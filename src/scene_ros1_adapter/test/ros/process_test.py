#!/usr/bin/env python3
"""Finite process smoke test with an owned master, ROS_HOME and child groups.

Run after sourcing the ROS underlay, passing the built xgc2_scene_adapter.
The CMake gtests exercise the same node without a master; this checks only
the real parameter, topic, timer, TF and process-lifecycle wiring.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import signal
import socket
import statistics
import subprocess
import tempfile
import threading
import time


def stop(process, signum):
    if process is None:
        return
    if process.poll() is None:
        os.killpg(process.pid, signum)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        raise AssertionError('Child exceeded its shutdown budget')


def wait_socket(address, process, unix=False):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise AssertionError('Child exited before its socket was available')
        try:
            with socket.socket(socket.AF_UNIX if unix else socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.settimeout(0.2)
                probe.connect(address)
            return
        except OSError:
            time.sleep(0.05)
    raise AssertionError('Child did not publish its socket')


def http_call(path, method, route, request_id, instance=None, body=None):
    payload = b'' if body is None else json.dumps(body, separators=(',', ':')).encode()
    headers = ['{} {} HTTP/1.1'.format(method, route), 'Host: localhost',
               'Connection: close', 'X-Xrpc-Timeout-Ms: 35000',
               'X-Request-ID: ' + request_id, 'Content-Length: ' + str(len(payload))]
    if instance:
        headers.append('X-Xrpc-Instance-ID: ' + instance)
    if body is not None:
        headers.append('Content-Type: application/json')
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(36)
        client.connect(str(path))
        client.sendall(('\r\n'.join(headers) + '\r\n\r\n').encode() + payload)
        chunks = []
        while True:
            chunk = client.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
    head, _, response = b''.join(chunks).partition(b'\r\n\r\n')
    status = int(head.split(b' ', 2)[1])
    assert status == 200, (status, response)
    return json.loads(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('executable', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='sa-node-process-') as root:
        root = Path(root)
        port = random.SystemRandom().randint(20000, 40000)
        # Reserve-check the chosen private port before launching the master.
        with socket.socket() as check:
            check.bind(('127.0.0.1', port))
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
        os.environ['ROS_HOME'] = str(root / 'ros-home')
        os.environ['ROS_IP'] = '127.0.0.1'
        os.environ.pop('ROS_HOSTNAME', None)
        scene = {'schema': 'xgc2.scene.v1', 'id': 'process-test', 'frame': 'world', 'obstacles': [
            {'id': 'held', 'parts': [{'id': 'body', 'geometry': {'type': 'box', 'size': [1, 2, 3]}}]},
            {'id': 'moving', 'motion': {'type': 'constant_twist', 'linear': [1, 0, 0]},
             'parts': [{'id': 'body', 'geometry': {'type': 'sphere', 'radius': 0.5}}]},
        ]}
        source = root / 'scene.yaml'
        source.write_text(json.dumps(scene))
        socket_path = root / 'scene.sock'
        run_id = 'smoke-{}'.format(port)
        namespace = '/sa_node_{}/'.format(port)
        master = node = None
        subscribers = []
        observed = {}
        states = []
        changed = threading.Condition()
        master_log = root / 'master.log'
        node_log = root / 'node.log'
        try:
            with master_log.open('w') as output:
                master = subprocess.Popen(['roscore', '-p', str(port)], stdout=output, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            wait_socket(('127.0.0.1', port), master)
            import rospy
            from std_msgs.msg import String
            from visualization_msgs.msg import MarkerArray
            from tf2_msgs.msg import TFMessage
            from xgc2_geometry_msgs.msg import SceneSnapshot, SceneState
            rospy.init_node('sa_node_observer_{}'.format(port), disable_signals=True)

            def receive(name, message):
                with changed:
                    observed[name] = message
                    if name == 'state':
                        states.append(message)
                    changed.notify_all()

            command_line = [str(args.executable.resolve()), '--socket-path', str(socket_path), '--target-id', 'fixture',
                            '--run-id', run_id, '--node-name', 'scene_' + run_id,
                            '--ros-master-uri', os.environ['ROS_MASTER_URI'], '--ros-ip', '127.0.0.1',
                            '_scene_file:=' + str(source), '_profile:=physical', '__ns:=' + namespace]
            with node_log.open('w') as output:
                node = subprocess.Popen(command_line, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            wait_socket(str(socket_path), node, unix=True)
            # Exactly one successful unbound readiness call; no describe polling.
            describe = http_call(socket_path, 'GET', '/v1/describe?wait_ready_ms=30000', 'readiness')
            assert describe['ready'] is True and describe['facts']['profile'] == 'physical', describe
            instance = describe['instance_id']
            assert describe['facts']['run_id'] == run_id
            # Subscribe after startup to prove that definition outputs latch.
            for name, message_type in [('snapshot', SceneSnapshot), ('state', SceneState),
                                       ('document', String), ('markers', MarkerArray)]:
                subscribers.append(rospy.Subscriber(namespace + name, message_type,
                                                    lambda message, name=name: receive(name, message)))
            subscribers.append(rospy.Subscriber('/tf_static', TFMessage, lambda message: receive('fixed', message)))
            subscribers.append(rospy.Subscriber('/tf', TFMessage, lambda message: receive('moving', message)))
            with changed:
                assert changed.wait_for(lambda: len(observed) == 6 and len(states) >= 10, timeout=10), list(observed)
                snapshot = observed['snapshot']
                assert snapshot.scene_id == 'process-test' and snapshot.revision == 1
                assert [body.id for body in snapshot.obstacles] == ['held', 'moving']
                assert observed['markers'].markers[0].action == 3
                prefix = 'xgc_scene_' + hashlib.sha256(namespace.encode()).hexdigest()[:12] + '/' + snapshot.epoch
                assert observed['fixed'].transforms[0].child_frame_id == prefix + '/fixed/held', observed['fixed']
                assert observed['moving'].transforms[0].child_frame_id == prefix + '/moving/moving', observed['moving']
                stamps = [message.header.stamp.to_sec() for message in states[-10:]]
                period = statistics.median(b - a for a, b in zip(stamps, stamps[1:]))
                assert 0.02 < period < 0.06, period
            before = http_call(socket_path, 'GET', '/v1/scene', 'before', instance)
            play = {'operation': 'play', 'expectedEpoch': before['epoch'], 'expectedRevision': 1}
            first = http_call(socket_path, 'POST', '/v1/commands', 'play', instance, play)
            assert first['success'] and first['playing']
            with changed:
                assert changed.wait_for(lambda: states[-1].playing and states[-1].scene_time > 0.1, timeout=5)
                assert states[-1].obstacles[0].pose.position.x == 0
                assert states[-1].obstacles[1].pose.position.x > 0
            assert http_call(socket_path, 'POST', '/v1/commands', 'play', instance, play) == first
            assert http_call(socket_path, 'POST', '/v1/commands/result', 'receipt', instance, {'request_id': 'play'}) == first
            stop(node, signal.SIGTERM)
            assert node.returncode == 0, node.returncode
            assert not socket_path.exists()
            with changed:
                assert changed.wait_for(lambda: not json.loads(observed['document'].data)['online'], timeout=3)
            # A fresh incarnation binds the same master and must end when that
            # dependency stays unreachable, with the common lifecycle status.
            with node_log.open('a') as output:
                node = subprocess.Popen(command_line, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            wait_socket(str(socket_path), node, unix=True)
            restarted = http_call(socket_path, 'GET', '/v1/describe?wait_ready_ms=30000', 'restart-readiness')
            assert restarted['ready'] and restarted['instance_id'] != instance
            stop(master, signal.SIGINT)
            node.wait(timeout=10)
            assert node.returncode == 4, node.returncode
            assert not socket_path.exists()
            print('PASS: private ROS master {}, latched outputs, TF, {:.2f} Hz, receipts, SIGTERM and master-loss shutdown'.format(port, 1 / period))
        except BaseException:
            if node_log.exists():
                print(node_log.read_text())
            raise
        finally:
            for subscriber in subscribers:
                subscriber.unregister()
            if 'rospy' in locals():
                rospy.signal_shutdown('Process smoke test completed')
            try:
                stop(node, signal.SIGTERM)
            finally:
                stop(master, signal.SIGINT)


if __name__ == '__main__':
    main()
