#!/usr/bin/env python3
"""Finite process smoke test with an owned master, ROS_HOME and child groups.

Run after sourcing the ROS underlay, passing the built xgc2_scene_adapter.
The CMake gtests exercise the same node without a master; this checks only
the real parameter, topic, timer, TF and process-lifecycle wiring.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import random
import signal
import socket
import socketserver
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


class SimulationServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    """Native application receipts, with a held startup for readiness checks."""
    def __init__(self, path):
        self.applying = threading.Event()
        self.release = threading.Event()
        self.refuse = False
        super().__init__(str(path), SimulationHandler)


class SimulationHandler(socketserver.StreamRequestHandler):
    def handle(self):
        line = self.rfile.readline().decode().strip()
        if not line:
            return
        method, route, _ = line.split()
        headers = {}
        while True:
            line = self.rfile.readline().decode().strip()
            if not line:
                break
            key, value = line.split(':', 1)
            headers[key.lower()] = value.strip()
        body = json.loads(self.rfile.read(int(headers['content-length'])))
        assert headers['x-xrpc-instance-id'] == 'simulation-fixture'
        assert method == 'POST' and route in ['/v1/extensions/scene/apply', '/v1/extensions/scene/motion']
        self.server.applying.set()
        assert self.server.release.wait(timeout=10), 'Native apply was not released'
        if self.server.refuse:
            receipt = {'id': headers['x-request-id'], 'state': 'failed', 'error': 'native application refused'}
        else:
            receipt = {'id': headers['x-request-id'], 'state': 'succeeded',
                       'result': {'epoch': body['epoch'], 'revision': body['revision']}}
        payload = json.dumps(receipt).encode()
        self.wfile.write(('HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n'
                          'X-Xrpc-Instance-ID: simulation-fixture\r\n'
                          'X-Request-ID: ' + headers['x-request-id'] + '\r\n' +
                          'Content-Length: {}\r\n\r\n'.format(len(payload))).encode() + payload)


def fleet_process(args, root, node_log, namespace, source):
    """Real adapter timer, both discovery sockets and enable control."""
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    path = root / 'fleet.sock'
    scene_socket = root / 'fleet-scene.sock'
    sim_path = root / 'simulation.sock'
    native = SimulationServer(sim_path)
    serving = threading.Thread(target=native.serve_forever, kwargs={'poll_interval': 0.05})
    serving.start()
    node = None
    subscribers = []
    poses = []
    observed = {}
    counts = {}
    changed = threading.Condition()
    def receive(name, message):
        with changed:
            observed[name] = message
            counts[name] = counts.get(name, 0) + 1
            changed.notify_all()
    session = {'runMode': 'simulation', 'robots': [
        {'namespace': '/lf_process/A', 'binding': {'simulationSensors': {'simpleLidar': {
            'enabled': True, 'mode': 'raycast', 'acceleration': 'cpu', 'publishBeams': True,
            'hRes': 1, 'vRes': 1, 'hFovDeg': 90, 'rateHz': 20}}}},
        {'namespace': '/lf_process/B', 'binding': {'simulationSensors': {'simpleLidar': True}}}],
        'visualizationTopics': [[{'topic': '/xgc/scene/reference_cloud',
            'messageType': 'sensor_msgs/PointCloud2', 'role': 'semantic', 'visible3d': True}]]}
    simulation_ref = {'target_id': 'fixture', 'service': 'xgc2.simulation', 'api_version': 'v1',
                      'instance_id': 'simulation-fixture', 'profile': 'http.v1',
                      'endpoint': {'kind': 'unix', 'address': str(sim_path)}}
    command = [str(args.executable.resolve()), '--socket-path', str(scene_socket), '--target-id', 'fixture',
               '--run-id', 'fleet-smoke', '--node-name', 'scene_fleet_smoke',
               '--ros-master-uri', os.environ['ROS_MASTER_URI'], '--ros-ip', '127.0.0.1',
               '_scene_file:=' + str(source), '_profile:=gazebo', '__ns:=' + namespace,
               '_simulation_service_ref_json:=' + json.dumps(simulation_ref),
               '_lidar_xrpc_socket:=' + str(path), '--session-stdin',
               '_lidar_defaults_json:={"worker_threads":2}', '_lidar_enabled:=false']
    try:
        with node_log.open('a') as output:
            node = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        node.stdin.write(json.dumps(session).encode())
        node.stdin.close()
        wait_socket(str(scene_socket), node, unix=True)
        wait_socket(str(path), node, unix=True)
        assert native.applying.wait(timeout=5), 'Native application did not start'
        for address in [scene_socket, path]:
            assert not http_call(address, 'GET', '/v1/describe?wait_ready_ms=0', 'starting')['ready']
        with ThreadPoolExecutor(max_workers=2) as calls:
            authoring = calls.submit(http_call, scene_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'scene-ready')
            lidar = calls.submit(http_call, path, 'GET', '/v1/describe?wait_ready_ms=30000', 'lidar-ready')
            time.sleep(0.1)
            assert not authoring.done() and not lidar.done()
            native.release.set()
            described, lidar_described = authoring.result(timeout=5), lidar.result(timeout=5)
        assert described['ready'] and lidar_described['ready']
        assert lidar_described['facts'] == {'target_id': 'fixture', 'profile': 'gazebo', 'sensors': 1, 'backend': 'cpu'}
        instance = lidar_described['instance_id']
        assert not any(topic.startswith('/lf_process/B/simple_lidar/') for topic, _ in rospy.get_published_topics())
        for robot in session['robots'][:1]:
            name = robot['namespace']
            for suffix in ['points', 'beams']:
                topic = name + '/simple_lidar/' + suffix
                subscribers.append(rospy.Subscriber(topic, PointCloud2, lambda msg, topic=topic: receive(topic, msg)))
            poses.append(rospy.Publisher('/vrpn_client_node' + name + '/pose', PoseStamped, queue_size=1))
        subscribers.append(rospy.Subscriber('/xgc/scene/reference_cloud', PointCloud2, lambda msg: receive('reference', msg)))
        def send(duration, predicate=None):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                for publisher in poses:
                    pose = PoseStamped()
                    pose.header.frame_id = 'world'
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = -4
                    pose.pose.orientation.w = 1
                    publisher.publish(pose)
                time.sleep(0.02)
                with changed:
                    if predicate and predicate():
                        return True
            return False
        send(0.4)
        assert not observed, 'Initially disabled fleet published data'
        receipt = http_call(path, 'POST', '/v1/configure', 'enable', instance,
                            {'expected_revision': 1, 'configuration': {'enabled': True}, 'operation_timeout_ms': 5000})
        assert receipt['state'] == 'succeeded'
        assert send(5, lambda: len(observed) == 3 and observed['reference'].width > 0), list(observed)
        with changed:
            for name, cloud in observed.items():
                assert cloud.header.frame_id == 'world' and cloud.height == 1 and cloud.is_dense, name
                assert cloud.point_step == (32 if name.endswith('/beams') else 12), name
                assert cloud.header.stamp.to_nsec() > 0, name
        receipt = http_call(path, 'POST', '/v1/configure', 'disable', instance,
                            {'expected_revision': 2, 'configuration': {'enabled': False}, 'operation_timeout_ms': 5000})
        assert receipt['state'] == 'succeeded'
        time.sleep(0.1)
        with changed:
            before = dict(counts)
        send(0.3)
        assert counts == before, 'Disabled fleet still published'
        receipt = http_call(path, 'POST', '/v1/configure', 'reenable', instance,
                            {'expected_revision': 3, 'configuration': {'enabled': True}, 'operation_timeout_ms': 5000})
        assert receipt['state'] == 'succeeded'
        assert send(3, lambda: counts['/lf_process/A/simple_lidar/points'] > before['/lf_process/A/simple_lidar/points'])
        current = http_call(scene_socket, 'GET', '/v1/scene', 'current', described['instance_id'])
        native.release.clear()
        native.applying.clear()
        with ThreadPoolExecutor(max_workers=1) as calls:
            edit = calls.submit(http_call, scene_socket, 'POST', '/v1/commands', 'native-refusal', described['instance_id'],
                                {'operation': 'clear', 'expectedEpoch': current['epoch'], 'expectedRevision': current['revision']})
            assert native.applying.wait(timeout=5)
            with changed:
                assert changed.wait_for(lambda: observed['reference'].width == 0, timeout=3), 'Pending map stayed latched'
            native.refuse = True
            native.release.set()
            refused = edit.result(timeout=5)
        assert not refused['synchronized']
        time.sleep(0.1)
        with changed:
            before = dict(counts)
        send(0.3)
        assert counts == before, 'Refused simulator revision left scans enabled'
        stop(node, signal.SIGTERM)
        assert node.returncode == 0
        assert not path.exists() and not scene_socket.exists()
        invalid = [arg for arg in command if not arg.startswith('_lidar_defaults_json:=')]
        invalid.append('_lidar_defaults_json:={"typo":1}')
        result = subprocess.run(invalid, input=json.dumps(session), text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, start_new_session=True, timeout=10)
        assert result.returncode == 1 and '~lidar_defaults_json' in result.stdout, result.stdout
        assert not path.exists() and not scene_socket.exists()
        for refusal in ['cpu', 'namespace', 'beams']:
            invalid_session = json.loads(json.dumps(session))
            selected = invalid_session['robots'][0]['binding']['simulationSensors']['simpleLidar']
            if refusal == 'cpu':
                selected.pop('acceleration')
                message = 'sampled-scene lidar requires an explicit CPU acceleration selection'
            elif refusal == 'namespace':
                invalid_session['robots'][0]['namespace'] = 'invalid namespace'
                message = 'invalid lidar namespace'
            else:
                selected['mode'] = 'penetrating'
                message = 'penetrating sampled clouds have no beams'
            result = subprocess.run(command, input=json.dumps(invalid_session), text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, start_new_session=True, timeout=10)
            assert result.returncode == 1 and message in result.stdout, result.stdout
            assert not path.exists() and not scene_socket.exists()
        for profile in ['gazebo', 'xsim', 'physical']:
            invalid_command = [arg.replace('_profile:=gazebo', '_profile:=' + profile) for arg in command]
            for invalid_text in ['{', '{"runMode":"invalid","robots":[],"visualizationTopics":[]}', ' ' * (8388608 + 1)]:
                result = subprocess.run(invalid_command, input=invalid_text, text=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, start_new_session=True, timeout=10)
                assert result.returncode == 2 and '--session-stdin' in result.stdout, result.stdout
                assert not path.exists() and not scene_socket.exists()
        print('PASS: two sockets held unbound readiness, sampled/default roster, beams/reference, enable control, stdin/refusal errors, SIGTERM')
    finally:
        native.release.set()
        stop(node, signal.SIGTERM)
        for subscriber in subscribers:
            subscriber.unregister()
        for publisher in poses:
            publisher.unregister()
        native.shutdown()
        serving.join()
        native.server_close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('executable', type=Path)
    parser.add_argument('--edge-test', type=Path)
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
        lidar_socket = root / 'empty-lidar.sock'
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
                            '_scene_file:=' + str(source), '_profile:=physical', '__ns:=' + namespace,
                            '_lidar_xrpc_socket:=' + str(lidar_socket)]
            with node_log.open('w') as output:
                node = subprocess.Popen(command_line, stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            # With no --session-stdin flag, even invalid input is not read.
            node.stdin.write(b'not a session document')
            node.stdin.close()
            wait_socket(str(socket_path), node, unix=True)
            # Exactly one successful unbound readiness call; no describe polling.
            describe = http_call(socket_path, 'GET', '/v1/describe?wait_ready_ms=30000', 'readiness')
            assert describe['ready'] is True and describe['facts']['profile'] == 'physical', describe
            instance = describe['instance_id']
            assert describe['facts']['run_id'] == run_id
            empty = http_call(lidar_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'empty-fleet')
            assert empty['ready'] and empty['facts'] == {'target_id': 'fixture', 'profile': 'physical', 'sensors': 0, 'backend': 'none'}
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
            assert not lidar_socket.exists()
            with changed:
                assert changed.wait_for(lambda: not json.loads(observed['document'].data)['online'], timeout=3)
            # A declared lidar service also answers in xsim with no engine,
            # even when stdin contains a roster with a sampled selection.
            with node_log.open('a') as output:
                node = subprocess.Popen(command_line + ['--session-stdin'], stdin=subprocess.DEVNULL,
                                        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            wait_socket(str(lidar_socket), node, unix=True)
            empty = http_call(lidar_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'empty-stdin')
            assert empty['ready'] and empty['facts']['sensors'] == 0
            stop(node, signal.SIGTERM)
            assert node.returncode == 0 and not lidar_socket.exists()
            xsim_command = [arg.replace('_profile:=physical', '_profile:=xsim') for arg in command_line]
            xsim_command.append('--session-stdin')
            session = {'runMode': 'simulation', 'robots': [{'namespace': '/lf_absent',
                'binding': {'simulationSensors': {'simpleLidar': {'enabled': True,
                    'mode': 'depth_frustum', 'acceleration': 'cpu'}}}}], 'visualizationTopics': []}
            with node_log.open('a') as output:
                node = subprocess.Popen(xsim_command, stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            node.stdin.write(json.dumps(session).encode())
            node.stdin.close()
            wait_socket(str(lidar_socket), node, unix=True)
            empty = http_call(lidar_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'xsim-fleet')
            assert empty['ready'] and empty['facts'] == {'target_id': 'fixture', 'profile': 'xsim', 'sensors': 0, 'backend': 'none'}
            assert not any(topic.startswith('/lf_absent/simple_lidar/') for topic, _ in rospy.get_published_topics())
            stop(node, signal.SIGTERM)
            assert node.returncode == 0 and not lidar_socket.exists()
            if args.edge_test:
                env = dict(os.environ, XGC_LIDAR_TEST_MASTER='1')
                child = subprocess.Popen([str(args.edge_test.resolve())], env=env, start_new_session=True)
                try:
                    assert child.wait(timeout=45) == 0, 'ROS edge transport assertions failed'
                finally:
                    stop(child, signal.SIGTERM)
            fleet_process(args, root, node_log, namespace, source)
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
