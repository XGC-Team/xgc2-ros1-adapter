#!/usr/bin/env python3
"""Private-master start order and peer replay checks; --long measures 60 s outages."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import random
import signal
import socket
import socketserver
import subprocess
import tempfile
import threading
import time
from process_test import http_call, scene_session, stop, wait_socket


def until(predicate, timeout=15):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('Condition did not recover')


def resources(pid):
    text = Path('/proc/{}/stat'.format(pid)).read_text()
    fields = text[text.rfind(')') + 2:].split()
    cpu = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    threads = len(list(Path('/proc/{}/task'.format(pid)).iterdir()))
    fds = len(list(Path('/proc/{}/fd'.format(pid)).iterdir()))
    return cpu, threads, fds


def idle_window(node, log, peer):
    baseline = resources(node.pid)
    maximum = list(baseline)
    end = time.monotonic() + 60
    while time.monotonic() < end:
        assert node.poll() is None
        time.sleep(max(0.001, min(1, end - time.monotonic())))
        current = resources(node.pid)
        maximum[1] = max(maximum[1], current[1])
        maximum[2] = max(maximum[2], current[2])
    final = resources(node.pid)
    cpu = final[0] - baseline[0]
    assert cpu < 1.2, (peer, cpu)
    assert final[1] <= baseline[1] and maximum[1] <= baseline[1], (peer, baseline, final, maximum)
    assert final[2] <= baseline[2] + 1 and maximum[2] <= baseline[2] + 2, (peer, baseline, final, maximum)
    lines = [line for line in log.read_text().splitlines() if 'scene adapter: ' + peer + ' waiting' in line]
    assert 1 <= len(lines) <= 7, (peer, lines)
    print('PASS: {} 60 s outage: {:.3f} s CPU, threads {}/{}, fds {}/{}, {} log lines'.format(
        peer, cpu, baseline[1], final[1], baseline[2], final[2], len(lines)), flush=True)


class World(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    def __init__(self, path, instance, refuse=False):
        self.path = path
        self.instance = instance
        self.applies = []
        self.motions = []
        self.describes = 0
        self.refuse = refuse
        super().__init__(str(path), Handler)
        self.owner = threading.Thread(target=self.serve_forever, kwargs={'poll_interval': 0.05})
        self.owner.start()

    def close(self):
        self.shutdown()
        self.owner.join()
        self.server_close()
        self.path.unlink(missing_ok=True)


class Handler(socketserver.StreamRequestHandler):
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
        text = self.rfile.read(int(headers.get('content-length', 0)))
        body = json.loads(text) if text else None
        world = self.server
        if method == 'GET':
            assert route == '/v1/describe?wait_ready_ms=30000'
            assert 'x-xrpc-instance-id' not in headers
            world.describes += 1
            value = {'service': 'xgc2.simulation', 'api_version': 'v1', 'ready': True, 'instance_id': world.instance,
                     'service_ref': {'target_id': 'fixture'}}
        else:
            if headers['x-xrpc-instance-id'] != world.instance:
                value = {'error': {'code': 'conflict', 'message': 'service instance identity mismatch'}}
            else:
                if route == '/v1/extensions/scene/apply':
                    world.applies.append(body)
                else:
                    assert route == '/v1/extensions/scene/motion'
                    world.motions.append(body)
                value = {'id': headers['x-request-id'], 'state': 'succeeded',
                         'result': {'epoch': body['epoch'], 'revision': body['revision'], 'scene_time': 0, 'playing': False}}
                if world.refuse and route.endswith('/apply'):
                    value = {'id': headers['x-request-id'], 'state': 'failed', 'error': 'stale epoch/revision conflict'}
        payload = json.dumps(value).encode()
        status = '409 Conflict' if 'error' in value and 'id' not in value else '200 OK'
        self.wfile.write(('HTTP/1.1 ' + status + '\r\nContent-Type: application/json\r\nConnection: close\r\n'
            'X-Xrpc-Instance-ID: ' + world.instance + '\r\nX-Request-ID: ' + headers['x-request-id'] + '\r\n' +
            'Content-Length: {}\r\n\r\n'.format(len(payload))).encode() + payload)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('executable', type=Path)
    parser.add_argument('--peer', choices=['master', 'world'], required=True)
    parser.add_argument('--long', action='store_true')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='lf-edge-startup-') as directory:
        root = Path(directory)
        while True:
            port = random.randint(20000, 40000)
            with socket.socket() as probe:
                try:
                    probe.bind(('127.0.0.1', port))
                    break
                except OSError:
                    pass
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:' + str(port), ROS_HOME=str(root / 'ros-home'), ROS_IP='127.0.0.1')
        scene_socket, lidar_socket, world_socket = [root / name for name in [
            'scene.sock', 'lidar.sock', '0123456789abcdef0123456789abcdef.sock']]
        source = root / 'scene.yaml'
        source.write_text(json.dumps({'schema': 'xgc2.scene.v1', 'id': 'start-order', 'frame': 'world', 'obstacles': [
            {'id': 'wall', 'name': 'Wall', 'pose': {'position': [4, 0, 0]},
             'parts': [{'id': 'main', 'geometry': {'type': 'box', 'size': [1, 4, 2]}}]}]}))
        log = root / 'node.log'
        master, node, world = None, None, None
        subscribers = []
        def start_master():
            with (root / 'master.log').open('a') as output:
                # This test replaces the master itself. Avoid roslaunch/rosout
                # shutdown/startup work consuming the 5-second liveness budget.
                process = subprocess.Popen(['rosmaster', '--core', '-p', str(port)], env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            wait_socket(('127.0.0.1', port), process)
            return process
        def start_node(profile):
            command = [str(args.executable.resolve()), '--socket-path', str(scene_socket), '--target-id', 'fixture',
                '--run-id', 'start-order', '--node-name', 'lf_start_order', '--ros-master-uri', env['ROS_MASTER_URI'],
                '--ros-ip', '127.0.0.1', '_scene_file:=' + str(source), '_lidar_xrpc_socket:=' + str(lidar_socket), '_profile:=' + profile]
            if profile == 'gazebo':
                command += ['_simulation_endpoint:=' + str(world_socket), '--session-stdin']
            with log.open('a') as output:
                process = subprocess.Popen(command, env=env, stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            if profile == 'gazebo':
                document = {'runMode': 'simulation', 'robots': [{'namespace': '/lf_start_order',
                    'binding': {'simulationSensors': {'simpleLidar': {'enabled': True, 'mode': 'depth_frustum', 'acceleration': 'cpu', 'hRes': 1, 'vRes': 1}}}}], 'visualizationTopics': []}
                scene_session(document, source, root, profile)
                process.stdin.write(json.dumps(document).encode())
            process.stdin.close()
            wait_socket(str(scene_socket), process, unix=True)
            wait_socket(str(lidar_socket), process, unix=True)
            return process
        try:
            if args.peer == 'master':
                node = start_node('physical')
                for address in [scene_socket, lidar_socket]:
                    starting = http_call(address, 'GET', '/v1/describe?wait_ready_ms=0', 'waiting')
                    assert not starting['ready'] and starting['facts']['ros_master'] == 'waiting', starting
                    before = time.monotonic()
                    held = http_call(address, 'GET', '/v1/describe?wait_ready_ms=300', 'held')
                    assert not held['ready'] and time.monotonic() - before >= 0.25
                if args.long:
                    before = http_call(scene_socket, 'GET', '/v1/describe', 'attempts-before')['facts']['master_probe_attempts']
                    idle_window(node, log, 'ros_master')
                    facts = http_call(scene_socket, 'GET', '/v1/describe', 'attempts')['facts']
                    attempts = facts['master_probe_attempts'] - before
                    assert attempts <= 15, facts
                    print('PASS: master liveness attempts in window {} ({} total)'.format(attempts, facts['master_probe_attempts']), flush=True)
                with ThreadPoolExecutor(max_workers=2) as calls:
                    scene_ready = calls.submit(http_call, scene_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'ready-scene')
                    lidar_ready = calls.submit(http_call, lidar_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'ready-lidar')
                    time.sleep(0.1)
                    assert not scene_ready.done() and not lidar_ready.done()
                    master = start_master()
                    assert scene_ready.result()['ready'] and lidar_ready.result()['ready']
                stop(master, signal.SIGINT)
                master = start_master()
                assert node.wait(timeout=10) == 3, 'Replaced master did not terminate its old binding'
                stop(master, signal.SIGINT)
                master = None
                node = start_node('physical')
                assert not http_call(scene_socket, 'GET', '/v1/describe', 'term-wait')['ready']
                stop(node, signal.SIGTERM)
                assert node.returncode == 0
                print('PASS: pre-master readiness, held describe, late master, replacement exit 3 and SIGTERM while waiting', flush=True)
            else:
                master = start_master()
                assert not world_socket.exists()
                if args.long:
                    # A stopped listener can leave its socket inode behind.
                    # Exercise transport backoff as well as the appearance watch.
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stale:
                        stale.bind(str(world_socket))
                node = start_node('gazebo')
                scene = http_call(scene_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'ready')
                lidar = http_call(lidar_socket, 'GET', '/v1/describe?wait_ready_ms=30000', 'lidar-ready')
                assert scene['ready'] and lidar['ready'] and lidar['facts']['sensors'] == 1
                assert not lidar['facts']['scene_ready']
                if not args.long:
                    assert not world_socket.exists(), 'Readiness required the world socket'
                instance = scene['instance_id']
                def envelope():
                    return http_call(scene_socket, 'GET', '/v1/scene', 'scene', instance)
                current = envelope()
                assert not current['synchronized'] and current['syncRetryable']
                obstacle = current['document']['obstacles'][0]
                obstacle['name'] = 'Offline edit'
                receipt = http_call(scene_socket, 'POST', '/v1/commands', 'offline-edit', instance,
                    {'operation': 'update', 'expectedEpoch': current['epoch'], 'expectedRevision': current['revision'], 'obstacle': obstacle})
                assert receipt['success'] and not receipt['synchronized'], receipt
                current = envelope()
                assert current['document']['obstacles'][0]['name'] == 'Offline edit'
                until(lambda: bool(envelope()['consumers']))
                if args.long:
                    time.sleep(0.3)
                    until(lambda: 'connect_attempts' in envelope()['consumers'][0])
                    before = envelope()['consumers'][0]['connect_attempts']
                    idle_window(node, log, 'world_host')
                    report = envelope()['consumers'][0]
                    attempts = report['connect_attempts'] - before
                    assert 0 < attempts <= 15, report
                    print('PASS: world discovery attempts in window {} ({} total)'.format(attempts, report['connect_attempts']), flush=True)
                    world_socket.unlink()
                world = World(world_socket, 'world-first')
                until(lambda: envelope()['synchronized'])
                assert len(world.applies) == 1 and world.applies[0]['document'] == current['document']
                assert world.applies[0]['epoch'] == current['epoch'] and world.applies[0]['revision'] == current['revision']
                assert envelope()['consumers'][0]['instance_id'] == 'world-first'
                time.sleep(0.15)
                assert len(world.applies) == 1 and world.describes == 1
                import rospy
                from geometry_msgs.msg import PoseStamped
                from sensor_msgs.msg import PointCloud2
                os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'], ROS_HOME=env['ROS_HOME'], ROS_IP='127.0.0.1')
                rospy.init_node('lf_start_order_observer', anonymous=True, disable_signals=True)
                clouds = []
                subscribers.append(rospy.Subscriber('/lf_start_order/simple_lidar/points', PointCloud2, clouds.append, queue_size=1))
                pose = rospy.Publisher('/lf_start_order/pose', PoseStamped, queue_size=1)
                until(lambda: pose.get_num_connections() > 0)
                time.sleep(0.3)
                assert not clouds, 'Fleet published without poses'
                def send():
                    sample = PoseStamped()
                    sample.header.frame_id = 'world'
                    sample.header.stamp = rospy.Time.now()
                    sample.pose.orientation.w = 1
                    pose.publish(sample)
                    return bool(clouds)
                until(send)
                assert clouds[-1].width > 0
                world.close()
                world = None
                until(lambda: not envelope()['synchronized'])
                assert http_call(scene_socket, 'GET', '/v1/describe', 'peer-down')['ready']
                until(lambda: not http_call(lidar_socket, 'GET', '/v1/describe', 'lidar-down')['facts']['scene_ready'])
                world = World(world_socket, 'world-second')
                until(lambda: envelope()['synchronized'])
                assert len(world.applies) == 1 and world.applies[0]['document'] == current['document']
                assert world.applies[0]['epoch'] == current['epoch'] and world.applies[0]['revision'] == current['revision']
                assert envelope()['consumers'][0]['instance_id'] == 'world-second'
                time.sleep(0.3)
                assert len(world.applies) == 1 and world.describes == 1
                # No socket event: a mismatch on the old pinned instance must
                # rediscover through the same world rendezvous path.
                world.instance = 'world-rebound'
                current = envelope()
                receipt = http_call(scene_socket, 'POST', '/v1/commands', 'rebind', instance,
                    {'operation': 'resync', 'expectedEpoch': current['epoch'], 'expectedRevision': current['revision']})
                assert receipt['success']
                until(lambda: envelope()['synchronized'])
                assert envelope()['consumers'][0]['instance_id'] == 'world-rebound'
                assert len(world.applies) == 2 and world.describes == 2
                world.close()
                world = World(world_socket, 'world-conflict', refuse=True)
                until(lambda: any('stale epoch' in value['message'] for value in envelope()['consumers']))
                assert http_call(scene_socket, 'GET', '/v1/describe', 'conflict')['ready']
                world.refuse = False
                current = envelope()
                receipt = http_call(scene_socket, 'POST', '/v1/commands', 'resync-conflict', instance,
                    {'operation': 'resync', 'expectedEpoch': current['epoch'], 'expectedRevision': current['revision']})
                assert receipt['success']
                until(lambda: envelope()['synchronized'])
                assert len(world.applies) == 2 and world.applies[0]['revision'] != world.applies[1]['revision']
                stop(node, signal.SIGTERM)
                assert node.returncode == 0
                print('PASS: late world, offline edits, new-instance replay, conflict resync, no-pose silence and pose recovery', flush=True)
        except BaseException:
            print(log.read_text() if log.exists() else 'No node log', flush=True)
            raise
        finally:
            for subscriber in subscribers:
                subscriber.unregister()
            if 'rospy' in locals():
                rospy.signal_shutdown('Start-order test completed')
            stop(node, signal.SIGTERM)
            if world:
                world.close()
            stop(master, signal.SIGINT)


if __name__ == '__main__':
    main()
