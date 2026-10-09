"""Private ROS/gRPC integration and resource measurement for all three servers.

Run inside an isolated validation container; ROBOT_SERVER_BUILD_ROOT points to
its build/devel tree. This owns its ROS master and never contacts a station.
"""
import concurrent.futures
from collections import OrderedDict
import json
import http.client
import socket
import os
from pathlib import Path
import signal
import struct
import socketserver
import platform
import subprocess
import tempfile
import threading
import time
import unittest
import uuid
import xmlrpc.client

import grpc
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, Twist
from sensor_msgs.msg import Imu
from xgc.semantic.common.v1 import telemetry_pb2
from xgc.semantic.ground.v1 import control_pb2
from xgc.semantic.aerial.v1 import control_pb2 as aerial_control
from xgc.semantic.common.v1 import control_pb2 as common_control
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse, CommandLong, CommandLongResponse
from xgc.robot.v1 import server_pb2 as wire
from xgc.adapter.v1 import adapter_pb2 as operation
from xgc.v1 import message_pb2 as payload

ROOT = Path(os.environ.get('ROBOT_SERVER_BUILD_ROOT', '/work'))
KINDS = {
    'px4': ('xgc_px4_multirotor_ros1_adapter', 'xgc2-px4-multirotor-ros1-adapter'),
    'scout': ('xgc_scout_mini_ros1_adapter', 'xgc2-scout-mini-ros1-adapter'),
    'mecanum': ('xgc_mecanum_ugv_ros1_adapter', 'xgc2-mecanum-ugv-ros1-adapter'),
}

def close_observer(process):
    if process.poll() is None:
        process.terminate()
        process.wait(timeout=15)

def imu_topic(kind, robot_id):
    return '/' + robot_id + ('/mavros/imu/data' if kind == 'px4' else '/imu/data_raw' if kind == 'scout' else '/imu')


def fixture_imu(stamp):
    message = Imu()
    message.header.stamp = stamp
    message.header.frame_id = 'source-body'
    message.orientation.w = 1
    message.angular_velocity.z = .5
    message.linear_acceleration.z = 9.80665
    message.orientation_covariance = [.001 if i % 4 == 0 else 0 for i in range(9)]
    message.angular_velocity_covariance = [.002 if i % 4 == 0 else 0 for i in range(9)]
    message.linear_acceleration_covariance = [.003 if i % 4 == 0 else 0 for i in range(9)]
    return message


def cgroup_cpu():
    path = Path('/sys/fs/cgroup/cpu.stat')
    return {key:int(value) for key,value in (line.split() for line in path.read_text().splitlines())} if path.exists() else {}


class Client:
    def __init__(self, kind, name=None, master=None, bootstrap=False):
        self._closed = False
        package, provider = KINDS[kind]
        self.kind = kind
        installed = os.environ.get('ROBOT_SERVER_INSTALL_PREFIX')
        manifests = Path('/usr/share/xgc2') if installed else ROOT / 'build' / kind / 'runtime-manifests'
        self.profiles = {p['profileId']:p for p in json.loads((manifests /
            'robot-adapter-profiles' / (provider + '.json')).read_text())['profiles']}
        self.profile = next(p for key,p in self.profiles.items() if key.endswith('.physical.vrpn'))
        self.manifest = json.loads((manifests /
            'adapter-definitions' / (provider + '.json')).read_text())['adapters'][0]
        self.socket = str(ROOT / ((name or kind) + '-socket') / 'server.sock')
        self.master_uri = master or os.environ['ROS_MASTER_URI']
        bootstrap_message = wire.RobotServerBootstrap(socket_path=self.socket, provider_definition_id=provider,
            target_id='test', ros_environment={'ros_master_uri': self.master_uri, 'ros_ip': '127.0.0.1'})
        path = ROOT / ((name or kind) + '.pb')
        path.write_bytes(bootstrap_message.SerializeToString())
        path.chmod(0o600)
        self.log = open(ROOT / ((name or kind) + '-server.log'), 'w')
        self.started = time.monotonic()
        binary = (Path(installed) if installed else ROOT / 'devel') / 'lib' / package / (package + '_node')
        self.binary, self.provider = str(binary), provider
        environment = dict(os.environ)
        if name:
            # Additional private fixtures may share the same master; do not
            # replace the main type server's fixed ROS node registration.
            environment['ROS_NAMESPACE'] = '/private_' + name.replace('-', '_')
        if environment.get('ROBOT_SERVER_ALLOCATION_PRELOAD'):
            environment['LD_PRELOAD'] = environment['ROBOT_SERVER_ALLOCATION_PRELOAD']
        arguments = ['--adapter-bootstrap-file', str(path)] if bootstrap else [
            '--socket-path', self.socket, '--provider', provider,
            '--target-id', 'test',
            '--ros-master-uri', self.master_uri, '--ros-ip', '127.0.0.1']
        self.process = subprocess.Popen([str(binary), *arguments], stdout=self.log, stderr=self.log, env=environment)
        self.channel = grpc.insecure_channel('unix:' + self.socket, options=[
            ('grpc.initial_reconnect_backoff_ms',10), ('grpc.min_reconnect_backoff_ms',100),
            ('grpc.max_reconnect_backoff_ms',100)])
        self.instance_id = None
        self.health = self.unary('Health', wire.HealthRequest, wire.HealthResponse)
        self.apply = self.unary('ApplyMembers', wire.ApplyMembersRequest, wire.ApplyMembersResponse)
        self.remove = self.unary('RemoveMembers', wire.RemoveMembersRequest, wire.RemoveMembersResponse)
        self.execute = self.bound_call(self.channel.unary_unary('/xgc.robot.v1.RobotAdapterServerService/Execute',
            request_serializer=wire.ExecuteRequest.SerializeToString,
            response_deserializer=lambda raw: wire.ExecuteResponse.FromString(raw).event))
        self._status = self.bound_call(self.channel.unary_stream('/xgc.robot.v1.RobotAdapterServerService/SubscribeStatus',
            request_serializer=wire.SubscribeStatusRequest.SerializeToString, response_deserializer=wire.SubscribeStatusResponse.FromString))
        startup_deadline = time.monotonic() + 10
        try:
            grpc.channel_ready_future(self.channel).result(timeout=10)
            request_id = uuid.uuid4().hex
            describe = self.channel.unary_unary('/xgc.robot.v1.RobotAdapterServerService/Describe',
                request_serializer=wire.DescribeRequest.SerializeToString, response_deserializer=wire.DescribeResponse.FromString)
            response, call = describe.with_call(wire.DescribeRequest(),
                metadata=(('x-request-id', request_id),), timeout=max(0, startup_deadline - time.monotonic()))
            reference = response.service_ref
            metadata = tuple(call.initial_metadata())
            if (reference.target_id != 'test' or reference.service != 'xgc2.robot-adapter' or
                    reference.api_version != 'v1' or reference.profile != 'grpc.v1' or
                    reference.endpoint.kind != 'unix' or reference.endpoint.address != self.socket or
                    not reference.instance_id or response.provider_definition_id != provider or
                    [value for key, value in metadata if key == 'x-xrpc-instance-id'] != [reference.instance_id] or
                    [value for key, value in metadata if key == 'x-request-id'] != [request_id]):
                raise RuntimeError('robot server discovery identity mismatch')
            self.instance_id = reference.instance_id
            health = self.health(wire.HealthRequest(), timeout=max(0, startup_deadline - time.monotonic()))
            if not health.serving or health.instance_id != self.instance_id:
                raise RuntimeError('robot server health identity mismatch or not serving')
        except Exception:
            before_stop = self.process.poll()
            try:
                self.close()
            finally:
                print('robot server startup failed: kind=%s before_stop=%s exit=%s socket=%s\n%s' % (
                    kind, before_stop, self.process.returncode, self.socket,
                    Path(self.log.name).read_text()), flush=True)
            raise
        self.startup_ms = (time.monotonic() - self.started) * 1000

    def unary(self, name, request, response):
        return self.bound_call(self.channel.unary_unary('/xgc.robot.v1.RobotAdapterServerService/' + name,
            request_serializer=request.SerializeToString, response_deserializer=response.FromString))

    def bound_call(self, call):
        def metadata():
            return (('x-xrpc-instance-id', self.instance_id), ('x-request-id', uuid.uuid4().hex))
        def invoke(request, **kwargs):
            return call(request, metadata=metadata(), **kwargs)
        invoke.future = lambda request, **kwargs: call.future(request, metadata=metadata(), **kwargs)
        return invoke

    def status(self, request, timeout=30):
        return self._status(request, timeout=timeout)

    def member(self, index, epoch=1, run=None, profile=None):
        profile = profile or self.profile
        name = self.kind + str(index)
        member = wire.MemberRegistration(identity=wire.MemberIdentity(target_id='test', run_id=run or ('observer-' + name),
            robot_id=name, connection_epoch=epoch))
        config = member.configuration
        config.robot_id = name
        config.profile_id = profile['profileId']
        config.profile_digest = profile['profileDigest']
        config.parameters.update({'namespace': '/' + name, 'mocap_rigid_body': name,
            'ros_master_uri': self.master_uri, 'ros_ip': '127.0.0.1',
            'mocap_source_root': '/vrpn_client_node', 'localization_offset_x': '1.25',
            'localization_offset_y': '-2', 'localization_offset_z': '0.1',
            'positioning_frame_number': '3', 'positioning_comparison_threshold_m': '0.001'})
        if config.profile_id.endswith('.xsim.ros'):
            for key in list(config.parameters):
                if key not in profile['parameters']: del config.parameters[key]
            config.parameters['localization_pose_topic'] = '/' + name + '/pose'
            config.parameters['localization_twist_topic'] = '/' + name + '/twist'
        for channel in profile['channels']:
            config.channels.add(channel_id=channel['id'], enabled=True)
        return member

    def command(self, member, work, endpoint='arm', value=b'\x08\x01', deadline_seconds=2):
        capabilities = self.manifest['capabilityManifest']['capabilities']
        descriptor = next(e for c in capabilities for e in c['endpoints'] if e['endpointId'] == endpoint)
        schema = descriptor['inputSchema']
        command = operation.OperationRequest(context=operation.WorkContext(work_id=work, endpoint_id=endpoint,
            deadline=operation.Deadline(deadline_unix_nanos=time.time_ns() + int(deadline_seconds * 1_000_000_000))),
            input=payload.Payload(encoding=payload.PAYLOAD_ENCODING_PROTOBUF, value=value))
        command.input.schema.message_id = schema['messageId']
        command.input.schema.type_name = schema['typeName']
        command.input.schema.schema_version = schema['schemaVersion']
        command.input.schema.schema_fingerprint = schema['schemaFingerprint']
        return wire.ExecuteRequest(identity=member.identity, operation=command)

    def resources(self):
        pid = self.process.pid
        threads = [p.read_text().strip() for p in Path(f'/proc/{pid}/task').glob('*/comm')]
        status = Path(f'/proc/{pid}/status').read_text()
        rss = int(next(line.split()[1] for line in status.splitlines() if line.startswith('VmRSS:')))
        children = Path(f'/proc/{pid}/task/{pid}/children').read_text().strip().split()
        app = sum(name in ('robot-commands', 'robot-members', 'robot-callback') for name in threads)
        return {'pid': pid, 'application_threads': app, 'total_threads': len(threads), 'thread_names': threads,
                'retired_grpc_threads': threads.count('robot-grpc'), 'children': children, 'rss_kib': rss}

    def cpu_ticks(self):
        result = {}
        for path in Path(f'/proc/{self.process.pid}/task').glob('*/stat'):
            try:
                fields = path.read_text().split()
                result[int(path.parent.name)] = (path.with_name('comm').read_text().strip(),
                                                int(fields[13]) + int(fields[14]))
            except FileNotFoundError: pass
        return result

    def cpu_delta(self, previous, elapsed):
        result = {}
        for tid, (name, ticks) in self.cpu_ticks().items():
            result[name] = result.get(name, 0) + ticks - previous.get(tid, (name, 0))[1]
        return {name: 100 * ticks / os.sysconf('SC_CLK_TCK') / elapsed
                for name, ticks in result.items()}

    def allocations(self):
        path = ROOT / ('alloc-' + str(self.process.pid) + '.bin')
        if not os.environ.get('ROBOT_SERVER_ALLOCATION_PRELOAD'): return {}
        data = path.read_bytes()
        magic, count, capacity, overflow = struct.unpack_from('QQQQ', data)
        if magic != 0x58474332414c4c4f or overflow: raise RuntimeError('allocation probe invalid or overflowed')
        result = {}
        for index in range(min(count, capacity)):
            values = struct.unpack_from('Q'*24,data,32+index*192)
            tid, mallocs, callocs, reallocs, frees, requested = values[:6]
            if tid: result[tid] = (mallocs+callocs+reallocs, frees, requested, *values[6:19])
        return result

    def allocation_delta(self, previous):
        groups = {}
        for tid, counts in self.allocations().items():
            try: name = Path(f'/proc/{self.process.pid}/task/{tid}/comm').read_text().strip()
            except FileNotFoundError: name = 'exited-library-thread'
            group = groups.setdefault(name, [0]*16)
            for i, value in enumerate(counts): group[i] += value - previous.get(tid,(0,)*16)[i]
        return {name:dict(allocations=v[0], frees=v[1], requested_bytes=v[2], ros_publish_calls=v[3], ros_publish_total_ns=v[4], ros_publish_histogram_ceil_us=[1,2,4,8,16,32,64,'above64'], ros_publish_histogram=v[6:14], poll_calls=v[14], poll_wait_ns=v[15]) for name,v in groups.items()}

    def close(self):
        if self._closed: return
        self.channel.close()
        if self.process.poll() is None: self.process.send_signal(signal.SIGTERM)
        self.process.wait(timeout=15)
        self.log.close()
        if Path(self.socket).exists() or list(Path(self.socket).parent.glob('.robot-grpc-*')):
            raise RuntimeError('robot server leaked its owned socket or private binding directory')
        self._closed = True


class ServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROBOT_SERVER_PRIVATE_TEST') != '1':
            raise RuntimeError('requires explicit isolated private ROS test environment')
        cls.ros_directory = tempfile.TemporaryDirectory(prefix='robot-server-ros-', dir=ROOT)
        cls.addClassCleanup(cls.ros_directory.cleanup)
        os.environ['ROS_HOME'] = str(Path(cls.ros_directory.name) / 'home')
        os.environ['ROS_LOG_DIR'] = str(Path(cls.ros_directory.name) / 'logs')
        Path(os.environ['ROS_HOME']).mkdir(mode=0o700)
        Path(os.environ['ROS_LOG_DIR']).mkdir(mode=0o700)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            master_port = reservation.getsockname()[1]
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(master_port)
        os.environ['ROS_IP'] = '127.0.0.1'
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'
        with open(ROOT / 'master.log', 'w') as log:
            cls.master = subprocess.Popen(['/opt/ros/noetic/bin/rosmaster', '--core', '-p', str(master_port)], stdout=log, stderr=subprocess.STDOUT)
        cls.addClassCleanup(close_observer, cls.master)
        proxy = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if cls.master.poll() is not None:
                raise RuntimeError('private ROS master exited before becoming ready')
            try:
                result = proxy.getPid('/private_test')
                if result[0] != 1 or result[2] != cls.master.pid:
                    raise RuntimeError('private ROS port belongs to another process')
                break
            except OSError:
                time.sleep(0.05)
        else:
            raise RuntimeError('private ROS master failed')
        rospy.init_node('robot_server_private_test', disable_signals=True)
        proxy('close')()
        cls.clients = {}
        for kind in KINDS:
            cls.clients[kind] = Client(kind)
            cls.addClassCleanup(cls.clients[kind].close)
        cls.measurements = []

    @classmethod
    def tearDownClass(cls):
        for client in cls.clients.values():
            client.close()
        rospy.signal_shutdown('private test completed')
        cls.master.terminate()
        cls.master.wait(timeout=10)
        (ROOT / 'robot-server-measurements.json').write_text(json.dumps(cls.measurements, indent=2))

    def assert_members(self, result, count):
        self.assertEqual(len(result.members), count)
        self.assertFalse(any(member.HasField('error') for member in result.members), str(result))

    def test_00_native_entry_health_deadline_and_bootstrap_compatibility(self):
        # --check cannot initialize ROS: a deliberately unreachable master and
        # a fresh ROS_HOME must have no bearing on a serving gRPC Health reply.
        environment = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:9',
                           ROS_HOME=str(ROOT / 'check-must-not-create-ros-home'))
        for client in self.clients.values():
            before = time.monotonic()
            response = subprocess.run([client.binary, '--check', '--target-id', 'test', '--socket-path', client.socket],
                                      env=environment, capture_output=True, text=True, timeout=4)
            self.assertEqual(response.returncode, 0, response.stderr)
            self.assertLess(time.monotonic() - before, 3)
        self.assertFalse(Path(environment['ROS_HOME']).exists())
        client = self.clients['scout']
        with tempfile.TemporaryDirectory(prefix='robot-bootstrap-fifo-', dir=ROOT) as directory:
            fifo = Path(directory) / 'bootstrap.fifo'
            os.mkfifo(fifo, 0o600)
            started = time.monotonic()
            response = subprocess.run([client.binary, '--adapter-bootstrap-file', str(fifo)],
                                      env=environment, capture_output=True, text=True, timeout=2)
            self.assertNotEqual(response.returncode, 0, response.stderr)
            self.assertIn('regular file', response.stderr)
            self.assertLess(time.monotonic() - started, 1)
            self.assertTrue(fifo.is_fifo())
        for arguments in [[], ['--socket-path'], ['--socket-path', '--provider'],
                          ['--check', '--check', '--target-id', 'test', '--socket-path', client.socket],
                          ['--socket-path', client.socket, '--provider', client.provider],
                          ['--socket-path', client.socket, '--provider', client.provider, '--target-id', 'invalid target'],
                          ['--socket-path', client.socket, '--provider', 'foreign', '--target-id', 'test'],
                          ['--socket-path', client.socket, '--socket-path', client.socket, '--provider', client.provider],
                          ['--adapter-bootstrap-file', str(ROOT / 'scout.pb'), '--socket-path', client.socket],
                          ['--check', '--target-id', 'test', '--socket-path', client.socket, '--ros-ip', ''],
                          ['--check', '--target-id', 'test', '--socket-path', client.socket, '--timeout-ms', '60001']]:
            response = subprocess.run([client.binary, *arguments], env=environment,
                                      capture_output=True, text=True, timeout=3)
            self.assertNotEqual(response.returncode, 0, arguments)
        for path in [str(ROOT / 'missing.sock'), str(ROOT / 'silent.sock')]:
            listener = None
            try:
                if path.endswith('silent.sock'):
                    listener = socket.socket(socket.AF_UNIX); listener.bind(path); listener.listen(4)
                before = time.monotonic()
                response = subprocess.run([client.binary, '--check', '--target-id', 'test', '--socket-path', path, '--timeout-ms', '200'],
                                          env=environment, capture_output=True, text=True, timeout=3)
                self.assertNotEqual(response.returncode, 0, response.stderr)
                self.assertGreaterEqual(time.monotonic() - before, .15)
                self.assertLess(time.monotonic() - before, 2)
            finally:
                if listener is not None: listener.close(); Path(path).unlink()
        fake_path = str(ROOT / 'false-health.sock')
        fake = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=1))
        fake_instance = uuid.uuid4().hex
        def fake_metadata(context):
            request_id = next(value for key, value in context.invocation_metadata() if key == 'x-request-id')
            context.send_initial_metadata((('x-xrpc-instance-id', fake_instance), ('x-request-id', request_id)))
        def fake_describe(request, context):
            fake_metadata(context)
            response = wire.DescribeResponse(provider_definition_id=client.provider)
            reference = response.service_ref
            reference.target_id = 'test'; reference.service = 'xgc2.robot-adapter'
            reference.api_version = 'v1'; reference.profile = 'grpc.v1'; reference.instance_id = fake_instance
            reference.endpoint.kind = 'unix'; reference.endpoint.address = fake_path
            return response
        def fake_health(request, context):
            fake_metadata(context)
            return wire.HealthResponse(serving=False, instance_id=fake_instance)
        fake.add_generic_rpc_handlers([grpc.method_handlers_generic_handler(
            'xgc.robot.v1.RobotAdapterServerService', {
                'Describe': grpc.unary_unary_rpc_method_handler(fake_describe,
                    request_deserializer=wire.DescribeRequest.FromString,
                    response_serializer=wire.DescribeResponse.SerializeToString),
                'Health': grpc.unary_unary_rpc_method_handler(fake_health,
                request_deserializer=wire.HealthRequest.FromString,
                response_serializer=wire.HealthResponse.SerializeToString)})])
        fake.add_insecure_port('unix:' + fake_path); fake.start()
        try:
            response = subprocess.run([client.binary, '--check', '--target-id', 'test', '--socket-path', fake_path],
                                      env=environment, capture_output=True, text=True, timeout=3)
            self.assertNotEqual(response.returncode, 0)
            self.assertIn('not serving', response.stderr)
        finally: fake.stop(0).wait()
        compatibility = Client('scout', name='bootstrap-compatible', bootstrap=True)
        try:
            member = compatibility.member(555)
            self.assert_members(compatibility.apply(wire.ApplyMembersRequest(members=[member]), timeout=5), 1)
            self.assert_members(compatibility.remove(wire.RemoveMembersRequest(members=[member.identity]), timeout=5), 1)
        finally: compatibility.close()

    def test_00_socket_lifecycle_preserves_foreign_paths(self):
        client = self.clients['scout']
        directory = ROOT / 'socket-ownership'; directory.mkdir(mode=0o700, exist_ok=True)
        path = directory / 'foreign.sock'
        def launch():
            return subprocess.run([client.binary, '--socket-path', str(path), '--provider', client.provider,
                                   '--target-id', 'test',
                                   '--ros-master-uri', os.environ['ROS_MASTER_URI'], '--ros-ip', ''],
                                  capture_output=True, text=True, timeout=5)
        foreign = socket.socket(socket.AF_UNIX); foreign.bind(str(path)); foreign.listen(1)
        before = path.lstat().st_ino
        try:
            self.assertNotEqual(launch().returncode, 0)
            self.assertEqual(path.lstat().st_ino, before)
        finally: foreign.close(); path.unlink()
        path.write_text('foreign bytes')
        try:
            self.assertNotEqual(launch().returncode, 0)
            self.assertEqual(path.read_text(), 'foreign bytes')
        finally: path.unlink()
        temporary = Client('scout', name='replacement-ownership')
        original_path = Path(temporary.socket); owned_path = original_path.with_suffix('.owned')
        self.assertEqual(original_path.parent.stat().st_mode & 0o777, 0o700)
        self.assertEqual(original_path.stat().st_mode & 0o777, 0o600)
        original_path.rename(owned_path)
        replacement = socket.socket(socket.AF_UNIX); replacement.bind(str(original_path)); replacement.listen(1)
        before = original_path.lstat().st_ino
        try:
            temporary.process.terminate(); temporary.process.wait(timeout=5)
            self.assertEqual(temporary.process.returncode, 0)
            self.assertEqual(original_path.lstat().st_ino, before)
        finally:
            replacement.close()
            if original_path.exists(): original_path.unlink()
            if owned_path.exists(): owned_path.unlink()
            temporary.close()

    def test_01_fixed_processes_and_threads_at_1_20_100(self):
        for kind, client in self.clients.items():
            members = []
            for count in (1, 20, 100):
                additions = [client.member(i) for i in range(len(members), count)]
                started = time.monotonic()
                self.assert_members(client.apply(wire.ApplyMembersRequest(members=additions), timeout=30), len(additions))
                members += additions
                resources = client.resources()
                self.assertEqual(resources['children'], [])
                self.assertEqual(resources['application_threads'], 4, resources)
                self.assertEqual(resources['retired_grpc_threads'], 0, resources)
                self.measurements.append(dict(resources, kind=kind, count=count,
                    startup_ms=client.startup_ms, registration_ms=(time.monotonic()-started)*1000))
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[member.identity for member in members]), timeout=30), 100)
        self.assertEqual(len({c.process.pid for c in self.clients.values()}), 3)

    def test_02_twenty_native_requests_before_any_response_and_fifo(self):
        client = self.clients['px4']
        members = [client.member(i) for i in range(20)]
        self.assert_members(client.apply(wire.ApplyMembersRequest(members=members), timeout=30), 20)
        received = []
        mutex = threading.Lock()
        release = threading.Event()
        all_received = threading.Event()

        def response(index, request):
            self.assertTrue(request.value)
            with mutex:
                received.append(index)
                if len(received) == 20:
                    all_received.set()
            release.wait(timeout=10)
            return CommandBoolResponse(success=True, result=0)

        services = [rospy.Service(f'/px4{i}/mavros/cmd/arming', CommandBool,
            lambda request, i=i: response(i, request)) for i in range(20)]
        try:
            calls = [client.execute.future(client.command(member, f'arm-{i}', deadline_seconds=5), timeout=6) for i, member in enumerate(members)]
            queued = client.execute.future(client.command(members[0], 'second-arm', deadline_seconds=5), timeout=6)
            self.assertTrue(all_received.wait(timeout=3.5), received)
            self.assertEqual(len(received), 20)
            release.set()
            self.assertTrue(all(call.result().phase == operation.OPERATION_PHASE_SUCCEEDED for call in calls))
            self.assertEqual(queued.result().phase, operation.OPERATION_PHASE_SUCCEEDED)
            self.assertEqual(received.count(0), 2)
        finally:
            release.set()
            for service in services:
                service.shutdown()
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[m.identity for m in members]), timeout=20),20)
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[member.identity for member in members]), timeout=10), 20)

    def test_03_unknown_blocks_only_same_robot_and_removal_cancels(self):
        client = self.clients['px4']
        members = [client.member(i) for i in range(2)]
        self.assert_members(client.apply(wire.ApplyMembersRequest(members=members), timeout=10), 2)
        entered = threading.Event()
        unblock = threading.Event()
        counts = [0, 0]

        def slow(request):
            counts[0] += 1
            entered.set()
            unblock.wait(timeout=10)
            return CommandBoolResponse(success=True, result=0)

        def fast(request):
            counts[1] += 1
            return CommandBoolResponse(success=True, result=0)

        services = [rospy.Service('/px4' + str(i) + '/mavros/cmd/arming', CommandBool, callback)
                    for i, callback in enumerate([slow, fast])]
        try:
            first = client.execute.future(client.command(members[0], 'slow'), timeout=4)
            self.assertTrue(entered.wait(timeout=1))
            queued = client.execute.future(client.command(members[0], 'never-send'), timeout=4)
            other = client.execute(client.command(members[1], 'fast'), timeout=2)
            self.assertEqual(other.phase, operation.OPERATION_PHASE_SUCCEEDED)
            # The offline service must leave another robot's native data live.
            sample = threading.Event()
            subscriber = rospy.Subscriber('/px41/pose', PoseStamped, lambda msg: sample.set(), queue_size=1)
            publisher = rospy.Publisher('/vrpn_client_node/px41/pose', PoseStamped, queue_size=1)
            until = time.monotonic() + 1
            while not sample.is_set() and time.monotonic() < until:
                msg = PoseStamped(); msg.header.stamp = rospy.Time.now(); msg.pose.orientation.w = 1
                publisher.publish(msg); time.sleep(.03)
            self.assertTrue(sample.is_set(), 'offline service blocked another robot data')
            subscriber.unregister(); publisher.unregister()
            self.assertEqual(first.result().phase, operation.OPERATION_PHASE_UNCERTAIN)
            self.assertEqual(queued.result().error.code, 'previous-result-unknown')
            self.assertEqual(counts, [1, 1])
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[members[0].identity]), timeout=3), 1)
            # A stale release must not remove a new connection generation.
            replacement = client.member(0, epoch=2)
            self.assert_members(client.apply(wire.ApplyMembersRequest(members=[replacement]), timeout=3), 1)
            conflict = client.remove(wire.RemoveMembersRequest(members=[members[0].identity]), timeout=3)
            self.assertTrue(conflict.members[0].HasField('error'))
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[replacement.identity, members[1].identity]), timeout=3), 2)
        finally:
            unblock.set()
            for service in services:
                service.shutdown()

    def test_04_forwarding_backpressure_offsets_updates_and_performance(self):
        # Canonical ROS stays at source cadence while dashboard streams coalesce.
        for kind, client in self.clients.items():
            for count in (1, 20, 100):
                members = [client.member(i) for i in range(count)]
                # A real station's native sources exist before observer Connect.
                publishers = [rospy.Publisher('/vrpn_client_node/' + kind + str(i) + '/pose', PoseStamped, queue_size=1) for i in range(count)]
                creation_allocations_before = client.allocations()
                started = time.monotonic()
                self.assert_members(client.apply(wire.ApplyMembersRequest(members=members), timeout=30), count)
                registered = time.monotonic()
                registered_wall = time.time()
                creation_allocations = client.allocation_delta(creation_allocations_before)
                first = threading.Event()
                observer_binary = ROOT / 'ros_forwarding_observer'
                observer = None
                if observer_binary.exists():
                    report_path = ROOT / f'{kind}-{count}-forwarding.json'
                    ready_path = ROOT / f'{kind}-{count}-forwarding.ready'
                    ready_path.unlink(missing_ok=True)
                    observer = subprocess.Popen([str(observer_binary),kind,str(count),str(report_path),str(ready_path)], stdout=subprocess.DEVNULL)
                    self.addCleanup(close_observer,observer)
                    until=time.monotonic()+25
                    while not ready_path.exists() and observer.poll() is None and time.monotonic()<until:time.sleep(.01)
                    self.assertTrue(ready_path.exists(), 'all-robot forwarding observer not ready')
                else:
                    raise RuntimeError('build common/test/ros_forwarding_observer.cpp for full forwarding measurements')
                slow = client.status(wire.SubscribeStatusRequest()) # intentionally never consume
                ready = time.monotonic() + 20
                while time.monotonic() < ready and any(p.get_num_connections() == 0 for p in publishers): time.sleep(.02)
                self.assertTrue(all(p.get_num_connections() for p in publishers), str([i for i,p in enumerate(publishers) if not p.get_num_connections()]))
                allocations_before = client.allocations()
                before = Path(f'/proc/{client.process.pid}/stat').read_text().split()
                ticks_before = int(before[13]) + int(before[14])
                input_started = time.monotonic()
                input_started_wall = time.time()
                for tick in range(90):
                    msg = PoseStamped(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = 'source-world'
                    msg.pose.position.x = 3; msg.pose.position.y = 4; msg.pose.position.z = 5; msg.pose.orientation.w = 1
                    for publisher in publishers:
                        msg.header.stamp = rospy.Time.now()
                        publisher.publish(msg)
                    delay = input_started + (tick + 1) / 30 - time.monotonic()
                    if delay > 0: time.sleep(delay)
                time.sleep(.1)
                observer.send_signal(signal.SIGTERM); observer.wait(timeout=5)
                full = json.loads(report_path.read_text())
                self.assertGreater(min(full['per_robot_samples']),70,'a native ROS output lost source cadence')
                self.assertEqual(full['invalid_samples'],0)
                fresh = client.status(wire.SubscribeStatusRequest(), timeout=3)
                batch = next(fresh)
                samples = [m for member in batch.members for m in member.messages if m.channel_id in ('state.mocap.pose', 'vrpn.position')]
                self.assertEqual(len(samples), count)
                decoded = telemetry_pb2.PoseEstimate.FromString(samples[0].message.payload.value)
                self.assertAlmostEqual(decoded.position.x, 4.25)
                fresh.cancel()
                after = Path(f'/proc/{client.process.pid}/stat').read_text().split()
                ticks = int(after[13]) + int(after[14]) - ticks_before
                elapsed = time.monotonic() - input_started

                resources = client.resources()
                self.assertEqual(resources['application_threads'], 4)
                self.measurements.append(dict(resources, kind=kind, count=count, input_hz=30, duration_s=elapsed,
                    canonical_samples=full['all_robot_samples'], canonical_hz=full['all_robot_samples']/count/3,
                    per_robot_samples=full['per_robot_samples'],
                    allocation_probe=bool(os.environ.get('ROBOT_SERVER_ALLOCATION_PRELOAD')),
                    allocations=client.allocation_delta(allocations_before),
                    creation_allocations=creation_allocations,
                    registration_ms=(registered-started)*1000,
                    first_sample_after_registration_ms=(full['first_sample_unix_ns']/1e9-registered_wall)*1000,
                    first_sample_after_input_ms=(full['first_sample_unix_ns']/1e9-input_started_wall)*1000,
                    forwarding_latency_ms=full['latency_ms'],
                    cpu_percent_one_core=100*ticks/os.sysconf('SC_CLK_TCK')/elapsed,
                    hardware=platform.machine(), cpu_model=next((line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')), 'unknown'),
                    private_container_cpu_limit=4))
                if count == 1:
                    # Configuration update touches this resource; a second use
                    # cannot silently overwrite incompatible shared resources.
                    duplicate = client.member(0, run='second-use')
                    self.assert_members(client.apply(wire.ApplyMembersRequest(members=[duplicate]), timeout=5), 1)
                    update = client.member(0); update.configuration.parameters['localization_offset_x'] = '2'
                    conflict = client.apply(wire.ApplyMembersRequest(members=[update]), timeout=5)
                    self.assertTrue(conflict.members[0].HasField('error'))
                    self.assert_members(client.remove(wire.RemoveMembersRequest(members=[duplicate.identity]), timeout=5), 1)
                    self.assert_members(client.apply(wire.ApplyMembersRequest(members=[update]), timeout=10), 1)
                    sample_messages=[]
                    subscriber=rospy.Subscriber('/'+kind+'0/pose',PoseStamped,lambda message:(sample_messages.append(message),first.set()),queue_size=1)
                    first.clear()
                    deadline=time.monotonic()+3
                    while not first.is_set() and time.monotonic()<deadline:
                        publishers[0].publish(msg);time.sleep(.05)
                    self.assertTrue(first.is_set())
                    self.assertAlmostEqual(sample_messages[-1].pose.position.x, 5)
                    subscriber.unregister()
                slow.cancel()
                for publisher in publishers: publisher.unregister()
                self.assert_members(client.remove(wire.RemoveMembersRequest(members=[m.identity for m in members]), timeout=30), count)

    def test_05_ground_commands_and_stop_keep_original_mapping(self):
        for kind in ('scout', 'mecanum'):
            client = self.clients[kind]; member = client.member(0)
            self.assert_members(client.apply(wire.ApplyMembersRequest(members=[member]), timeout=5), 1)
            commands = []; received = threading.Event()
            subscriber = rospy.Subscriber('/' + kind + '0/cmd_vel', Twist, lambda msg: (commands.append(msg), received.set()), queue_size=100)
            time.sleep(.3); received.clear(); commands.clear()
            invalid=common_control.RemoteControlIntentRequest(gear=0,longitudinal=1)
            rejected=client.execute(client.command(member,'invalid-'+kind,endpoint='set-motion-intent',value=invalid.SerializeToString()),timeout=3)
            self.assertEqual(rejected.phase,operation.OPERATION_PHASE_REJECTED)
            self.assertEqual(rejected.error.code,'invalid-command-input')
            self.assertEqual(commands,[])
            intent = (common_control.RemoteControlIntentRequest(gear=3, longitudinal=1, yaw=1) if kind == 'scout' else common_control.RemoteControlIntentRequest(gear=3, longitudinal=1, lateral=1, yaw=1))
            result = client.execute(client.command(member, 'motion-'+kind, endpoint='set-motion-intent', value=intent.SerializeToString()), timeout=3)
            self.assertEqual(result.phase, operation.OPERATION_PHASE_SUCCEEDED)
            self.assertTrue(received.wait(2)); time.sleep(.4)
            self.assertGreaterEqual(len(commands), 3)
            moving = commands[-1]
            self.assertGreater(moving.linear.x, 0); self.assertGreater(moving.angular.z, 0)
            self.assertEqual(moving.linear.y == 0, kind == 'scout')
            result = client.execute(client.command(member, 'release-'+kind, endpoint='release-motion-intent', value=intent.SerializeToString()), timeout=3)
            self.assertEqual(result.phase, operation.OPERATION_PHASE_SUCCEEDED)
            time.sleep(.15)
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[member.identity]), timeout=5), 1)
            time.sleep(.15)
            self.assertEqual((commands[-1].linear.x, commands[-1].linear.y, commands[-1].angular.z), (0,0,0))
            size=len(commands);time.sleep(.3);self.assertEqual(len(commands),size)
            subscriber.unregister()

    def test_06_removal_cancels_actual_inflight_and_late_response(self):
        client=self.clients['px4']; member=client.member(0)
        self.assert_members(client.apply(wire.ApplyMembersRequest(members=[member]),timeout=5),1)
        entered=threading.Event(); release=threading.Event(); count=[]
        def slow(request):
            count.append(request.value);entered.set();release.wait(5);return CommandBoolResponse(success=True,result=0)
        service=rospy.Service('/px40/mavros/cmd/arming',CommandBool,slow)
        try:
            pending=client.execute.future(client.command(member,'removed-inflight'),timeout=4)
            self.assertTrue(entered.wait(1))
            before=time.monotonic()
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[member.identity]),timeout=2),1)
            self.assertLess(time.monotonic()-before,1)
            self.assertEqual(pending.result().phase,operation.OPERATION_PHASE_UNCERTAIN)
            release.set();time.sleep(.1)
            self.assertEqual(count,[True])
            stale=client.execute(client.command(member,'stale-after-remove'),timeout=2)
            self.assertEqual(stale.error.code,'member-unavailable')
        finally:
            release.set();service.shutdown()

    def test_07_silent_master_management_and_shutdown_return(self):
        class Silent(socketserver.BaseRequestHandler):
            def handle(self):
                self.request.settimeout(15)
                try:
                    while self.request.recv(8192): pass
                except OSError: pass
        master=socketserver.ThreadingTCPServer(('127.0.0.1',0),Silent);master.daemon_threads=True
        thread=threading.Thread(target=master.serve_forever);thread.start()
        client=None
        try:
            client=Client('scout',name='silent-master',master='http://127.0.0.1:'+str(master.server_address[1]))
            member=client.member(777)
            for channel in member.configuration.channels: channel.enabled = channel.channel_id == 'vrpn.position'
            before=time.monotonic()
            response=client.apply(wire.ApplyMembersRequest(members=[member]),timeout=20)
            self.assertTrue(response.members[0].HasField('error'))
            self.assertLess(time.monotonic()-before,15)
            client.health(wire.HealthRequest(),timeout=1)
            self.assertEqual(client.resources()['application_threads'],4)
            client.close();client=None
        finally:
            try:
                if client is not None: client.close()
            finally:
                master.shutdown();master.server_close();thread.join()

    def test_08_full_canonical_outputs_and_px4_vision_at_100(self):
        for kind,client in self.clients.items():
            members=[client.member(i) for i in range(100)]
            source_report=ROOT/(kind+'-all-input.json');source_ready=ROOT/(kind+'-all-input.ready');source_start=ROOT/(kind+'-all-input.start')
            source_ready.unlink(missing_ok=True);source_start.unlink(missing_ok=True);source_report.unlink(missing_ok=True)
            source=subprocess.Popen([str(ROOT/'ros_forwarding_source'),kind,'100',str(source_report),str(source_ready),str(source_start)],stdout=subprocess.DEVNULL)
            self.addCleanup(close_observer,source)
            started=time.monotonic()
            self.assert_members(client.apply(wire.ApplyMembersRequest(members=members),timeout=30),100)
            registered=time.monotonic()
            report_path=ROOT/(kind+'-all-channels.json');ready_path=ROOT/(kind+'-all-channels.ready')
            ready_path.unlink(missing_ok=True)
            observer=subprocess.Popen([str(ROOT/'ros_forwarding_observer'),kind,'100',str(report_path),str(ready_path),'all'],stdout=subprocess.DEVNULL)
            self.addCleanup(close_observer,observer)
            until=time.monotonic()+25
            while not ready_path.exists() and observer.poll() is None and time.monotonic()<until:time.sleep(.01)
            self.assertTrue(ready_path.exists(),'full canonical receiver not ready')
            until=time.monotonic()+20
            while time.monotonic()<until and not source_ready.exists() and source.poll() is None:time.sleep(.02)
            self.assertTrue(source_ready.exists(),'full roscpp source connections not ready')
            slow=client.status(wire.SubscribeStatusRequest())
            before=Path(f'/proc/{client.process.pid}/stat').read_text().split();ticks_before=int(before[13])+int(before[14])
            allocations_before=client.allocations();input_started=time.monotonic()
            source_start.write_text('start\n')
            until=time.monotonic()+10
            while not source_report.exists() and source.poll() is None and time.monotonic()<until:time.sleep(.01)
            self.assertTrue(source_report.exists(),'source did not finish its input window')
            time.sleep(.1)
            after=Path(f'/proc/{client.process.pid}/stat').read_text().split()
            elapsed=time.monotonic()-input_started
            allocations=client.allocation_delta(allocations_before)
            self.assertEqual(source.wait(timeout=15),0)
            observer.send_signal(signal.SIGTERM);observer.wait(timeout=5)
            full=json.loads(report_path.read_text());self.assertEqual(full['invalid_samples'],0)
            for channel,counts in full['channel_samples'].items():
                self.assertGreaterEqual(min(counts),60 if channel=='vision' else 70,(kind,channel,min(counts)))
            ticks=int(after[13])+int(after[14])-ticks_before
            resources=client.resources();self.assertEqual(resources['application_threads'],4)
            self.measurements.append(dict(resources,kind=kind,count=100,input_hz_per_channel=30,input_channels=['pose','twist','accel'],
                duration_s=elapsed,registration_ms=(registered-started)*1000,output=full,input=json.loads(source_report.read_text()),
                cpu_percent_one_core=100*ticks/os.sysconf('SC_CLK_TCK')/elapsed,
                allocations=allocations,allocation_probe=bool(os.environ.get('ROBOT_SERVER_ALLOCATION_PRELOAD'))))
            fresh=client.status(wire.SubscribeStatusRequest(),timeout=3);batch=next(fresh)
            self.assertEqual(len(batch.members),100);fresh.cancel();slow.cancel()
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[m.identity for m in members]),timeout=30),100)

    def test_09_native_service_parameters_and_result_semantics(self):
        client=self.clients['px4'];member=client.member(9)
        self.assert_members(client.apply(wire.ApplyMembersRequest(members=[member]),timeout=5),1)
        received=[]
        def arm(request):
            received.append(('arm',request.value));return CommandBoolResponse(success=False,result=3)
        def mode(request):
            received.append(('mode',request.base_mode,request.custom_mode));return SetModeResponse(mode_sent=True)
        def reboot(request):
            received.append(('long',request.broadcast,request.command,request.confirmation,request.param1,request.param2))
            return CommandLongResponse(success=True,result=0)
        services=[rospy.Service('/px49/mavros/cmd/arming',CommandBool,arm),
            rospy.Service('/px49/mavros/set_mode',SetMode,mode),
            rospy.Service('/px49/mavros/cmd/command',CommandLong,reboot)]
        state=rospy.Publisher('/px49/mavros/state',State,queue_size=1)
        try:
            result=client.execute(client.command(member,'unsupported-arm'),timeout=3)
            self.assertEqual(result.phase,operation.OPERATION_PHASE_REJECTED)
            self.assertEqual(result.error.code,'px4-unsupported');self.assertEqual(result.native_code,3)
            result=client.execute(client.command(member,'mode-sent',endpoint='set-flight-mode',value=aerial_control.ModeRequest(mode='OFFBOARD').SerializeToString()),timeout=3)
            self.assertEqual(result.phase,operation.OPERATION_PHASE_SUCCEEDED)
            until=time.monotonic()+2
            while not state.get_num_connections() and time.monotonic()<until:time.sleep(.01)
            self.assertTrue(state.get_num_connections())
            for _ in range(3):state.publish(State(connected=True,armed=False));time.sleep(.03)
            result=client.execute(client.command(member,'reboot-accepted',endpoint='reboot-autopilot',value=aerial_control.AutopilotRebootRequest().SerializeToString()),timeout=3)
            self.assertEqual(result.phase,operation.OPERATION_PHASE_SUCCEEDED)
            self.assertEqual(received,[('arm',True),('mode',0,'OFFBOARD'),('long',False,246,0,1.0,0.0)])
        finally:
            state.unregister()
            for service in services:service.shutdown()
            self.assert_members(client.remove(wire.RemoveMembersRequest(members=[member.identity]),timeout=5),1)


    def test_10_mixed_profiles_direct_topics_health_and_independent_removal(self):
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for kind, client in self.clients.items():
            profiles = [client.profile,
                next(p for key,p in client.profiles.items() if key.endswith('.gazebo.vrpn')),
                next(p for key,p in client.profiles.items() if key.endswith('.xsim.ros'))]
            members = [client.member(101+i, profile=profile) for i,profile in enumerate(profiles)]
            members[2].configuration.parameters.update(localization_pose_topic='/custom/'+kind+'/pose', localization_twist_topic='/custom/'+kind+'/twist')
            self.assert_members(client.apply(wire.ApplyMembersRequest(members=members),timeout=15),3)
            stream = client.status(wire.SubscribeStatusRequest(),timeout=15)
            latest = {}; lock = threading.Lock()
            def receive():
                try:
                    for batch in stream:
                        with lock:
                            for member in batch.members:
                                for message in member.messages: latest[(member.identity.robot_id,message.channel_id)] = message
                except grpc.RpcError: pass
            receiver = threading.Thread(target=receive); receiver.start()
            publishers = []
            for i,member in enumerate(members):
                root = '/custom/'+kind if i==2 else '/vrpn_client_node/'+member.identity.robot_id
                publishers += [rospy.Publisher(root+'/pose',PoseStamped,queue_size=2), rospy.Publisher(root+'/twist',TwistStamped,queue_size=2)]
            imu_publishers = [rospy.Publisher(imu_topic(kind, m.identity.robot_id), Imu, queue_size=2) for m in members]
            try:
                deadline=time.monotonic()+8
                while time.monotonic()<deadline and not all(p.get_num_connections() for p in publishers+imu_publishers): time.sleep(.02)
                self.assertTrue(all(p.get_num_connections() for p in publishers+imu_publishers))
                pose_channel='state.mocap.pose' if kind=='px4' else 'vrpn.position'
                def send(tick):
                    pose=PoseStamped();pose.header.stamp=rospy.Time.now();pose.pose.orientation.w=1;pose.pose.position.x=3+tick*.01
                    twist=TwistStamped();twist.header.stamp=pose.header.stamp;twist.twist.linear.x=1
                    for i,publisher in enumerate(publishers): publisher.publish(pose if i%2==0 else twist)
                    for publisher in imu_publishers: publisher.publish(fixture_imu(pose.header.stamp))
                for tick in range(60): send(tick); time.sleep(.025)
                with lock:
                    for i,member in enumerate(members):
                        message=latest[(member.identity.robot_id,pose_channel)]
                        pose=telemetry_pb2.PoseEstimate.FromString(message.message.payload.value)
                        self.assertAlmostEqual(pose.position.x,3.59+(1.25 if i==0 else 0),delta=.08)
                    health=telemetry_pb2.VehicleHealth.FromString(latest[(members[2].identity.robot_id,'state.health')].message.payload.value)
                    self.assertGreaterEqual(health.positioning.sample_count,3, str((client.kind,health)))
                    self.assertLess(health.positioning.observed_age_ms,100)
                    for member in members:
                        imu=telemetry_pb2.ImuEstimate.FromString(latest[(member.identity.robot_id,'state.imu')].message.payload.value)
                        self.assertEqual(imu.frame_id,'source-body')
                        self.assertEqual(imu.orientation.w,1)
                        self.assertEqual(imu.angular_velocity.z,.5)
                        self.assertEqual(imu.linear_acceleration.z,9.80665)
                        self.assertEqual(list(imu.orientation_covariance),list(fixture_imu(rospy.Time()).orientation_covariance))
                        self.assertEqual(list(imu.angular_velocity_covariance),list(fixture_imu(rospy.Time()).angular_velocity_covariance))
                        self.assertEqual(list(imu.linear_acceleration_covariance),list(fixture_imu(rospy.Time()).linear_acceleration_covariance))
                    previous=[latest[(m.identity.robot_id,pose_channel)].message.sequence for m in members[:2]]
                    previous_imu=[latest[(m.identity.robot_id,'state.imu')].message.sequence for m in members[:2]]
                code,_,graph=master.getSystemState('/mixed_profiles_test');self.assertEqual(code,1)
                node_prefix='/' + KINDS[kind][0]
                owned_publishers=[topic for topic,nodes in graph[0] if any(node.startswith(node_prefix) for node in nodes)]
                self.assertNotIn('/custom/'+kind+'/pose',owned_publishers)
                self.assertNotIn('/custom/'+kind+'/twist',owned_publishers)
                self.assertFalse(any(topic.startswith('/'+members[2].identity.robot_id+'/mavros/vision_pose') for topic in owned_publishers))
                subscriptions={topic:nodes for topic,nodes in graph[1]}
                self.assertFalse(any(node.startswith(node_prefix) for topic,nodes in subscriptions.items() if topic.startswith('/vrpn_client_node/'+members[2].identity.robot_id+'/')))
                self.assert_members(client.remove(wire.RemoveMembersRequest(members=[members[2].identity]),timeout=10),1)
                for tick in range(60,75):send(tick);time.sleep(.025)
                with lock:
                    for i,member in enumerate(members[:2]):
                        self.assertGreater(latest[(member.identity.robot_id,pose_channel)].message.sequence,previous[i])
                        self.assertGreater(latest[(member.identity.robot_id,'state.imu')].message.sequence,previous_imu[i])
                self.assertEqual(client.resources()['application_threads'],4)
                master('close')()
            finally:
                stream.cancel();receiver.join(3)
                for publisher in publishers+imu_publishers:publisher.unregister()
                self.assert_members(client.remove(wire.RemoveMembersRequest(members=[m.identity for m in members]),timeout=15),3)

    def test_11_production_xsim_direct_custom_topics(self):
        binary=os.environ.get('ROBOT_SERVER_XSIM_BINARY')
        if not binary:self.skipTest('production xsim binary is supplied by the isolated combination test')
        entities=[];members=[];streams=[];threads=[];latest={};lock=threading.Lock()
        imu_inputs={};imu_subscribers=[];imu_source_stamps={}
        for kind,client in self.clients.items():
            profile=next(p for key,p in client.profiles.items() if key.endswith('.xsim.ros'))
            member=client.member(201,profile=profile)
            member.configuration.parameters.update(localization_pose_topic='/production/'+kind+'/pose',localization_twist_topic='/production/'+kind+'/twist')
            members.append((client,member))
            imu_inputs[member.identity.robot_id]=OrderedDict()
            imu_source_stamps[member.identity.robot_id]=[]
            def observe_imu(message, robot_id=member.identity.robot_id):
                with lock:
                    samples=imu_inputs[robot_id]
                    stamp=message.header.stamp.to_nsec()
                    samples[stamp]=message
                    imu_source_stamps[robot_id].append(stamp)
                    if len(samples)>120:samples.popitem(last=False)
            imu_subscribers.append(rospy.Subscriber(imu_topic(kind,member.identity.robot_id),Imu,observe_imu,queue_size=20,tcp_nodelay=True))
            entities.append(dict(name=member.identity.robot_id,kind='fs150' if kind=='px4' else kind,position=[2,-1,.2],yaw=0,ros=dict(localization_pose_topic='/production/'+kind+'/pose',localization_twist_topic='/production/'+kind+'/twist',mocap_noise=[1e-7]*3,mocap_seed=1)))
        config=dict(instance_id='adapter-xsim-combination',epoch_ns=time.time_ns(),model_step_ns=1000000,output_period_ns=10000000,input_poll_ns=2000000,publish_clock=False,scene={},entities=entities)
        path=ROOT/'production-xsim.json';path.write_text(json.dumps(config));sock=str(ROOT/'production-xsim.sock')
        log=open(ROOT/'production-xsim.log','w');plant=subprocess.Popen([binary,'--config',str(path),'--socket',sock],stdout=log,stderr=subprocess.STDOUT)
        class UnixHTTP(http.client.HTTPConnection):
            def connect(self):self.sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);self.sock.settimeout(5);self.sock.connect(sock)
        def request(method,path,body=None):
            connection=UnixHTTP('localhost')
            try:
                connection.request(method,path,json.dumps(body) if body is not None else None,{'Content-Type':'application/json'})
                response=connection.getresponse();result=json.loads(response.read());self.assertIn(response.status,(200,202));return result
            finally:connection.close()
        try:
            until=time.monotonic()+10
            while not Path(sock).exists() and time.monotonic()<until:
                self.assertIsNone(plant.poll());time.sleep(.02)
            roster=request('GET','/entities')
            for index,entity in enumerate(roster['entities']):
                request_id='provider-'+str(index)
                request('POST','/entities/'+str(entity['id'])+'/provider',dict(instance_id=config['instance_id'],request_id=request_id,action='start',generation=entity['generation'],timeout_ms=5000))
                until=time.monotonic()+5
                while time.monotonic()<until:
                    receipt=request('GET','/requests/'+request_id)
                    if receipt['phase']=='applied':break
                    self.assertNotEqual(receipt['phase'],'failed');time.sleep(.01)
                self.assertTrue(receipt['result']['success'])
            for client,member in members:
                self.assert_members(client.apply(wire.ApplyMembersRequest(members=[member]),timeout=10),1)
                stream=client.status(wire.SubscribeStatusRequest(),timeout=15);streams.append(stream)
                def receive(stream=stream):
                    try:
                        for batch in stream:
                            with lock:
                                for status in batch.members:
                                    for message in status.messages:latest[(status.identity.robot_id,message.channel_id)]=message
                    except grpc.RpcError:pass
                thread=threading.Thread(target=receive);thread.start();threads.append(thread)
            until=time.monotonic()+8
            while time.monotonic()<until:
                with lock:
                    ready=all((member.identity.robot_id,'state.mocap.pose' if client.kind=='px4' else 'vrpn.position') in latest and (member.identity.robot_id,'state.mocap.velocity' if client.kind=='px4' else 'vrpn.velocity') in latest and (member.identity.robot_id,'state.imu') in latest and (member.identity.robot_id,'state.health') in latest and telemetry_pb2.VehicleHealth.FromString(latest[(member.identity.robot_id,'state.health')].message.payload.value).positioning.sample_count>=3 for client,member in members)
                if ready:break
                time.sleep(.02)
            self.assertTrue(ready,'production xsim did not reach Adapter gRPC')
            time.sleep(1)
            rates={}
            for client,member in members:
                with lock:
                    pose=telemetry_pb2.PoseEstimate.FromString(latest[(member.identity.robot_id,'state.mocap.pose' if client.kind=='px4' else 'vrpn.position')].message.payload.value)
                    health=telemetry_pb2.VehicleHealth.FromString(latest[(member.identity.robot_id,'state.health')].message.payload.value)
                    message=latest[(member.identity.robot_id,'state.imu')]
                    imu=telemetry_pb2.ImuEstimate.FromString(message.message.payload.value)
                    source=imu_inputs[member.identity.robot_id].get(message.message.source_time.nanoseconds)
                    stamps=list(imu_source_stamps[member.identity.robot_id])
                self.assertAlmostEqual(pose.position.x,2,delta=.001)
                self.assertGreaterEqual(health.positioning.sample_count,3, str((client.kind,health)))
                self.assertLess(health.positioning.observed_age_ms,100)
                self.assertIsNotNone(source,'Adapter IMU did not preserve the actual xsim source timestamp')
                self.assertEqual(imu.frame_id,source.header.frame_id)
                self.assertEqual((imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w),
                                 (source.orientation.x,source.orientation.y,source.orientation.z,source.orientation.w))
                self.assertEqual((imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z),
                                 (source.angular_velocity.x,source.angular_velocity.y,source.angular_velocity.z))
                self.assertEqual((imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z),
                                 (source.linear_acceleration.x,source.linear_acceleration.y,source.linear_acceleration.z))
                self.assertEqual(list(imu.orientation_covariance),list(source.orientation_covariance))
                self.assertEqual(list(imu.angular_velocity_covariance),list(source.angular_velocity_covariance))
                self.assertEqual(list(imu.linear_acceleration_covariance),list(source.linear_acceleration_covariance))
                self.assertLess((time.time_ns()-message.message.source_time.nanoseconds)/1e6,200)
                self.assertGreaterEqual(len(stamps),20)
                rates[client.kind]=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0])
                self.assertGreater(rates[client.kind],25)
                self.assertLess(rates[client.kind],35)
            self.measurements.append(dict(test='production-xsim-combination',types=3,direct_custom_topics=True,
                                          positioning_health=True,actual_imu_field_mapping=True,imu_hz=rates))
        finally:
            for subscriber in imu_subscribers:subscriber.unregister()
            for stream in streams:stream.cancel()
            for thread in threads:thread.join(3)
            for client,member in members:self.assert_members(client.remove(wire.RemoveMembersRequest(members=[member.identity]),timeout=10),1)
            plant.terminate();plant.wait(timeout=15);log.close()

    def test_12_xsim_100_members_at_125_hz(self):
        # Match xsim's actual localization cadence with one native ROS source;
        # the simulator's physics cost is excluded from Adapter CPU measurements.
        for kind, client in self.clients.items():
            profile=next(p for key,p in client.profiles.items() if key.endswith('.xsim.ros'))
            members=[client.member(i,profile=profile) for i in range(100)]
            began=time.monotonic()
            self.assert_members(client.apply(wire.ApplyMembersRequest(members=members),timeout=30),100)
            registration_ms=(time.monotonic()-began)*1000
            stream=client.status(wire.SubscribeStatusRequest(),timeout=20)
            counts={}; delays=[]; callback_delays=[]; failures=[]; imu_counts={}
            pose_channel='state.mocap.pose' if kind=='px4' else 'vrpn.position'
            twist_channel='state.mocap.velocity' if kind=='px4' else 'vrpn.velocity'
            def receive():
                try:
                    for batch in stream:
                        now=time.time_ns()
                        for status in batch.members:
                            for message in status.messages:
                                if message.channel_id == 'state.imu':
                                    imu_counts[status.identity.robot_id] = imu_counts.get(status.identity.robot_id,0)+1
                                    imu = telemetry_pb2.ImuEstimate.FromString(message.message.payload.value)
                                    if (imu.frame_id != 'source-body' or imu.orientation.w != 1 or
                                        imu.angular_velocity.z != .5 or imu.linear_acceleration.z != 9.80665 or
                                        list(imu.orientation_covariance) != [.001 if i % 4 == 0 else 0 for i in range(9)] or
                                        list(imu.angular_velocity_covariance) != [.002 if i % 4 == 0 else 0 for i in range(9)] or
                                        list(imu.linear_acceleration_covariance) != [.003 if i % 4 == 0 else 0 for i in range(9)]):
                                        failures.append('IMU field mapping changed')
                                if message.channel_id not in (pose_channel,twist_channel): continue
                                stamp=message.message.source_time.nanoseconds
                                if not stamp: continue
                                key=(status.identity.robot_id,message.channel_id)
                                counts[key]=counts.get(key,0)+1
                                delays.append((now-stamp)/1e6)
                                callback_delays.append((message.message.observed_unix_nanos-stamp)/1e6)
                                if message.channel_id==pose_channel:
                                    pose=telemetry_pb2.PoseEstimate.FromString(message.message.payload.value)
                                    if abs(pose.position.x-3)>1e-8: failures.append('direct pose changed')
                except grpc.RpcError: pass
            receiver=threading.Thread(target=receive);receiver.start()
            base=ROOT/(kind+'-xsim-125')
            report,ready,start=(Path(str(base)+suffix) for suffix in ('.json','.ready','.start'))
            report.unlink(missing_ok=True);ready.unlink(missing_ok=True);start.unlink(missing_ok=True)
            source=subprocess.Popen([str(ROOT/'ros_forwarding_source'),kind,'100',str(report),str(ready),str(start),'direct','125'],stdout=subprocess.DEVNULL)
            try:
                until=time.monotonic()+20
                while not ready.exists() and source.poll() is None and time.monotonic()<until:time.sleep(.01)
                self.assertTrue(ready.exists(),'125 Hz direct source did not connect every member')
                allocations_before=client.allocations()
                thread_cpu_before=client.cpu_ticks()
                before=Path(f'/proc/{client.process.pid}/stat').read_text().split()
                began=time.monotonic();start.write_text('start')
                until=time.monotonic()+12
                while not report.exists() and source.poll() is None and time.monotonic()<until:time.sleep(.01)
                self.assertTrue(report.exists(),'source did not finish publishing')
                after=Path(f'/proc/{client.process.pid}/stat').read_text().split()
                elapsed=time.monotonic()-began
                allocation_delta=client.allocation_delta(allocations_before)
                thread_cpu_percent=client.cpu_delta(thread_cpu_before,elapsed)
                self.assertEqual(source.wait(timeout=12),0)
                ticks=(int(after[13])+int(after[14]))-(int(before[13])+int(before[14]))
                stream.cancel();receiver.join(3)
                self.assertFalse(failures,failures)
                self.assertEqual(len(counts),200)
                self.assertGreaterEqual(min(counts.values()),24)
                self.assertEqual(len(imu_counts),100)
                self.assertGreaterEqual(min(imu_counts.values()),24)
                self.assertEqual(json.loads(report.read_text())['imu_samples'],9000)
                resources=client.resources();self.assertEqual(resources['application_threads'],4)
                percentile=lambda values,p: sorted(values)[int(p*(len(values)-1))]
                self.measurements.append(dict(resources,test='xsim-125hz',kind=kind,count=100,input_hz_per_channel=125,
                    input_channels=['pose','twist','imu'],input_hz={'pose':125,'twist':125,'imu':30},
                    source=json.loads(report.read_text()),registration_ms=registration_ms,
                    measurement_duration_s=elapsed,allocation_probe=bool(os.environ.get('ROBOT_SERVER_ALLOCATION_PRELOAD')),
                    allocation_delta=allocation_delta,thread_cpu_percent_one_core=thread_cpu_percent,
                    minimum_semantic_samples_per_channel=min(counts.values()),semantic_samples=sum(counts.values()),
                    minimum_imu_semantic_samples=min(imu_counts.values()),imu_semantic_samples=sum(imu_counts.values()),
                    source_to_client_ms={q:percentile(delays,p) for q,p in [('p50',.5),('p95',.95),('p99',.99)]},
                    source_to_adapter_observation_ms={q:percentile(callback_delays,p) for q,p in [('p50',.5),('p95',.95),('p99',.99)]},
                    cpu_percent_one_core=100*ticks/os.sysconf('SC_CLK_TCK')/elapsed))
            finally:
                stream.cancel();receiver.join(3)
                if source.poll() is None:source.terminate();source.wait(timeout=5)
                self.assert_members(client.remove(wire.RemoveMembersRequest(members=[member.identity for member in members]),timeout=30),100)

    def test_13_single_member_burst_does_not_starve_other_members(self):
        # Exercise two real sources, not just xsim: raw motion capture retains
        # every normal forwarding sample, while direct localization has no
        # output publisher. One member sends 500 extra frames per input tick.
        for kind, client in self.clients.items():
            for direct in (False, True):
                with self.subTest(kind=kind, direct=direct):
                    profile = (next(p for key, p in client.profiles.items() if key.endswith('.xsim.ros'))
                               if direct else client.profile)
                    rate = 125 if direct else 30
                    members = [client.member(i, profile=profile) for i in range(100)]
                    self.assert_members(client.apply(wire.ApplyMembersRequest(members=members), timeout=30), 100)
                    stream = client.status(wire.SubscribeStatusRequest(), timeout=25)
                    slow = client.status(wire.SubscribeStatusRequest())  # never read this stream
                    pose_channel = 'state.mocap.pose' if kind == 'px4' else 'vrpn.position'
                    twist_channel = 'state.mocap.velocity' if kind == 'px4' else 'vrpn.velocity'
                    counts, latest, delays, command_delays, failures = {}, {}, [], [], []
                    sequences, input_rates = {}, {}
                    lock = threading.Lock()

                    def receive():
                        try:
                            for batch in stream:
                                with lock:
                                    for status in batch.members:
                                        for message in status.messages:
                                            if message.channel_id == 'diagnostic.stream-health':
                                                health = telemetry_pb2.StreamHealthReport.FromString(message.message.payload.value)
                                                for channel in health.channels:
                                                    if channel.channel_id in (pose_channel, twist_channel):
                                                        input_rates[(status.identity.robot_id, channel.channel_id)] = dict(
                                                            source_hz=channel.source_rate_hz, output_hz=channel.output_rate_hz,
                                                            source_age_ms=channel.source_age_ms)
                                            if message.channel_id not in (pose_channel, twist_channel): continue
                                            stamp = message.message.source_time.nanoseconds
                                            if not stamp: continue
                                            key = (status.identity.robot_id, message.channel_id)
                                            counts[key] = counts.get(key, 0) + 1
                                            sequence = message.message.sequence
                                            if key not in sequences: sequences[key] = [sequence, sequence]
                                            else: sequences[key][1] = sequence
                                            latest[key] = stamp
                                            if status.identity.robot_id != kind + '0':
                                                delays.append((message.message.observed_unix_nanos - stamp) / 1e6)
                                            if message.channel_id == pose_channel:
                                                pose = telemetry_pb2.PoseEstimate.FromString(message.message.payload.value)
                                                if abs(pose.position.x - (3 if direct else 4.25)) > 1e-8:
                                                    failures.append('localization mapping changed')
                        except grpc.RpcError: pass

                    receiver = threading.Thread(target=receive)
                    receiver.start()
                    base = ROOT / (kind + ('-direct' if direct else '-raw') + '-burst')
                    report, ready, start = (Path(str(base) + suffix) for suffix in ('.json', '.ready', '.start'))
                    burst, recovered = Path(str(report) + '.burst'), Path(str(report) + '.recovered')
                    for path in (report, ready, start, burst, recovered): path.unlink(missing_ok=True)
                    observation, observation_ready = Path(str(base) + '-output.json'), Path(str(base) + '-output.ready')
                    observation_ready.unlink(missing_ok=True)
                    # Direct localization has no Adapter output publisher. Tap
                    # one healthy source to distinguish source delays from
                    # Adapter ingress without doubling the entire offered load.
                    observer = subprocess.Popen([str(ROOT / 'ros_forwarding_observer'), kind,
                        '1' if direct else '100', str(observation), str(observation_ready),
                        'direct' if direct else 'all'] + (['1'] if direct else []), stdout=subprocess.DEVNULL)
                    source = subprocess.Popen([str(ROOT / 'ros_forwarding_source'), kind, '100',
                        str(report), str(ready), str(start), 'direct' if direct else 'raw', str(rate), '500'],
                        stdout=subprocess.DEVNULL)
                    service = (rospy.Service('/px41/mavros/cmd/arming', CommandBool,
                               lambda request: CommandBoolResponse(success=True, result=0)) if kind == 'px4' else None)
                    try:
                        until = time.monotonic() + 25
                        while time.monotonic() < until and (not ready.exists() or
                              (observer is not None and not observation_ready.exists())):
                            self.assertIsNone(source.poll(), 'burst source exited before connecting')
                            time.sleep(.01)
                        self.assertTrue(ready.exists(), 'burst source did not connect all members')
                        if observer is not None: self.assertTrue(observation_ready.exists())
                        baseline_rss = client.resources()['rss_kib']
                        peak_rss = baseline_rss
                        thread_cpu_before = client.cpu_ticks()
                        cgroup_cpu_before = cgroup_cpu()
                        before = Path(f'/proc/{client.process.pid}/stat').read_text().split()
                        began = time.monotonic()
                        start.write_text('start')
                        until = time.monotonic() + 8
                        while not burst.exists() and time.monotonic() < until: time.sleep(.01)
                        self.assertTrue(burst.exists(), 'source never entered its burst')
                        for i in range(10):
                            command = (client.command(members[1], 'burst-arm-' + str(i)) if kind == 'px4' else
                                client.command(members[1], 'burst-intent-' + str(i), endpoint='set-motion-intent',
                                    value=common_control.RemoteControlIntentRequest(gear=3).SerializeToString()))
                            sent = time.monotonic()
                            result = client.execute(command, timeout=3)
                            command_delays.append((time.monotonic() - sent) * 1000)
                            self.assertEqual(result.phase, operation.OPERATION_PHASE_SUCCEEDED)
                            peak_rss = max(peak_rss, client.resources()['rss_kib'])
                            time.sleep(.05)
                        while not report.exists() and source.poll() is None and time.monotonic() < until:
                            peak_rss = max(peak_rss, client.resources()['rss_kib'])
                            time.sleep(.02)
                        self.assertTrue(report.exists(), 'source did not finish publishing')
                        after = Path(f'/proc/{client.process.pid}/stat').read_text().split()
                        elapsed = time.monotonic() - began
                        thread_cpu_percent = client.cpu_delta(thread_cpu_before, elapsed)
                        cgroup_cpu_after = cgroup_cpu()
                        self.assertEqual(source.wait(timeout=5), 0)
                        source_exit_elapsed = time.monotonic() - began
                        self.assertTrue(recovered.exists())
                        time.sleep(.15)
                        with lock:
                            final_latest = dict(latest)
                        cancel_began = time.monotonic()
                        stream.cancel()
                        receiver.join(3)
                        cancellation_ms = (time.monotonic() - cancel_began) * 1000
                        healthy_counts = [counts.get((kind + str(i), channel), 0)
                                          for i in range(1, 100) for channel in (pose_channel, twist_channel)]
                        self.assertFalse(failures, failures)
                        # The final half-second must reach every slot, including
                        # the previously hot member; no old-frame catch-up tail.
                        recovery_ns = recovered.stat().st_mtime_ns
                        self.assertEqual(len(final_latest), 200)
                        self.assertGreaterEqual(min(final_latest.values()), recovery_ns,
                            'not every channel received a post-burst frame: ' + str(sorted(final_latest.items(), key=lambda x: x[1])[:4]))
                        p99 = sorted(delays)[int(.99 * (len(delays) - 1))]
                        resources = client.resources()
                        self.assertEqual(resources['application_threads'], 4)
                        self.assertEqual(resources['children'], [])
                        forwarding = None
                        if observer is not None:
                            close_observer(observer)
                            self.assertEqual(observer.returncode, 0)
                            forwarding = json.loads(observation.read_text())
                            self.assertEqual(forwarding['invalid_samples'], 0)
                            if not direct:
                                for channel, samples in forwarding['channel_samples'].items():
                                    self.assertGreaterEqual(min(samples[1:]), 70, channel + ' lost normal input cadence')
                        source_report = json.loads(report.read_text())
                        self.assertGreater(source_report['burst_samples'], 0)
                        self.assertLess(source_report['duration_s'], 4.5, 'source itself could not maintain the offered load')
                        ticks = (int(after[13]) + int(after[14])) - (int(before[13]) + int(before[14]))
                        self.measurements.append(dict(resources, test='single-member-burst', kind=kind, count=100,
                            profile=profile['profileId'], input_hz_per_channel=rate, extra_frames_per_hot_tick=500,
                            measurement_duration_s=elapsed, test_source_exit_elapsed_s=source_exit_elapsed,
                            source=source_report, minimum_other_member_semantic_samples=min(healthy_counts),
                            other_member_channels=[dict(robot_id=kind + str(i), channel_id=channel,
                                received=counts.get((kind + str(i), channel), 0),
                                sequence_range=sequences.get((kind + str(i), channel)),
                                input_rates=input_rates.get((kind + str(i), channel)))
                                for i in range(1, 100) for channel in (pose_channel, twist_channel)],
                            thread_cpu_percent_one_core=thread_cpu_percent,
                            cgroup_cpu_delta={key:value-cgroup_cpu_before.get(key,0) for key,value in cgroup_cpu_after.items()},
                            other_member_source_to_observation_p99_ms=p99, command_max_ms=max(command_delays),
                            baseline_rss_kib=baseline_rss, peak_rss_kib=peak_rss,
                            forwarding=forwarding if not direct else None,
                            source_observation=forwarding if direct else None,
                            minimum_post_burst_source_time_ns=min(final_latest.values()), recovery_marker_unix_ns=recovery_ns,
                            test_stream_cancellation_ms=cancellation_ms,
                            cpu_percent_one_core=100 * ticks / os.sysconf('SC_CLK_TCK') / elapsed))
                        self.assertGreaterEqual(min(healthy_counts), 24, 'a burst starved another member')
                        self.assertLess(p99, 200, 'healthy localization stayed behind the source')
                        self.assertLess(max(command_delays), 500, 'ingress blocked an unrelated command')
                        self.assertLess(peak_rss - baseline_rss, 32768, 'burst retained unbounded history')
                    finally:
                        stream.cancel(); slow.cancel(); receiver.join(3)
                        if source.poll() is None: source.terminate(); source.wait(timeout=5)
                        if observer is not None: close_observer(observer)
                        if service is not None: service.shutdown()
                        self.assert_members(client.remove(wire.RemoveMembersRequest(
                            members=[member.identity for member in members]), timeout=30), 100)

if __name__ == '__main__':
    unittest.main(verbosity=2)
