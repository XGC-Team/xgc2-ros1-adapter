"""Fixtures for the integration tests of the ROS1 type servers.

The tests run the executables against a private roscore: one master on a port
of its own, one runtime directory, one ROS home. They need

  XGC_TEST_DEVEL     the catkin devel space: lib/<package>/ holds the executables
                     under test and the gRPC test client (robot_server_client)
  XGC_TEST_BUILD     the catkin build space holding the generated profile
                     catalogs (<package>/robot-adapter-profiles/*.json)
  XGC_TEST_REGISTRY  the protobuf message registry (registry.json)

or, to test the installed packages, XGC_TEST_INSTALL_ROOT (the root of the file
system the packages are installed into: "/" in the release container, a DESTDIR
elsewhere) in place of XGC_TEST_BUILD and for the executables, which are then
<root>/opt/ros/noetic/lib/<package>/<executable>, and the catalogs
<root>/usr/share/xgc2/robot-adapter-profiles/<provider>.json. The libraries of
the packages have to be on LD_LIBRARY_PATH, as setup.bash makes them. A sourced
ROS environment (rospy, rosgraph, roscore, rosnode) is required.
"""

import http.client
import json
import os
import random
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import uuid
from pathlib import Path

import yaml

# A call to a master that never answers must fail the test, not hang it.
socket.setdefaulttimeout(15)

REPOSITORY = Path(__file__).resolve().parents[2]
PROFILES = REPOSITORY / "profiles" / "ros1"
FAKE_ROBOTS = Path(__file__).resolve().parent / "fake_robots.py"

UGV = "xgc_ugv_ros1_adapter/xgc_ugv_ros1_adapter_node"
PX4 = "xgc_px4_multirotor_ros1_adapter/xgc_px4_multirotor_ros1_adapter_node"
TOOLS = "xgc_ros1_tools_adapter/xgc_ros1_tools_adapter_node"
CLIENT = "xgc_ros1_adapter_common/robot_server_client"

SCOUT = "xgc2-scout-mini-ros1-adapter"
MECANUM = "xgc2-mecanum-ugv-ros1-adapter"
PX4_PROVIDER = "xgc2-px4-multirotor-ros1-adapter"
CATALOG_DIRECTORY = {
    SCOUT: "ugv_ros1_adapter",
    MECANUM: "ugv_ros1_adapter",
    PX4_PROVIDER: "px4_multirotor_ros1_adapter",
}
PROFILE_PREFIX = {SCOUT: "scout-mini", MECANUM: "mecanum-ugv", PX4_PROVIDER: "px4-multirotor"}

EXIT_OK, EXIT_FAILURE, EXIT_USAGE = 0, 1, 2
EXIT_MASTER_CHANGED, EXIT_MASTER_UNREACHABLE, EXIT_ROS_SHUTDOWN = 3, 4, 5


def environment_path(name):
    value = os.environ.get(name)
    if not value:
        raise unittest.SkipTest("{} is not set; see test/integration/harness.py".format(name))
    return Path(value)


def devel_file(relative):
    """An executable under test: installed when XGC_TEST_INSTALL_ROOT is set."""
    install = os.environ.get("XGC_TEST_INSTALL_ROOT")
    path = (Path(install) / "opt/ros/noetic" if install else environment_path("XGC_TEST_DEVEL")) \
        / "lib" / relative
    if not path.exists():
        raise unittest.SkipTest("{} is not built".format(path))
    return path


def client_binary():
    """The gRPC client binary: always from the catkin workspace."""
    path = environment_path("XGC_TEST_DEVEL") / "lib" / CLIENT
    if not path.exists():
        raise unittest.SkipTest("{} is not built".format(path))
    return path


def free_port():
    for _ in range(200):
        port = random.randint(20000, 39999)
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                continue
            return port
    raise RuntimeError("no free port")


def wait_until(predicate, timeout, interval=0.02, message="condition"):
    deadline = time.monotonic() + timeout
    while True:
        value = predicate()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise AssertionError("timed out waiting for " + message)
        time.sleep(interval)


class Workspace:
    """A private runtime directory and ROS home; removed at the end."""

    def __init__(self):
        self.root = Path(tempfile.mkdtemp(prefix="xgc-ros1-it-"))
        os.chmod(self.root, 0o700)
        self.sockets = self.root / "sockets"
        self.sockets.mkdir(mode=0o700)
        self.ros_home = self.root / "ros_home"
        self.ros_home.mkdir()
        self.processes = []

    def close(self):
        for process in reversed(self.processes):
            process.cleanup()
        shutil.rmtree(self.root, ignore_errors=True)


class Roscore:
    """A roscore on its own port, in its own process group."""

    def __init__(self, workspace, port=None):
        self.workspace = workspace
        self.port = port or free_port()
        self.uri = "http://127.0.0.1:{}".format(self.port)
        self.process = None
        self.log = workspace.root / "roscore-{}.log".format(self.port)

    def environment(self):
        env = dict(os.environ)
        env.update(
            ROS_MASTER_URI=self.uri,
            ROS_IP="127.0.0.1",
            ROS_HOME=str(self.workspace.ros_home),
            ROS_LOG_DIR=str(self.workspace.ros_home / "log"),
        )
        env.pop("ROS_HOSTNAME", None)
        return env

    def start(self):
        with open(self.log, "ab") as log:
            self.process = subprocess.Popen(
                ["roscore", "-p", str(self.port)],
                env=self.environment(), stdout=log, stderr=log, start_new_session=True,
            )
        self.workspace.processes.append(self)
        wait_until(self.alive, 30, 0.1, "roscore on " + self.uri)
        return self

    def call(self, method, *arguments):
        """One master API call; the connection is closed again."""
        import rosgraph
        master = rosgraph.Master("/xgc_integration_test", master_uri=self.uri)
        try:
            return getattr(master, method)(*arguments)
        finally:
            master.handle("close")()

    def alive(self):
        try:
            self.call("getPid")
            return True
        except Exception:
            return False

    def pid(self):
        return self.call("getPid")

    def _tree(self, pid):
        """The process and all its descendants (roslaunch puts its children in
        sessions of their own, so a process group does not reach them)."""
        out = subprocess.run(["pgrep", "-P", str(pid)], capture_output=True).stdout.split()
        pids = [pid]
        for child in out:
            pids.extend(self._tree(int(child)))
        return pids

    def kill(self):
        """Ends the master the hard way, as a crash would."""
        if self.process is not None and self.process.poll() is None:
            for pid in reversed(self._tree(self.process.pid)):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self.process.wait()
        wait_until(lambda: not self.alive(), 10, 0.05, "the master to be gone")

    def restart(self):
        """A new master on the same URI: another process, another run id."""
        self.kill()
        time.sleep(0.2)
        return self.start()

    def _master_pids(self):
        out = subprocess.run(["pgrep", "-f", "rosmaster.*-p {}".format(self.port)],
                             capture_output=True).stdout
        return [int(pid) for pid in out.split()]

    def stop_responding(self):
        """A master that accepts connections but never answers."""
        for pid in self._master_pids():
            os.kill(pid, signal.SIGSTOP)

    def resume(self):
        for pid in self._master_pids():
            os.kill(pid, signal.SIGCONT)

    def run_id(self):
        """The id roslaunch gave this master run (the /run_id parameter)."""
        return self.call("getParam", "/run_id")

    def system_state(self):
        publishers, subscribers, services = self.call("getSystemState")
        return {
            "publishers": {topic: set(nodes) for topic, nodes in publishers},
            "subscribers": {topic: set(nodes) for topic, nodes in subscribers},
            "services": {name: set(nodes) for name, nodes in services},
        }

    def nodes(self):
        out = subprocess.check_output(["rosnode", "list"], env=self.environment(), timeout=20)
        return set(out.decode().split())

    def cleanup(self):
        if self.process is not None and self.process.poll() is None:
            for pid in reversed(self._tree(self.process.pid)):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self.process.wait()


class Client:
    """The gRPC client binary, one RPC per call, JSON in and out."""

    def __init__(self, socket_path):
        self.socket_path = str(socket_path)
        self.binary = client_binary()

    def run(self, *arguments, stdin=None, timeout=60, deadline_ms=10000):
        out = subprocess.run(
            [str(self.binary), "--socket", self.socket_path, "--deadline-ms", str(deadline_ms), *arguments],
            input=None if stdin is None else json.dumps(stdin).encode(),
            capture_output=True, timeout=timeout,
        )
        if not out.stdout:
            raise AssertionError("client printed nothing: " + out.stderr.decode())
        return json.loads(out.stdout.decode().strip().splitlines()[-1])

    def describe(self, wait_ms=0):
        return self.run("describe", str(wait_ms), deadline_ms=wait_ms + 10000)

    def health(self):
        return self.run("health")

    def apply(self, members, deadline_ms=10000):
        return self.run("apply", stdin={"members": members}, deadline_ms=deadline_ms,
                        timeout=deadline_ms / 1000.0 + 30)

    def remove(self, identities):
        return self.run("remove", stdin={"members": identities})

    def execute(self, identity, endpoint, work_id, input_payload=None, deadline_s=30):
        operation = {
            "context": {
                "work_id": work_id,
                "endpoint_id": endpoint,
                "deadline": {"deadline_unix_nanos": str(int((time.time() + deadline_s) * 1e9))},
            },
        }
        if input_payload is not None:
            operation["input"] = input_payload
        return self.run("execute", stdin={"identity": identity, "operation": operation})

    def shutdown(self):
        return self.run("shutdown", deadline_ms=30000, timeout=60)

    def watch(self, seconds):
        """Starts a status stream; frames arrive through next_frames()."""
        return Watch(self, seconds)


class Watch:
    def __init__(self, client, seconds):
        self.process = subprocess.Popen(
            [str(client.binary), "--socket", client.socket_path, "watch", str(seconds)],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        self.frames = []
        import threading
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                frame = json.loads(line.decode())
            except ValueError:
                continue
            with self._lock:
                self.frames.append(frame)

    def members(self):
        """Latest message per robot and channel, and the robots removed."""
        latest, removed = {}, set()
        with self._lock:
            frames = list(self.frames)
        for frame in frames:
            for member in frame.get("members", []):
                robot = member["identity"]["robot_id"]
                if member.get("removed"):
                    removed.add(robot)
                    latest.pop(robot, None)
                    continue
                removed.discard(robot)
                for message in member.get("messages", []):
                    latest.setdefault(robot, {})[message["channel_id"]] = message
        return latest, removed

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
        self.process.wait(timeout=10)
        self._thread.join(timeout=5)
        self.process.stdout.close()


class Server:
    """One type server process of a Run."""

    def __init__(self, workspace, roscore, provider, run_id, name=None, extra=(), executable=None):
        self.workspace = workspace
        self.roscore = roscore
        self.provider = provider
        self.run_id = run_id
        self.name = name or "xgc_{}_{}".format(PROFILE_PREFIX[provider].replace("-", "_"), run_id)
        self.socket = workspace.sockets / "{}.sock".format(run_id)
        self.log = workspace.root / "{}.log".format(run_id)
        self.executable = executable or devel_file(PX4 if provider == PX4_PROVIDER else UGV)
        self.extra = list(extra)
        self.process = None

    def command(self):
        return [
            str(self.executable),
            "--socket-path", str(self.socket),
            "--run-id", self.run_id,
            "--node-name", self.name,
            "--provider", self.provider,
            "--target-id", "local",
            "--ros-master-uri", self.roscore.uri,
            "--ros-ip", "127.0.0.1",
            *self.extra,
        ]

    def start(self):
        with open(self.log, "ab") as log:
            self.process = subprocess.Popen(
                self.command(), env=self.roscore.environment(), stdout=log, stderr=log,
                start_new_session=True,
            )
        self.workspace.processes.append(self)
        wait_until(lambda: self.socket.exists() or self.process.poll() is not None, 20, 0.01,
                   "the server socket")
        self.connect()
        return self

    def connect(self):
        self.client = Client(self.socket)

    @property
    def node(self):
        """The ROS graph name; characters illegal in a ROS name become "_"."""
        return "/" + re.sub(r"[^A-Za-z0-9_]", "_", self.name)

    @property
    def pid(self):
        return self.process.pid

    def cleanup(self):
        if self.process is not None and self.process.poll() is None:
            self.process.kill()
            self.process.wait()

    def wait(self, timeout):
        try:
            return self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            raise AssertionError("the server did not exit within {} s\n{}".format(timeout, self.output()))

    def signal(self, number):
        os.kill(self.process.pid, number)

    def threads(self):
        return len(os.listdir("/proc/{}/task".format(self.process.pid)))

    def output(self):
        return self.log.read_text(errors="replace") if self.log.exists() else ""

    def exited(self):
        return self.process.poll() is not None


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout):
        super().__init__("localhost", timeout=timeout)
        self.path = str(path)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


class ToolsClient:
    """HTTP over the Tools server's socket, with the XRPC request headers."""

    def __init__(self, socket_path):
        self.socket_path = socket_path

    def request(self, method, target, body=None, instance=None, timeout_ms=10000):
        connection = UnixConnection(self.socket_path, timeout_ms / 1000.0 + 5)
        headers = {"X-Request-ID": uuid.uuid4().hex, "X-Xrpc-Timeout-Ms": str(timeout_ms)}
        payload = None
        if instance:
            headers["X-Xrpc-Instance-ID"] = instance
        if body is not None:
            headers["Content-Type"] = "application/json"
            payload = json.dumps(body)
        try:
            connection.request(method, target, body=payload, headers=headers)
            response = connection.getresponse()
            data = response.read()
            return response.status, dict(response.getheaders()), (json.loads(data) if data else None)
        finally:
            connection.close()

    def describe(self, instance=None, wait_ms=None):
        target = "/v1/describe" + ("" if wait_ms is None else "?wait_ready_ms={}".format(wait_ms))
        return self.request("GET", target, instance=instance, timeout_ms=(wait_ms or 0) + 10000)


class ToolsServer(Server):
    """The ROS1 Tools process of a Run."""

    def __init__(self, workspace, roscore, run_id, name=None):
        super().__init__(workspace, roscore, None, run_id,
                         name=name or "xgc_ros1_tools_" + run_id, executable=devel_file(TOOLS))

    def command(self):
        return [
            str(self.executable),
            "--socket-path", str(self.socket),
            "--target-id", "local",
            "--run-id", self.run_id,
            "--node-name", self.name,
            "--ros-master-uri", self.roscore.uri,
            "--ros-ip", "127.0.0.1",
            "--ros-hostname", "",
        ]

    def connect(self):
        self.http = ToolsClient(self.socket)

    def instance(self):
        status, _, body = self.http.describe()
        assert status == 200, body
        return body["instance_id"]


def registry_entry(message_id):
    registry = json.loads(environment_path("XGC_TEST_REGISTRY").read_text())
    for entry in registry["messages"]:
        if entry["id"] == message_id:
            return entry
    raise KeyError(message_id)


def _varint(value):
    value &= (1 << 64) - 1
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def payload(message_id, body):
    """A protobuf Payload (as protobuf JSON) for message `message_id`."""
    import base64
    entry = registry_entry(message_id)
    return {
        "schema": {
            "message_id": message_id,
            "type_name": entry["type"],
            "schema_version": entry["version"],
            "schema_fingerprint": str(entry["fingerprint"]),
        },
        "encoding": "PAYLOAD_ENCODING_PROTOBUF",
        "value": base64.b64encode(body).decode(),
    }


def motion_intent_payload(gear, longitudinal, lateral, yaw):
    """RemoteControlIntentRequest{gear, longitudinal, lateral, yaw}."""
    body = b""
    for number, value in ((1, gear), (2, longitudinal), (3, lateral), (4, yaw)):
        if value:
            body += _varint(number << 3) + _varint(value)
    return payload(3205, body)


def arm_payload(armed):
    """ArmRequest{armed}."""
    return payload(3201, _varint(1 << 3) + _varint(1) if armed else b"")


def mode_payload(mode):
    """ModeRequest{mode}."""
    text = mode.encode()
    return payload(3202, _varint(1 << 3 | 2) + _varint(len(text)) + text)


def load_catalog(provider):
    if os.environ.get("XGC_TEST_INSTALL_ROOT"):
        path = Path(os.environ["XGC_TEST_INSTALL_ROOT"]) / "usr/share/xgc2/robot-adapter-profiles" \
            / (provider + ".json")
    else:
        build = environment_path("XGC_TEST_BUILD")
        path = build / CATALOG_DIRECTORY[provider] / "robot-adapter-profiles" / (provider + ".json")
    if not path.exists():
        raise unittest.SkipTest("{} is not built".format(path))
    return {entry["profileId"]: entry for entry in json.loads(path.read_text())["profiles"]}


def profile_yaml(profile_id):
    for path in PROFILES.glob("*.yaml"):
        document = yaml.safe_load(path.read_text())
        if document["profile_id"] == profile_id:
            return document
    raise KeyError(profile_id)


def top_name(root, name):
    clean_root = root.strip("/")
    clean_name = name.strip("/")
    return "/{}/{}".format(clean_root, clean_name) if clean_root else "/" + clean_name


def resolve(template, parameters):
    return re.sub(r"\{([a-z_]+)\}", lambda match: parameters[match.group(1)], template)


class Robot:
    """One robot of a Run: its member registration and the ROS interface its
    profile implies, so the fake robots can stand in for the vehicle."""

    def __init__(self, provider, robot_id, profile_suffix, namespace=None, run_id="run-it",
                 epoch=1, **extra):
        self.provider = provider
        self.robot_id = robot_id
        self.profile_id = "{}.{}".format(PROFILE_PREFIX[provider], profile_suffix)
        self.namespace = namespace or "/" + robot_id
        self.run_id = run_id
        self.epoch = epoch
        self.catalog = load_catalog(provider)[self.profile_id]
        self.document = profile_yaml(self.profile_id)
        self.parameters = {
            "namespace": self.namespace,
            "positioning_frame_number": "5",
            "positioning_comparison_threshold_m": "1e-10",
        }
        if "mocap_rigid_body" in self.document["parameters"]:
            self.parameters["mocap_rigid_body"] = robot_id.capitalize()
            self.parameters["mocap_source_root"] = "/vrpn_client_node_test"
            for axis in "xyz":
                self.parameters["localization_offset_" + axis] = "0"
        if "localization_pose_topic" in self.document["parameters"] and \
                self.document["parameters"]["localization_pose_topic"]["required"]:
            self.parameters["localization_pose_topic"] = "/{}/xsim/pose".format(robot_id)
            self.parameters["localization_twist_topic"] = "/{}/xsim/twist".format(robot_id)
        self.parameters.update(extra)

    @property
    def identity(self):
        return {"target_id": "local", "run_id": self.run_id, "robot_id": self.robot_id,
                "connection_epoch": str(self.epoch)}

    def registration(self):
        return {
            "identity": self.identity,
            "configuration": {
                "robot_id": self.robot_id,
                "profile_id": self.profile_id,
                "profile_digest": self.catalog["profileDigest"],
                "parameters": dict(self.parameters),
                "channels": [{"channel_id": channel["id"], "enabled": True}
                             for channel in self.catalog["channels"]],
            },
        }

    def topic(self, name, scope):
        resolved = resolve(name, self.parameters)
        if scope == "global":
            return top_name(self.parameters.get("mocap_source_root", ""), resolved) \
                if "mocap_rigid_body" in self.parameters else "/" + resolved.lstrip("/")
        return top_name(self.namespace, resolved)

    def input_topics(self):
        """(topic, type) the vehicle publishes for this robot."""
        topics = {}
        for channel in self.document["channels"]:
            for item in (channel.get("inputs") or {}).values():
                topics[self.topic(item["name"], item.get("scope", "robot_namespace"))] = item["type"]
        return topics

    def services(self):
        return {top_name(self.namespace, channel["service"]["name"]): channel["service"]["type"]
                for channel in self.document["channels"] if "service" in channel}

    def command_topics(self):
        """(topic, type) the server publishes the robot's commands on."""
        return {top_name(self.namespace, channel["output"]["name"]): channel["output"]["type"]
                for channel in self.document["channels"]
                if channel["kind"] == "operation" and "output" in channel}


class FakeRobots:
    """The ROS side of some vehicles: publishes what their profiles read and
    records what the server publishes to them. `publish`, `record` and
    `services` ({name: type}) add interfaces of their own."""

    def __init__(self, workspace, roscore, robots=(), rate=20, publish=None, record=None,
                 services=None, service_delay=0):
        self.workspace = workspace
        self.roscore = roscore
        self.robots = robots
        self.rate = rate
        self.service_delay = service_delay
        self.extra = (publish or {}, record or {}, services or {})
        self.record = workspace.root / "fake-{}.jsonl".format(id(self))
        self.process = None

    def start(self):
        publish, record, services = (dict(part) for part in self.extra)
        for robot in self.robots:
            publish.update(robot.input_topics())
            record.update(robot.command_topics())
            services.update(robot.services())
        for topic in record:
            publish.pop(topic, None)  # the server publishes the commands
        config = self.workspace.root / "fake-{}.json".format(id(self))
        config.write_text(json.dumps({"publish": publish, "record": record,
                                      "services": services, "rate": self.rate,
                                      "service_delay": self.service_delay,
                                      "log": str(self.record)}))
        self.process = subprocess.Popen(
            [sys.executable, str(FAKE_ROBOTS), str(config)], env=self.roscore.environment(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
        )
        self.workspace.processes.append(self)
        line = self.process.stdout.readline().decode().strip()
        if line != "ready":
            raise AssertionError("fake robots failed to start: " + line)
        return self

    def received(self, topic):
        if not self.record.exists():
            return []
        out = []
        for line in self.record.read_text().splitlines():
            entry = json.loads(line)
            if entry["topic"] == topic:
                out.append(entry)
        return out

    def cleanup(self):
        if self.process is None or self.process.stdout.closed:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.process.stdout.close()


def new_run_id():
    return "run-" + uuid.uuid4().hex[:8]


def process_exists(pid):
    """Whether a process is still running (a zombie is not)."""
    try:
        with open("/proc/{}/stat".format(pid)) as stream:
            return stream.read().rsplit(")", 1)[1].split()[0] != "Z"
    except FileNotFoundError:
        return False


def registrations(roscore, node):
    """What the master holds for a node: topics published and subscribed,
    services provided."""
    state = roscore.system_state()
    return {kind: sorted(name for name, nodes in table.items() if node in nodes)
            for kind, table in state.items()}


def is_zero(message):
    return all(value == 0 for key in ("linear", "angular") for value in message.get(key, []))


class MasterCase(unittest.TestCase):
    """Tests that share one private master. Every test starts its servers
    with a Run id of its own and leaves nothing behind."""

    @classmethod
    def setUpClass(cls):
        environment_path("XGC_TEST_DEVEL")
        cls.workspace = Workspace()
        cls.roscore = Roscore(cls.workspace).start()

    @classmethod
    def tearDownClass(cls):
        cls.workspace.close()

    def start_server(self, provider, run_id=None, **options):
        server = Server(self.workspace, self.roscore, provider, run_id or new_run_id(), **options)
        server.start()
        self.addCleanup(server.cleanup)
        return server

    def start_fake_robots(self, robots, **options):
        fake = FakeRobots(self.workspace, self.roscore, robots, **options).start()
        self.addCleanup(fake.cleanup)
        return fake

    def ready(self, server):
        reply = server.client.describe(wait_ms=20000)
        self.assertEqual(reply["status"]["code"], 0, reply)
        self.assertTrue(reply["response"]["ready"], reply)
        return reply["response"]

    def apply(self, server, robots):
        reply = server.client.apply([robot.registration() for robot in robots])
        self.assertEqual(reply["status"]["code"], 0, reply)
        members = reply["response"]["members"]
        self.assertEqual([m["identity"]["robot_id"] for m in members], [r.robot_id for r in robots])
        return members

    def apply_all(self, server, robots):
        for member in self.apply(server, robots):
            self.assertNotIn("error", member, member)

    def watch(self, server, seconds=60):
        watch = server.client.watch(seconds)
        self.addCleanup(watch.close)
        return watch

    def node_of(self, server):
        return server.node

    def assert_gone(self, server, exit_code=EXIT_OK):
        """The process ended with `exit_code` and the master holds nothing of it."""
        self.assertEqual(server.wait(60), exit_code, server.output())
        self.assertFalse(process_exists(server.pid))
        self.assertNotIn(self.node_of(server), self.roscore.nodes())
        self.assertEqual(registrations(self.roscore, self.node_of(server)),
                         {"publishers": [], "subscribers": [], "services": []})
