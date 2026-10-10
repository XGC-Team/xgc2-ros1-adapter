"""Exercise the Python code embedded in the actual trusted recording recipe.

No ROS installation is required for unit/recipe-stub tests. These tests do not
claim camera precision, a live ROS transport, or native rosbag acquisition.
"""
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from xmlrpc.server import SimpleXMLRPCServer


def recipe_script():
    override = os.environ.get("RECORD_FACTS_TEST_SCRIPT")
    if override:
        return Path(override).read_text()
    return (Path(__file__).resolve().parents[1] / "scripts/rosbag_recorder.py").read_text()

SCRIPT = recipe_script()
RUNTIME_SCRIPT = SCRIPT[:SCRIPT.index('try:\n    if sys.argv[1:] == ["--prepare"]:')] + SCRIPT[SCRIPT.index('\ndef require_archive_path'):]

module = ast.parse(SCRIPT)
namespace = {}
# Compile production imports/class without launching the recorder at import.
selected = [node for node in module.body if isinstance(node, (ast.Import, ast.ImportFrom)) or
            isinstance(node, ast.ClassDef) and node.name == "RecordFacts"]
exec(compile(ast.Module(body=selected, type_ignores=[]), "record-recipe", "exec"), namespace)
RecordFacts = namespace["RecordFacts"]


def applied(sequence=1, instance="epoch-A", gain=2.5, event="applied", ros=None):
    return {
        "schemaVersion": 1,
        "source": {"id": "camera:world:optical", "kind": "camera-extrinsic",
                   "instanceId": instance, "resolutionId": "frozen-selection-A"},
        "sequence": sequence, "event": event, "evidence": "producer-applied",
        "effectiveAt": {"rosTimeNs": str(ros if ros is not None else sequence * 10),
                        "unixTimeNs": str(900000000000000000 + sequence),
                        "monotonicTimeNs": str(sequence * 100)},
        "values": {"gain": gain, "groundZ": None,
                   "transforms": [{"parentFrame": "world", "childFrame": "optical",
                                   "translation": [1.0, 2.0, 3.0], "quaternionXyzw": [0, 0, 0, 1]}]},
        "provenance": {"boundary": "complete-tf-chain-broadcast-returned",
                       "appliesTo": "camera-transform-publication",
                       "overrides": [{"source": "workflow-binding", "value": gain}]},
    }


class RecordFactsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.facts = RecordFacts({}, str(self.directory / "record"))

    def document(self):
        result = {}
        self.facts.snapshot(result)
        return result["recordFacts"]

    def body(self, event, directory=None):
        return (Path(directory or self.facts.directory) / event["content"]["relativePath"]).read_bytes()

    def test_file_revisions_are_portable_and_do_not_claim_applied(self):
        source = self.directory / "intrinsics.yaml"
        declaration = {"role": "intrinsic", "path": str(source)}
        self.facts.file(declaration, "calibration-file")
        source.write_bytes(b"K: [100, 0, 50]\n")
        self.facts.file(declaration, "calibration-file")
        self.facts.file(declaration, "calibration-file")
        source.write_bytes(b"K: [200, 0, 50]\n")
        self.facts.file(declaration, "calibration-file")
        source.unlink()
        self.facts.file(declaration, "calibration-file")
        events = self.document()["events"]
        self.assertEqual(len(events), 4)
        self.assertEqual(events[0]["evidence"], "unavailable")
        self.assertNotIn("content", events[0])
        destination = self.directory / "elsewhere"
        shutil.copytree(self.facts.directory, destination)
        shutil.rmtree(self.facts.directory)
        self.assertEqual(self.body(events[1], destination), b"K: [100, 0, 50]\n")
        self.assertEqual(self.body(events[2], destination), b"K: [200, 0, 50]\n")
        for event in events[1:3]:
            self.assertEqual(event["evidence"], "snapshot-observed")
            self.assertIsNone(event["validUntil"])
            self.assertEqual(hashlib.sha256(self.body(event, destination)).hexdigest(), event["content"]["sha256"])

    def test_only_declared_parameters_and_runtime_changes_are_observed(self):
        server = SimpleXMLRPCServer(("127.0.0.1", 0), logRequests=False)
        values = {"/controller/gain": 2.5}
        calls = []
        def get_parameter(caller, name):
            calls.append((caller, name))
            return [1, "ok", values[name]] if name in values else [-1, "missing", 0]
        server.register_function(get_parameter, "getParam")
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.facts.declarations = {"parameters": [{"name": "/controller/gain", "purpose": "final gain", "requestedValue": 1}]}
        with patch.dict(os.environ, {"ROS_MASTER_URI": "http://127.0.0.1:%s" % server.server_address[1]}):
            self.facts.poll()
            values["/controller/gain"] = 4.0
            self.facts.poll()
            del values["/controller/gain"]
            self.facts.poll()
            self.facts.poll()
            values["/controller/gain"] = 4.0
            self.facts.poll()
        events = self.document()["events"]
        self.assertEqual(len(events), 4)
        self.assertEqual(json.loads(self.body(events[0])), 2.5)
        self.assertEqual(json.loads(self.body(events[1])), 4.0)
        self.assertEqual(events[2]["evidence"], "unavailable")
        self.assertEqual(events[3]["evidence"], "snapshot-observed")
        self.assertTrue(all(name == "/controller/gain" for _, name in calls))
        self.assertEqual(self.facts.declarations["parameters"][0]["requestedValue"], 1)

    def test_actual_producer_raw_bytes_values_overrides_and_intervals(self):
        first = json.dumps(applied(1), indent=2)
        observed = {"unixTimeNs": "2", "monotonicTimeNs": "3"}
        self.facts.producer(first, observed)
        self.facts.producer(first, observed)  # Repeated latched message.
        self.facts.producer(json.dumps(applied(2, gain=8)), observed)
        self.facts.producer(json.dumps(applied(3, gain=8, event="stopped")), observed)
        events = self.document()["events"]
        self.assertEqual(len(events), 3)
        self.assertEqual(self.body(events[0]), first.encode())
        self.assertEqual(events[0]["validity"]["until"], applied(2)["effectiveAt"])
        self.assertEqual(events[1]["validity"]["endReason"], "stopped")
        self.assertIsNone(events[2]["validity"])
        self.assertEqual(events[0]["observedAt"]["unixTimeNs"], "2")
        self.assertEqual(json.loads(self.body(events[1]))["provenance"]["overrides"][0]["value"], 8)
        self.assertEqual(json.loads(self.body(events[0]))["values"]["transforms"][0]["translation"], [1, 2, 3])

    def test_late_recording_restart_and_sequence_gaps_never_fill_history(self):
        self.facts.producer(json.dumps(applied(7)), self.facts.clock())
        self.facts.producer(json.dumps(applied(9)), self.facts.clock())
        self.facts.producer(json.dumps(applied(1, instance="epoch-B")), self.facts.clock())
        result = self.document()
        events = result["events"]
        self.assertEqual(events[0]["continuity"], "prior-events-unobserved")
        self.assertEqual(events[0]["validity"]["endReason"], "sequence-gap")
        self.assertIsNone(events[0]["validity"]["until"])
        self.assertIsNone(events[1]["validity"]["until"])
        self.assertEqual(events[2]["source"]["instanceId"], "epoch-B")
        self.assertEqual(result["coverage"]["beforeObservation"], "unknown")

    def test_ros_clock_reset_is_not_a_synthetic_alignment(self):
        self.facts.producer(json.dumps(applied(1, ros=1000)), self.facts.clock())
        self.facts.producer(json.dumps(applied(2, ros=10)), self.facts.clock())
        result = self.document()
        self.assertEqual(result["events"][1]["continuity"], "ros-clock-regression")
        self.assertIsNone(result["events"][0]["validity"]["until"])
        self.assertEqual(result["coverage"]["clockAlignment"], "not-inferred")

    def test_invalid_or_non_finite_receipts_never_become_applied(self):
        bad = []
        for key, value in (("schemaVersion", 2), ("sequence", True), ("evidence", "requested"), ("event", "maybe")):
            body = applied()
            body[key] = value
            bad.append(json.dumps(body))
        body = applied()
        body["effectiveAt"]["rosTimeNs"] = "NaN"
        bad.append(json.dumps(body))
        bad.extend(['{"schemaVersion":1,"schemaVersion":1}', json.dumps(applied()).replace('2.5', 'NaN')])
        for text in bad:
            self.facts.enqueue("producer", "/xgc/record_facts", text)
            self.facts.process_messages()
        self.assertTrue(all(event["evidence"] == "unavailable" for event in self.document()["events"]))

    def test_sequence_conflict_is_rejected_without_overwriting_previous_bytes(self):
        self.facts.producer(json.dumps(applied()), self.facts.clock())
        with self.assertRaises(ValueError):
            self.facts.producer(json.dumps(applied(gain=99)), self.facts.clock())
        events = self.document()["events"]
        self.assertEqual(len(events), 1)
        self.assertEqual(json.loads(self.body(events[0]))["values"]["gain"], 2.5)

    def test_symlink_fifo_and_oversize_files_do_not_block_or_forge_hashes(self):
        regular = self.directory / "real"
        regular.write_bytes(b"secret")
        link = self.directory / "link"
        link.symlink_to(regular)
        fifo = self.directory / "fifo"
        os.mkfifo(fifo)
        large = self.directory / "large"
        with large.open("wb") as stream:
            stream.truncate(self.facts.MAX_BYTES + 1)
        started = time.monotonic()
        for path in (link, fifo, large):
            self.facts.file({"role": path.name, "path": str(path)}, "algorithm-file")
        self.assertLess(time.monotonic() - started, 1)
        self.assertTrue(all(event["evidence"] == "unavailable" and "content" not in event for event in self.document()["events"]))

    def test_overflow_and_shutdown_are_explicit_not_fake_complete(self):
        self.facts.MAX_EVENTS = 1
        self.facts.observation("parameter", "/gain", 1)
        self.facts.observation("parameter", "/gain", 2)
        for _ in range(300):
            self.facts.enqueue("producer", "/xgc/record_facts", json.dumps(applied()))
        document = {}
        self.facts.finish(document)
        result = document["recordFacts"]
        self.assertGreater(result["coverage"]["droppedEvents"], 0)
        self.assertEqual(result["coverage"]["pendingAtClose"], 256)
        self.assertEqual(result["status"], "closed")
        self.assertIsNotNone(result["observationEndedAt"])

    def test_manifest_revision_does_not_change_producer_wire_version(self):
        receipt = applied()
        self.assertEqual(receipt["schemaVersion"], 1)
        self.facts.producer(json.dumps(receipt), self.facts.clock())
        document = self.document()
        self.assertEqual(document["schemaVersion"], 2)
        self.assertEqual(document["contract"], "xgc.record-facts/v2")
        self.assertEqual(json.loads(self.body(document["events"][0]))["schemaVersion"], 1)

    def test_readable_names_and_empty_declarations(self):
        self.facts.declarations = {"calibrationFiles": None, "algorithmSidecarFiles": None, "parameters": None}
        self.facts.poll()
        self.facts.observation("parameter", "/controller/gain", 2)
        document = self.document()
        event = document["events"][0]
        self.assertIn("parameter-controller-gain-revision-", event["content"]["relativePath"])
        self.assertNotIn(event["content"]["sha256"], event["content"]["relativePath"])
        self.assertEqual(document["contract"], "xgc.record-facts/v2")
        self.assertEqual(document["schemaVersion"], 2)

    def test_ros_subscriptions_retain_late_producers_and_camera_observations(self):
        from types import SimpleNamespace as NS
        from unittest import mock
        callbacks = {}
        def subscribe(topic, message_type, callback, **options):
            callbacks[topic] = callback
            return NS(unregister=lambda: None)
        modules = {"rospy": NS(init_node=lambda *a, **k: None, Subscriber=subscribe),
                   "std_msgs": NS(), "std_msgs.msg": NS(String=object),
                   "sensor_msgs": NS(), "sensor_msgs.msg": NS(CameraInfo=object)}
        with mock.patch.dict(sys.modules, modules):
            self.facts.ros(["/xgc/camera/world"])
        self.assertEqual(set(callbacks), {"/xgc/record_facts", "/xgc/camera/world/camera_info"})
        callbacks["/xgc/record_facts"](NS(data=json.dumps(applied())))
        message = NS(header=NS(frame_id="camera_optical", stamp=NS(to_nsec=lambda: 120)),
            width=640, height=480, distortion_model="plumb_bob", D=[0], K=[1]*9,
            R=[1]*9, P=[1]*12, binning_x=0, binning_y=0,
            roi=NS(x_offset=0,y_offset=0,height=480,width=640,do_rectify=False))
        callbacks["/xgc/camera/world/camera_info"](message)
        self.facts.process_messages()
        message.K[0] = 2
        message.header.stamp.to_nsec = lambda: 130
        callbacks["/xgc/camera/world/camera_info"](message)
        self.facts.process_messages()
        events = self.document()["events"]
        camera = [event for event in events if event["kind"] == "camera-info"]
        self.assertEqual(len(camera), 2)
        self.assertEqual(camera[0]["observedAt"]["rosHeaderTimeNs"], "120")
        self.assertEqual(camera[1]["observedAt"]["rosHeaderTimeNs"], "130")
        self.assertEqual(json.loads(self.body(camera[0]))["K"][0], 1)
        self.assertTrue(all(event["evidence"] == "snapshot-observed" for event in camera))
        self.assertTrue(any(event["kind"] == "producer" and event["evidence"] == "producer-applied" for event in events))

    def test_native_recipe_still_finalizes_its_existing_record_identity(self):
        self.run_native_recipe()

    def test_blocked_ros_registration_and_shutdown_do_not_hold_the_recorder(self):
        # The callback crosses the real observer process pipe. Both registration
        # and its shutdown hook then hang, as rospy can do after losing a master.
        setup = '''import sys, types, time
def block(*args, **kwargs):
    while True:
        time.sleep(1)
rospy = types.ModuleType("rospy")
rospy.init_node = lambda *args, **kwargs: None
rospy.signal_shutdown = block
rospy.spin = block
def subscribe(topic, message_type, callback, **kwargs):
    callback(types.SimpleNamespace(data=RECEIPT))
    block()
rospy.Subscriber = subscribe
sys.modules["rospy"] = rospy
sys.modules["std_msgs"] = types.ModuleType("std_msgs")
sys.modules["std_msgs.msg"] = types.SimpleNamespace(String=object)
sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
sys.modules["sensor_msgs.msg"] = types.SimpleNamespace(CameraInfo=object)
'''.replace("RECEIPT", repr(json.dumps(applied())))
        document = self.run_native_recipe(setup)
        producers = [event for event in document["recordFacts"]["events"] if event["kind"] == "producer"]
        self.assertEqual(len(producers), 1)
        self.assertEqual(json.loads(self.body(producers[0], self.directory / "Experiments" / "Data")), applied())

    def run_native_recipe(self, setup=""):
        if "sys.argv[1:]" not in SCRIPT:
            self.skipTest("standalone fragment run; full recipe exercised in isolated checkout")
        root = self.directory / "ros"
        native = root / "lib/rosbag/record"
        native.parent.mkdir(parents=True)
        source = self.directory / "calibration.yaml"
        source.write_text("K: 1\n")
        native.write_text("#!/usr/bin/env python3\nimport os,sys,time\nfrom pathlib import Path\na=sys.argv\np=Path(a[a.index('--output-name')+1]+'_0.bag.active')\np.write_bytes(b'bag-stub')\ntime.sleep(1.4)\nPath(os.environ['FACT_TEST_FILE']).write_text('K: 2\\n')\ntime.sleep(1.4)\np.rename(str(p)[:-7])\n")
        native.chmod(0o700)
        topics = ["/xgc/record_facts", "/tf"]
        manifest = {"schemaVersion": 1, "recordNamingVersion": 1, "recordingId": "a" * 32,
                    "recordingProfile": "general", "topics": topics, "estimatedBytes": 0,
                    "experimentName": "experiment", "runMode": "simulation",
                    "factSources": {"calibrationFiles": [{"role": "calibration", "path": str(source)}]}}
        directory = self.directory / "Experiments" / "Data"
        prefix = directory / "record"
        command = [sys.executable, "-c", setup + RUNTIME_SCRIPT, str(root), str(prefix), json.dumps(topics),
                   "64", "1", "1", "none", "true", "1", "0", "1", json.dumps(manifest)]
        result = subprocess.run(command, text=True, capture_output=True, timeout=10,
                                env=dict(os.environ, FACT_TEST_FILE=str(source),
                                         XGC_USER_FILES_DIR=str(self.directory)))
        self.assertEqual(result.returncode, 0, result.stderr)
        document = json.loads((directory / "session-manifest.json").read_text())
        self.assertEqual(document["bagFiles"][0]["id"], "bag." + "a" * 32 + ".0")
        self.assertEqual(document["archiveFinalizationState"], "finalized")
        events = [event for event in document["recordFacts"]["events"] if event["kind"] == "calibration-file"]
        self.assertEqual(len(events), 2)
        self.assertEqual(self.body(events[0], directory), b"K: 1\n")
        self.assertEqual(self.body(events[1], directory), b"K: 2\n")
        self.assertEqual(document["recordingStartedAtEvidence"], "recorder-launch-request")
        self.assertEqual(document["firstBagObservedAtEvidence"], "file-presence-observed-not-first-sample")
        self.assertEqual(document["recordFacts"]["status"], "closed")
        return document


if __name__ == "__main__":
    unittest.main()
