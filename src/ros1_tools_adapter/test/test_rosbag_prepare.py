"""Recorder preparation cases moved from the retired Go domain compiler."""
import copy
import io
import json
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/rosbag_recorder.py"
script = SCRIPT.read_text()
namespace = {}
exec(compile(script[:script.index('try:\n    if sys.argv[1:] == ["--prepare"]:')], "native-recorder-prepare", "exec"), namespace)
prepare = namespace["prepare"]


def request():
    return {"parameters": {}, "robots": {"experimentResourceId": "experiment-a", "experimentCommitId": "commit-a", "experimentDigest": "a" * 64, "robotSelectionDigest": "b" * 64,
            "robots": [{"id": "uav-1", "namespace": "/uav1", "profileId": "px4-multirotor.xsim.ros", "kind": "px4-multirotor", "px4": {"mocapRigidBodyName": "Body1"}, "visualization": {}}]},
            "context": {"experimentName": "Frozen experiment", "runMode": "simulation", "scene": {"simulator": "xsim"},
                        "rosMasterUri": "http://master.test:27181"}}


class PreparationTests(unittest.TestCase):
    def test_defaults_and_topics_keep_the_frozen_roster(self):
        document = request()
        parameters, resolved, manifest = prepare(document)
        self.assertEqual(resolved, {"slotIds": ["uav-1"], "topics": ["/clock", "/tf", "/tf_static"]})
        self.assertEqual((parameters["splitSizeMiB"], parameters["maxSplits"], parameters["compression"]), (1024, 10, "lz4"))
        self.assertEqual(manifest["experimentResourceId"], "experiment-a")
        self.assertNotIn("localizationOffset", manifest)
        self.assertEqual(document, request())

    def test_measurement_topics_use_only_frozen_parameters_in_every_mode(self):
        document = request()
        document["parameters"] = {"globalTopics": [], "includeMotionCapture": True}
        document["robots"]["robots"][0].update(runtimeParameters={
            "localization_pose_topic": "/truth/custom_pose", "localization_twist_topic": "/truth/custom_twist"},
            simulationPoseTopic="/ignored/pose", simulationTwistTopic="/ignored/twist")
        for mode in ("simulation", "physical", "hybrid"):
            for simulator in ("xsim", "gazebo"):
                document["context"].update(runMode=mode, scene={"simulator": simulator})
                with self.subTest(mode=mode, simulator=simulator):
                    self.assertEqual(prepare(document)[1]["topics"], ["/truth/custom_pose", "/truth/custom_twist"])
        document["parameters"]["slotIds"] = ["absent"]
        with self.assertRaisesRegex(ValueError, "outside"):
            prepare(document)

    def test_missing_or_invalid_frozen_measurement_topics_never_infer_a_fallback(self):
        for parameters in ({}, {"localization_pose_topic": "/pose"},
                           {"localization_twist_topic": "/twist"},
                           {"localization_pose_topic": "relative", "localization_twist_topic": "/twist"}):
            document = request()
            document["parameters"]["includeMotionCapture"] = True
            document["robots"]["robots"][0]["runtimeParameters"] = parameters
            with self.subTest(parameters=parameters), self.assertRaisesRegex(ValueError, "explicit frozen localization topics"):
                prepare(document)

    def test_recording_requires_the_frozen_master_but_preview_does_not(self):
        for master in (None, "", "   ", "not-a-url"):
            document = request()
            if master is None:
                del document["context"]["rosMasterUri"]
            else:
                document["context"]["rosMasterUri"] = master
            with self.subTest(master=master), self.assertRaisesRegex(ValueError, "rosMasterUri"):
                prepare(document)
        document = request()
        del document["context"]["rosMasterUri"]
        document["parameters"]["rosMasterUri"] = "http://authored.test:11311"
        with self.assertRaisesRegex(ValueError, "frozen Session rosMasterUri"):
            prepare(document)
        del document["parameters"]["rosMasterUri"]
        self.assertNotIn("rosMasterUri", prepare(document, preview=True)[0])
        document = request()
        document["parameters"]["rosMasterUri"] = "http://other.test:11311"
        with self.assertRaisesRegex(ValueError, "disagrees"):
            prepare(document)

    def test_preview_cli_uses_selection_protocol_without_a_master(self):
        document = request()
        del document["context"]["rosMasterUri"]
        result = subprocess.run([sys.executable, str(SCRIPT), "--prepare"], input=json.dumps(document),
                                text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), {"slotIds": ["uav-1"],
                         "topics": ["/clock", "/tf", "/tf_static"], "estimatedBytes": 0})

    def test_record_and_preview_validators_reject_each_others_shapes(self):
        session = {"parameters": {}, "context": {"robots": [], "experimentResourceId": "experiment-a",
                   "experimentCommitId": "commit-a", "simulation": {"rosMasterUri": "http://master.test:27181", "rosIp": ""}}}
        def read(name, document):
            with patch.object(sys, "stdin", types.SimpleNamespace(buffer=io.BytesIO(json.dumps(document).encode()))):
                return namespace[name]()
        self.assertEqual(read("read_record_request", session)["context"]["rosMasterUri"], "http://master.test:27181")
        self.assertEqual(read("read_preview_request", request()), request())
        with self.assertRaisesRegex(ValueError, "frozen Session"):
            read("read_record_request", request())
        with self.assertRaisesRegex(ValueError, "frozen Robot selection"):
            read("read_preview_request", session)
        with patch.object(sys, "stdin", types.SimpleNamespace(buffer=io.BytesIO(b'{"parameters":{},"parameters":{},"context":{}}'))):
            with self.assertRaisesRegex(ValueError, "duplicate field"):
                namespace["read_record_request"]()

    def test_scientific_facts_must_be_explicit_and_agree(self):
        document = request()
        document["parameters"].update(recordingProfile="camera_scientific", cameras=[{"root": "/camera", "topics": ["video", "frame_timing"]}])
        with self.assertRaisesRegex(ValueError, "sessionId"):
            prepare(document)
        self.assertIn("/camera/frame_timing", prepare(document, preview=True)[1]["topics"])
        document["context"].update(sessionId="session-a", localizationOffset={"x": 1, "y": 2, "z": 3}, worldBoundary=None)
        parameters, _, manifest = prepare(document)
        self.assertEqual(manifest["localizationOffset"], document["context"]["localizationOffset"])
        self.assertIsNone(manifest["worldBoundary"])
        self.assertEqual(manifest["scientificTotalLimitBytes"], 10 << 30)
        self.assertEqual(parameters["compression"], "none")
        document["parameters"]["sessionId"] = "other-session"
        with self.assertRaisesRegex(ValueError, "disagrees"):
            prepare(document)

    def test_capacity_exclusion_and_bounded_transport_are_fail_closed(self):
        for extra in ({"splitSizeMiB": 64, "maxSplits": 1, "estimatedVideoBitrateMbps": 24},
                      {"excludedTopics": [{"topic": "/clock", "reason": "excluded"}]},
                      {"worldBoundary": "not-an-object"}, {"unknown": True}):
            document = request()
            document["parameters"].update(copy.deepcopy(extra))
            with self.assertRaises(ValueError):
                prepare(document)


if __name__ == "__main__":
    unittest.main()
