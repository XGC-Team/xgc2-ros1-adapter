"""Recorder preparation cases moved from the retired Go domain compiler."""
import copy
from pathlib import Path
import unittest

script = (Path(__file__).resolve().parents[1] / "scripts/rosbag_recorder.py").read_text()
namespace = {}
exec(compile(script[:script.index('try:\n    if sys.argv[1:] == ["--prepare"]:')], "native-recorder-prepare", "exec"), namespace)
prepare = namespace["prepare"]


def request():
    return {"parameters": {}, "robots": {"experimentResourceId": "experiment-a", "experimentCommitId": "commit-a", "experimentDigest": "a" * 64, "robotSelectionDigest": "b" * 64,
            "robots": [{"id": "uav-1", "namespace": "/uav1", "profileId": "px4-multirotor.xsim.ros", "kind": "px4-multirotor", "px4": {"mocapRigidBodyName": "Body1"}, "visualization": {}}]},
            "context": {"experimentName": "Frozen experiment", "runMode": "simulation", "scene": {"simulator": "xsim"}}}


class PreparationTests(unittest.TestCase):
    def test_defaults_and_topics_keep_the_frozen_roster(self):
        document = request()
        parameters, resolved, manifest = prepare(document)
        self.assertEqual(resolved, {"slotIds": ["uav-1"], "topics": ["/clock", "/tf", "/tf_static"]})
        self.assertEqual((parameters["splitSizeMiB"], parameters["maxSplits"], parameters["compression"]), (1024, 10, "lz4"))
        self.assertEqual(manifest["experimentResourceId"], "experiment-a")
        self.assertNotIn("localizationOffset", manifest)
        self.assertEqual(document, request())

    def test_native_measurement_projection_uses_explicit_public_topics(self):
        document = request()
        document["parameters"] = {"globalTopics": [], "includeMotionCapture": True}
        document["robots"]["robots"][0].update(simulationPoseTopic="/truth/custom_pose", simulationTwistTopic="/truth/custom_twist")
        self.assertEqual(prepare(document)[1]["topics"], ["/truth/custom_pose", "/truth/custom_twist"])
        document["context"]["runMode"] = "physical"
        self.assertEqual(prepare(document)[1]["topics"], ["/vrpn_client_node/Body1/pose", "/vrpn_client_node/Body1/twist"])
        document["parameters"]["slotIds"] = ["absent"]
        with self.assertRaisesRegex(ValueError, "outside"):
            prepare(document)

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
