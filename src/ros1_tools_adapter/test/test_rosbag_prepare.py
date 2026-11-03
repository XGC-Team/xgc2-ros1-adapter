"""Recorder preparation cases moved from the retired Go domain compiler."""
import copy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
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

    def test_exact_slot_topics_preserve_aliases_and_do_not_multiply_other_members(self):
        document = request()
        second = copy.deepcopy(document["robots"]["robots"][0])
        second.update(id="uav-2", namespace="/uav2")
        document["robots"]["robots"].append(second)
        document["parameters"] = {"slotIds": ["uav-1"], "globalTopics": [], "topicsBySlot": {
            "uav-1": ["/xgc/act1/uav1/odom", "/command"],
            "uav-2": ["/xgc/act1/uav2/odom", "/command"]}}
        self.assertEqual(prepare(document)[1], {"slotIds": ["uav-1"], "topics": ["/command", "/xgc/act1/uav1/odom"]})
        document["parameters"]["slotIds"] = ["uav-2"]
        self.assertEqual(prepare(document)[1]["topics"], ["/command", "/xgc/act1/uav2/odom"])
        del document["parameters"]["topicsBySlot"]["uav-2"]
        with self.assertRaisesRegex(ValueError, "lack explicit recording topics"):
            prepare(document)
        document["parameters"]["topicsBySlot"]["foreign"] = ["/foreign/pose"]
        with self.assertRaisesRegex(ValueError, "outside the frozen"):
            prepare(document)

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
        for parameters in ({}, {"localization_twist_topic": "/twist"},
                           {"localization_pose_topic": "relative", "localization_twist_topic": "/twist"}):
            document = request()
            document["parameters"]["includeMotionCapture"] = True
            document["robots"]["robots"][0]["runtimeParameters"] = parameters
            with self.subTest(parameters=parameters), self.assertRaisesRegex(ValueError, "explicit frozen localization pose topic"):
                prepare(document)
        document = request()
        document["parameters"] = {"globalTopics": [], "includeMotionCapture": True}
        document["robots"]["robots"][0]["runtimeParameters"] = {"localization_pose_topic": "/pose"}
        self.assertEqual(prepare(document)[1]["topics"], ["/pose"])
        document["robots"]["robots"][0]["runtimeParameters"]["localization_twist_topic"] = "relative"
        with self.assertRaisesRegex(ValueError, "invalid frozen localization twist topic"):
            prepare(document)

    def test_pose_only_slot_output_and_effective_tracker_reach_the_manifest(self):
        document = request()
        document["parameters"] = {"globalTopics": [], "includeMotionCapture": True}
        document["robots"]["robots"][0].update(
            runtimeParameters={"localization_pose_topic": "/uav1/pose",
                               "mocap_rigid_body": "slot-uav1"},
            px4={"mocapRigidBodyName": "physical-body"})
        parameters, resolved, manifest = prepare(document)
        self.assertEqual(resolved["topics"], ["/uav1/pose"])
        self.assertEqual(manifest["robots"][0]["mocapRigidBodyName"], "slot-uav1")
        self.assertNotIn("localization_twist_topic", parameters)

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

    def test_member_endpoint_is_target_scoped_without_rewriting_core_simulation(self):
        robot = {"source": "simulation", "slotId": "uav1", "namespace": "/uav1", "kind": "px4-multirotor",
                 "profile": {"id": "p", "parameters": {}}, "binding": {"initialPose": {}},
                 "adapter": {"targetId": "agent-a"}}
        session = {"robots": [robot], "targetId": "agent-a", "experimentResourceId": "exp",
                   "experimentCommitId": "commit", "simulation": {"rosMasterUri": "http://core:11311", "rosIp": "core"},
                   "recordingEndpoint": {"targetId": "agent-a", "rosMasterUri": "http://member:11311", "rosIp": "member"}}
        document = {"parameters": {"slotIds": ["uav1"], "rosMasterUri": "http://member:11311", "rosIp": "member"}, "context": session}
        def read():
            with patch.object(sys, "stdin", types.SimpleNamespace(buffer=io.BytesIO(json.dumps(document).encode()))):
                return namespace["read_record_request"]()
        result = read()
        self.assertEqual(result["context"]["rosMasterUri"], "http://member:11311")
        self.assertEqual(result["context"]["slotId"], "uav1")
        self.assertEqual(result["context"]["simulation"]["rosMasterUri"], "http://core:11311")
        namespace["normalize_parameters"](result["parameters"], result["context"], False)
        document["parameters"]["rosMasterUri"] = "http://forged:11311"
        with self.assertRaisesRegex(ValueError, "disagrees"):
            result = read()
            namespace["normalize_parameters"](result["parameters"], result["context"], False)
        for change in ("target", "slot", "adapter"):
            bad = copy.deepcopy(document)
            if change == "target": bad["context"]["recordingEndpoint"]["targetId"] = "other"
            if change == "slot": bad["parameters"]["slotIds"] = ["absent"]
            if change == "adapter": bad["context"]["robots"][0]["adapter"]["targetId"] = "other"
            with patch.object(sys, "stdin", types.SimpleNamespace(buffer=io.BytesIO(json.dumps(bad).encode()))):
                with self.assertRaisesRegex(ValueError, "target"):
                    namespace["read_record_request"]()

    def test_frozen_robots_are_model_backed_with_optional_physical_binding(self):
        def frozen(**extra):
            return dict({"source": "simulation", "slotId": "uav1", "namespace": "/uav1", "kind": "px4-multirotor",
                         "profile": {"id": "p", "parameters": {}},
                         "binding": {"slotDeployment": {"resource": {"kind": "asset-agent"}, "plant": "simulation"}, "initialPose": {}},
                         "derived": {"modelId": "fs150", "visualization": {}}}, **extra)
        def read(robot):
            session = {"parameters": {}, "context": {"robots": [robot], "experimentResourceId": "experiment-a",
                       "experimentCommitId": "commit-a", "simulation": {"rosMasterUri": "http://master.test:27181", "rosIp": ""}}}
            with patch.object(sys, "stdin", types.SimpleNamespace(buffer=io.BytesIO(json.dumps(session).encode()))):
                return namespace["read_record_request"]()["robots"]["robots"][0]
        model = read(frozen())
        self.assertEqual((model["modelId"], model["robotAssetId"], model["robotAssetCommitId"]), ("fs150", None, None))
        self.assertEqual(model["hybridSource"], "simulation")
        real = frozen(source="physical")
        real["binding"]["hybridSource"] = "simulation"  # Non-onboard declaration does not override the frozen source.
        self.assertEqual(read(real)["hybridSource"], "physical")
        device = read(frozen(physicalBinding={"assetId": "asset-1", "assetCommitId": "commit-1",
                                              "asset": {"px4": {"mocapRigidBodyName": "Body1"}}}))
        self.assertEqual((device["robotAssetId"], device["robotAssetCommitId"], device["px4"]),
                         ("asset-1", "commit-1", {"mocapRigidBodyName": "Body1"}))
        extension = read(frozen(physicalBinding={"assetId": "asset-2", "assetCommitId": "commit-2",
                                                 "asset": {"extension": {"kind": "custom"}}}))
        self.assertEqual(extension["extension"], {"kind": "custom"})

    def test_resolved_topic_bounds_admit_a_hundred_robot_roster_and_stay_closed(self):
        document = request()
        document["parameters"]["globalTopics"] = ["/g%d" % index for index in range(1024)]
        self.assertEqual(len(prepare(document)[1]["topics"]), 1024)
        document["parameters"]["globalTopics"].append("/g1024")
        with self.assertRaisesRegex(ValueError, "globalTopics"):
            prepare(document)
        document = request()
        document["parameters"]["robotTopics"] = ["t%d" % index for index in range(256)]
        document["robots"]["robots"] = [dict(document["robots"]["robots"][0], id="uav-%d" % index, namespace="/uav%d" % index)
                                         for index in range(9)]
        with self.assertRaisesRegex(ValueError, "between 1 and 2048"):
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

    def test_real_parent_segment_sidecar_paths_are_read_not_shape_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "ros1_ws").mkdir()
            install = root / "ws_impc_dr/install"
            install.mkdir(parents=True)
            (install / "build-inputs.yaml").write_text("build: {}\n")
            (install / "calibration.yml").write_text("calibration: {}\n")
            algorithm_path = str(root / "ros1_ws" / ".." / "ws_impc_dr/install/build-inputs.yaml")
            calibration_path = str(root / "ros1_ws" / ".." / "ws_impc_dr/install/calibration.yml")
            for path in (algorithm_path, calibration_path):
                self.assertNotEqual(os.path.normpath(path), path)
                self.assertTrue(os.path.isfile(path))
            document = request()
            document["parameters"] = {
                "algorithmSidecarFiles": [{"role": "build-inputs", "path": algorithm_path}],
                "calibrationFiles": [{"role": "camera-calibration", "path": calibration_path}]}
            parameters, _, manifest = prepare(document)
            self.assertEqual(parameters["algorithmSidecarFiles"][0]["path"], algorithm_path)
            self.assertEqual(parameters["calibrationFiles"][0]["path"], calibration_path)
            self.assertEqual(manifest["factSources"]["algorithmSidecarFiles"], parameters["algorithmSidecarFiles"])
            self.assertEqual(manifest["factSources"]["calibrationFiles"], parameters["calibrationFiles"])
            self.assertEqual(manifest["requestedParameters"]["algorithmSidecarFiles"][0]["path"], algorithm_path)

    def test_sidecar_path_safety_keeps_abs_null_extension_and_unique_role(self):
        for field, path in (("algorithmSidecarFiles", "relative/build-inputs.yaml"),
                            ("algorithmSidecarFiles", "/tmp/build-inputs.yaml\x00"),
                            ("algorithmSidecarFiles", "/tmp/build-inputs.txt"),
                            ("calibrationFiles", "/tmp/calibration.json")):
            document = request()
            document["parameters"] = {field: [{"role": "sidecar", "path": path}]}
            with self.subTest(field=field, path=path), self.assertRaisesRegex(ValueError, "invalid or duplicate sidecar"):
                prepare(document)
        document = request()
        document["parameters"] = {"algorithmSidecarFiles": [
            {"role": "same", "path": "/tmp/a.yaml"}, {"role": "same", "path": "/tmp/b.yaml"}]}
        with self.assertRaisesRegex(ValueError, "invalid or duplicate sidecar"):
            prepare(document)
        document = request()
        document["parameters"] = {"rosInstallPath": "/opt/ros/../noetic"}
        with self.assertRaisesRegex(ValueError, "rosInstallPath must be a clean absolute path"):
            prepare(document)


if __name__ == "__main__":
    unittest.main()
