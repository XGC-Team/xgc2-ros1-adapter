import copy
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("projection", Path(__file__).parents[1] / "scripts/project_robot_assets.py")
projection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(projection)

FAKE_ENDPOINTS = ("192.0.2.", "198.51.100.", "20108", "20508", "management_ip", "physical_fcu_url", "simulation_fcu_url")
EXPECTED_MODEL_IDS = {"px4_multirotor": "fs150", "scout_mini": "scout-mini", "mecanum_ugv": "mecanum-ugv"}


def context(mode="simulation", simulator="xsim", offset=None):
    return dict(runMode=mode, simulator=simulator, rosMasterUri="http://127.0.0.1:11311",
                rosIp="127.0.0.1", simulatorHostname="198.51.100.19",
                localizationOffset=offset or dict(x=1, y=-2, z=3))


def model(kind, ordinal):
    return dict(robotKind=kind, modelId=projection.INSTALLED_MODEL_IDS[kind], ordinal=ordinal)


def binding(slot, namespace, dynamics=True, source="simulation"):
    return dict(id=slot, namespace=namespace, hybridSource=source, applyModelDynamics=dynamics)


def scout_asset():
    return dict(kind="scout_mini", profileId="scout-mini.physical.vrpn", scout=dict(
        mocapRigidBodyName="tracked", positioningFrameNumber=5,
        positioningComparisonThresholdM=0.001, managementAddress="192.0.2.8",
        connector="swarm_ros_bridge", telemetryRemotePort=9090, controlLocalPort=9091))


def px4_asset():
    return dict(kind="px4_multirotor", profileId="px4-multirotor.physical.vrpn", px4=dict(
        mocapRigidBodyName="tracked_body", positioningFrameNumber=5,
        positioningComparisonThresholdM=0.001, modelId="fs150", mavSystemId=8,
        managementIp="192.0.2.8", physicalMavrosLocalPort=9018, physicalFcuRemotePort=14560,
        simulationLocalPort=20108, simulationRemotePort=20508))


def mecanum_asset():
    return dict(kind="mecanum_ugv", profileId="mecanum-ugv.physical.vrpn", mecanum=dict(
        mocapRigidBodyName="tracked_mecanum", positioningFrameNumber=5,
        positioningComparisonThresholdM=0.001))


class AssetProjectionTest(unittest.TestCase):
    def test_body_returns_follows_scope_independently_of_sensor_enabled(self):
        local = {"enabled": True, "bodyReturns": False}
        binding_row = {"id": "slot", "simulationSensors": {"simpleLidar": local}}
        for enabled in (False, True):
            for body_returns in (False, True):
                with self.subTest(enabled=enabled, body_returns=body_returns):
                    settings = {"enabled": enabled, "bodyReturns": body_returns}
                    scene = {"parameters": {"lidar": {"scope": "global", "settings": settings}}}
                    projected = projection.effective_binding(binding_row, scene)
                    self.assertEqual(projected["simulationSensors"]["simpleLidar"], settings)
                    scene["parameters"]["lidar"]["scope"] = "individual"
                    self.assertEqual(projection.effective_binding(binding_row, scene), binding_row)
                    self.assertEqual(binding_row["simulationSensors"]["simpleLidar"], local)

    def test_global_lidar_overrides_effective_binding_and_individual_values_survive(self):
        asset = scout_asset()
        local = dict(enabled=False, mode="raycast", acceleration="cpu", rangeMeters=8)
        bindings = [dict(binding("slot1", "/robot1"), simulationSensors=dict(simpleLidar=local)),
                    binding("slot2", "/robot2")]
        global_lidar = dict(enabled=True, mode="penetrating", preset="", acceleration="gpu",
                            rateHz=8, rangeMeters=12, hFovDeg=270, vFovDeg=45,
                            hRes=640, vRes=32, publishBeams=False)
        for simulator in ("gazebo", "xsim"):
            scene_context = context(simulator=simulator)
            scene_context["scene"] = dict(parameters=dict(lidar=dict(scope="global", settings=global_lidar)))
            request = dict(context=scene_context, robots=[
                dict(model=model("scout_mini", index), binding=row, physicalBinding=dict(
                    assetId="scout-asset", assetCommitId="commit", asset=asset))
                for index, row in enumerate(bindings)])
            original = copy.deepcopy(request)
            projected = projection.project_batch(request)
            for index, row in enumerate(projected):
                self.assertEqual(row["binding"], dict(bindings[index], simulationSensors=dict(simpleLidar=global_lidar)))
            self.assertEqual(request, original)
            scene_context["scene"]["parameters"]["lidar"]["scope"] = "individual"
            self.assertEqual([row["binding"] for row in projection.project_batch(request)], bindings)
            scene_context["scene"]["parameters"]["lidar"].update(scope="global", settings=dict(global_lidar, enabled=False))
            self.assertTrue(all(not row["binding"]["simulationSensors"]["simpleLidar"]["enabled"]
                                for row in projection.project_batch(request)))

    def test_authored_parameters_survive_for_shared_profile_resolution(self):
        authored = dict(localization_pose_topic="/measured/custom_pose",
                        localization_twist_topic="/measured/custom_twist")
        row_binding = dict(binding("slot", "/robot7", source="physical"), runtimeParameters=authored)
        request = dict(context=context(mode="physical", simulator="none"), robots=[dict(
            model=model("scout_mini", 0), binding=row_binding,
            physicalBinding=dict(assetId="scout-asset", assetCommitId="commit", asset=scout_asset()))])
        before = copy.deepcopy(request)
        result = projection.project_batch(request)[0]
        self.assertEqual(result["binding"]["runtimeParameters"], authored)
        self.assertEqual(result["parameters"]["localization_pose_topic"], "/vrpn_client_node/tracked/pose")
        self.assertEqual(result["parameters"]["localization_twist_topic"], "/vrpn_client_node/tracked/twist")
        self.assertEqual(request, before)
        self.assertIsNot(result["binding"]["runtimeParameters"], authored)

    def test_xsim_three_kinds_have_no_endpoints(self):
        robots = []
        for ordinal, kind in enumerate(("px4_multirotor", "scout_mini", "mecanum_ugv")):
            robots.append(dict(model=model(kind, ordinal), binding=binding("slot-%d" % ordinal, "/robot%d" % (ordinal + 1))))
        request = dict(context=context(), robots=robots)
        before = copy.deepcopy(request)
        projected = projection.project_batch(request)
        self.assertEqual(request, before)
        prefixes = {"px4_multirotor": "px4-multirotor", "scout_mini": "scout-mini", "mecanum_ugv": "mecanum-ugv"}
        for ordinal, row in enumerate(projected):
            kind = robots[ordinal]["model"]["robotKind"]
            self.assertEqual(row["kind"], kind)
            self.assertEqual(row["derived"]["modelId"], EXPECTED_MODEL_IDS[kind])
            self.assertNotIn("model", row)
            self.assertNotIn("physicalBinding", row)
            self.assertNotIn("sitlInstanceId", row["derived"])
            self.assertNotIn("simulationFcuUrl", row["derived"])
            self.assertEqual(row["profileId"], prefixes[kind] + ".xsim.ros")
            self.assertEqual(row["parameters"]["localization_pose_topic"], "/robot%d/pose" % (ordinal + 1))
            self.assertEqual(row["parameters"]["positioning_frame_number"], "5")
            self.assertEqual(row["parameters"]["positioning_comparison_threshold_m"], "1e-10")
            for key in FAKE_ENDPOINTS:
                self.assertNotIn(key, row["parameters"])
            self.assertNotIn("198.51.100.19", "".join(row["parameters"].values()))

    def test_xsim_batches_of_20_and_100_stay_local_topics_only(self):
        for count in (20, 100):
            kinds = ("px4_multirotor", "scout_mini", "mecanum_ugv")
            robots = [dict(model=model(kinds[i % 3], i), binding=binding("slot-%03d" % i, "/robot%d" % (i + 1)))
                      for i in range(count)]
            projected = projection.project_batch(dict(context=context(), robots=robots))
            self.assertEqual(len(projected), count)
            topics = set()
            for i, row in enumerate(projected):
                self.assertEqual(row["kind"], kinds[i % 3])
                self.assertEqual(row["derived"]["modelId"], EXPECTED_MODEL_IDS[kinds[i % 3]])
                self.assertNotIn("model", row)
                self.assertNotIn("sitlInstanceId", row["derived"])
                self.assertNotIn("simulationFcuUrl", row["derived"])
                topic = row["parameters"]["localization_pose_topic"]
                self.assertEqual(topic, "/robot%d/pose" % (i + 1))
                topics.add(topic)
                blob = " ".join(list(row["parameters"].values()) + [str(value) for value in row["derived"].values() if not isinstance(value, dict)])
                for key in ("management_ip", "ssh", "physical_fcu_url", "simulation_fcu_url", "management_address", "simulationFcuUrl"):
                    self.assertNotIn(key, row["parameters"])
                    self.assertNotIn(key, row["derived"])
                self.assertNotIn("192.0.2.", blob)
                self.assertNotIn("198.51.100.", blob)
                self.assertNotIn("udp://", blob)
            self.assertEqual(len(topics), count)

    def test_none_models_use_ros_observation_without_fcu(self):
        prefixes = {"px4_multirotor": "px4-multirotor", "scout_mini": "scout-mini", "mecanum_ugv": "mecanum-ugv"}
        for mode in ("simulation", "hybrid"):
            for ordinal, kind in enumerate(prefixes):
                for dynamics in (False, True):
                    with self.subTest(mode=mode, kind=kind, dynamics=dynamics):
                        request = dict(context=context(mode=mode, simulator="none"), robots=[dict(
                            model=model(kind, ordinal),
                            binding=binding("slot-%d" % ordinal, "/robot%d" % (ordinal + 1), dynamics=dynamics))])
                        before = copy.deepcopy(request)
                        row = projection.project_batch(request)[0]
                        self.assertEqual(request, before)
                        self.assertEqual(row["profileId"], prefixes[kind] + ".xsim.ros")
                        self.assertEqual(row["derived"]["modelId"], EXPECTED_MODEL_IDS[kind])
                        self.assertNotIn("model", row)
                        self.assertNotIn("physicalBinding", row)
                        self.assertNotIn("sitlInstanceId", row["derived"])
                        self.assertEqual(row["parameters"]["localization_pose_topic"], "/robot%d/pose" % (ordinal + 1))
                        self.assertEqual(row["parameters"]["localization_twist_topic"], "/robot%d/twist" % (ordinal + 1))
                        for axis in ("x", "y", "z"):
                            self.assertEqual(row["parameters"]["localization_offset_" + axis], "0")
                        for port in ("mavlink_tcp_port", "mavlink_udp_port", "qgc_udp_port", "sdk_udp_port"):
                            self.assertNotIn(port, row["parameters"])
                        for key in ("simulationFcuUrl", "mavSystemId", "mav_system_id", *FAKE_ENDPOINTS):
                            self.assertNotIn(key, row["parameters"])
                            self.assertNotIn(key, row["derived"])

    def test_none_asset_session_uses_ros_profile_without_mutating_inventory(self):
        for factory in (px4_asset, scout_asset, mecanum_asset):
            for mode in ("simulation", "hybrid"):
                with self.subTest(kind=factory()["kind"], mode=mode):
                    asset = factory()
                    kind = asset["kind"]
                    request = dict(context=context(mode=mode, simulator="none"), robots=[dict(
                        model=model(kind, 4), binding=binding("slot", "/robot7", dynamics=False),
                        physicalBinding=dict(assetId="real-asset", assetCommitId="saved-commit", asset=asset))])
                    before = copy.deepcopy(request)
                    row = projection.project_batch(request)[0]
                    self.assertEqual(request, before)
                    self.assertEqual(asset, before["robots"][0]["physicalBinding"]["asset"])
                    self.assertEqual(row["physicalBinding"], before["robots"][0]["physicalBinding"])
                    self.assertIsNot(row["physicalBinding"]["asset"], asset)
                    self.assertEqual(row["profileId"], asset["profileId"].replace(".physical.vrpn", ".xsim.ros"))
                    self.assertEqual(row["physicalBinding"]["asset"]["profileId"], asset["profileId"])
                    self.assertEqual(row["parameters"]["localization_pose_topic"], "/robot7/pose")
                    self.assertEqual(row["parameters"]["localization_twist_topic"], "/robot7/twist")
                    self.assertNotIn("sitlInstanceId", row["derived"])
                    for port in ("mavlink_tcp_port", "mavlink_udp_port", "qgc_udp_port", "sdk_udp_port"):
                        self.assertNotIn(port, row["parameters"])

    def test_physical_sources_keep_vrpn_profiles_and_observation(self):
        for factory in (px4_asset, scout_asset, mecanum_asset):
            for mode in ("physical", "hybrid"):
                for simulator in ("none", "xsim", "gazebo"):
                    with self.subTest(kind=factory()["kind"], mode=mode, simulator=simulator):
                        asset = factory()
                        kind = asset["kind"]
                        request = dict(context=context(mode=mode, simulator=simulator), robots=[dict(
                            model=model(kind, 4), binding=binding("slot", "/robot7", source="physical"),
                            physicalBinding=dict(assetId="real-asset", assetCommitId="saved-commit", asset=asset))])
                        before = copy.deepcopy(request)
                        row = projection.project_batch(request)[0]
                        arm = projection.MODELS[kind][0]
                        root = "/vrpn_client_node" if mode == "physical" else "/vrpn_client_node_physical"
                        topic = root + "/" + asset[arm]["mocapRigidBodyName"]
                        self.assertEqual(request, before)
                        self.assertEqual(row["profileId"], asset["profileId"])
                        self.assertEqual(row["physicalBinding"], request["robots"][0]["physicalBinding"])
                        self.assertEqual(row["parameters"]["localization_pose_topic"], topic + "/pose")
                        self.assertEqual(row["parameters"]["localization_twist_topic"], topic + "/twist")
                        self.assertNotIn("sitlInstanceId", row["derived"])
                        self.assertEqual(row["parameters"]["localization_offset_y"], "-2")

    def test_gazebo_derives_local_sitl_endpoints_from_ordinal(self):
        robots = [dict(model=model("px4_multirotor", ordinal),
                       binding=binding("uav-%03d" % ordinal, "/uav%d" % (ordinal + 1)))
                  for ordinal in range(20)]
        projected = projection.project_batch(dict(context=context(simulator="gazebo"), robots=robots))
        urls = set()
        for ordinal, row in enumerate(projected):
            self.assertEqual(row["kind"], "px4_multirotor")
            self.assertNotIn("model", row)
            self.assertEqual(row["profileId"], "px4-multirotor.gazebo.vrpn")
            self.assertEqual(row["derived"]["modelId"], "fs150")
            self.assertEqual(row["derived"]["sitlInstanceId"], ordinal)
            self.assertNotIn("mavSystemId", row["derived"])
            self.assertNotIn("simulationFcuUrl", row["derived"])
            self.assertEqual(row["parameters"]["mavlink_tcp_port"], str(4560 + ordinal))
            self.assertEqual(row["parameters"]["mavlink_udp_port"], str(14560 + ordinal))
            self.assertEqual(row["parameters"]["qgc_udp_port"], "14550")
            self.assertEqual(row["parameters"]["sdk_udp_port"], str(15000 + ordinal))
            for key in ("management_ip", "ssh", "physical_fcu_url", "simulation_fcu_url"):
                self.assertNotIn(key, row["parameters"])
                self.assertNotIn(key, row["derived"])
            urls.add(row["parameters"]["sdk_udp_port"])
        self.assertEqual(len(urls), 20)
        edge = projection.project(dict(model=model("px4_multirotor", 243), binding=binding("edge", "/edge")),
                                  context(simulator="gazebo"))
        self.assertEqual(edge["parameters"]["mavlink_tcp_port"], "4803")
        self.assertEqual(edge["parameters"]["sdk_udp_port"], "15243")
        self.assertEqual(edge["parameters"]["qgc_udp_port"], "14550")
        with self.assertRaises(ValueError):
            projection.project(dict(model=model("px4_multirotor", 244), binding=binding("over", "/over")),
                               context(simulator="gazebo"))

    def test_device_attachment_keeps_physical_profile_and_gazebo_replaces_authored_sitl(self):
        asset = px4_asset()
        row_binding = binding("slot", "/robot7", source="physical")
        physical = projection.project(dict(
            model=model("px4_multirotor", 4), binding=row_binding,
            physicalBinding=dict(assetId="px4-asset", assetCommitId="commit-1", asset=asset)),
            context(mode="physical", simulator="none"))
        self.assertEqual(physical["kind"], "px4_multirotor")
        self.assertEqual(physical["derived"]["modelId"], "fs150")
        self.assertNotIn("model", physical)
        self.assertEqual(physical["physicalBinding"]["assetId"], "px4-asset")
        self.assertEqual(physical["profileId"], "px4-multirotor.physical.vrpn")
        self.assertNotIn("sitlInstanceId", physical["derived"])
        self.assertEqual(physical["parameters"]["mocap_rigid_body"], "tracked_body")
        for key in ("management_ip", "ssh", "physical_fcu_url", "simulation_fcu_url", "mav_system_id"):
            self.assertNotIn(key, physical["parameters"])
        self.assertEqual(physical["parameters"]["localization_offset_y"], "-2")
        gazebo_binding = binding("slot", "/robot7")
        gazebo = projection.project(dict(
            model=model("px4_multirotor", 6), binding=gazebo_binding,
            physicalBinding=dict(assetId="px4-asset", assetCommitId="commit-1", asset=copy.deepcopy(asset))),
            context(simulator="gazebo"))
        self.assertEqual(gazebo["kind"], "px4_multirotor")
        self.assertEqual(gazebo["derived"]["modelId"], "fs150")
        self.assertNotIn("model", gazebo)
        self.assertEqual(gazebo["profileId"], "px4-multirotor.gazebo.vrpn")
        for key in ("management_ip", "ssh", "physical_fcu_url"):
            self.assertNotIn(key, gazebo["parameters"])
        self.assertEqual(gazebo["derived"]["sitlInstanceId"], 6)
        self.assertNotIn("mavSystemId", gazebo["derived"])
        self.assertEqual(gazebo["parameters"]["mavlink_tcp_port"], "4566")
        self.assertEqual(gazebo["parameters"]["mavlink_udp_port"], "14566")
        self.assertEqual(gazebo["parameters"]["qgc_udp_port"], "14550")
        self.assertEqual(gazebo["parameters"]["sdk_udp_port"], "15006")
        self.assertNotIn("simulation_fcu_url", gazebo["parameters"])
        self.assertNotIn("20108", " ".join(gazebo["parameters"].values()))

    def test_perfect_tracker_emits_no_fcu(self):
        for kind in ("px4_multirotor", "scout_mini", "mecanum_ugv"):
            row = projection.project(dict(model=model(kind, 3), binding=binding("slot", "/robot3", dynamics=False)),
                                     context(simulator="gazebo"))
            self.assertEqual(row["kind"], kind)
            self.assertEqual(row["derived"]["modelId"], EXPECTED_MODEL_IDS[kind])
            self.assertNotIn("model", row)
            self.assertNotIn("sitlInstanceId", row["derived"])
            for port in ("mavlink_tcp_port", "mavlink_udp_port", "qgc_udp_port", "sdk_udp_port"):
                self.assertNotIn(port, row["parameters"])
            self.assertNotIn("simulationFcuUrl", row["derived"])
            for key in ("physical_fcu_url", "simulation_fcu_url", "management_ip", "ssh"):
                self.assertNotIn(key, row["parameters"])
                self.assertNotIn(key, row["derived"])
        xsim = projection.project(dict(model=model("px4_multirotor", 3), binding=binding("slot", "/robot3", dynamics=False)),
                                  context())
        self.assertNotIn("sitlInstanceId", xsim["derived"])
        for key in ("management_ip", "ssh", "physical_fcu_url", "simulation_fcu_url", "mavlink_tcp_port"):
            self.assertNotIn(key, xsim["parameters"])
        with self.assertRaises(ValueError):
            projection.project(dict(asset=px4_asset(), binding=binding("slot", "/robot7")), context())
        missing = dict(model=model("px4_multirotor", 1), binding=dict(id="slot", namespace="/robot1", hybridSource="simulation"))
        with self.assertRaises(ValueError):
            projection.project(missing, context(simulator="gazebo"))
        self.assertNotIn("applyModelDynamics", missing["binding"])


if __name__ == "__main__":
    unittest.main()
