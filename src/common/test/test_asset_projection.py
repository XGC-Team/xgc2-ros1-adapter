import copy
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("projection", Path(__file__).parents[1] / "scripts/project_robot_assets.py")
projection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(projection)


class AssetProjectionTest(unittest.TestCase):
    def test_body_returns_follows_scope_independently_of_sensor_enabled(self):
        local = {"enabled": True, "bodyReturns": False}
        binding = {"id": "slot", "simulationSensors": {"simpleLidar": local}}
        for enabled in (False, True):
            for body_returns in (False, True):
                with self.subTest(enabled=enabled, body_returns=body_returns):
                    settings = {"enabled": enabled, "bodyReturns": body_returns}
                    scene = {"parameters": {"lidar": {"scope": "global", "settings": settings}}}
                    projected = projection.effective_binding(binding, scene)
                    self.assertEqual(projected["simulationSensors"]["simpleLidar"], settings)
                    scene["parameters"]["lidar"]["scope"] = "individual"
                    self.assertEqual(projection.effective_binding(binding, scene), binding)
                    self.assertEqual(binding["simulationSensors"]["simpleLidar"], local)

    def test_global_lidar_overrides_effective_binding_and_individual_values_survive(self):
        asset = dict(kind="scout_mini", profileId="scout-mini.physical.vrpn", scout=dict(
            mocapRigidBodyName="tracked", positioningFrameNumber=5,
            positioningComparisonThresholdM=0.001, managementAddress="192.0.2.8",
            connector="swarm_ros_bridge", telemetryRemotePort=9090, controlLocalPort=9091))
        local = dict(enabled=False, mode="raycast", acceleration="cpu", rangeMeters=8)
        bindings = [dict(id="slot1", namespace="/robot1", hybridSource="simulation",
                         simulationSensors=dict(simpleLidar=local)),
                    dict(id="slot2", namespace="/robot2", hybridSource="simulation")]
        global_lidar = dict(enabled=True, mode="penetrating", preset="", acceleration="gpu",
                            rateHz=8, rangeMeters=12, hFovDeg=270, vFovDeg=45,
                            hRes=640, vRes=32, publishBeams=False)
        for simulator in ("gazebo", "xsim"):
            context = dict(runMode="simulation", simulator=simulator, rosMasterUri="http://localhost:11311",
                           rosIp="127.0.0.1", simulatorHostname="198.51.100.19", localizationOffset=dict(x=0, y=0, z=0),
                           scene=dict(parameters=dict(lidar=dict(scope="global", settings=global_lidar))))
            request = dict(context=context, robots=[dict(asset=asset, binding=b) for b in bindings])
            original = copy.deepcopy(request)
            projected = projection.project_batch(request)
            for index, row in enumerate(projected):
                self.assertEqual(row["binding"], dict(bindings[index], simulationSensors=dict(simpleLidar=global_lidar)))
            self.assertEqual(request, original)
            context["scene"]["parameters"]["lidar"]["scope"] = "individual"
            self.assertEqual([row["binding"] for row in projection.project_batch(request)], bindings)
            context["scene"]["parameters"]["lidar"].update(scope="global", settings=dict(global_lidar, enabled=False))
            self.assertTrue(all(not row["binding"]["simulationSensors"]["simpleLidar"]["enabled"]
                                for row in projection.project_batch(request)))

    def test_authored_parameters_survive_for_shared_profile_resolution(self):
        asset = dict(kind="scout_mini", profileId="scout-mini.physical.vrpn", scout=dict(
            mocapRigidBodyName="tracked", positioningFrameNumber=5,
            positioningComparisonThresholdM=0.001, managementAddress="192.0.2.8",
            connector="swarm_ros_bridge", telemetryRemotePort=9090, controlLocalPort=9091))
        authored = dict(localization_pose_topic="/measured/custom_pose",
                        localization_twist_topic="/measured/custom_twist")
        binding = dict(id="slot", namespace="/robot7", hybridSource="physical",
                       runtimeParameters=authored)
        context = dict(runMode="physical", simulator="none", rosMasterUri="http://localhost:11311",
                       rosIp="127.0.0.1", simulatorHostname="198.51.100.19",
                       localizationOffset=dict(x=0, y=0, z=0))
        request = dict(context=context, robots=[dict(asset=asset, binding=binding)])
        before = copy.deepcopy(request)
        result = projection.project_batch(request)[0]
        # Product injection and authored values both reach the generic profile
        # resolver, which owns declaration validation and override precedence.
        self.assertEqual(result["binding"]["runtimeParameters"], authored)
        self.assertEqual(result["parameters"]["localization_pose_topic"], "/vrpn_client_node/tracked/pose")
        self.assertEqual(result["parameters"]["localization_twist_topic"], "/vrpn_client_node/tracked/twist")
        self.assertEqual(request, before)
        self.assertIsNot(result["binding"]["runtimeParameters"], authored)

    def test_profiles_offsets_and_transport_for_all_sources(self):
        for kind, (arm, prefix, *_) in projection.MODELS.items():
            body = dict(mocapRigidBodyName="tracked_body", positioningFrameNumber=5,
                        positioningComparisonThresholdM=0.001, modelId="fs150", mavSystemId=8,
                        managementIp="192.0.2.8", physicalMavrosLocalPort=9018, physicalFcuRemotePort=14560,
                        simulationLocalPort=20108, simulationRemotePort=20508,
                        managementAddress="192.0.2.8", connector="swarm_ros_bridge",
                        telemetryRemotePort=9090, controlLocalPort=9091)
            asset = dict(kind=kind, profileId=prefix + ".physical.vrpn", **{arm: body})
            before = copy.deepcopy(asset)
            for mode in ("simulation", "physical", "hybrid"):
                for backend in ("gazebo", "xsim"):
                    for source in ("physical", "simulation"):
                        context = dict(runMode=mode, simulator=backend, rosMasterUri="http://localhost:11311",
                                       rosIp="127.0.0.1", simulatorHostname="198.51.100.19", localizationOffset=dict(x=1, y=-2, z=3))
                        row = dict(asset=asset, binding=dict(id="slot", namespace="/robot7", hybridSource=source))
                        result = projection.project_batch(dict(context=context, robots=[row]))[0]
                        simulated = mode == "simulation" or mode == "hybrid" and source == "simulation"
                        self.assertEqual(result["profileId"], prefix + ("." + backend + (".ros" if backend == "xsim" else ".vrpn") if simulated else ".physical.vrpn"))
                        self.assertEqual(result["parameters"]["mocap_rigid_body"], "robot7" if simulated else "tracked_body")
                        parameters = result["parameters"]
                        root = parameters["mocap_source_root"] + '/' + parameters["mocap_rigid_body"]
                        self.assertEqual(parameters["localization_pose_topic"], "/robot7/pose" if simulated and backend == "xsim" else root + '/pose')
                        self.assertEqual(parameters["localization_twist_topic"], "/robot7/twist" if simulated and backend == "xsim" else root + '/twist')
                        self.assertNotIn("physical_gcs_url", parameters)
                        self.assertEqual(result["parameters"]["localization_offset_y"], "0" if simulated else "-2")
                        if arm == "px4":
                            self.assertEqual(result["derived"]["sitlInstanceId"], 7)
                            self.assertEqual(result["derived"]["simulationFcuUrl"], "udp://:20108@198.51.100.19:20508")
            self.assertEqual(asset, before)


if __name__ == "__main__":
    unittest.main()
