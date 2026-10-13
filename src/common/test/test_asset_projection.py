import copy
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("projection", Path(__file__).parents[1] / "scripts/project_robot_assets.py")
projection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(projection)


class AssetProjectionTest(unittest.TestCase):
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
                                       rosIp="127.0.0.1", localizationOffset=dict(x=1, y=-2, z=3))
                        row = dict(asset=asset, binding=dict(id="slot", namespace="/robot7", hybridSource=source))
                        result = projection.project_batch(dict(context=context, robots=[row]))[0]
                        simulated = mode == "simulation" or mode == "hybrid" and source == "simulation"
                        self.assertEqual(result["profileId"], prefix + ("." + backend + (".ros" if backend == "xsim" else ".vrpn") if simulated else ".physical.vrpn"))
                        self.assertEqual(result["parameters"]["mocap_rigid_body"], "robot7" if simulated else "tracked_body")
                        self.assertEqual(result["parameters"]["localization_offset_y"], "0" if simulated else "-2")
                        if arm == "px4":
                            self.assertEqual(result["derived"]["sitlInstanceId"], 7)
                            self.assertEqual(result["derived"]["simulationFcuUrl"], "udp://:20108@127.0.0.1:20508")
            self.assertEqual(asset, before)


if __name__ == "__main__":
    unittest.main()
