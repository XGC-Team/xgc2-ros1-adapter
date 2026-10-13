#!/usr/bin/env python3
"""Project a frozen robot batch into the installed ROS1 profile contract.

This finite, side-effect-free native entry point owns robot transport, source
selection and model facts. Core supplies pinned documents and freezes the result.
"""
import json
import sys


MODELS = {
    "px4_multirotor": ("px4", "px4-multirotor", "fs150", "fs150_description", "fs150_visual.urdf",
                       [("base_link", "mesh"), *[("rotor_" + str(i), "mesh") for i in range(4)]]),
    "scout_mini": ("scout", "scout-mini", "scout", "scout_description", "scout_visual.urdf",
                   [(x, "mesh") for x in ("base_link", "box_link", "front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link", "rear_right_wheel_link")]),
    "mecanum_ugv": ("mecanum", "mecanum-ugv", "mecanum", "mecanum_description", "mecanum_visual.urdf",
                    [("base_link", "mesh"), ("base_link", "box"), *[(x, "mesh") for x in ("upper_left_wheel", "lower_left_wheel", "upper_right_wheel", "lower_right_wheel", "upper_left_wheel_shaft", "upper_right_wheel_shaft", "lower_left_wheel_shaft", "lower_right_wheel_shaft", "front_sensor", "left_sensor", "right_sensor", "rear_sensor")]]),
}


def scalar(value):
    return format(value, ".15g") if isinstance(value, float) else str(value)


def project(row, context):
    asset, binding = row["asset"], row["binding"]
    arm, prefix, scene, package, description, visuals = MODELS[asset["kind"]]
    spec = asset[arm]
    params = {"mocap_rigid_body": spec["mocapRigidBodyName"],
              "positioning_frame_number": scalar(spec["positioningFrameNumber"]),
              "positioning_comparison_threshold_m": scalar(spec["positioningComparisonThresholdM"])}
    derived = {}
    if arm == "px4":
        if spec["modelId"] != "fs150":
            raise ValueError("Unsupported PX4 model: " + spec["modelId"])
        mapping = {"model_id": "modelId", "mav_system_id": "mavSystemId", "management_ip": "managementIp",
                   "physical_mavros_local_port": "physicalMavrosLocalPort", "physical_fcu_remote_port": "physicalFcuRemotePort",
                   "simulation_local_port": "simulationLocalPort", "simulation_remote_port": "simulationRemotePort"}
        params.update({key: scalar(spec[field]) for key, field in mapping.items()})
        params["physical_fcu_url"] = "udp://:%s@%s:%s" % (spec["physicalMavrosLocalPort"], spec["managementIp"], spec["physicalFcuRemotePort"])
        params["physical_gcs_url"] = "udp://0.0.0.0:%s@127.0.0.1:14550" % (24554 + spec["mavSystemId"])
        params["simulation_fcu_url"] = "udp://:%s@127.0.0.1:%s" % (spec["simulationLocalPort"], spec["simulationRemotePort"])
        derived.update(sitlInstanceId=spec["mavSystemId"] - 1, simulationFcuUrl=params["simulation_fcu_url"])
    else:
        params.update({key: scalar(spec[field]) for key, field in {
            "management_address": "managementAddress", "connector": "connector",
            "telemetry_remote_port": "telemetryRemotePort", "control_local_port": "controlLocalPort"}.items()})
    seen = {}
    rendered = []
    for link, geometry in visuals:
        index = seen.get(link, 0)
        rendered.append(dict(link=link, index=index, geometryType=geometry))
        seen[link] = index + 1
    view = dict(sceneClass=scene, descriptionPackage=package, descriptionFile="urdf/" + description,
                robotStatePublisher=False, jointStateTopic="joint_states", pathTopic="path", visuals=rendered)
    if arm != "px4":
        view["odometryTopic"] = "odom"
    derived["visualization"] = view
    mode, simulator = context["runMode"], context["simulator"]
    simulated = mode == "simulation" or mode == "hybrid" and binding["hybridSource"] == "simulation"
    profile = asset["profileId"]
    if profile.startswith(prefix + ".") and simulated and simulator in ("gazebo", "xsim"):
        profile = prefix + "." + simulator + (".vrpn" if simulator == "gazebo" else ".ros")
    params["ros_master_uri"], params["ros_ip"] = context["rosMasterUri"], context["rosIp"]
    params["mocap_source_root"] = "/vrpn_client_node" if mode != "hybrid" else "/vrpn_client_node_simulation" if simulated else "/vrpn_client_node_physical"
    offset = context["localizationOffset"]
    if simulated:
        params["mocap_rigid_body"] = binding["namespace"].lstrip("/")
        offset = dict(x=0, y=0, z=0)
    params.update({"localization_offset_" + axis: scalar(offset[axis]) for axis in ("x", "y", "z")})
    if simulated and simulator == "xsim":
        params["localization_pose_topic"] = binding["namespace"] + "/pose"
        params["localization_twist_topic"] = binding["namespace"] + "/twist"
    cameras = []
    if binding.get(arm, {}).get("imageSimulationEnabled"):
        cameras.append(dict(id="robot:" + binding["id"], kind="robot", robotSlotId=binding["id"]))
    return dict(slotId=binding["id"], profileId=profile, parameters=params, derived=derived, cameras=cameras)


def project_batch(request):
    return [project(row, request["context"]) for row in request["robots"]]


if __name__ == "__main__":
    try:
        json.dump(project_batch(json.load(sys.stdin)), sys.stdout, allow_nan=False)
    except (KeyError, TypeError, ValueError) as exc:
        print("robot asset projection: " + str(exc), file=sys.stderr)
        sys.exit(2)
