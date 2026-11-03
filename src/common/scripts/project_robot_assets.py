#!/usr/bin/env python3
"""Project a frozen robot batch into the installed ROS1 profile contract.

This finite, side-effect-free native entry point owns robot transport, source
selection and model facts. Core supplies pinned documents and freezes the result.
Input is binding plus model{robotKind, modelId, ordinal} and an optional
physicalBinding. The frozen selector is kind plus derived.modelId. The model
object stays input-only. physicalBinding is inventory and is not read from row asset.
"""
import copy
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
# Installed model ids. The third MODELS field is the visualization scene class, which
# matches only for FS150; Scout and Mecanum scene classes are not their model ids.
INSTALLED_MODEL_IDS = {
    "px4_multirotor": "fs150",
    "scout_mini": "scout-mini",
    "mecanum_ugv": "mecanum-ugv",
}
# Product defaults for a model slot that has no robot asset. These are not endpoints.
MODEL_POSITIONING_FRAME = 5
MODEL_POSITIONING_THRESHOLD_M = 1e-10
# Gazebo MAVLink plugin ports for sitl instance N. QGC stays on 14550.
SITL_ORDINAL_MAX = 243
SITL_TCP_BASE = 4560
SITL_UDP_BASE = 14560
SITL_QGC_PORT = 14550
SITL_SDK_BASE = 15000


def scalar(value):
    return str(int(value)) if isinstance(value, float) and value.is_integer() else repr(value) if isinstance(value, float) else str(value)


def whole_number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, int) or value < low or value > high:
        raise ValueError("%s is out of range" % name)
    return value


def effective_binding(binding, scene):
    result = copy.deepcopy(binding)
    lidar = (scene or {}).get("parameters", {}).get("lidar")
    if lidar is None:
        return result
    if lidar["scope"] not in ("global", "individual"):
        raise ValueError("Scene lidar scope must be global or individual")
    if lidar["scope"] == "global":
        result.setdefault("simulationSensors", {})["simpleLidar"] = copy.deepcopy(lidar["settings"])
    return result


def simulated_source(row):
    source = row["source"]
    if source not in ("simulation", "physical"):
        raise ValueError("effective robot source must be simulation or physical")
    return source == "simulation"


def gazebo_sitl(kind, binding, context, simulated):
    # Absent applyModelDynamics is not false and is not a Gazebo SITL admission.
    return (kind == "px4_multirotor" and context.get("simulator") == "gazebo" and simulated
            and binding.get("applyModelDynamics") is True)


def sitl_endpoint(ordinal):
    """Gazebo SITL identity. sitlInstanceId is the model ordinal; ports follow that id."""
    instance = whole_number(ordinal, "sitlInstanceId", 0, SITL_ORDINAL_MAX)
    return dict(sitlInstanceId=instance,
                mavlinkTcpPort=SITL_TCP_BASE + instance,
                mavlinkUdpPort=SITL_UDP_BASE + instance,
                qgcUdpPort=SITL_QGC_PORT,
                sdkUdpPort=SITL_SDK_BASE + instance)


def visualization(arm, scene, package, description, visuals):
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
    return view


def apply_runtime(params, derived, binding, context, simulated):
    mode, simulator = context["runMode"], context["simulator"]
    params["ros_master_uri"], params["ros_ip"] = context["rosMasterUri"], context["rosIp"]
    offset = context["localizationOffset"]
    params["mocap_source_root"] = "/vrpn_client_node" if mode != "hybrid" else "/vrpn_client_node_simulation" if simulated else "/vrpn_client_node_physical"
    if simulated:
        params["mocap_rigid_body"] = binding["namespace"].lstrip("/")
        offset = dict(x=0, y=0, z=0)
    params.update({"localization_offset_" + axis: scalar(offset[axis]) for axis in ("x", "y", "z")})
    authored = binding.get("runtimeParameters") or {}
    # A frozen explicit binding selects the shared Zenoh input. Until the
    # source producer supplies endpoint/source, keep the existing ROS path
    # rather than removing its root and leaving the Adapter with no input.
    if authored.get("zenoh_endpoint") and authored.get("localization_source"):
        for name in ("zenoh_endpoint", "localization_source", "mocap_sensor", "mocap_frame_id"):
            if name in authored:
                params[name] = str(authored[name])
        params.pop("mocap_source_root", None)
        params["localization_pose_topic"] = binding["namespace"] + "/pose"
        params["localization_twist_topic"] = binding["namespace"] + "/twist"
    elif simulated and simulator in ("xsim", "none"):
        params["localization_pose_topic"] = binding["namespace"] + "/pose"
        params["localization_twist_topic"] = binding["namespace"] + "/twist"
    elif simulated:
        # Same-machine Gazebo keeps the existing local measurement short chain.
        root = params["mocap_source_root"].rstrip('/') + '/' + params["mocap_rigid_body"]
        params["localization_pose_topic"] = root + '/pose'
        params["localization_twist_topic"] = root + '/twist'
    else:
        # External localization reaches the Adapter through Zenoh. The selected
        # source/endpoint/sensor producer is still Open; do not restore the old
        # VRPN root as a compatibility path.
        params.pop("mocap_source_root", None)
        params["localization_pose_topic"] = binding["namespace"] + "/pose"
        params["localization_twist_topic"] = binding["namespace"] + "/twist"
    return params, derived


def finish(binding, kind, model_id, ordinal, arm, profile, params, derived, context, visuals, simulated, physical=None):
    derived["modelId"] = model_id
    derived["visualization"] = visuals
    apply_runtime(params, derived, binding, context, simulated)
    cameras = []
    if binding.get(arm, {}).get("imageSimulationEnabled"):
        cameras.append(dict(id="robot:" + binding["id"], kind="robot", robotSlotId=binding["id"]))
    result = dict(slotId=binding["id"], kind=kind, binding=binding, profileId=profile,
                  parameters=params, derived=derived, cameras=cameras)
    if physical is not None:
        result["physicalBinding"] = physical
    return result


def require_model(row):
    if "asset" in row:
        raise ValueError("projection reads physicalBinding.asset, not row asset")
    model = row["model"]
    if not isinstance(model, dict):
        raise ValueError("model is required")
    kind, model_id = model["robotKind"], model["modelId"]
    if INSTALLED_MODEL_IDS.get(kind) != model_id:
        raise ValueError("unsupported installed model %s/%s" % (kind, model_id))
    ordinal = whole_number(model["ordinal"], "ordinal", 0, 255)
    return kind, model_id, ordinal


def project_model(row, binding, context, kind, model_id, ordinal):
    if row.get("physicalBinding") is not None:
        raise ValueError("model projection cannot carry an asset")
    arm, prefix, scene, package, description, visuals = MODELS[kind]
    simulator = context["simulator"]
    simulated = simulated_source(row)
    if not simulated or simulator not in ("gazebo", "xsim", "none"):
        raise ValueError("model projection requires gazebo, xsim or none")
    profile = prefix + (".gazebo.vrpn" if simulator == "gazebo" else ".xsim.ros")
    params = {"positioning_frame_number": scalar(MODEL_POSITIONING_FRAME),
              "positioning_comparison_threshold_m": scalar(MODEL_POSITIONING_THRESHOLD_M)}
    derived = {}
    if gazebo_sitl(kind, binding, context, simulated):
        endpoint = sitl_endpoint(ordinal)
        derived["sitlInstanceId"] = endpoint["sitlInstanceId"]
        params.update(mavlink_tcp_port=scalar(endpoint["mavlinkTcpPort"]),
                      mavlink_udp_port=scalar(endpoint["mavlinkUdpPort"]),
                      qgc_udp_port=scalar(endpoint["qgcUdpPort"]),
                      sdk_udp_port=scalar(endpoint["sdkUdpPort"]))
    return finish(binding, kind, model_id, ordinal, arm, profile, params, derived, context,
                  visualization(arm, scene, package, description, visuals), simulated)


def project_device(row, binding, context, kind, model_id, ordinal):
    physical = row.get("physicalBinding")
    if not isinstance(physical, dict) or "asset" not in physical:
        raise ValueError("asset projection requires physicalBinding.asset")
    asset = physical["asset"]
    if asset["kind"] != kind:
        raise ValueError("physicalBinding kind does not match model.robotKind")
    arm, prefix, scene, package, description, visuals = MODELS[kind]
    spec = asset[arm]
    if arm == "px4" and spec["modelId"] != model_id:
        raise ValueError("Unsupported PX4 model: " + spec["modelId"])
    params = {"mocap_rigid_body": spec["mocapRigidBodyName"],
              "positioning_frame_number": scalar(spec["positioningFrameNumber"]),
              "positioning_comparison_threshold_m": scalar(spec["positioningComparisonThresholdM"])}
    # physicalBinding stays inventory. It does not become FCU URLs or management addresses.
    derived = {}
    simulated = simulated_source(row)
    if gazebo_sitl(kind, binding, context, simulated):
        endpoint = sitl_endpoint(ordinal)
        derived["sitlInstanceId"] = endpoint["sitlInstanceId"]
        params.update(mavlink_tcp_port=scalar(endpoint["mavlinkTcpPort"]),
                      mavlink_udp_port=scalar(endpoint["mavlinkUdpPort"]),
                      qgc_udp_port=scalar(endpoint["qgcUdpPort"]),
                      sdk_udp_port=scalar(endpoint["sdkUdpPort"]))
    profile = asset["profileId"]
    if profile.startswith(prefix + ".") and simulated and context["simulator"] in ("gazebo", "xsim", "none"):
        profile = prefix + (".gazebo.vrpn" if context["simulator"] == "gazebo" else ".xsim.ros")
    return finish(binding, kind, model_id, ordinal, arm, profile, params, derived, context,
                  visualization(arm, scene, package, description, visuals), simulated, physical=copy.deepcopy(physical))


def project(row, context):
    binding = effective_binding(row["binding"], context.get("scene"))
    if binding.get("applyModelDynamics") is not True and binding.get("applyModelDynamics") is not False:
        raise ValueError("applyModelDynamics must be true or false")
    kind, model_id, ordinal = require_model(row)
    if row.get("physicalBinding") is None:
        return project_model(row, binding, context, kind, model_id, ordinal)
    return project_device(row, binding, context, kind, model_id, ordinal)


def project_batch(request):
    return [project(row, request["context"]) for row in request["robots"]]


if __name__ == "__main__":
    try:
        json.dump(project_batch(json.load(sys.stdin)), sys.stdout, allow_nan=False)
    except (KeyError, TypeError, ValueError) as exc:
        print("robot asset projection: " + str(exc), file=sys.stderr)
        sys.exit(2)
