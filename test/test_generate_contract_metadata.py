#!/usr/bin/env python3

import copy
import importlib.util
import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = REPOSITORY_ROOT / "tools" / "generate_contract_metadata.py"
SCHEMA_PATH = (
    REPOSITORY_ROOT
    / "profiles"
    / "schema"
    / "robot-adapter-profile-v4.schema.json"
)
PX4_PROFILE_PATH = REPOSITORY_ROOT / "profiles" / "ros1" / "px4-multirotor-ros1-v7.yaml"
SCOUT_PROFILE_PATH = REPOSITORY_ROOT / "profiles" / "ros1" / "scout-mini-ros1-v6.yaml"
MECANUM_PROFILE_PATH = (
    REPOSITORY_ROOT / "profiles" / "ros1" / "mecanum-ugv-ros1-v3.yaml"
)
B2_PROFILE_PATH = REPOSITORY_ROOT / "profiles" / "ros1" / "unitree-b2-v1.yaml"
MOCAP_ROTOR_PROFILE_PATH = (
    REPOSITORY_ROOT / "profiles" / "ros1" / "mocap-rotor-ros1-v1.yaml"
)
PX4_DEFINITION_ID = "xgc2-px4-multirotor-ros1-adapter"
MECANUM_DEFINITION_ID = "xgc2-mecanum-ugv-ros1-adapter"
MOCAP_ROTOR_DEFINITION_ID = "xgc2-mocap-rotor-ros1-adapter"

SPEC = importlib.util.spec_from_file_location("contract_generator", GENERATOR_PATH)
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)


MESSAGE_ROLES = {
    1: "request",
    2001: "telemetry",
    2002: "telemetry",
    2003: "telemetry",
    2004: "telemetry",
    2005: "telemetry",
    2006: "telemetry",
    2007: "telemetry",
    2010: "diagnostic",
    2011: "diagnostic",
    3001: "telemetry",
    3002: "telemetry",
    3003: "telemetry",
    3004: "diagnostic",
    3005: "diagnostic",
    3102: "telemetry",
    3103: "telemetry",
    3104: "telemetry",
    3201: "request",
    3202: "request",
    3203: "request",
    3204: "request",
    3205: "request",
    4001: "configuration",
    4002: "telemetry",
}

TYPE_NAMES = {
    2005: "xgc.semantic.common.v1.VehicleHealth",
    2006: "xgc.semantic.common.v1.SpeedEstimate",
    2007: "xgc.semantic.common.v1.DistanceEstimate",
    3001: "xgc.semantic.aerial.v1.FlightStatus",
    3102: "xgc.semantic.ground.v1.ChassisStatus",
    3103: "xgc.semantic.ground.v1.LocomotionStatus",
    3104: "xgc.semantic.ground.v1.JointStateSet",
    3201: "xgc.semantic.aerial.v1.ArmRequest",
    3202: "xgc.semantic.aerial.v1.ModeRequest",
    3203: "xgc.semantic.aerial.v1.AutopilotRebootRequest",
    3204: "xgc.semantic.ground.v1.MotionIntentRequest",
    3205: "xgc.semantic.common.v1.RemoteControlIntentRequest",
}


def registry_document():
    return {
        "registry_version": 1,
        "registry_fingerprint": 987654321,
        "messages": [
            {
                "id": message_id,
                "type": TYPE_NAMES.get(
                    message_id, "test.semantic.Message{}".format(message_id)
                ),
                "version": 1,
                "fingerprint": 100000 + message_id,
                "roles": [role],
            }
            for message_id, role in sorted(MESSAGE_ROLES.items())
        ],
    }


class ContractGeneratorTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.temp = Path(self.temporary_directory.name)
        self.registry_path = self.temp / "registry.json"
        self.registry_path.write_text(
            json.dumps(registry_document()), encoding="utf-8"
        )
        self.registry_fingerprint, self.messages = GENERATOR.load_registry(
            self.registry_path
        )

    def write_profile(self, document):
        path = self.temp / "profile.yaml"
        path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        return path

    def test_mocap_rotor_profile_is_a_dedicated_read_only_zenoh_projection(self):
        profiles = GENERATOR.load_profile(
            MOCAP_ROTOR_PROFILE_PATH, SCHEMA_PATH, self.messages
        )
        profile = profiles["px4.mocap-rotor.ros1.v1"]
        channels = {channel["id"]: channel for channel in profile["channels"]}

        self.assertEqual(profile["robot_kind"], "px4_multirotor")
        self.assertEqual(
            set(profile["parameters"]),
            {"namespace", "robot_id", "wire_transport", "zenoh_listen"},
        )
        self.assertEqual(
            profile["parameters"]["wire_transport"]["pattern"], "^zenoh$"
        )
        self.assertEqual(
            set(channels),
            {
                "state.pose",
                "state.velocity",
                "state.speed",
                "state.imu",
                "state.power",
                "state.health",
                "state.flight",
                "diagnostic.link",
                "diagnostic.stream-health",
            },
        )
        self.assertFalse(
            any(channel["kind"] == "operation" for channel in channels.values())
        )
        self.assertEqual(
            channels["state.pose"]["endpoints"][0]["name_template"],
            "xgc2/{robot_id}/up/local_pose",
        )
        self.assertEqual(
            channels["diagnostic.link"]["endpoints"][0]["name_template"],
            "xgc2/{robot_id}/up/forwarder_hb",
        )
        self.assertEqual(
            {
                endpoint["name_template"]
                for channel in channels.values()
                for endpoint in channel["endpoints"]
            },
            {
                "xgc2/{robot_id}/up/local_pose",
                "xgc2/{robot_id}/up/local_velocity",
                "xgc2/{robot_id}/up/imu",
                "xgc2/{robot_id}/up/power",
                "xgc2/{robot_id}/up/flight_state",
                "xgc2/{robot_id}/up/forwarder_hb",
            },
        )

        serialized = MOCAP_ROTOR_PROFILE_PATH.read_text(encoding="utf-8").lower()
        for forbidden in ("mavros/", "gps", "navsatfix", "setpoint", "operation."):
            self.assertNotIn(forbidden, serialized)

        body = GENERATOR.catalog_profile_body(
            profile, self.messages, MOCAP_ROTOR_DEFINITION_ID
        )
        self.assertEqual(
            body["providerDefinitionId"], MOCAP_ROTOR_DEFINITION_ID
        )
        self.assertEqual(body["semantics"]["operations"], [])
        self.assertEqual(
            body["semantics"]["operationalReadyConditions"],
            [
                {"channelId": "diagnostic.link", "maximumAgeMillis": 3000},
                {"channelId": "state.pose", "maximumAgeMillis": 1000},
            ],
        )

    def test_local_profiles_are_source_of_complete_generated_metadata(self):
        px4_profiles = GENERATOR.load_profile(
            PX4_PROFILE_PATH, SCHEMA_PATH, self.messages
        )
        scout_profiles = GENERATOR.load_profile(
            SCOUT_PROFILE_PATH, SCHEMA_PATH, self.messages
        )
        mecanum_profiles = GENERATOR.load_profile(
            MECANUM_PROFILE_PATH, SCHEMA_PATH, self.messages
        )
        b2_profiles = GENERATOR.load_profile(
            B2_PROFILE_PATH, SCHEMA_PATH, self.messages
        )

        px4 = px4_profiles["px4.multirotor.ros1.v7"]
        px4_channels = {channel["id"]: channel for channel in px4["channels"]}
        self.assertEqual(
            set(px4_channels),
            {
                "state.pose",
                "state.mocap.pose",
                "state.velocity",
                "state.mocap.velocity",
                "state.mocap.speed",
                "state.localization.error",
                "state.imu",
                "state.power",
                "state.health",
                "state.flight",
                "setpoint.local",
                "setpoint.attitude",
                "diagnostic.fcu-link",
                "diagnostic.offboard-input",
                "diagnostic.stream-health",
                "operation.arm",
                "operation.mode",
                "operation.autopilot-reboot",
                "operation.motion-intent",
            },
        )
        self.assertEqual(px4_channels["operation.arm"]["input_message_id"], 3201)
        self.assertEqual(px4_channels["operation.arm"]["output_message_id"], 1)
        self.assertEqual(
            px4_channels["operation.arm"]["operation_timeout_millis"], 5000
        )
        self.assertEqual(px4_channels["operation.mode"]["input_message_id"], 3202)
        self.assertEqual(
            px4_channels["operation.autopilot-reboot"]["input_message_id"], 3203
        )
        self.assertEqual(px4_channels["state.pose"]["stale_after_millis"], 1000)
        self.assertEqual(
            px4_channels["state.mocap.pose"]["stale_after_millis"], 500
        )
        self.assertEqual(px4_channels["state.velocity"]["stale_after_millis"], 2000)
        self.assertEqual(
            px4_channels["state.mocap.velocity"]["stale_after_millis"], 500
        )
        self.assertEqual(
            px4_channels["state.mocap.speed"]["stale_after_millis"], 500
        )
        self.assertEqual(
            px4_channels["state.mocap.velocity"]["output_message_id"], 2002
        )
        self.assertEqual(
            px4_channels["state.mocap.speed"]["output_message_id"], 2006
        )
        self.assertEqual(
            px4_channels["state.localization.error"]["output_message_id"], 2007
        )
        self.assertEqual(
            px4_channels["state.localization.error"]["stale_after_millis"], 500
        )
        self.assertEqual(
            px4_channels["state.mocap.velocity"]["endpoints"][0]["name_template"],
            "vrpn_client_node/{mocap_rigid_body}/twist",
        )
        self.assertEqual(
            px4_channels["state.mocap.pose"]["endpoints"][0],
            {
                "kind": "input",
                "role": "pose",
                "name_template": "vrpn_client_node/{mocap_rigid_body}/pose",
                "ros_type": "geometry_msgs/PoseStamped",
                "scope": "global",
            },
        )
        self.assertEqual(
            px4_channels["operation.autopilot-reboot"]["operation_contract"],
            {
                "side_effect": "non-idempotent",
                "idempotency": "required",
                "cancellation_supported": False,
                "deadline_required": True,
            },
        )
        self.assertNotIn("digest", px4)
        px4_body = GENERATOR.catalog_profile_body(
            px4, self.messages, PX4_DEFINITION_ID
        )
        self.assertIn(
            {"channelId": "state.mocap.pose", "maximumAgeMillis": 500},
            px4_body["semantics"]["operationalReadyConditions"],
        )
        operations = {
            operation["id"]: operation
            for operation in px4_body["semantics"]["operations"]
        }
        self.assertEqual(
            operations["arm"],
            {
                "id": "arm",
                "channelId": "operation.arm",
                "timeoutMillis": 5000,
                "parameterSchema": {
                    "type": "object",
                    "required": ["armed"],
                    "properties": {"armed": {"type": "boolean"}},
                    "additionalProperties": False,
                },
            },
        )
        self.assertEqual(
            operations["set-flight-mode"]["parameterSchema"]["properties"][
                "mode"
            ]["enum"],
            ["OFFBOARD", "POSCTL", "ALTCTL", "STABILIZED"],
        )
        self.assertEqual(
            operations["reboot-autopilot"]["parameterSchema"],
            {
                "type": "object",
                "required": [],
                "properties": {},
                "additionalProperties": False,
            },
        )
        px4_digest = GENERATOR.profile_contract_digest(px4_body)

        scout = scout_profiles["scout-mini.ros1.v6"]
        scout_channels = {channel["id"]: channel for channel in scout["channels"]}
        self.assertNotIn("state.pose", scout_channels)
        self.assertNotIn("state.velocity", scout_channels)
        self.assertEqual(
            scout_channels["vrpn.position"]["endpoints"][0],
            {
                "kind": "input",
                "role": "pose",
                "name_template": "vrpn_client_node/{mocap_rigid_body}/pose",
                "ros_type": "geometry_msgs/PoseStamped",
                "scope": "global",
            },
        )
        self.assertEqual(
            scout_channels["command.velocity"]["endpoints"][0],
            {
                "kind": "input",
                "role": "command",
                "name_template": "cmd_vel",
                "ros_type": "geometry_msgs/Twist",
                "scope": "robot_namespace",
            },
        )
        self.assertEqual(
            scout_channels["vrpn.velocity"]["endpoints"][0],
            {
                "kind": "input",
                "role": "velocity",
                "name_template": "vrpn_client_node/{mocap_rigid_body}/twist",
                "ros_type": "geometry_msgs/TwistStamped",
                "scope": "global",
            },
        )
        self.assertEqual(
            scout_channels["state.power"]["endpoints"][0],
            {
                "kind": "input",
                "role": "battery",
                "name_template": "PowerVoltage",
                "ros_type": "std_msgs/Float32",
                "scope": "robot_namespace",
            },
        )
        self.assertEqual(
            scout_channels["state.chassis"]["endpoints"][0],
            {
                "kind": "input",
                "role": "chassis_state",
                "name_template": "scout/chassis_state",
                "ros_type": "std_msgs/UInt32",
                "scope": "robot_namespace",
            },
        )
        self.assertEqual(
            scout_channels["vrpn.speed"]["output_message_id"], 2006
        )
        self.assertEqual(
            {endpoint["role"] for endpoint in scout_channels["vrpn.speed"]["endpoints"]},
            {"pose", "velocity"},
        )
        motion = scout_channels["operation.motion-intent"]
        self.assertEqual(motion["kind"], "operation")
        self.assertEqual(motion["operation_id"], "set-motion-intent")
        self.assertEqual(motion["input_message_id"], 3205)
        self.assertEqual(motion["output_message_id"], 1)
        self.assertEqual(
            motion["endpoints"],
            [
                {
                    "kind": "output",
                    "role": "output",
                    "name_template": "cmd_vel",
                    "ros_type": "geometry_msgs/Twist",
                    "scope": "robot_namespace",
                }
            ],
        )
        scout_body = GENERATOR.catalog_profile_body(
            scout, self.messages, "xgc2-scout-mini-ros1-adapter"
        )
        self.assertEqual(
            scout_body["semantics"]["operations"],
            [
                {
                    "id": "set-motion-intent",
                    "channelId": "operation.motion-intent",
                    "timeoutMillis": 1000,
                    "parameterSchema": {
                        "type": "object",
                        "required": ["gear", "lateral", "longitudinal", "yaw"],
                        "properties": {
                            "gear": {
                                "type": "integer",
                                "minimum": 1,
                                "maximum": 3,
                            },
                            "longitudinal": {
                                "type": "integer",
                                "minimum": -1,
                                "maximum": 1,
                            },
                            "lateral": {
                                "type": "integer",
                                "minimum": -1,
                                "maximum": 1,
                            },
                            "yaw": {
                                "type": "integer",
                                "minimum": -1,
                                "maximum": 1,
                            },
                        },
                        "additionalProperties": False,
                    },
                }
            ],
        )

        mecanum = mecanum_profiles["mecanum-ugv.ros1.v3"]
        mecanum_channels = {
            channel["id"]: channel for channel in mecanum["channels"]
        }
        b2 = b2_profiles["unitree.b2.v1"]
        b2_channels = {channel["id"]: channel for channel in b2["channels"]}
        self.assertEqual(len(b2_channels), 10)
        self.assertEqual(b2_channels["state.locomotion"]["output_message_id"], 3103)
        self.assertEqual(b2_channels["state.joints"]["output_message_id"], 3104)
        self.assertEqual(b2_channels["state.arm-joints"]["output_message_id"], 3104)
        self.assertFalse(any(channel["kind"] == "operation" for channel in b2_channels.values()))
        self.assertEqual(
            b2_channels["state.pose"]["endpoints"][0]["name_template"],
            "xgc2/{robot_id}/up/odom",
        )
        self.assertEqual(
            set(mecanum_channels),
            {
                "vrpn.position",
                "vrpn.velocity",
                "vrpn.speed",
                "command.velocity",
                # The on-board IMU is this chassis's liveness stream: the Robot
                # Profile gates online on it and leaves VRPN to gate readiness.
                "state.imu",
                "state.power",
                "state.health",
                "operation.motion-intent",
                "diagnostic.stream-health",
            },
        )
        self.assertEqual(
            scout_channels["diagnostic.stream-health"]["output_message_id"],
            2011,
        )
        self.assertNotIn("diagnostic.channel-health", scout_channels)
        self.assertEqual(
            mecanum_channels["diagnostic.stream-health"]["output_message_id"],
            2011,
        )
        self.assertNotIn("diagnostic.channel-health", mecanum_channels)
        for channel_id in (
            "vrpn.position",
            "vrpn.velocity",
            "vrpn.speed",
            "command.velocity",
            "diagnostic.stream-health",
        ):
            self.assertEqual(mecanum_channels[channel_id]["output_rate_hz"], 10)
        self.assertEqual(mecanum_channels["vrpn.position"]["output_message_id"], 2001)
        self.assertEqual(mecanum_channels["vrpn.velocity"]["output_message_id"], 2002)
        self.assertEqual(mecanum_channels["vrpn.speed"]["output_message_id"], 2006)
        self.assertEqual(mecanum_channels["command.velocity"]["output_message_id"], 2002)
        self.assertEqual(mecanum_channels["state.imu"]["output_message_id"], 2003)
        self.assertEqual(mecanum_channels["state.power"]["output_message_id"], 2004)
        self.assertEqual(mecanum_channels["state.power"]["output_rate_hz"], 1)
        self.assertEqual(
            mecanum_channels["state.power"]["endpoints"][0],
            {
                "kind": "input",
                "role": "battery",
                "name_template": "PowerVoltage",
                "ros_type": "std_msgs/Float32",
                "scope": "robot_namespace",
            },
        )
        self.assertEqual(
            {endpoint["role"] for endpoint in mecanum_channels["vrpn.speed"]["endpoints"]},
            {"pose", "velocity"},
        )
        mecanum_body = GENERATOR.catalog_profile_body(
            mecanum, self.messages, MECANUM_DEFINITION_ID
        )
        self.assertEqual(mecanum_body["robotKind"], "mecanum_ugv")
        # Both ground vehicles carry the identical readiness contract: the
        # on-board IMU proves the vehicle is online, VRPN proves it is localized,
        # and a robot is ready only when both hold.
        self.assertEqual(
            mecanum_body["semantics"]["onlineConditions"],
            [{"channelId": "state.imu", "maximumAgeMillis": 1000}],
        )
        self.assertEqual(
            mecanum_body["semantics"]["operationalReadyConditions"],
            [{"channelId": "vrpn.position", "maximumAgeMillis": 1000}],
        )
        self.assertEqual(
            mecanum_body["semantics"]["onlineConditions"],
            scout_body["semantics"]["onlineConditions"],
        )
        self.assertEqual(
            mecanum_body["semantics"]["operationalReadyConditions"],
            scout_body["semantics"]["operationalReadyConditions"],
        )
        self.assertEqual(
            mecanum_body["semantics"]["operations"],
            scout_body["semantics"]["operations"],
        )

        header = GENERATOR.generate(
            self.registry_fingerprint,
            self.messages,
            px4_profiles,
            PX4_DEFINITION_ID,
            "xgc_px4_multirotor_ros1_adapter",
        )
        self.assertIn('if (profile_id == "px4.multirotor.ros1.v7")', header)
        self.assertIn('kProfileId = "px4.multirotor.ros1.v7"', header)
        self.assertIn('"namespace", ParameterType::kString, true', header)
        self.assertNotIn("kNamespaceParameter", header)
        self.assertIn("struct ParameterMetadata", header)
        self.assertIn("struct EndpointMetadata", header)
        self.assertIn("struct OperationMetadata", header)
        self.assertIn("inline const OperationMetadata* profileOperations(", header)
        self.assertIn("struct PolicyMetadata", header)
        self.assertIn("inline const ChannelMetadata* profileChannels(", header)
        self.assertIn('"operation.arm", ChannelKind::kOperation', header)
        self.assertIn('"arm", 3201u, 1u, 0.0, 5000u, 0u', header)
        self.assertIn('"mavros/cmd/arming", "mavros_msgs/CommandBool"', header)
        self.assertIn('"source_timeout_ms", PolicyValueKind::kInteger', header)
        self.assertIn("inline bool channelPolicyStringArray(", header)
        self.assertIn('"xgc.semantic.aerial.v1.ModeRequest"', header)
        self.assertIn(px4_digest, header)
        self.assertNotIn("xgc::adapter::v1", header)
        self.assertNotIn("ProfileAdvertisement", header)

        scout_header = GENERATOR.generate(
            self.registry_fingerprint,
            self.messages,
            scout_profiles,
            "xgc2-scout-mini-ros1-adapter",
            "xgc_scout_mini_ros1_adapter",
        )
        self.assertIn('"set-motion-intent", 3205u, 1u, 0.0, 1000u, 0u', scout_header)
        self.assertIn('EndpointKind::kOutput, "output", "cmd_vel"', scout_header)
        self.assertIn('"xgc.semantic.common.v1.RemoteControlIntentRequest"', scout_header)

        mecanum_header = GENERATOR.generate(
            self.registry_fingerprint,
            self.messages,
            mecanum_profiles,
            MECANUM_DEFINITION_ID,
            "xgc_mecanum_ugv_ros1_adapter",
        )
        self.assertIn('kProfileId = "mecanum-ugv.ros1.v3"', mecanum_header)
        self.assertIn('"mecanum-ugv.set-motion-intent"', mecanum_header)
        self.assertIn('EndpointKind::kOutput, "output", "cmd_vel"', mecanum_header)

    def test_px4_native_operation_policies_are_explicit(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        channels = {channel["id"]: channel for channel in profile["channels"]}

        self.assertEqual(
            channels["operation.arm"]["service"],
            {"name": "mavros/cmd/arming", "type": "mavros_msgs/CommandBool"},
        )
        self.assertEqual(
            channels["operation.arm"]["operation_contract"],
            {
                "side_effect": "idempotent",
                "idempotency": "required",
                "cancellation_supported": False,
                "deadline_required": True,
            },
        )
        self.assertEqual(
            channels["operation.mode"]["policy"]["allowed_modes"],
            ["OFFBOARD", "POSCTL", "ALTCTL", "STABILIZED"],
        )
        reboot_policy = channels["operation.autopilot-reboot"]["policy"]
        self.assertEqual(reboot_policy["mav_command"], 246)
        self.assertEqual(reboot_policy["normal_reboot_param1"], 1)
        for condition in (
            "require_state_known",
            "require_state_fresh",
            "require_connected",
            "require_disarmed",
        ):
            self.assertTrue(reboot_policy[condition])

    def test_profile_digest_canonicalizes_public_contract_only(self):
        def contract_digest(path, definition_id=PX4_DEFINITION_ID):
            profiles = GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)
            source_profile = next(iter(profiles.values()))
            body = GENERATOR.catalog_profile_body(
                source_profile, self.messages, definition_id
            )
            return GENERATOR.profile_contract_digest(body)

        baseline = contract_digest(PX4_PROFILE_PATH)
        document = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))

        reordered = dict(reversed(list(document.items())))
        reordered["channels"] = list(reversed(reordered["channels"]))
        traits = reordered["semantic_traits"]
        traits["channel_stale_after_ms"] = dict(
            reversed(list(traits["channel_stale_after_ms"].items()))
        )
        traits["online_conditions"] = list(
            reversed(traits["online_conditions"])
        )
        traits["operational_ready_conditions"] = list(
            reversed(traits["operational_ready_conditions"])
        )
        reordered_path = self.temp / "reordered-profile.yaml"
        reordered_path.write_text(
            "# formatting and key order are not profile identity\n\n"
            + yaml.safe_dump(reordered, sort_keys=True),
            encoding="utf-8",
        )
        self.assertEqual(contract_digest(reordered_path), baseline)

        implementation_only = yaml.safe_load(
            PX4_PROFILE_PATH.read_text(encoding="utf-8")
        )
        implementation_only["description"] = "Changed package documentation."
        implementation_only["channels"][0]["processor"] = "px4.pose-v2"
        implementation_only["channels"][0]["inputs"]["pose"]["name"] = (
            "mavros/alternate_pose"
        )
        implementation_path = self.write_profile(implementation_only)
        self.assertEqual(contract_digest(implementation_path), baseline)

        public_change = yaml.safe_load(
            PX4_PROFILE_PATH.read_text(encoding="utf-8")
        )
        public_change["semantic_traits"]["default_stale_after_ms"] += 1
        public_path = self.write_profile(public_change)
        self.assertNotEqual(contract_digest(public_path), baseline)

        delivery_change = yaml.safe_load(
            PX4_PROFILE_PATH.read_text(encoding="utf-8")
        )
        delivery_change["parameters"]["namespace"]["delivery"] = "core_local"
        delivery_path = self.write_profile(delivery_change)
        self.assertNotEqual(contract_digest(delivery_path), baseline)

        operation_schema_change = yaml.safe_load(
            PX4_PROFILE_PATH.read_text(encoding="utf-8")
        )
        mode_channel = next(
            channel
            for channel in operation_schema_change["channels"]
            if channel.get("operation_id") == "set-flight-mode"
        )
        mode_channel["policy"]["allowed_modes"].append("ACRO")
        operation_schema_path = self.write_profile(operation_schema_change)
        self.assertNotEqual(contract_digest(operation_schema_path), baseline)

        operation_timeout_change = yaml.safe_load(
            PX4_PROFILE_PATH.read_text(encoding="utf-8")
        )
        arm_channel = next(
            channel
            for channel in operation_timeout_change["channels"]
            if channel.get("operation_id") == "arm"
        )
        arm_channel["policy"]["timeout_ms"] = 4999
        operation_timeout_path = self.write_profile(operation_timeout_change)
        self.assertNotEqual(contract_digest(operation_timeout_path), baseline)
        self.assertNotEqual(
            contract_digest(PX4_PROFILE_PATH, "another-provider"), baseline
        )

    def test_cmake_uses_one_provider_identity_for_header_and_catalog(self):
        packages = (
            (
                "xgc_px4_multirotor_ros1_adapter",
                "xgc2-px4-multirotor-ros1-adapter",
            ),
            (
                "xgc_scout_mini_ros1_adapter",
                "xgc2-scout-mini-ros1-adapter",
            ),
            (
                "xgc_mecanum_ugv_ros1_adapter",
                "xgc2-mecanum-ugv-ros1-adapter",
            ),
        )
        for package, definition_id in packages:
            cmake = (
                REPOSITORY_ROOT / "src" / package / "CMakeLists.txt"
            ).read_text(encoding="utf-8")
            with self.subTest(package=package):
                self.assertIn(
                    'set(ADAPTER_DEFINITION_ID "{}")'.format(definition_id),
                    cmake,
                )
                self.assertIn(
                    '--definition-id "${ADAPTER_DEFINITION_ID}"', cmake
                )
                self.assertIn(
                    "robot-adapter-profile-v4.schema.json", cmake
                )
                self.assertNotIn(
                    "robot-adapter-profile-v3.schema.json", cmake
                )
                self.assertIn(
                    "xgc2_add_robot_runtime_manifests(\n"
                    "  ${PROJECT_NAME}_node\n"
                    "  ${ADAPTER_DEFINITION_ID}",
                    cmake,
                )

    def test_complete_descriptor_header_compiles_as_cxx14(self):
        compiler = shutil.which("c++")
        if compiler is None:
            self.skipTest("C++ compiler is unavailable")
        profiles = GENERATOR.load_profile(
            PX4_PROFILE_PATH, SCHEMA_PATH, self.messages
        )
        header = self.temp / "generated_contract.hpp"
        header.write_text(
            GENERATOR.generate(
                self.registry_fingerprint,
                self.messages,
                profiles,
                PX4_DEFINITION_ID,
                "fixture_robot_adapter",
            ),
            encoding="utf-8",
        )
        translation_unit = self.temp / "contract_test.cpp"
        translation_unit.write_text(
            """
#include "generated_contract.hpp"

int main() {
  using namespace fixture_robot_adapter::contract;
  std::size_t channel_count = 0u;
  const ChannelMetadata* channels = profileChannels(kProfileId, &channel_count);
  ChannelMetadata arm{};
  if (channels == nullptr || channel_count == 0u ||
      !channelMetadata(kProfileId, "operation.arm", &arm)) {
    return 1;
  }
  const EndpointMetadata* service =
      channelEndpoint(arm, EndpointKind::kService, "service");
  std::int64_t timeout = 0;
  if (service == nullptr ||
      !channelPolicyInteger(arm, "timeout_ms", &timeout)) {
    return 2;
  }
  OperationMetadata mode{};
  if (!operationMetadata(kProfileId, "set-flight-mode", &mode)) {
    return 3;
  }
  const std::string schema(mode.parameter_schema_json);
  return timeout == 5000 && arm.output_message_id == 1u &&
                 mode.timeout_millis == 5000u &&
                 schema.find("additionalProperties") != std::string::npos &&
                 schema.find("OFFBOARD") != std::string::npos
             ? 0
             : 4;
}
""",
            encoding="utf-8",
        )
        completed = subprocess.run(
            [
                compiler,
                "-std=c++14",
                "-Wall",
                "-Wextra",
                "-Wpedantic",
                "-Werror",
                "-fsyntax-only",
                str(translation_unit),
            ],
            cwd=self.temp,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_schema_rejects_unowned_native_fields(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["central_profile_alias"] = "legacy"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["namespace_parameter"] = "namespace"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["parameters"]["namespace"]["type"] = "ros_namespace"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        del profile["parameters"]["namespace"]["delivery"]
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["parameters"]["namespace"]["delivery"] = "remote_guess"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["profile_id"] = "1robot.ros1.v4"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["parameters"]["namespace"]["default"] = "/uav1"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_profile_identity_and_parameters_have_exact_source_bounds(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["profile_id"] = "a" * 125 + ".v7"
        path = self.write_profile(profile)
        GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile["profile_id"] = "a" * 126 + ".v7"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        optional_parameter = {
            "type": "string",
            "required": False,
            "delivery": "target_binding",
            "description": "Optional product-owned runtime parameter.",
        }
        profile["parameters"]["a" * 64] = dict(optional_parameter)
        path = self.write_profile(profile)
        GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        del profile["parameters"]["a" * 64]
        profile["parameters"]["a" * 65] = dict(optional_parameter)
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        for index in range(62):
            profile["parameters"]["optional_{:02d}".format(index)] = dict(
                optional_parameter
            )
        path = self.write_profile(profile)
        GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile["parameters"]["overflow"] = dict(optional_parameter)
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_profile_channels_are_only_stream_outputs_or_operations(self):
        for unsupported_kind in ("stream_in", "request_response"):
            with self.subTest(kind=unsupported_kind):
                profile = yaml.safe_load(
                    PX4_PROFILE_PATH.read_text(encoding="utf-8")
                )
                profile["channels"][0]["kind"] = unsupported_kind
                path = self.write_profile(profile)
                with self.assertRaisesRegex(ValueError, "schema validation failed"):
                    GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_robot_kind_and_semantic_traits_use_exact_asset_vocabulary(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(profile["robot_kind"], "px4_multirotor")
        self.assertEqual(
            profile["semantic_traits"]["online_conditions"][0]["predicate"],
            "xgc.semantic.aerial.flight.connected",
        )

        profile["robot_kind"] = "px4-multirotor"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_semantic_traits_must_reference_compatible_stream_outputs(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["semantic_traits"]["channel_stale_after_ms"][
            "operation.arm"
        ] = 1000
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "must be an existing stream_out"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["semantic_traits"]["online_conditions"][0]["channel_id"] = (
            "operation.arm"
        )
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "must be an existing stream_out"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["semantic_traits"]["online_conditions"][0]["channel_id"] = (
            "state.pose"
        )
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "requires a FlightStatus"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_semantic_durations_and_operation_identities_are_bounded(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["semantic_traits"]["default_stale_after_ms"] = 1 << 32
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        duplicate_arm = copy.deepcopy(
            next(
                channel
                for channel in profile["channels"]
                if channel.get("operation_id") == "arm"
            )
        )
        duplicate_arm["id"] = "operation.arm-duplicate"
        profile["channels"].append(duplicate_arm)
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "duplicate operation identity"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        del profile["channels"][-1]["policy"]["timeout_ms"]
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["channels"][-1]["policy"]["timeout_ms"] = "5000"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["channels"][-1]["policy"]["timeout_ms"] = 5001
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed|safety limit"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        del profile["channels"][-1]["operation_contract"]
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        del profile["semantic_traits"]
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_operation_parameter_contracts_reject_unknown_or_malformed_sources(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        mode = next(
            channel
            for channel in profile["channels"]
            if channel.get("operation_id") == "set-flight-mode"
        )
        mode["policy"]["allowed_modes"] = []
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed|allowed_modes"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        arm = next(
            channel
            for channel in profile["channels"]
            if channel.get("operation_id") == "arm"
        )
        arm["operation_id"] = "unknown-operation"
        path = self.write_profile(profile)
        with self.assertRaisesRegex(
            ValueError, "schema validation failed|no owned parameter contract"
        ):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        arm = next(
            channel
            for channel in profile["channels"]
            if channel.get("operation_id") == "arm"
        )
        arm["input_message_id"] = 3202
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed|input must be"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        malformed = {
            "type": "object",
            "required": ["armed"],
            "properties": {"armed": {"type": "boolean"}},
            "additionalProperties": False,
            "legacy": True,
        }
        with self.assertRaisesRegex(ValueError, "exact strict object schema"):
            GENERATOR.validate_operation_parameter_schema(
                malformed, "fixture parameterSchema"
            )

        malformed_bounds = {
            "type": "object",
            "required": ["gear"],
            "properties": {
                "gear": {"type": "integer", "minimum": 3, "maximum": 1}
            },
            "additionalProperties": False,
        }
        with self.assertRaisesRegex(ValueError, "invalid bounds"):
            GENERATOR.validate_operation_parameter_schema(
                malformed_bounds, "fixture parameterSchema"
            )

    def test_operation_native_endpoint_is_exactly_one_service_or_output(self):
        profile = yaml.safe_load(SCOUT_PROFILE_PATH.read_text(encoding="utf-8"))
        motion = next(
            channel
            for channel in profile["channels"]
            if channel.get("operation_id") == "set-motion-intent"
        )
        motion["service"] = {
            "name": "cmd_vel",
            "type": "geometry_msgs/Twist",
        }
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed|exactly one"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(SCOUT_PROFILE_PATH.read_text(encoding="utf-8"))
        motion = next(
            channel
            for channel in profile["channels"]
            if channel.get("operation_id") == "set-motion-intent"
        )
        motion["output_message_id"] = 2001
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(SCOUT_PROFILE_PATH.read_text(encoding="utf-8"))
        motion = next(
            channel
            for channel in profile["channels"]
            if channel.get("operation_id") == "set-motion-intent"
        )
        del motion["output"]
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed|exactly one"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_endpoint_parameters_must_be_declared_required_strings(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["channels"][1]["inputs"]["pose"]["name"] = (
            "vrpn_client_node/{undeclared}/pose"
        )
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "undeclared parameter"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_parameter_patterns_use_the_portable_v1_regex_subset(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        for pattern in (
            "mocap.*",
            "^(?=mocap)mocap$",
            r"^(mocap)\\1$",
            "^[z-a]+$",
            "^[A-z]+$",
            "^[A-Z]{2}$",
        ):
            with self.subTest(pattern=pattern):
                mutated = yaml.safe_load(yaml.safe_dump(profile))
                mutated["parameters"]["mocap_rigid_body"]["pattern"] = pattern
                path = self.write_profile(mutated)
                with self.assertRaisesRegex(
                    ValueError, "schema validation|portable|range|construct|anchored"
                ):
                    GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        self.assertEqual(
            GENERATOR.validate_portable_parameter_pattern(
                "^[A-Za-z][A-Za-z0-9_]*$", "fixture"
            ),
            "^[A-Za-z][A-Za-z0-9_]*$",
        )

    def test_mocap_rigid_body_pattern_rejects_ros1_illegal_hyphens(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["parameters"]["mocap_rigid_body"]["pattern"] = (
            "^[A-Za-z][A-Za-z0-9_-]*$"
        )
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "schema validation failed"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_generic_profile_parameters_can_be_empty_without_a_namespace_role(self):
        profile = yaml.safe_load(SCOUT_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["parameters"] = {}
        channels = {channel["id"]: channel for channel in profile["channels"]}
        channels["vrpn.position"]["inputs"]["pose"]["name"] = (
            "vrpn_client_node/fixture/pose"
        )
        channels["vrpn.velocity"]["inputs"]["velocity"]["name"] = (
            "vrpn_client_node/fixture/twist"
        )
        channels["vrpn.speed"]["inputs"]["velocity"]["name"] = (
            "vrpn_client_node/fixture/twist"
        )
        channels["vrpn.speed"]["inputs"]["pose"]["name"] = (
            "vrpn_client_node/fixture/pose"
        )
        path = self.write_profile(profile)
        profiles = GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)
        header = GENERATOR.generate(
            self.registry_fingerprint,
            self.messages,
            profiles,
            "fixture-robot-adapter",
            "fixture_robot_adapter",
        )

        self.assertNotIn("kNamespaceParameter", header)
        self.assertNotIn("kRosNamespace", header)
        self.assertIn(
            'if (profile_id == "scout-mini.ros1.v6") {\n'
            "    *count = 0u;\n"
            "    return nullptr;",
            header,
        )

    def test_native_nodes_own_their_explicit_ros_namespace_parameter(self):
        implementations = (
            REPOSITORY_ROOT
            / "src"
            / "xgc_px4_multirotor_ros1_adapter"
            / "src"
            / "px4_multirotor_ros1_adapter_node.cpp",
            REPOSITORY_ROOT
            / "src"
            / "xgc_px4_multirotor_ros1_adapter"
            / "src"
            / "robot_runtime.cpp",
            REPOSITORY_ROOT
            / "src"
            / "xgc_scout_mini_ros1_adapter"
            / "src"
            / "scout_mini_ros1_adapter_node.cpp",
            REPOSITORY_ROOT
            / "src"
            / "xgc_scout_mini_ros1_adapter"
            / "src"
            / "robot_runtime.cpp",
            REPOSITORY_ROOT
            / "src"
            / "xgc_mecanum_ugv_ros1_adapter"
            / "src"
            / "mecanum_ugv_ros1_adapter_node.cpp",
            REPOSITORY_ROOT
            / "src"
            / "xgc_mecanum_ugv_ros1_adapter"
            / "src"
            / "robot_runtime.cpp",
            REPOSITORY_ROOT
            / "src"
            / "xgc_unitree_b2_ros1_adapter"
            / "src"
            / "unitree_b2_ros1_adapter_node.cpp",
            REPOSITORY_ROOT
            / "src"
            / "xgc_unitree_b2_ros1_adapter"
            / "src"
            / "robot_runtime.cpp",
        )
        for implementation in implementations:
            source = implementation.read_text(encoding="utf-8")
            with self.subTest(implementation=implementation.name):
                self.assertIn("contract::kProfileId", source)
                self.assertNotIn("contract::kNamespaceParameter", source)
                self.assertIn('find("namespace")', source)
                self.assertNotIn("px4.multirotor.ros1.v7", source)
                self.assertNotIn("scout-mini.ros1.v6", source)

        # Adapter Runtime applications consume a supervisor bootstrap. The
        # separately packaged onboard Mocap Rotor Forwarder is intentionally
        # not an Adapter Runtime application and owns explicit ROS parameters.
        for launch_file in REPOSITORY_ROOT.glob(
            "src/*_ros1_adapter/launch/*.launch"
        ):
            launch = launch_file.read_text(encoding="utf-8")
            with self.subTest(launch=launch_file.name):
                self.assertIn("--adapter-bootstrap-file", launch)
                self.assertNotIn("<param", launch)
                self.assertNotIn("<rosparam", launch)
                self.assertNotIn("robot_state_publisher", launch)

    def test_every_channel_message_must_exist_with_a_compatible_role(self):
        missing_reboot = dict(self.messages)
        missing_reboot.pop(3203)
        with self.assertRaisesRegex(ValueError, "unregistered input_message_id 3203"):
            GENERATOR.load_profile(PX4_PROFILE_PATH, SCHEMA_PATH, missing_reboot)

        wrong_role = dict(self.messages)
        wrong_role[3201] = dict(wrong_role[3201])
        wrong_role[3201]["roles"] = {"telemetry"}
        with self.assertRaisesRegex(ValueError, "incompatible roles"):
            GENERATOR.load_profile(PX4_PROFILE_PATH, SCHEMA_PATH, wrong_role)

    def test_registry_rejects_duplicate_message_ids(self):
        document = registry_document()
        document["messages"].append(dict(document["messages"][0]))
        self.registry_path.write_text(json.dumps(document), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "duplicate message ID"):
            GENERATOR.load_registry(self.registry_path)

    def test_endpoint_observes_policy_and_owned_message_identity_are_generated(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["channels"][0]["inputs"]["pose"].update(
            {
                "name": "external/pose",
                "type": "geometry_msgs/PoseWithCovarianceStamped",
                "scope": "global",
            }
        )
        profile["channels"][1]["policy"]["source_timeout_ms"] = 321
        next(
            channel
            for channel in profile["channels"]
            if channel["id"] == "diagnostic.offboard-input"
        )["observes"] = ["state.flight"]
        path = self.write_profile(profile)

        profiles = GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)
        channels = {
            channel["id"]: channel
            for channel in profiles["px4.multirotor.ros1.v7"]["channels"]
        }
        self.assertEqual(
            channels["state.pose"]["endpoints"][0],
            {
                "kind": "input",
                "role": "pose",
                "name_template": "external/pose",
                "ros_type": "geometry_msgs/PoseWithCovarianceStamped",
                "scope": "global",
            },
        )
        self.assertEqual(
            channels["state.mocap.pose"]["policy"]["source_timeout_ms"], 321
        )
        self.assertEqual(
            channels["diagnostic.offboard-input"]["observes"], ["state.flight"]
        )
        header = GENERATOR.generate(
            self.registry_fingerprint,
            self.messages,
            profiles,
            "fixture-robot-adapter",
            "fixture_robot_adapter",
        )
        self.assertIn(
            'EndpointKind::kInput, "pose", "external/pose", '
            '"geometry_msgs/PoseWithCovarianceStamped", EndpointScope::kGlobal',
            header,
        )
        self.assertIn(
            '"source_timeout_ms", PolicyValueKind::kInteger, "", 321LL',
            header,
        )
        self.assertIn('"operation.arm", ChannelKind::kOperation', header)
        self.assertIn('"arm", 3201u, 1u', header)

    def test_operation_contract_fields_are_strict(self):
        for field, value in (
            ("side_effect", "unknown"),
            ("idempotency", "optional"),
            ("cancellation_supported", True),
            ("deadline_required", False),
        ):
            with self.subTest(field=field):
                profile = yaml.safe_load(
                    PX4_PROFILE_PATH.read_text(encoding="utf-8")
                )
                profile["channels"][-1]["operation_contract"][field] = value
                path = self.write_profile(profile)
                with self.assertRaisesRegex(ValueError, "schema validation failed"):
                    GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_typed_policy_values_must_be_cxx14_representable(self):
        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["channels"][1]["policy"]["source_timeout_ms"] = 1 << 63
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "outside int64"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        next(
            channel
            for channel in profile["channels"]
            if channel["id"] == "diagnostic.offboard-input"
        )["policy"]["minimum_rate_hz"] = float("inf")
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "finite"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

        profile = yaml.safe_load(PX4_PROFILE_PATH.read_text(encoding="utf-8"))
        profile["channels"][0]["output_rate_hz"] = float("inf")
        path = self.write_profile(profile)
        with self.assertRaisesRegex(ValueError, "finite"):
            GENERATOR.load_profile(path, SCHEMA_PATH, self.messages)

    def test_generator_has_no_central_profile_allowlist(self):
        self.assertFalse(hasattr(GENERATOR, "PROFILE_FILES"))
        self.assertFalse(hasattr(GENERATOR, "EXPECTED_CHANNELS"))
        self.assertFalse(hasattr(GENERATOR, "EXPECTED_PROCESSORS"))


if __name__ == "__main__":
    unittest.main()
