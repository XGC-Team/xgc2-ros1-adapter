#!/usr/bin/env python3

import copy
import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TOOLS = REPOSITORY_ROOT / "tools"
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location(
    "runtime_manifest_generator", TOOLS / "generate_runtime_manifests.py"
)
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)
CONTRACT_GENERATOR = sys.modules["generate_contract_metadata"]
VERIFY_SPEC = importlib.util.spec_from_file_location(
    "runtime_manifest_verifier", TOOLS / "verify_runtime_manifests.py"
)
VERIFIER = importlib.util.module_from_spec(VERIFY_SPEC)
VERIFY_SPEC.loader.exec_module(VERIFIER)

SCHEMA = REPOSITORY_ROOT / "profiles/schema/robot-adapter-profile-v4.schema.json"
PX4_PROFILE = REPOSITORY_ROOT / "profiles/ros1/px4-multirotor-ros1-v6.yaml"
SCOUT_PROFILE = REPOSITORY_ROOT / "profiles/ros1/scout-mini-ros1-v4.yaml"
MECANUM_PROFILE = REPOSITORY_ROOT / "profiles/ros1/mecanum-ugv-ros1-v1.yaml"
ROS_NOETIC_ENVIRONMENT = {
    "CMAKE_PREFIX_PATH": "/opt/ros/noetic",
    "LD_LIBRARY_PATH": "/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/opt/ros/noetic/lib/aarch64-linux-gnu",
    "PATH": "/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
    "PKG_CONFIG_PATH": "/opt/ros/noetic/lib/pkgconfig",
    "PYTHONPATH": "/opt/ros/noetic/lib/python3/dist-packages",
    "ROS_DISTRO": "noetic",
    "ROS_ETC_DIR": "/opt/ros/noetic/etc/ros",
    "ROS_PACKAGE_PATH": "/opt/ros/noetic/share",
    "ROS_PYTHON_VERSION": "3",
    "ROS_ROOT": "/opt/ros/noetic/share/ros",
    "ROS_VERSION": "1",
}

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
    3201: "request",
    3202: "request",
    3203: "request",
    3204: "request",
    4001: "configuration",
    4002: "telemetry",
}
TYPE_NAMES = {
    1: "xgc.v1.Empty",
    2005: "xgc.semantic.common.v1.VehicleHealth",
    2006: "xgc.semantic.common.v1.SpeedEstimate",
    2007: "xgc.semantic.common.v1.DistanceEstimate",
    3001: "xgc.semantic.aerial.v1.FlightStatus",
    3102: "xgc.semantic.ground.v1.ChassisStatus",
    3201: "xgc.semantic.aerial.v1.ArmRequest",
    3202: "xgc.semantic.aerial.v1.ModeRequest",
    3203: "xgc.semantic.aerial.v1.AutopilotRebootRequest",
    3204: "xgc.semantic.ground.v1.MotionIntentRequest",
    4001: "xgc.robot.v1.RobotAdapterSpec",
    4002: "xgc.robot.v1.RobotMessage",
}


class RuntimeManifestGeneratorTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.temp = Path(temporary.name)
        self.registry = self.temp / "registry.json"
        self.registry.write_text(
            json.dumps(
                {
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
            ),
            encoding="utf-8",
        )

    def arguments(self, profile_file):
        return SimpleNamespace(
            executable="/bin/true",
            artifact_path="/usr/bin/fixture-adapter",
            registry=str(self.registry),
            profile_file=str(profile_file),
            profile_schema=str(SCHEMA),
            definition_id="fixture-robot-adapter",
            version="1.0.0",
            label="Fixture",
            description="Fixture Adapter",
        )

    def test_catalog_carries_exact_kind_and_semantic_traits(self):
        _, _, catalog = GENERATOR.build_documents(self.arguments(PX4_PROFILE))
        self.assertEqual(catalog["schema"], "xgc.robot.adapter-profile-catalog/v4")
        profile = catalog["profiles"][0]
        self.assertEqual(
            set(profile),
            {
                "profileId",
                "profileDigest",
                "providerDefinitionId",
                "robotKind",
                "parameters",
                "semantics",
                "channels",
            },
        )
        self.assertEqual(profile["robotKind"], "px4_multirotor")
        self.assertEqual(
            profile["parameters"]["namespace"],
            {
                "type": "string",
                "required": True,
                "delivery": "target_binding",
            },
        )
        self.assertEqual(
            profile["semantics"]["operations"],
            [
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
                {
                    "id": "reboot-autopilot",
                    "channelId": "operation.autopilot-reboot",
                    "timeoutMillis": 5000,
                    "parameterSchema": {
                        "type": "object",
                        "required": [],
                        "properties": {},
                        "additionalProperties": False,
                    },
                },
                {
                    "id": "set-flight-mode",
                    "channelId": "operation.mode",
                    "timeoutMillis": 5000,
                    "parameterSchema": {
                        "type": "object",
                        "required": ["mode"],
                        "properties": {
                            "mode": {
                                "type": "string",
                                "enum": [
                                    "OFFBOARD",
                                    "POSCTL",
                                    "ALTCTL",
                                    "STABILIZED",
                                ],
                            }
                        },
                        "additionalProperties": False,
                    },
                },
            ],
        )
        self.assertEqual(
            profile["semantics"]["onlineConditions"][0]["predicate"],
            "xgc.semantic.aerial.flight.connected",
        )
        operation_channels = {
            channel["id"]: channel
            for channel in profile["channels"]
            if channel["kind"] == "operation"
        }
        self.assertEqual(operation_channels["operation.arm"]["messageId"], 3201)

        _, _, scout_catalog = GENERATOR.build_documents(
            self.arguments(SCOUT_PROFILE)
        )
        scout = scout_catalog["profiles"][0]
        self.assertEqual(scout["robotKind"], "scout_mini")
        self.assertEqual(
            scout["semantics"]["operations"],
            [
                {
                    "id": "set-motion-intent",
                    "channelId": "operation.motion-intent",
                    "timeoutMillis": 1000,
                    "parameterSchema": {
                        "type": "object",
                        "required": ["gear", "longitudinal", "yaw"],
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
        self.assertEqual(
            [
                condition["channelId"]
                for condition in scout["semantics"]["onlineConditions"]
            ],
            ["state.health", "vrpn.position"],
        )
        self.assertEqual(
            scout["semantics"]["onlineConditions"][0]["predicate"],
            "xgc.semantic.common.vehicle-health.online",
        )

        _, _, mecanum_catalog = GENERATOR.build_documents(
            self.arguments(MECANUM_PROFILE)
        )
        mecanum = mecanum_catalog["profiles"][0]
        self.assertEqual(mecanum["profileId"], "mecanum-ugv.ros1.v1")
        self.assertEqual(mecanum["robotKind"], "mecanum_ugv")
        self.assertEqual(
            mecanum["semantics"]["onlineConditions"],
            [{"channelId": "vrpn.position", "maximumAgeMillis": 1000}],
        )
        self.assertEqual(
            [channel["id"] for channel in mecanum["channels"]],
            [
                "command.velocity",
                "diagnostic.channel-health",
                "operation.motion-intent",
                "vrpn.position",
                "vrpn.speed",
                "vrpn.velocity",
            ],
        )

    def test_profile_v4_contract_and_legacy_schema_are_explicit(self):
        self.assertEqual(
            CONTRACT_GENERATOR.PROFILE_SCHEMA_ID,
            "xgc.robot.adapter-profile/v4",
        )
        self.assertEqual(
            CONTRACT_GENERATOR.PROFILE_CONTRACT_DIGEST_SCHEMA,
            "xgc.robot.profile-contract-digest/v4",
        )

        legacy_schema = json.loads(
            (REPOSITORY_ROOT / "profiles/schema/robot-adapter-profile-v3.schema.json").read_text(
                encoding="utf-8"
            )
        )
        current_schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        legacy_parameter = legacy_schema["definitions"]["parameter"]
        current_parameter = current_schema["definitions"]["parameter"]
        self.assertEqual(
            legacy_schema["properties"]["schema"]["const"],
            "xgc.robot.adapter-profile/v3",
        )
        self.assertNotIn("delivery", legacy_parameter["properties"])
        self.assertNotIn("delivery", legacy_parameter["required"])
        self.assertEqual(
            current_schema["properties"]["schema"]["const"],
            "xgc.robot.adapter-profile/v4",
        )
        self.assertIn("delivery", current_parameter["required"])
        self.assertEqual(
            current_parameter["properties"]["delivery"]["enum"],
            ["target_binding", "core_local"],
        )
        for legacy in (
            "profiles/schema/robot-adapter-profile-v{}.schema.json".format(2),
            "profiles/ros1/px4-multirotor-ros1-v{}.yaml".format(4),
            "profiles/ros1/scout-mini-ros1-v{}.yaml".format(2),
        ):
            self.assertFalse((REPOSITORY_ROOT / legacy).exists(), legacy)

    def test_header_and_catalog_share_the_canonical_profile_digest(self):
        _, _, catalog = GENERATOR.build_documents(self.arguments(PX4_PROFILE))
        installed_profile = catalog["profiles"][0]
        profile_body = {
            key: value
            for key, value in installed_profile.items()
            if key != "profileDigest"
        }
        expected_digest = hashlib.sha256(
            json.dumps(
                {
                    "schema": "xgc.robot.profile-contract-digest/v4",
                    "profile": profile_body,
                },
                sort_keys=True,
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        self.assertEqual(installed_profile["profileDigest"], expected_digest)

        registry_fingerprint, messages = CONTRACT_GENERATOR.load_registry(
            self.registry
        )
        profiles = CONTRACT_GENERATOR.load_profile(
            PX4_PROFILE, SCHEMA, messages
        )
        header = CONTRACT_GENERATOR.generate(
            registry_fingerprint,
            messages,
            profiles,
            "fixture-robot-adapter",
            "fixture_robot_adapter",
        )
        self.assertIn(expected_digest, header)

    def test_capabilities_and_direct_process_environment_are_exact(self):
        px4_adapter, px4_process, px4_catalog = GENERATOR.build_documents(
            self.arguments(PX4_PROFILE)
        )
        self.assertEqual(
            {
                capability["ref"]["id"]
                for capability in px4_adapter["adapters"][0][
                    "capabilityManifest"
                ]["capabilities"]
            },
            {"xgc.robot.command", "xgc.robot.telemetry"},
        )
        command_capability = next(
            capability
            for capability in px4_adapter["adapters"][0]["capabilityManifest"][
                "capabilities"
            ]
            if capability["ref"]["id"] == "xgc.robot.command"
        )
        self.assertTrue(
            all(
                endpoint["defaultTimeoutMillis"] == 5000
                and endpoint["maximumTimeoutMillis"] == 5000
                for endpoint in command_capability["endpoints"]
            )
        )
        endpoints = {
            endpoint["endpointId"]: endpoint
            for endpoint in command_capability["endpoints"]
        }
        profile_operations = {
            operation["id"]: operation
            for operation in px4_catalog["profiles"][0]["semantics"][
                "operations"
            ]
        }
        self.assertEqual(set(profile_operations), set(endpoints))
        for operation_id, operation in profile_operations.items():
            self.assertEqual(
                operation["timeoutMillis"],
                endpoints[operation_id]["defaultTimeoutMillis"],
            )
            self.assertLessEqual(
                operation["timeoutMillis"],
                endpoints[operation_id]["maximumTimeoutMillis"],
            )
        self.assertEqual(endpoints["arm"]["sideEffect"], "idempotent")
        self.assertEqual(
            endpoints["reboot-autopilot"]["sideEffect"], "non-idempotent"
        )
        self.assertEqual(endpoints["arm"]["outputSchema"]["messageId"], 1)
        self.assertEqual(endpoints["arm"]["idempotency"], "required")
        self.assertFalse(endpoints["arm"]["cancellationSupported"])
        command = px4_process["definitions"][0]["command"]
        self.assertEqual(
            command,
            {
                "executable": "/usr/bin/fixture-adapter",
                "args": [
                    "--adapter-bootstrap-file",
                    "${adapterBootstrapFile}",
                ],
                "directExecutable": True,
                "env": ROS_NOETIC_ENVIRONMENT,
            },
        )

        scout_adapter, scout_process, _ = GENERATOR.build_documents(
            self.arguments(SCOUT_PROFILE)
        )
        self.assertEqual(
            {
                capability["ref"]["id"]
                for capability in scout_adapter["adapters"][0][
                    "capabilityManifest"
                ]["capabilities"]
            },
            {"xgc.robot.command", "xgc.robot.telemetry"},
        )
        scout_command = next(
            capability
            for capability in scout_adapter["adapters"][0][
                "capabilityManifest"
            ]["capabilities"]
            if capability["ref"]["id"] == "xgc.robot.command"
        )
        self.assertEqual(len(scout_command["endpoints"]), 1)
        self.assertEqual(
            scout_command["endpoints"][0]["endpointId"],
            "set-motion-intent",
        )
        self.assertEqual(
            scout_command["endpoints"][0]["inputSchema"]["messageId"], 3204
        )
        self.assertEqual(
            scout_command["endpoints"][0]["defaultTimeoutMillis"], 1000
        )
        self.assertEqual(
            scout_process["definitions"][0]["command"]["env"],
            ROS_NOETIC_ENVIRONMENT,
        )

    def test_artifact_and_manifest_digests_are_deterministic(self):
        executable = self.temp / "fixture-adapter"
        executable.write_bytes(b"fixture-adapter-v1\n")
        arguments = self.arguments(PX4_PROFILE)
        arguments.executable = str(executable)

        first = GENERATOR.build_documents(arguments)
        second = GENERATOR.build_documents(arguments)
        self.assertEqual(first, second)
        expected_build_digest = "sha256:" + hashlib.sha256(
            executable.read_bytes()
        ).hexdigest()
        self.assertEqual(
            first[0]["adapters"][0]["definition"]["buildDigest"],
            expected_build_digest,
        )

        executable.write_bytes(b"fixture-adapter-changed\n")
        changed = GENERATOR.build_documents(arguments)
        self.assertNotEqual(
            changed[0]["adapters"][0]["definition"]["buildDigest"],
            expected_build_digest,
        )
        self.assertEqual(
            changed[0]["adapters"][0]["definition"][
                "trustedManifestDigest"
            ],
            first[0]["adapters"][0]["definition"][
                "trustedManifestDigest"
            ],
        )
        self.assertEqual(
            changed[0]["adapters"][0]["capabilityManifest"],
            first[0]["adapters"][0]["capabilityManifest"],
        )
        self.assertEqual(changed[1:], first[1:])

    def test_installed_package_check_uses_the_current_verifier_cli(self):
        script = (
            REPOSITORY_ROOT / ".xgc2/scripts/check_installed_packages.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("--profile-schema", script)
        self.assertNotIn("--with-commands", script)

    def test_ground_adapters_stop_motion_before_ros_transport_shutdown(self):
        for package, node_source in (
            ("scout", "xgc_scout_mini_ros1_adapter/src/scout_mini_ros1_adapter_node.cpp"),
            ("mecanum", "xgc_mecanum_ugv_ros1_adapter/src/mecanum_ugv_ros1_adapter_node.cpp"),
        ):
            with self.subTest(package=package):
                source = (REPOSITORY_ROOT / "src" / node_source).read_text(encoding="utf-8")
                self.assertIn("client_.reset();", source)
                self.assertLess(source.index("node.Shutdown();"), source.index("ros::shutdown();"))

    def test_duplicate_operation_identity_is_rejected_without_fallback(self):
        profile = yaml.safe_load(PX4_PROFILE.read_text(encoding="utf-8"))
        duplicate_arm = copy.deepcopy(
            next(
                channel
                for channel in profile["channels"]
                if channel.get("operation_id") == "arm"
            )
        )
        duplicate_arm["id"] = "operation.arm-duplicate"
        profile["channels"].append(duplicate_arm)
        path = self.temp / "duplicate-operation.yaml"
        path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "duplicate operation identity|operation identities must be unique"):
            GENERATOR.build_documents(self.arguments(path))

    def test_unknown_operation_contract_is_rejected_without_fallback(self):
        profile = yaml.safe_load(PX4_PROFILE.read_text(encoding="utf-8"))
        extra = copy.deepcopy(
            next(channel for channel in profile["channels"] if channel["id"] == "operation.arm")
        )
        extra["id"] = "operation.arm-secondary"
        extra["operation_id"] = "arm-secondary"
        extra["operation_contract"]["side_effect"] = "non-idempotent"
        profile["channels"].append(extra)
        path = self.temp / "extra-operation.yaml"
        path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
        with self.assertRaisesRegex(
            ValueError, "schema validation failed|no owned parameter contract"
        ):
            GENERATOR.build_documents(self.arguments(path))

    def test_native_operation_timeout_limit_is_enforced_before_manifest_generation(self):
        profile = yaml.safe_load(PX4_PROFILE.read_text(encoding="utf-8"))
        profile["channels"][-1]["policy"]["timeout_ms"] = 5001
        path = self.temp / "unsafe-timeout.yaml"
        path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "schema validation failed|safety limit"):
            GENERATOR.build_documents(self.arguments(path))

    def test_verifier_rejects_every_profile_identity_and_semantics_mutation(self):
        adapter, process, catalog = GENERATOR.build_documents(
            self.arguments(PX4_PROFILE)
        )
        adapter_path = self.temp / "adapter.json"
        process_path = self.temp / "process.json"
        profile_path = self.temp / "profile.json"
        adapter_path.write_text(json.dumps(adapter), encoding="utf-8")
        process_path.write_text(json.dumps(process), encoding="utf-8")
        profile_path.write_text(json.dumps(catalog), encoding="utf-8")
        arguments = SimpleNamespace(
            executable="/bin/true",
            artifact_path="/usr/bin/fixture-adapter",
            definition_id="fixture-robot-adapter",
            registry=str(self.registry),
            profile_file=str(PX4_PROFILE),
            profile_schema=str(SCHEMA),
            adapter_manifest=str(adapter_path),
            process_manifest=str(process_path),
            profile_catalog=str(profile_path),
        )
        VERIFIER.verify(arguments)

        for field, value in (
            ("artifact", "/tmp/removed-legacy-field"),
            ("placement", {"mode": "managed-host"}),
        ):
            with self.subTest("removed definition field " + field):
                tampered_adapter = copy.deepcopy(adapter)
                tampered_adapter["adapters"][0]["definition"][field] = value
                adapter_path.write_text(json.dumps(tampered_adapter), encoding="utf-8")
                with self.assertRaises(ValueError):
                    VERIFIER.verify(arguments)
        adapter_path.write_text(json.dumps(adapter), encoding="utf-8")

        tampered_adapter = copy.deepcopy(adapter)
        installed = tampered_adapter["adapters"][0]
        telemetry = next(
            capability
            for capability in installed["capabilityManifest"]["capabilities"]
            if capability["ref"]["id"] == "xgc.robot.telemetry"
        )
        telemetry["endpoints"][0]["endpointId"] = "not-telemetry"
        telemetry["contractDigest"] = VERIFIER.digest(
            VERIFIER.canonical(
                {"ref": telemetry["ref"], "endpoints": telemetry["endpoints"]}
            )
        )
        canonical_manifest = {
            "formatVersion": installed["capabilityManifest"]["formatVersion"],
            "capabilities": sorted(
                installed["capabilityManifest"]["capabilities"],
                key=lambda capability: "{}@{}".format(
                    capability["ref"]["id"], capability["ref"]["version"]
                ),
            ),
        }
        installed["definition"]["trustedManifestDigest"] = VERIFIER.digest(
            VERIFIER.canonical(canonical_manifest)
        )
        adapter_path.write_text(json.dumps(tampered_adapter), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "exact native contract"):
            VERIFIER.verify(arguments)
        adapter_path.write_text(json.dumps(adapter), encoding="utf-8")

        for name, mutate in (
            (
                "missing ROS environment",
                lambda command: command["env"].pop("ROS_ROOT"),
            ),
            (
                "additional inherited environment",
                lambda command: command["env"].update({"HOME": "/root"}),
            ),
            (
                "shell executable",
                lambda command: command.update({"executable": "/bin/sh"}),
            ),
            (
                "source through a shell argument",
                lambda command: command.update(
                    {"args": ["-c", "source /opt/ros/noetic/setup.bash"]}
                ),
            ),
        ):
            with self.subTest(name):
                tampered_process = copy.deepcopy(process)
                mutate(tampered_process["definitions"][0]["command"])
                process_path.write_text(
                    json.dumps(tampered_process), encoding="utf-8"
                )
                with self.assertRaises(ValueError):
                    VERIFIER.verify(arguments)
        process_path.write_text(json.dumps(process), encoding="utf-8")

        def mutate_profile_id(profile):
            profile["profileId"] = "tampered.profile.v1"

        def mutate_profile_digest(profile):
            profile["profileDigest"] = "0" * 64

        def mutate_default_staleness(profile):
            profile["semantics"]["defaultStaleAfterMillis"] += 1

        def mutate_channel_staleness(profile):
            profile["semantics"]["channelStaleAfterMillis"]["state.pose"] += 1

        def mutate_online(profile):
            profile["semantics"]["onlineConditions"][0]["maximumAgeMillis"] += 1

        def mutate_ready(profile):
            profile["semantics"]["operationalReadyConditions"].pop()

        def mutate_channel_schema(profile):
            profile["channels"][0]["schemaFingerprint"] += 1

        def mutate_robot_kind(profile):
            profile["robotKind"] = "scout_mini"

        def mutate_parameter_contract(profile):
            profile["parameters"]["namespace"]["required"] = False

        def mutate_parameter_delivery(profile):
            profile["parameters"]["namespace"]["delivery"] = "core_local"

        def mutate_removed_namespace_role(profile):
            profile["namespaceParameter"] = "ros_ns"

        def mutate_channel_kind(profile):
            profile["channels"][0]["kind"] = "operation"

        def mutate_operation_parameter_schema(profile):
            profile["semantics"]["operations"][0]["parameterSchema"][
                "legacy"
            ] = True

        def mutate_operation_timeout(profile):
            profile["semantics"]["operations"][0]["timeoutMillis"] += 1

        for name, mutate in (
            ("profileId", mutate_profile_id),
            ("profile digest", mutate_profile_digest),
            ("default staleness", mutate_default_staleness),
            ("channel staleness", mutate_channel_staleness),
            ("online condition", mutate_online),
            ("operational ready condition", mutate_ready),
            ("channel schema", mutate_channel_schema),
            ("robot kind", mutate_robot_kind),
            ("parameter contract", mutate_parameter_contract),
            ("parameter delivery", mutate_parameter_delivery),
            ("removed namespace parameter role", mutate_removed_namespace_role),
            ("channel kind", mutate_channel_kind),
            ("operation parameter schema", mutate_operation_parameter_schema),
            ("operation timeout", mutate_operation_timeout),
        ):
            with self.subTest(name):
                tampered = copy.deepcopy(catalog)
                mutate(tampered["profiles"][0])
                profile_path.write_text(json.dumps(tampered), encoding="utf-8")
                with self.assertRaises(ValueError):
                    VERIFIER.verify(arguments)

        tampered = copy.deepcopy(catalog)
        mutate_channel_schema(tampered["profiles"][0])
        profile_path.write_text(json.dumps(tampered), encoding="utf-8")
        optimized = subprocess.run(
            [
                sys.executable,
                "-O",
                str(TOOLS / "verify_runtime_manifests.py"),
                "--executable",
                "/bin/true",
                "--artifact-path",
                "/usr/bin/fixture-adapter",
                "--definition-id",
                "fixture-robot-adapter",
                "--registry",
                str(self.registry),
                "--profile-file",
                str(PX4_PROFILE),
                "--profile-schema",
                str(SCHEMA),
                "--adapter-manifest",
                str(adapter_path),
                "--process-manifest",
                str(process_path),
                "--profile-catalog",
                str(profile_path),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(
            optimized.returncode,
            0,
            "optimized Python disabled manifest verification: {}".format(
                optimized.stdout
            ),
        )


if __name__ == "__main__":
    unittest.main()
