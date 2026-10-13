#!/usr/bin/env python3
"""The shipped process definitions must match the executables they start."""

import json
import re
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCES = REPOSITORY_ROOT / "src"
SCHEMA_ID = "xgc.execution.process/v1"
DEFINITION_FILES = sorted(SOURCES.glob("*/process/*.json"))
KNOWN_KEYS = {
    "id", "version", "label", "description", "parameters", "setupFiles", "command",
    "services", "readiness", "stop", "logs",
}
OPTIONS_SOURCE = SOURCES / "common" / "src" / "robot_server_options.cpp"
TOOLS_SOURCE = SOURCES / "ros1_tools_adapter" / "src" / "main.cpp"
ROBOT_PACKAGES = {
    "xgc_px4_multirotor_ros1_adapter": ["xgc2-px4-multirotor-ros1-adapter"],
    "xgc_ugv_ros1_adapter": [
        "xgc2-scout-mini-ros1-adapter",
        "xgc2-mecanum-ugv-ros1-adapter",
    ],
}


def flags_of(path, marker):
    """The command-line flags a source accepts: the quoted --flags after marker."""
    text = path.read_text(encoding="utf-8")
    block = text[text.index(marker):]
    block = block[: block.index("}")]
    return set(re.findall(r'"(--[a-z-]+)"', block))


def load(path):
    document = json.loads(path.read_text(encoding="utf-8"))
    assert document["apiVersion"] == SCHEMA_ID, path
    assert len(document["definitions"]) == 1, path
    return document["definitions"][0]


class ProcessDefinitionTest(unittest.TestCase):
    def test_every_adapter_ships_its_definitions(self):
        names = {path.stem for path in DEFINITION_FILES}
        self.assertEqual(
            names,
            {
                "xgc2-px4-multirotor-ros1-adapter",
                "xgc2-scout-mini-ros1-adapter",
                "xgc2-mecanum-ugv-ros1-adapter",
                "xgc2-ros1-tools-adapter",
                "xgc2-ros1-scene-adapter",
            },
        )
        for path in DEFINITION_FILES:
            with self.subTest(definition=path.stem):
                self.assertEqual(load(path)["id"], path.stem)

    def test_definitions_use_the_current_schema_and_only_current_fields(self):
        for path in DEFINITION_FILES:
            with self.subTest(definition=path.stem):
                document = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(document["apiVersion"], SCHEMA_ID)
                self.assertEqual(set(document), {"apiVersion", "definitions"})
                definition = load(path)
                self.assertLessEqual(set(definition), KNOWN_KEYS)
                self.assertTrue(
                    {"id", "version", "label", "command", "parameters", "services",
                     "readiness", "stop"} <= set(definition)
                )
                self.assertEqual(definition["version"], "0.7.0")
                for fields, allowed in (
                    (definition["parameters"], {"properties", "required", "groups"}),
                    (definition["command"], {"executable", "args", "workDir", "env", "stdinParameter"}),
                    (definition["readiness"], {"kind", "startGraceMs", "timeoutMs", "address", "masterUri"}),
                    (definition["stop"], {"graceMs", "rpc"}),
                    (definition.get("logs", {}), {"maxBytes", "files"}),
                ):
                    self.assertLessEqual(set(fields), allowed)
                for parameter in definition["parameters"]["properties"].values():
                    self.assertLessEqual(
                        set(parameter),
                        {"type", "default", "enum", "minimum", "maximum", "sensitive", "output", "fixedOnly", "description"},
                    )
                self.assertEqual(definition["readiness"]["kind"], "describe")
                readiness = definition["readiness"]
                self.assertGreater(readiness["startGraceMs"], 0)
                self.assertLessEqual(readiness["startGraceMs"], readiness["timeoutMs"])
                self.assertLessEqual(readiness["timeoutMs"], 30000 + 1000)

    def test_the_command_is_the_installed_executable_started_directly(self):
        for path in DEFINITION_FILES:
            with self.subTest(definition=path.stem):
                command = load(path)["command"]
                executable = command["executable"]
                match = re.fullmatch(r"/opt/ros/noetic/lib/([a-z0-9_]+)/([a-z0-9_]+)", executable)
                self.assertIsNotNone(match, executable)
                package, binary = match.groups()
                self.assertTrue((SOURCES / "common").is_dir())
                package_xml = [
                    candidate
                    for candidate in SOURCES.glob("*/package.xml")
                    if "<name>{}</name>".format(package) in candidate.read_text(encoding="utf-8")
                ]
                self.assertEqual(len(package_xml), 1, package)
                cmake = (package_xml[0].parent / "CMakeLists.txt").read_text(encoding="utf-8")
                if path.stem == "xgc2-ros1-scene-adapter":
                    self.assertEqual(package, "xgc_scene_ros1_adapter")
                    self.assertEqual(binary, "xgc2_scene_adapter")
                    self.assertIn("install(TARGETS xgc2_scene_adapter", cmake)
                else:
                    self.assertEqual(binary, package + "_node")
                    self.assertIn("add_executable(${PROJECT_NAME}_node", cmake)
                # No wrapper: no interpreter, no shell, no rosrun/roslaunch.
                self.assertNotIn("-c", command["args"])
                self.assertNotIn("env", command)

    def test_arguments_are_exactly_the_flags_the_executable_parses(self):
        robot_flags = flags_of(OPTIONS_SOURCE, "static const std::set<std::string> flags")
        tools_flags = flags_of(TOOLS_SOURCE, "const std::set<std::string> flags")
        self.assertEqual(
            robot_flags,
            {"--socket-path", "--run-id", "--node-name", "--provider", "--target-id",
             "--ros-master-uri", "--ros-ip"},
        )
        for path in DEFINITION_FILES:
            with self.subTest(definition=path.stem):
                definition = load(path)
                args = definition["command"]["args"]
                if path.stem == "xgc2-ros1-scene-adapter":
                    self.assertIn("--session-stdin", args)
                    self.assertEqual(args.count("--session-stdin"), 1)
                    remappings = dict(argument.split(":=", 1) for argument in args if ":=" in argument)
                    native_args = [argument for argument in args
                                   if ":=" not in argument and argument != "--session-stdin"]
                    self.assertEqual(len(remappings) + len(native_args) + 1, len(args))
                    self.assertEqual(len(native_args) % 2, 0)
                    self.assertEqual(len(native_args[0::2]), len(set(native_args[0::2])))
                    self.assertEqual(set(native_args[0::2]), robot_flags - {"--provider"})
                    self.assertEqual(dict(zip(native_args[0::2], native_args[1::2])), {
                        "--socket-path": "${socketPath}",
                        "--target-id": "${targetId}",
                        "--run-id": "${runId}",
                        "--node-name": "xgc_scene_${runId}",
                        "--ros-master-uri": "${rosMasterUri}",
                        "--ros-ip": "${rosIp}",
                    })
                    scene_main = (SOURCES / "scene_ros1_adapter" / "src" / "node_main.cpp").read_text(encoding="utf-8")
                    self.assertIn('args.insert(args.end(), {"--provider", "xgc2-scene-adapter"});', scene_main)
                    self.assertIn("adapter::ParseServerOptions(", scene_main)
                    self.assertEqual(remappings, {
                        "__ns": "${namespace}",
                        "__master": "${rosMasterUri}",
                        "__ip": "${rosIp}",
                        "_scene_file": "${sceneFile}",
                        "_working_file": "${workingFile}",
                        "_generation_parameters_json": "${generationParametersJson}",
                        "_save_directory": "${saveDirectory}",
                        "_target_id": "${targetId}",
                        "_xrpc_socket": "${socketPath}",
                        "_frozen": "${frozen}",
                        "_profile": "${profile}",
                        "_simulation_endpoint": "${simulationEndpoint}",
                        "_lidar_xrpc_socket": "${lidarSocketPath}",
                        "_lidar_enabled": "${lidarEnabled}",
                        "_lidar_defaults_json": "${lidarDefaultsJson}",
                    })
                    declared = definition["parameters"]["properties"]
                    used = set(re.findall(r"\$\{([A-Za-z0-9]+)\}", " ".join(args)))
                    stdin_parameter = definition["command"]["stdinParameter"]
                    self.assertEqual(stdin_parameter, "sessionJson")
                    self.assertIn(stdin_parameter, declared)
                    self.assertEqual(declared[stdin_parameter]["type"], "string")
                    used.add(stdin_parameter)
                    self.assertEqual(used, set(declared))
                    self.assertEqual(
                        set(definition["parameters"]["required"]),
                        {"socketPath", "runId", "lidarSocketPath", "targetId", "rosMasterUri", "sceneFile"},
                    )
                    self.assertEqual(declared["profile"]["enum"], ["gazebo", "xsim", "physical"])
                    self.assertEqual(set(declared), {
                        "socketPath", "runId", "lidarSocketPath", "targetId", "rosMasterUri", "rosIp", "namespace",
                        "sceneFile", "workingFile", "generationParametersJson", "saveDirectory", "frozen",
                        "profile", "simulationEndpoint", "sessionJson", "lidarEnabled", "lidarDefaultsJson",
                    })
                    self.assertEqual(declared["simulationEndpoint"]["type"], "string")
                    self.assertTrue(declared["simulationEndpoint"]["fixedOnly"])
                    self.assertEqual(declared["simulationEndpoint"]["default"], "")
                    self.assertIn("$session.endpoints.world", declared["simulationEndpoint"]["description"])
                    self.assertIn("discovers the instance itself", declared["simulationEndpoint"]["description"])
                    self.assertEqual(declared["sessionJson"]["default"], "")
                    self.assertIn("unchanged", declared["sessionJson"]["description"])
                    self.assertEqual(declared["lidarEnabled"], {"type": "boolean", "default": True})
                    self.assertEqual(declared["lidarDefaultsJson"], {"type": "string", "default": "{}"})
                    continue
                flags = args[0::2]
                values = args[1::2]
                self.assertEqual(len(args) % 2, 0)
                accepted = tools_flags if path.stem == "xgc2-ros1-tools-adapter" else robot_flags
                self.assertEqual(set(flags), accepted)
                self.assertEqual(len(flags), len(set(flags)))
                declared = definition["parameters"]["properties"]
                used = set()
                for value in values:
                    used.update(re.findall(r"\$\{([A-Za-z0-9]+)\}", value))
                self.assertLessEqual(used, set(declared))
                self.assertEqual(used, set(declared))
                self.assertLessEqual(set(definition["parameters"]["required"]), set(declared))
                arguments = dict(zip(flags, values))
                self.assertEqual(arguments["--run-id"], "${runId}")
                self.assertTrue(arguments["--node-name"].endswith("${runId}"))
                self.assertEqual(arguments["--socket-path"], "${socketPath}")
                if "--provider" in arguments:
                    self.assertEqual(arguments["--provider"], definition["id"])

    def test_providers_are_served_by_the_executable_they_name(self):
        ugv = (SOURCES / "ugv_ros1_adapter" / "src" / "variant.cpp").read_text(encoding="utf-8")
        px4 = (SOURCES / "px4_multirotor_ros1_adapter" / "src"
               / "px4_multirotor_ros1_adapter_node.cpp").read_text(encoding="utf-8")
        for package, providers in ROBOT_PACKAGES.items():
            for provider in providers:
                with self.subTest(provider=provider):
                    path = SOURCES.glob("*/process/{}.json".format(provider))
                    definition = load(next(path))
                    self.assertIn(
                        "/lib/{}/".format(package), definition["command"]["executable"]
                    )
                    self.assertIn('"{}"'.format(provider), ugv if "ugv" in package else px4)

    def test_services_are_published_on_the_socket_parameter(self):
        for path in DEFINITION_FILES:
            with self.subTest(definition=path.stem):
                definition = load(path)
                endpoints = set()
                for service in definition["services"]:
                    self.assertEqual(set(service), {"service", "api_version", "profile", "endpointParameter"})
                    self.assertNotIn(service["endpointParameter"], endpoints)
                    endpoints.add(service["endpointParameter"])
                    socket = definition["parameters"]["properties"][service["endpointParameter"]]
                    self.assertEqual(socket, {"type": "string", "fixedOnly": True})
                    self.assertEqual(
                        service["profile"],
                        "http.v1" if path.stem in {"xgc2-ros1-tools-adapter", "xgc2-ros1-scene-adapter"} else "grpc.v1",
                    )
                if path.stem == "xgc2-ros1-scene-adapter":
                    self.assertEqual(definition["services"], [
                        {"service": "xgc2.scene.authoring", "api_version": "v1", "profile": "http.v1", "endpointParameter": "socketPath"},
                        {"service": "xgc2.sensor.lidar", "api_version": "v1", "profile": "http.v1", "endpointParameter": "lidarSocketPath"},
                    ])
                else:
                    self.assertEqual(len(definition["services"]), 1)

    def test_stopping_asks_the_server_to_drain_before_the_signal(self):
        for path in DEFINITION_FILES:
            with self.subTest(definition=path.stem):
                stop = load(path)["stop"]
                self.assertGreater(stop["graceMs"], 0)
                if path.stem in {"xgc2-ros1-tools-adapter", "xgc2-ros1-scene-adapter"}:
                    self.assertNotIn("rpc", stop)
                else:
                    self.assertEqual(stop["rpc"], "Shutdown")
                    # The drain budgets of the server fit in the grace period.
                    self.assertGreaterEqual(stop["graceMs"], 22500)


if __name__ == "__main__":
    unittest.main()
