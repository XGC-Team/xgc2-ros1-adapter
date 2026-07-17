#!/usr/bin/env python3
"""Generate exact installed manifests for one robot Adapter executable."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from generate_contract_metadata import (
    catalog_profile_body,
    load_profile,
    load_registry,
    profile_contract_digest,
)


ROBOT_CONFIGURATION_MESSAGE_ID = 4001
ROBOT_TELEMETRY_MESSAGE_ID = 4002
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


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )


def sha256_bytes(value: bytes) -> str:
    return "sha256:" + hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def schema_reference(messages: dict[int, dict[str, Any]], message_id: int) -> dict[str, Any]:
    message = messages.get(message_id)
    if message is None:
        raise ValueError("message ID {} is not registered".format(message_id))
    return {
        "messageId": message_id,
        "typeName": message["type"],
        "schemaVersion": message["version"],
        "schemaFingerprint": message["fingerprint"],
    }


def operation_endpoint(
    endpoint_id: str,
    input_schema: dict[str, Any],
    output_schema: dict[str, Any],
    side_effect: str,
    idempotency: str,
    cancellation_supported: bool,
    deadline_required: bool,
    default_timeout_ms: int,
    maximum_timeout_ms: int,
) -> dict[str, Any]:
    return {
        "endpointId": endpoint_id,
        "interaction": "operation",
        "inputSchema": input_schema,
        "outputSchema": output_schema,
        "sideEffect": side_effect,
        "idempotency": idempotency,
        "cancellationSupported": cancellation_supported,
        "deadlineRequired": deadline_required,
        "defaultTimeoutMillis": default_timeout_ms,
        "maximumTimeoutMillis": maximum_timeout_ms,
        "limits": {
            "maximumRequestBytes": 65536,
            "maximumResponseBytes": 65536,
            "maximumConcurrency": 32,
            "maximumStreams": 0,
            "maximumStreamChunkBytes": 0,
            "maximumStreamChunkMessages": 0,
        },
    }


def telemetry_endpoint(output_schema: dict[str, Any]) -> dict[str, Any]:
    return {
        "endpointId": "telemetry",
        "interaction": "stream-source",
        "outputSchema": output_schema,
        "sideEffect": "read-only",
        "idempotency": "not-supported",
        "cancellationSupported": True,
        "deadlineRequired": False,
        "defaultTimeoutMillis": 30000,
        "maximumTimeoutMillis": 86400000,
        "limits": {
            "maximumRequestBytes": 4096,
            "maximumResponseBytes": 1048576,
            "maximumConcurrency": 128,
            "maximumStreams": 1024,
            "maximumStreamChunkBytes": 8388608,
            "maximumStreamChunkMessages": 1024,
        },
    }


def contract(capability_id: str, endpoints: list[dict[str, Any]]) -> dict[str, Any]:
    endpoints = sorted(endpoints, key=lambda item: item["endpointId"])
    body = {
        "ref": {"id": capability_id, "version": 1},
        "endpoints": endpoints,
    }
    return {
        "ref": body["ref"],
        "contractDigest": sha256_bytes(canonical_json(body)),
        "endpoints": endpoints,
    }


def validate_profile_operation_endpoints(
    profile_body: dict[str, Any], capability_manifest: dict[str, Any]
) -> None:
    profile_operations = {
        operation["id"]: operation
        for operation in profile_body["semantics"]["operations"]
    }
    command_capability = next(
        (
            capability
            for capability in capability_manifest["capabilities"]
            if capability["ref"]["id"] == "xgc.robot.command"
        ),
        None,
    )
    endpoints = {
        endpoint["endpointId"]: endpoint
        for endpoint in (
            command_capability["endpoints"] if command_capability else []
        )
    }
    if set(profile_operations) != set(endpoints):
        raise ValueError(
            "Profile operations and provider command endpoints disagree"
        )
    for operation_id, operation in profile_operations.items():
        endpoint = endpoints[operation_id]
        timeout = operation["timeoutMillis"]
        if (
            not isinstance(timeout, int)
            or isinstance(timeout, bool)
            or timeout <= 0
            or timeout != endpoint["defaultTimeoutMillis"]
            or timeout > endpoint["maximumTimeoutMillis"]
        ):
            raise ValueError(
                "Profile operation {} timeout disagrees with its provider endpoint".format(
                    operation_id
                )
            )


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def build_documents(args: argparse.Namespace) -> tuple[dict[str, Any], ...]:
    executable = Path(args.executable)
    if not executable.is_file():
        raise ValueError("Adapter executable does not exist: {}".format(executable))

    registry_fingerprint, messages = load_registry(Path(args.registry))
    del registry_fingerprint
    profile_file = Path(args.profile_file)
    profiles = load_profile(profile_file, Path(args.profile_schema), messages)
    if len(profiles) != 1:
        raise ValueError("one robot profile is required")
    _, generated_profile = next(iter(profiles.items()))
    operation_channels = sorted(
        [
            channel
            for channel in generated_profile["channels"]
            if channel["kind"] == "operation"
        ],
        key=lambda channel: channel["operation_id"],
    )

    expected_types = {
        ROBOT_CONFIGURATION_MESSAGE_ID: "xgc.robot.v1.RobotAdapterSpec",
        ROBOT_TELEMETRY_MESSAGE_ID: "xgc.robot.v1.RobotMessage",
    }
    for message_id, type_name in expected_types.items():
        actual = messages.get(message_id, {}).get("type")
        if actual != type_name:
            raise ValueError(
                "message ID {} must be {}, got {}".format(
                    message_id, type_name, actual or "<missing>"
                )
            )

    telemetry = contract(
        "xgc.robot.telemetry",
        [telemetry_endpoint(schema_reference(messages, ROBOT_TELEMETRY_MESSAGE_ID))],
    )
    contracts = [telemetry]
    if operation_channels:
        contracts.append(
            contract(
                "xgc.robot.command",
                [
                    operation_endpoint(
                        channel["operation_id"],
                        schema_reference(messages, channel["input_message_id"]),
                        schema_reference(messages, channel["output_message_id"]),
                        channel["operation_contract"]["side_effect"],
                        channel["operation_contract"]["idempotency"],
                        channel["operation_contract"]["cancellation_supported"],
                        channel["operation_contract"]["deadline_required"],
                        channel["operation_timeout_millis"],
                        channel["operation_timeout_millis"],
                    )
                    for channel in operation_channels
                ],
            )
        )
    contracts.sort(key=lambda item: "{}@{}".format(item["ref"]["id"], item["ref"]["version"]))
    capability_manifest = {"formatVersion": 1, "capabilities": contracts}
    manifest_digest = sha256_bytes(canonical_json(capability_manifest))

    adapter_manifest = {
        "apiVersion": "xgc.adapter.definition/v1",
        "adapters": [
            {
                "definition": {
                    "id": args.definition_id,
                    "version": args.version,
                    "processDefinitionId": args.definition_id,
                    "buildDigest": sha256_file(executable),
                    "trustedManifestDigest": manifest_digest,
                    "configuration": {
                        "schema": schema_reference(
                            messages, ROBOT_CONFIGURATION_MESSAGE_ID
                        ),
                        "allowedEncodings": ["protobuf"],
                    },
                    "activation": {
                        "mode": "on-demand",
                        "idleTimeoutNanos": 300000000000,
                    },
                    "scope": {
                        "kind": "robot-group",
                        "requiredAttributes": ["provider", "run-id", "target-id"],
                        "allowAdditionalAttributes": False,
                        "sharing": "shared",
                    },
                },
                "capabilityManifest": capability_manifest,
            }
        ],
    }

    process_manifest = {
        "apiVersion": "xgc.execution.process/v1",
        "definitions": [
            {
                "id": args.definition_id,
                "version": args.version,
                "label": args.label,
                "description": args.description,
                "drivers": ["host"],
                "parameters": {
                    "properties": {
                        "adapterBootstrapFile": {
                            "type": "string",
                            "description": "Trusted mode-0600 binary AdapterProcessBootstrap path.",
                        }
                    },
                    "required": ["adapterBootstrapFile"],
                    "additionalProperties": False,
                },
                "command": {
                    "executable": args.artifact_path,
                    "args": ["--adapter-bootstrap-file", "${adapterBootstrapFile}"],
                    "directExecutable": True,
                    "env": dict(ROS_NOETIC_ENVIRONMENT),
                },
                "readiness": {"kind": "process"},
                "liveness": {"kind": "process"},
                "stop": {"gracePeriod": 10000000000},
                "restart": {"mode": "never"},
                "internal": True,
            }
        ],
    }

    profile_body = catalog_profile_body(
        generated_profile, messages, args.definition_id
    )
    validate_profile_operation_endpoints(profile_body, capability_manifest)
    installed_profile = {
        "profileId": profile_body["profileId"],
        "profileDigest": profile_contract_digest(profile_body),
        **{
            key: value
            for key, value in profile_body.items()
            if key != "profileId"
        },
    }
    profile_catalog = {
        "schema": "xgc.robot.adapter-profile-catalog/v3",
        "profiles": [installed_profile],
    }
    return adapter_manifest, process_manifest, profile_catalog


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--artifact-path", required=True)
    parser.add_argument("--registry", required=True)
    parser.add_argument("--profile-file", required=True)
    parser.add_argument("--profile-schema", required=True)
    parser.add_argument("--definition-id", required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--description", required=True)
    parser.add_argument("--adapter-output", required=True)
    parser.add_argument("--process-output", required=True)
    parser.add_argument("--profile-output", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    adapter, process, profile = build_documents(args)
    write_json(Path(args.adapter_output), adapter)
    write_json(Path(args.process_output), process)
    write_json(Path(args.profile_output), profile)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
