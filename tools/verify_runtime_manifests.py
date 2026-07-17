#!/usr/bin/env python3
"""Verify artifact, contract and profile identities in robot Adapter manifests."""

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


def canonical(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )


def digest(value: bytes) -> str:
    return "sha256:" + hashlib.sha256(value).hexdigest()


def file_digest(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(chunk)
    return "sha256:" + hasher.hexdigest()


def schema_reference(
    messages: dict[int, dict[str, Any]], message_id: int
) -> dict[str, Any]:
    message = messages.get(message_id)
    require(message is not None, "required message is absent from registry")
    return {
        "messageId": message_id,
        "typeName": message["type"],
        "schemaVersion": message["version"],
        "schemaFingerprint": message["fingerprint"],
    }


def expected_capability_endpoints(
    messages: dict[int, dict[str, Any]],
    source_profile: dict[str, Any],
) -> dict[str, list[dict[str, Any]]]:
    expected = {
        "xgc.robot.telemetry": [
            {
                "endpointId": "telemetry",
                "interaction": "stream-source",
                "outputSchema": schema_reference(messages, 4002),
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
        ]
    }
    operations = sorted(
        [
            channel
            for channel in source_profile["channels"]
            if channel["kind"] == "operation"
        ],
        key=lambda channel: channel["operation_id"],
    )
    if operations:
        expected["xgc.robot.command"] = sorted(
            [
                {
                    "endpointId": channel["operation_id"],
                    "interaction": "operation",
                    "inputSchema": schema_reference(
                        messages, channel["input_message_id"]
                    ),
                    "outputSchema": schema_reference(
                        messages, channel["output_message_id"]
                    ),
                    "sideEffect": channel["operation_contract"]["side_effect"],
                    "idempotency": channel["operation_contract"]["idempotency"],
                    "cancellationSupported": channel["operation_contract"][
                        "cancellation_supported"
                    ],
                    "deadlineRequired": channel["operation_contract"][
                        "deadline_required"
                    ],
                    "defaultTimeoutMillis": channel[
                        "operation_timeout_millis"
                    ],
                    "maximumTimeoutMillis": channel[
                        "operation_timeout_millis"
                    ],
                    "limits": {
                        "maximumRequestBytes": 65536,
                        "maximumResponseBytes": 65536,
                        "maximumConcurrency": 32,
                        "maximumStreams": 0,
                        "maximumStreamChunkBytes": 0,
                        "maximumStreamChunkMessages": 0,
                    },
                }
                for channel in operations
            ],
            key=lambda endpoint: endpoint["endpointId"],
        )
    return expected


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def verify(args: argparse.Namespace) -> None:
    executable = Path(args.executable)
    require(executable.is_file(), "Adapter executable does not exist")
    require(Path(args.artifact_path).is_absolute(), "Adapter artifact path must be absolute")
    _, messages = load_registry(Path(args.registry))
    profiles = load_profile(
        Path(args.profile_file), Path(args.profile_schema), messages
    )
    require(len(profiles) == 1, "one robot profile is required")
    _, source_profile = next(iter(profiles.items()))
    adapter = json.loads(Path(args.adapter_manifest).read_text(encoding="utf-8"))
    process = json.loads(Path(args.process_manifest).read_text(encoding="utf-8"))
    profile = json.loads(Path(args.profile_catalog).read_text(encoding="utf-8"))

    require(adapter["apiVersion"] == "xgc.adapter.definition/v1", "invalid Adapter manifest apiVersion")
    require(len(adapter["adapters"]) == 1, "Adapter manifest must contain exactly one definition")
    installed = adapter["adapters"][0]
    definition = installed["definition"]
    require(
        set(definition)
        == {
            "id",
            "version",
            "processDefinitionId",
            "buildDigest",
            "trustedManifestDigest",
            "configuration",
            "activation",
            "scope",
        },
        "Adapter definition fields do not match the exact schema",
    )
    require(definition["id"] == args.definition_id, "Adapter definition identity mismatch")
    require(definition["buildDigest"] == file_digest(executable), "Adapter build digest mismatch")
    require(definition["configuration"]["schema"]["messageId"] == 4001, "Adapter configuration schema mismatch")
    require(definition["configuration"]["allowedEncodings"] == ["protobuf"], "Adapter configuration encoding mismatch")
    require(
        definition["scope"]
        == {
            "kind": "robot-group",
            "requiredAttributes": ["provider", "run-id", "target-id"],
            "allowAdditionalAttributes": False,
            "sharing": "shared",
        },
        "Adapter scope contract mismatch",
    )

    manifest = installed["capabilityManifest"]
    require(manifest["formatVersion"] == 1, "invalid capability manifest formatVersion")
    capabilities_by_id = {}
    for capability in manifest["capabilities"]:
        endpoints = sorted(capability["endpoints"], key=lambda item: item["endpointId"])
        body = {"ref": capability["ref"], "endpoints": endpoints}
        require(capability["contractDigest"] == digest(canonical(body)), "capability contract digest mismatch")
        capability_id = capability["ref"]["id"]
        require(capability_id not in capabilities_by_id, "Adapter capability is duplicated")
        capabilities_by_id[capability_id] = capability
        for endpoint in endpoints:
            require("inputSchema" not in endpoint or endpoint["inputSchema"]["messageId"] > 0, "endpoint input schema is invalid")
            require(endpoint["outputSchema"]["messageId"] > 0, "endpoint output schema is invalid")
            require(endpoint["deadlineRequired"] == (endpoint["interaction"] == "operation"), "endpoint deadline policy mismatch")
    expected_endpoints = expected_capability_endpoints(messages, source_profile)
    require(
        set(capabilities_by_id) == set(expected_endpoints),
        "Adapter capability set mismatch",
    )
    for capability_id, endpoints in expected_endpoints.items():
        capability = capabilities_by_id[capability_id]
        require(
            capability["ref"] == {"id": capability_id, "version": 1},
            "Adapter capability reference mismatch",
        )
        require(
            capability["endpoints"] == endpoints,
            "Adapter capability endpoints do not match the exact native contract",
        )

    canonical_manifest = {
        "formatVersion": manifest["formatVersion"],
        "capabilities": sorted(
            manifest["capabilities"],
            key=lambda item: "{}@{}".format(item["ref"]["id"], item["ref"]["version"]),
        ),
    }
    require(definition["trustedManifestDigest"] == digest(canonical(canonical_manifest)), "trusted capability manifest digest mismatch")

    require(process["apiVersion"] == "xgc.execution.process/v1", "invalid process manifest apiVersion")
    require(len(process["definitions"]) == 1, "process manifest must contain exactly one definition")
    process_definition = process["definitions"][0]
    require(process_definition["id"] == definition["processDefinitionId"], "process definition identity mismatch")
    require(process_definition["internal"] is True, "Adapter process must be internal")
    require(process_definition["restart"]["mode"] == "never", "Adapter process restart policy mismatch")
    require(
        process_definition["command"]
        == {
            "executable": args.artifact_path,
            "args": ["--adapter-bootstrap-file", "${adapterBootstrapFile}"],
            "directExecutable": True,
            "env": ROS_NOETIC_ENVIRONMENT,
        },
        "Adapter process command mismatch",
    )

    require(profile["schema"] == "xgc.robot.adapter-profile-catalog/v1", "invalid profile catalog schema")
    require(len(profile["profiles"]) == 1, "profile catalog must contain exactly one profile")
    installed_profile = profile["profiles"][0]
    expected_profile_body = catalog_profile_body(
        source_profile, messages, definition["id"]
    )
    expected_profile = {
        "profileId": expected_profile_body["profileId"],
        "profileDigest": profile_contract_digest(expected_profile_body),
        **{
            key: value
            for key, value in expected_profile_body.items()
            if key != "profileId"
        },
    }
    require(
        installed_profile == expected_profile,
        "installed profile catalog entry does not exactly match its canonical public contract",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--artifact-path", required=True)
    parser.add_argument("--definition-id", required=True)
    parser.add_argument("--registry", required=True)
    parser.add_argument("--profile-file", required=True)
    parser.add_argument("--profile-schema", required=True)
    parser.add_argument("--adapter-manifest", required=True)
    parser.add_argument("--process-manifest", required=True)
    parser.add_argument("--profile-catalog", required=True)
    verify(parser.parse_args())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
