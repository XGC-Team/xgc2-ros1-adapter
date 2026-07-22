#!/usr/bin/env python3

"""Validate one repository-owned ROS1 profile and emit C++ metadata.

The public protobuf product owns generic messages and semantic schemas. This
repository owns the native ROS profile that maps those messages to a robot.
The generated metadata is deliberately independent of the generic Adapter
Runtime protocol: profile channel kinds are robot-domain implementation facts.
"""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

import jsonschema
import yaml


PROFILE_SCHEMA_IDS = {
    "xgc.robot.adapter-profile/v4",
    "xgc.robot.adapter-profile/v5",
}
PROFILE_CONTRACT_DIGEST_SCHEMA = "xgc.robot.profile-contract-digest/v5"
KINDS = {"stream_out", "operation"}
INPUT_KINDS = {"operation"}
OUTPUT_KINDS = {"stream_out", "operation"}
INPUT_ROLES = {"control", "request"}
OUTPUT_ROLES = {"telemetry", "diagnostic", "response"}
OPERATION_OUTPUT_ROLES = {"request", "response"}
ENDPOINT_PARAMETER = re.compile(r"\{([a-z][a-z0-9_]*)\}")
CPP_NAMESPACE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
UINT32_MAX = (1 << 32) - 1
UINT64_MAX = (1 << 64) - 1
INT64_MIN = -(1 << 63)
INT64_MAX = (1 << 63) - 1
MAX_PARAMETER_PATTERN_LENGTH = 256
NATIVE_OPERATION_TIMEOUT_MAX_MILLIS = 5000

OPERATION_INPUT_TYPES = {
    "arm": "xgc.semantic.aerial.v1.ArmRequest",
    "set-flight-mode": "xgc.semantic.aerial.v1.ModeRequest",
    "reboot-autopilot": "xgc.semantic.aerial.v1.AutopilotRebootRequest",
    "set-motion-intent": "xgc.semantic.ground.v1.MotionIntentRequest",
}

KIND_ENUM = {
    "stream_out": "ChannelKind::kStreamOut",
    "operation": "ChannelKind::kOperation",
}

PARAMETER_TYPE_ENUM = {
    "string": "ParameterType::kString",
    "integer": "ParameterType::kInteger",
    "number": "ParameterType::kNumber",
    "boolean": "ParameterType::kBoolean",
}

ENDPOINT_KIND_ENUM = {
    "input": "EndpointKind::kInput",
    "output": "EndpointKind::kOutput",
    "service": "EndpointKind::kService",
}

ENDPOINT_SCOPE_ENUM = {
    "robot_namespace": "EndpointScope::kRobotNamespace",
    "global": "EndpointScope::kGlobal",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate compile-time metadata from one local ROS1 profile"
    )
    parser.add_argument("--registry", required=True)
    parser.add_argument("--profile-file", required=True)
    parser.add_argument("--profile-schema", required=True)
    parser.add_argument("--definition-id", required=True)
    parser.add_argument("--cpp-namespace", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def cpp_string(value):
    return json.dumps(str(value))


def cpp_bool(value):
    return "true" if value else "false"


def cpp_int64(value):
    if value == INT64_MIN:
        return "(-9223372036854775807LL - 1LL)"
    return "{}LL".format(value)


def normalized_endpoint(kind, role, endpoint):
    return {
        "kind": kind,
        "role": role,
        "name_template": endpoint["name"],
        "ros_type": endpoint["type"],
        "scope": endpoint.get("scope", "robot_namespace"),
    }


def validate_policy_value(value, label):
    if isinstance(value, bool) or isinstance(value, str):
        return value
    if isinstance(value, int):
        if value < INT64_MIN or value > INT64_MAX:
            raise ValueError("{} integer is outside int64".format(label))
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("{} number must be finite".format(label))
        return value
    if isinstance(value, list) and value and all(
        isinstance(item, str) for item in value
    ):
        return value
    raise ValueError("{} has an unsupported value".format(label))


def validate_operation_parameter_schema(schema, label):
    if not isinstance(schema, dict) or set(schema) != {
        "type",
        "required",
        "properties",
        "additionalProperties",
    }:
        raise ValueError("{} must be an exact strict object schema".format(label))
    if schema["type"] != "object" or schema["additionalProperties"] is not False:
        raise ValueError("{} root must be a closed object".format(label))
    required = schema["required"]
    properties = schema["properties"]
    if (
        not isinstance(required, list)
        or any(not isinstance(name, str) or not name for name in required)
        or len(required) != len(set(required))
        or not isinstance(properties, dict)
        or set(required) != set(properties)
    ):
        raise ValueError(
            "{} required must name every property exactly once".format(label)
        )
    for name, definition in properties.items():
        property_label = "{} property {}".format(label, name)
        if not isinstance(name, str) or not name:
            raise ValueError("{} has an invalid property name".format(label))
        if not isinstance(definition, dict) or set(definition) not in (
            {"type"},
            {"type", "enum"},
            {"type", "minimum", "maximum"},
        ):
            raise ValueError(
                "{} must declare only type with an optional enum or bounds".format(
                    property_label
                )
            )
        if definition["type"] not in {
            "string",
            "integer",
            "number",
            "boolean",
        }:
            raise ValueError("{} has an invalid type".format(property_label))
        if "enum" in definition:
            values = definition["enum"]
            if (
                definition["type"] != "string"
                or not isinstance(values, list)
                or not values
                or any(not isinstance(value, str) or not value for value in values)
                or len(values) != len(set(values))
            ):
                raise ValueError(
                    "{} enum must be unique non-empty strings".format(
                        property_label
                    )
                )
        if "minimum" in definition:
            if definition["type"] not in {"integer", "number"}:
                raise ValueError(
                    "{} bounds require an integer or number type".format(
                        property_label
                    )
                )
            minimum = definition["minimum"]
            maximum = definition["maximum"]
            if (
                isinstance(minimum, bool)
                or isinstance(maximum, bool)
                or not isinstance(minimum, (int, float))
                or not isinstance(maximum, (int, float))
                or not math.isfinite(minimum)
                or not math.isfinite(maximum)
                or minimum > maximum
                or (
                    definition["type"] == "integer"
                    and (not isinstance(minimum, int) or not isinstance(maximum, int))
                )
            ):
                raise ValueError("{} has invalid bounds".format(property_label))
    return schema


def operation_parameter_schema(profile_path, channel, messages):
    operation_id = channel["operation_id"]
    expected_input_type = OPERATION_INPUT_TYPES.get(operation_id)
    if expected_input_type is None:
        raise ValueError(
            "{}: operation {} has no owned parameter contract".format(
                profile_path, operation_id
            )
        )
    input_message = messages.get(channel["input_message_id"])
    actual_input_type = input_message.get("type") if input_message else None
    if actual_input_type != expected_input_type:
        raise ValueError(
            "{}: operation {} input must be {}, got {}".format(
                profile_path,
                operation_id,
                expected_input_type,
                actual_input_type or "<missing>",
            )
        )

    if operation_id == "arm":
        properties = {"armed": {"type": "boolean"}}
    elif operation_id == "set-flight-mode":
        allowed_modes = channel.get("policy", {}).get("allowed_modes")
        if (
            not isinstance(allowed_modes, list)
            or not allowed_modes
            or any(not isinstance(mode, str) or not mode for mode in allowed_modes)
            or len(allowed_modes) != len(set(allowed_modes))
        ):
            raise ValueError(
                "{}: operation {} policy.allowed_modes must be unique non-empty strings".format(
                    profile_path, operation_id
                )
            )
        properties = {
            "mode": {"type": "string", "enum": list(allowed_modes)}
        }
    elif operation_id == "set-motion-intent":
        properties = {
            "gear": {"type": "integer", "minimum": 1, "maximum": 3},
            "longitudinal": {
                "type": "integer",
                "minimum": -1,
                "maximum": 1,
            },
            "yaw": {"type": "integer", "minimum": -1, "maximum": 1},
        }
    else:
        properties = {}

    return validate_operation_parameter_schema(
        {
            "type": "object",
            "required": sorted(properties),
            "properties": properties,
            "additionalProperties": False,
        },
        "{}: operation {} parameterSchema".format(profile_path, operation_id),
    )


def require_positive_integer(value, label, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("{} must be a positive integer".format(label))
    if value > maximum:
        raise ValueError("{} exceeds {}".format(label, maximum))
    return value


def validate_portable_parameter_pattern(pattern, label):
    """Accept the deliberately small regex dialect shared by Python/Go/C++.

    Robot profile patterns cross three regex engines. Keeping v1 to anchored
    ASCII literals/classes and simple quantifiers prevents a source profile
    from passing package generation but failing later in Core's RE2 runtime.
    """
    if not isinstance(pattern, str) or not 3 <= len(pattern) <= MAX_PARAMETER_PATTERN_LENGTH:
        raise ValueError("{} must be a 3..{} character string".format(label, MAX_PARAMETER_PATTERN_LENGTH))
    if not pattern.isascii() or not pattern.startswith("^") or not pattern.endswith("$"):
        raise ValueError("{} must be an anchored ASCII portable regex".format(label))
    body = pattern[1:-1]
    if not body:
        raise ValueError("{} must contain at least one atom".format(label))

    index = 0
    while index < len(body):
        character = body[index]
        if character == "[":
            close = body.find("]", index + 1)
            if close < 0:
                raise ValueError("{} contains an unterminated character class".format(label))
            content = body[index + 1 : close]
            if not content or content.startswith("^"):
                raise ValueError("{} contains an empty or negated character class".format(label))
            class_index = 0
            while class_index < len(content):
                item = content[class_index]
                if item == "-":
                    if class_index != len(content) - 1:
                        raise ValueError("{} contains an ambiguous literal hyphen".format(label))
                    class_index += 1
                    continue
                if not (item.isascii() and (item.isalnum() or item == "_")):
                    raise ValueError("{} contains a non-portable character class atom".format(label))
                if class_index + 1 < len(content) and content[class_index + 1] == "-":
                    if class_index + 2 >= len(content):
                        class_index += 1
                        continue
                    end = content[class_index + 2]
                    same_range = (
                        item.isdigit() and end.isdigit()
                        or item.islower() and end.islower()
                        or item.isupper() and end.isupper()
                    )
                    if (
                        not (end.isascii() and end.isalnum())
                        or not same_range
                        or ord(item) > ord(end)
                    ):
                        raise ValueError("{} contains an invalid character range".format(label))
                    class_index += 3
                    continue
                class_index += 1
            index = close + 1
        elif character.isascii() and (character.isalnum() or character in "_/-"):
            index += 1
        else:
            raise ValueError("{} contains a non-portable regex construct".format(label))
        if index < len(body) and body[index] in "?*+":
            index += 1

    try:
        re.compile(pattern, re.ASCII)
    except re.error as error:
        raise ValueError("{} is invalid: {}".format(label, error))
    return pattern


def load_registry(path):
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("{}: invalid message registry: {}".format(path, error))

    if not isinstance(document, dict):
        raise ValueError("{}: message registry root must be an object".format(path))
    fingerprint = require_positive_integer(
        document.get("registry_fingerprint"),
        "{}: registry_fingerprint".format(path),
        UINT64_MAX,
    )
    require_positive_integer(
        document.get("registry_version"),
        "{}: registry_version".format(path),
        UINT32_MAX,
    )
    raw_messages = document.get("messages")
    if not isinstance(raw_messages, list) or not raw_messages:
        raise ValueError("{}: message registry is empty".format(path))

    messages = {}
    for index, item in enumerate(raw_messages):
        label = "{}: messages[{}]".format(path, index)
        if not isinstance(item, dict):
            raise ValueError("{} must be an object".format(label))
        message_id = require_positive_integer(
            item.get("id"), "{}.id".format(label), UINT32_MAX
        )
        if message_id in messages:
            raise ValueError("{}: duplicate message ID {}".format(path, message_id))
        message_type = item.get("type")
        if not isinstance(message_type, str) or not message_type:
            raise ValueError("{}.type must be a non-empty string".format(label))
        version = require_positive_integer(
            item.get("version"), "{}.version".format(label), UINT32_MAX
        )
        message_fingerprint = require_positive_integer(
            item.get("fingerprint"), "{}.fingerprint".format(label), UINT64_MAX
        )
        roles = item.get("roles")
        if (
            not isinstance(roles, list)
            or not roles
            or any(not isinstance(role, str) or not role for role in roles)
        ):
            raise ValueError("{}.roles must be a non-empty string array".format(label))
        if len(roles) != len(set(roles)):
            raise ValueError("{}.roles contains duplicates".format(label))
        messages[message_id] = {
            "type": message_type,
            "version": version,
            "fingerprint": message_fingerprint,
            "roles": set(roles),
        }
    return fingerprint, messages


def load_profile_schema(path):
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        jsonschema.Draft7Validator.check_schema(document)
    except (OSError, UnicodeError, json.JSONDecodeError, jsonschema.SchemaError) as error:
        raise ValueError("{}: invalid profile schema: {}".format(path, error))
    schema_marker = document.get("properties", {}).get("schema", {}).get("const")
    if schema_marker not in PROFILE_SCHEMA_IDS:
        raise ValueError(
            "{}: profile schema marker {} is unsupported".format(
                path, schema_marker or "<missing>"
            )
        )
    return document, schema_marker


def validate_endpoint_parameters(profile_path, profile, channel_id, endpoint):
    parameters = profile["parameters"]
    for parameter_name in ENDPOINT_PARAMETER.findall(endpoint["name"]):
        definition = parameters.get(parameter_name)
        if definition is None:
            raise ValueError(
                "{}: channel {} endpoint references undeclared parameter {}".format(
                    profile_path, channel_id, parameter_name
                )
            )
        if not definition.get("required"):
            raise ValueError(
                "{}: channel {} endpoint parameter {} must be required".format(
                    profile_path, channel_id, parameter_name
                )
            )
        if definition["type"] != "string":
            raise ValueError(
                "{}: channel {} endpoint parameter {} must be a string".format(
                    profile_path, channel_id, parameter_name
                )
            )


def require_registered_message(messages, profile_path, channel, field, roles):
    message_id = channel.get(field)
    message = messages.get(message_id)
    if message is None:
        raise ValueError(
            "{}: channel {} references unregistered {} {}".format(
                profile_path, channel["id"], field, message_id
            )
        )
    if not message["roles"].intersection(roles):
        raise ValueError(
            "{}: channel {} message {} has incompatible roles {}".format(
                profile_path,
                channel["id"],
                message["type"],
                sorted(message["roles"]),
            )
        )
    return message_id


def validate_profile_document(
    profile_path, profile, schema, profile_schema_id, messages
):
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(
        validator.iter_errors(profile),
        key=lambda error: tuple(str(item) for item in error.absolute_path),
    )
    if errors:
        error = errors[0]
        location = ".".join(str(item) for item in error.absolute_path) or "<root>"
        raise ValueError(
            "{}: schema validation failed at {}: {}".format(
                profile_path, location, error.message
            )
        )

    if profile["schema"] != profile_schema_id:
        raise ValueError(
            "{}: unsupported profile schema {}".format(profile_path, profile["schema"])
        )
    if profile["native_protocol"] != "ros1":
        raise ValueError("{}: profile is not a ROS1 native mapping".format(profile_path))

    version_match = re.search(r"\.v([1-9][0-9]*)$", profile["profile_id"])
    if not version_match or int(version_match.group(1)) != profile["profile_version"]:
        raise ValueError(
            "{}: profile_id suffix and profile_version disagree".format(profile_path)
        )

    for parameter_name, definition in profile["parameters"].items():
        if "pattern" in definition:
            validate_portable_parameter_pattern(
                definition["pattern"],
                "{}: parameter {} pattern".format(profile_path, parameter_name),
            )

    seen_channels = set()
    channels_by_id = {}
    operation_ids = set()
    pulse_endpoint_ids = set()
    ordered_channels = []
    for channel in profile["channels"]:
        channel_id = channel["id"]
        kind = channel["kind"]
        if channel_id in seen_channels:
            raise ValueError("{}: duplicate channel ID {}".format(profile_path, channel_id))
        if kind not in KINDS:
            raise ValueError(
                "{}: channel {} has invalid kind {}".format(
                    profile_path, channel_id, kind
                )
            )

        operation_id = channel.get("operation_id")
        operation_contract = channel.get("operation_contract")
        if kind == "operation":
            if operation_id in operation_ids:
                raise ValueError(
                    "{}: duplicate operation identity {}".format(
                        profile_path, operation_id
                    )
                )
            operation_ids.add(operation_id)
        elif operation_id is not None or operation_contract is not None:
            raise ValueError(
                "{}: channel {} kind {} cannot declare operation metadata".format(
                    profile_path, channel_id, kind
                )
            )

        native_fields = {
            field
            for field in ("inputs", "observes", "output", "service")
            if field in channel
        }
        allowed_native_fields = {
            "stream_out": {"inputs", "observes", "output"},
            "operation": {"service", "output"},
        }[kind]
        unexpected_native_fields = native_fields - allowed_native_fields
        if unexpected_native_fields:
            raise ValueError(
                "{}: channel {} kind {} has incompatible native fields {}".format(
                    profile_path,
                    channel_id,
                    kind,
                    sorted(unexpected_native_fields),
                )
            )
        if kind == "operation" and len(native_fields) != 1:
            raise ValueError(
                "{}: operation channel {} must declare exactly one native service or output endpoint".format(
                    profile_path, channel_id
                )
            )

        input_id = 0
        if kind in INPUT_KINDS:
            input_id = require_registered_message(
                messages, profile_path, channel, "input_message_id", INPUT_ROLES
            )
        elif "input_message_id" in channel:
            raise ValueError(
                "{}: channel {} kind {} cannot declare input_message_id".format(
                    profile_path, channel_id, kind
                )
            )

        output_id = 0
        if kind in OUTPUT_KINDS:
            output_roles = (
                OPERATION_OUTPUT_ROLES if kind == "operation" else OUTPUT_ROLES
            )
            output_id = require_registered_message(
                messages,
                profile_path,
                channel,
                "output_message_id",
                output_roles,
            )
        elif "output_message_id" in channel:
            raise ValueError(
                "{}: channel {} kind {} cannot declare output_message_id".format(
                    profile_path, channel_id, kind
                )
            )

        rate = 0.0
        if kind == "stream_out":
            rate = float(channel["output_rate_hz"])
            if not math.isfinite(rate):
                raise ValueError(
                    "{}: channel {} output_rate_hz must be finite".format(
                        profile_path, channel_id
                    )
                )
        elif "output_rate_hz" in channel:
            raise ValueError(
                "{}: channel {} kind {} cannot declare output_rate_hz".format(
                    profile_path, channel_id, kind
                )
            )

        timeout_ms = 0
        if kind == "operation":
            policy = channel.get("policy")
            if not isinstance(policy, dict) or "timeout_ms" not in policy:
                raise ValueError(
                    "{}: operation channel {} requires policy.timeout_ms".format(
                        profile_path, channel_id
                    )
                )
            timeout_ms = require_positive_integer(
                policy["timeout_ms"],
                "{}: operation channel {} policy.timeout_ms".format(
                    profile_path, channel_id
                ),
                UINT32_MAX,
            )
            if timeout_ms > NATIVE_OPERATION_TIMEOUT_MAX_MILLIS:
                raise ValueError(
                    "{}: operation channel {} policy.timeout_ms exceeds the native {} ms safety limit".format(
                        profile_path,
                        channel_id,
                        NATIVE_OPERATION_TIMEOUT_MAX_MILLIS,
                    )
                )

        endpoints = []
        for endpoint in channel.get("inputs", {}).values():
            validate_endpoint_parameters(profile_path, profile, channel_id, endpoint)
        for role, endpoint in sorted(channel.get("inputs", {}).items()):
            endpoints.append(normalized_endpoint("input", role, endpoint))
        for endpoint_name in ("output", "service"):
            endpoint = channel.get(endpoint_name)
            if endpoint is not None:
                validate_endpoint_parameters(profile_path, profile, channel_id, endpoint)
                endpoints.append(
                    normalized_endpoint(endpoint_name, endpoint_name, endpoint)
                )

        policy = dict(channel.get("policy", {}))
        for key, value in policy.items():
            validate_policy_value(
                value,
                "{}: channel {} policy {}".format(
                    profile_path, channel_id, key
                ),
            )
        parameter_schema = (
            operation_parameter_schema(profile_path, channel, messages)
            if kind == "operation"
            else None
        )
        lease = channel.get("lease")
        if lease is not None:
            if kind != "operation":
                raise ValueError(
                    "{}: channel {} lease requires an operation".format(
                        profile_path, channel_id
                    )
                )
            pulse_endpoint_id = lease["pulse_endpoint_id"]
            if (
                pulse_endpoint_id == operation_id
                or pulse_endpoint_id in operation_ids
                or pulse_endpoint_id in pulse_endpoint_ids
            ):
                raise ValueError(
                    "{}: operation {} lease pulse endpoint {} is not unique".format(
                        profile_path, operation_id, pulse_endpoint_id
                    )
                )
            pulse_endpoint_ids.add(pulse_endpoint_id)
            heartbeat_interval_ms = require_positive_integer(
                lease["heartbeat_interval_ms"],
                "{}: operation {} lease heartbeat_interval_ms".format(
                    profile_path, operation_id
                ),
                NATIVE_OPERATION_TIMEOUT_MAX_MILLIS,
            )
            timeout_ms = require_positive_integer(
                lease["timeout_ms"],
                "{}: operation {} lease timeout_ms".format(
                    profile_path, operation_id
                ),
                NATIVE_OPERATION_TIMEOUT_MAX_MILLIS,
            )
            if timeout_ms < 2 * heartbeat_interval_ms:
                raise ValueError(
                    "{}: operation {} lease timeout must be at least twice its heartbeat interval".format(
                        profile_path, operation_id
                    )
                )
            if not lease["volatile_supported"]:
                raise ValueError(
                    "{}: operation {} lease pulse must explicitly opt in to volatile dispatch".format(
                        profile_path, operation_id
                    )
                )
            if not operation_contract.get("cancellation_supported"):
                raise ValueError(
                    "{}: leased operation {} must support cancellation".format(
                        profile_path, operation_id
                    )
                )
            properties = parameter_schema["properties"]
            inactive = lease["inactive_parameter_values"]
            for name, value in inactive.items():
                definition = properties.get(name)
                if definition is None or not jsonschema.Draft7Validator(
                    definition
                ).is_valid(value):
                    raise ValueError(
                        "{}: operation {} inactive lease value {} does not match its parameter schema".format(
                            profile_path, operation_id, name
                        )
                    )

        ordered_channels.append(
            {
                "id": channel_id,
                "kind": kind,
                "processor": channel["processor"],
                "operation_id": operation_id or "",
                "input_message_id": input_id,
                "output_message_id": output_id,
                "output_rate_hz": rate,
                "operation_timeout_millis": timeout_ms,
                "endpoints": endpoints,
                "observes": list(channel.get("observes", [])),
                "policy": policy,
                "operation_contract": dict(operation_contract or {}),
                "operation_parameter_schema": parameter_schema,
                "lease": dict(lease or {}),
            }
        )
        seen_channels.add(channel_id)
        channels_by_id[channel_id] = channel

    for channel in profile["channels"]:
        for observed_channel in channel.get("observes", []):
            if observed_channel == channel["id"]:
                raise ValueError(
                    "{}: channel {} cannot observe itself".format(
                        profile_path, channel["id"]
                    )
                )
            if observed_channel not in seen_channels:
                raise ValueError(
                    "{}: channel {} observes unknown channel {}".format(
                        profile_path, channel["id"], observed_channel
                    )
                )

    traits = profile["semantic_traits"]
    default_stale_after_ms = require_positive_integer(
        traits["default_stale_after_ms"],
        "{}: semantic_traits.default_stale_after_ms".format(profile_path),
        UINT32_MAX,
    )
    for channel_id, maximum_age in traits["channel_stale_after_ms"].items():
        channel = channels_by_id.get(channel_id)
        if channel is None or channel["kind"] != "stream_out":
            raise ValueError(
                "{}: staleness policy channel {} must be an existing stream_out".format(
                    profile_path, channel_id
                )
            )
        require_positive_integer(
            maximum_age,
            "{}: staleness policy channel {}".format(profile_path, channel_id),
            UINT32_MAX,
        )

    for channel in ordered_channels:
        channel["stale_after_millis"] = (
            traits["channel_stale_after_ms"].get(
                channel["id"], default_stale_after_ms
            )
            if channel["kind"] == "stream_out"
            else 0
        )

    for trait_name in ("online_conditions", "operational_ready_conditions"):
        seen_conditions = set()
        for condition in traits[trait_name]:
            channel_id = condition["channel_id"]
            channel = channels_by_id.get(channel_id)
            if channel is None or channel["kind"] != "stream_out":
                raise ValueError(
                    "{}: {} channel {} must be an existing stream_out".format(
                        profile_path, trait_name, channel_id
                    )
                )
            if channel_id in seen_conditions:
                raise ValueError(
                    "{}: {} repeats channel {}".format(
                        profile_path, trait_name, channel_id
                    )
                )
            seen_conditions.add(channel_id)
            require_positive_integer(
                condition["maximum_age_ms"],
                "{}: {} channel {} maximum_age_ms".format(
                    profile_path, trait_name, channel_id
                ),
                UINT32_MAX,
            )
            predicate = condition.get("predicate")
            if predicate == "xgc.semantic.aerial.flight.connected":
                output_message = messages.get(channel.get("output_message_id"))
                if (
                    output_message is None
                    or output_message["type"]
                    != "xgc.semantic.aerial.v1.FlightStatus"
                ):
                    raise ValueError(
                        "{}: predicate {} requires a FlightStatus stream_out channel".format(
                            profile_path, predicate
                        )
                    )
            if predicate == "xgc.semantic.common.vehicle-health.online":
                output_message = messages.get(channel.get("output_message_id"))
                if (
                    output_message is None
                    or output_message["type"]
                    != "xgc.semantic.common.v1.VehicleHealth"
                ):
                    raise ValueError(
                        "{}: predicate {} requires a VehicleHealth stream_out channel".format(
                            profile_path, predicate
                        )
                    )

    return ordered_channels


def load_profile(profile_path, schema_path, messages):
    raw = profile_path.read_bytes()
    try:
        profile = yaml.safe_load(raw.decode("utf-8"))
    except (UnicodeError, yaml.YAMLError) as error:
        raise ValueError("{}: invalid profile YAML: {}".format(profile_path, error))
    if not isinstance(profile, dict):
        raise ValueError("{}: profile root must be an object".format(profile_path))
    schema, profile_schema_id = load_profile_schema(schema_path)
    channels = validate_profile_document(
        profile_path, profile, schema, profile_schema_id, messages
    )
    return {
        profile["profile_id"]: {
            "profile_id": profile["profile_id"],
            "profile_version": profile["profile_version"],
            "robot_kind": profile["robot_kind"],
            "description": profile.get("description", ""),
            "parameters": profile["parameters"],
            "semantic_traits": profile["semantic_traits"],
            "channels": channels,
        }
    }


def catalog_parameters(source_profile):
    return {
        name: {
            "type": definition["type"],
            "required": definition["required"],
            "delivery": definition["delivery"],
            **(
                {"pattern": definition["pattern"]}
                if "pattern" in definition
                else {}
            ),
        }
        for name, definition in sorted(source_profile["parameters"].items())
    }


def catalog_conditions(source_profile, name):
    conditions = [
        {
            "channelId": item["channel_id"],
            "maximumAgeMillis": item["maximum_age_ms"],
            **(
                {"predicate": item["predicate"]}
                if "predicate" in item
                else {}
            ),
        }
        for item in source_profile["semantic_traits"][name]
    ]
    return sorted(
        conditions,
        key=lambda item: (
            item["channelId"],
            item.get("predicate", ""),
            item["maximumAgeMillis"],
        ),
    )


def catalog_semantics(source_profile):
    return {
        "operations": sorted(
            [
                {
                    "id": channel["operation_id"],
                    "channelId": channel["id"],
                    "timeoutMillis": channel["operation_timeout_millis"],
                    "parameterSchema": channel[
                        "operation_parameter_schema"
                    ],
                    **(
                        {
                            "lease": {
                                "pulseEndpointId": channel["lease"][
                                    "pulse_endpoint_id"
                                ],
                                "heartbeatIntervalMillis": channel["lease"][
                                    "heartbeat_interval_ms"
                                ],
                                "timeoutMillis": channel["lease"][
                                    "timeout_ms"
                                ],
                                "inactiveParameterValues": dict(
                                    sorted(
                                        channel["lease"][
                                            "inactive_parameter_values"
                                        ].items()
                                    )
                                ),
                            }
                        }
                        if channel["lease"]
                        else {}
                    ),
                }
                for channel in source_profile["channels"]
                if channel["kind"] == "operation"
            ],
            key=lambda item: item["id"],
        ),
        "defaultStaleAfterMillis": source_profile["semantic_traits"][
            "default_stale_after_ms"
        ],
        "channelStaleAfterMillis": {
            channel_id: maximum_age
            for channel_id, maximum_age in sorted(
                source_profile["semantic_traits"][
                    "channel_stale_after_ms"
                ].items()
            )
        },
        "onlineConditions": catalog_conditions(
            source_profile, "online_conditions"
        ),
        "operationalReadyConditions": catalog_conditions(
            source_profile, "operational_ready_conditions"
        ),
    }


def catalog_channels(source_profile, messages):
    channels = []
    for source in source_profile["channels"]:
        message_id = source["input_message_id"] or source["output_message_id"]
        message = messages.get(message_id)
        if message is None:
            raise ValueError(
                "profile channel {} message {} is absent from registry".format(
                    source["id"], message_id
                )
            )
        channels.append(
            {
                "id": source["id"],
                "kind": source["kind"],
                "messageId": message_id,
                "typeName": message["type"],
                "schemaVersion": message["version"],
                "schemaFingerprint": message["fingerprint"],
            }
        )
    return sorted(channels, key=lambda item: item["id"])


def catalog_profile_body(source_profile, messages, provider_definition_id):
    if not isinstance(provider_definition_id, str) or not provider_definition_id:
        raise ValueError("provider definition ID must be a non-empty string")
    return {
        "profileId": source_profile["profile_id"],
        "providerDefinitionId": provider_definition_id,
        "robotKind": source_profile["robot_kind"],
        "parameters": catalog_parameters(source_profile),
        "semantics": catalog_semantics(source_profile),
        "channels": catalog_channels(source_profile, messages),
    }


def profile_contract_digest(profile_body):
    envelope = {
        "schema": PROFILE_CONTRACT_DIGEST_SCHEMA,
        "profile": profile_body,
    }
    canonical = json.dumps(
        envelope,
        sort_keys=True,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def generate(
    registry_fingerprint,
    messages,
    profiles,
    provider_definition_id,
    cpp_namespace,
):
    if not CPP_NAMESPACE.fullmatch(cpp_namespace):
        raise ValueError("invalid C++ namespace: {}".format(cpp_namespace))
    if len(profiles) != 1:
        raise ValueError("one robot profile is required per native Adapter binary")
    required_messages = sorted(
        {1, 4001, 4002}
        | {
            message_id
            for profile in profiles.values()
            for channel in profile["channels"]
            for message_id in (
                channel["input_message_id"],
                channel["output_message_id"],
            )
            if message_id
        }
    )
    missing_runtime_messages = [
        message_id for message_id in required_messages if message_id not in messages
    ]
    if missing_runtime_messages:
        raise ValueError(
            "registry is missing robot Adapter Runtime messages {}".format(
                missing_runtime_messages
            )
        )
    profile_digests = {
        profile_id: profile_contract_digest(
            catalog_profile_body(profile, messages, provider_definition_id)
        )
        for profile_id, profile in profiles.items()
    }
    lines = [
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <string>",
        "",
        "namespace {} {{".format(cpp_namespace),
        "namespace contract {",
        "",
        "constexpr const char* kProfileId = {};".format(
            cpp_string(next(iter(profiles)))
        ),
        "",
        "constexpr std::uint64_t kRegistryFingerprint = {}ULL;".format(
            registry_fingerprint
        ),
        "",
        "enum class ParameterType {",
        "  kString,",
        "  kInteger,",
        "  kNumber,",
        "  kBoolean,",
        "};",
        "",
        "enum class ChannelKind {",
        "  kStreamOut,",
        "  kOperation,",
        "};",
        "",
        "enum class EndpointKind {",
        "  kInput,",
        "  kOutput,",
        "  kService,",
        "};",
        "",
        "enum class EndpointScope {",
        "  kRobotNamespace,",
        "  kGlobal,",
        "};",
        "",
        "enum class PolicyValueKind {",
        "  kString,",
        "  kInteger,",
        "  kNumber,",
        "  kBoolean,",
        "  kStringArray,",
        "};",
        "",
        "struct MessageMetadata {",
        "  const char* type_name;",
        "  std::uint32_t version;",
        "  std::uint64_t fingerprint;",
        "};",
        "",
        "struct ParameterMetadata {",
        "  const char* name;",
        "  ParameterType type;",
        "  bool required;",
        "  const char* pattern;",
        "  const char* description;",
        "};",
        "",
        "struct EndpointMetadata {",
        "  EndpointKind kind;",
        "  const char* role;",
        "  const char* name_template;",
        "  const char* ros_type;",
        "  EndpointScope scope;",
        "};",
        "",
        "struct PolicyMetadata {",
        "  const char* key;",
        "  PolicyValueKind kind;",
        "  const char* string_value;",
        "  std::int64_t integer_value;",
        "  double number_value;",
        "  bool boolean_value;",
        "  const char* const* string_array_value;",
        "  std::size_t string_array_size;",
        "};",
        "",
        "struct OperationContractMetadata {",
        "  const char* side_effect;",
        "  const char* idempotency;",
        "  bool cancellation_supported;",
        "  bool deadline_required;",
        "};",
        "",
        "struct OperationMetadata {",
        "  const char* operation_id;",
        "  const char* channel_id;",
        "  std::uint32_t timeout_millis;",
        "  const char* parameter_schema_json;",
        "};",
        "",
        "struct ChannelMetadata {",
        "  const char* channel_id;",
        "  ChannelKind kind;",
        "  const char* processor;",
        "  const char* operation_id;",
        "  std::uint32_t input_message_id;",
        "  std::uint32_t output_message_id;",
        "  double output_rate_hz;",
        "  std::uint32_t operation_timeout_millis;",
        "  std::uint32_t stale_after_millis;",
        "  const EndpointMetadata* endpoints;",
        "  std::size_t endpoint_count;",
        "  const char* const* observes;",
        "  std::size_t observes_count;",
        "  const PolicyMetadata* policy;",
        "  std::size_t policy_count;",
        "  OperationContractMetadata operation_contract;",
        "};",
        "",
        "inline bool messageMetadata(std::uint32_t id, MessageMetadata* out) {",
        "  if (out == nullptr) {",
        "    return false;",
        "  }",
        "  switch (id) {",
    ]
    for message_id in required_messages:
        item = messages[message_id]
        lines.extend(
            [
                "    case {}u:".format(message_id),
                "      *out = MessageMetadata{{{}, {}u, {}ULL}};".format(
                    cpp_string(item["type"]),
                    item["version"],
                    item["fingerprint"],
                ),
                "      return true;",
            ]
        )
    lines.extend(
        [
            "    default:",
            "      return false;",
            "  }",
            "}",
            "",
            "inline const char* profileDigest(const std::string& profile_id) {",
        ]
    )
    for profile_id, profile in profiles.items():
        lines.append(
            "  if (profile_id == {}) return {};".format(
                cpp_string(profile_id), cpp_string(profile_digests[profile_id])
            )
        )
    lines.extend(
        [
            "  return nullptr;",
            "}",
            "",
            "inline const ParameterMetadata* profileParameters(",
            "    const std::string& profile_id, std::size_t* count) {",
            "  if (count == nullptr) {",
            "    return nullptr;",
            "  }",
        ]
    )
    for profile_index, (profile_id, profile) in enumerate(profiles.items()):
        lines.append("  if (profile_id == {}) {{".format(cpp_string(profile_id)))
        if not profile["parameters"]:
            lines.extend(
                [
                    "    *count = 0u;",
                    "    return nullptr;",
                    "  }",
                ]
            )
            continue
        lines.append("    static const ParameterMetadata values[] = {")
        for name, definition in sorted(profile["parameters"].items()):
            lines.append(
                "      {{{}, {}, {}, {}, {}}},".format(
                    cpp_string(name),
                    PARAMETER_TYPE_ENUM[definition["type"]],
                    cpp_bool(definition["required"]),
                    cpp_string(definition.get("pattern", "")),
                    cpp_string(definition["description"]),
                )
            )
        lines.extend(
            [
                "    };",
                "    *count = sizeof(values) / sizeof(values[0]);",
                "    return values;",
                "  }",
            ]
        )
    lines.extend(
        [
            "  *count = 0u;",
            "  return nullptr;",
            "}",
            "",
            "inline bool parameterMetadata(",
            "    const std::string& profile_id, const std::string& name, ParameterMetadata* out) {",
            "  if (out == nullptr) {",
            "    return false;",
            "  }",
            "  std::size_t count = 0u;",
            "  const ParameterMetadata* values = profileParameters(profile_id, &count);",
            "  for (std::size_t index = 0u; index < count; ++index) {",
            "    if (name == values[index].name) {",
            "      *out = values[index];",
            "      return true;",
            "    }",
            "  }",
            "  return false;",
            "}",
            "",
            "inline const OperationMetadata* profileOperations(",
            "    const std::string& profile_id, std::size_t* count) {",
            "  if (count == nullptr) {",
            "    return nullptr;",
            "  }",
        ]
    )
    for profile_id, profile in profiles.items():
        operations = catalog_semantics(profile)["operations"]
        lines.append("  if (profile_id == {}) {{".format(cpp_string(profile_id)))
        if not operations:
            lines.extend(
                [
                    "    *count = 0u;",
                    "    return nullptr;",
                    "  }",
                ]
            )
            continue
        lines.append("    static const OperationMetadata values[] = {")
        for operation in operations:
            parameter_schema_json = json.dumps(
                operation["parameterSchema"],
                sort_keys=True,
                ensure_ascii=False,
                separators=(",", ":"),
            )
            lines.append(
                "      {{{}, {}, {}u, {}}},".format(
                    cpp_string(operation["id"]),
                    cpp_string(operation["channelId"]),
                    operation["timeoutMillis"],
                    cpp_string(parameter_schema_json),
                )
            )
        lines.extend(
            [
                "    };",
                "    *count = sizeof(values) / sizeof(values[0]);",
                "    return values;",
                "  }",
            ]
        )
    lines.extend(
        [
            "  *count = 0u;",
            "  return nullptr;",
            "}",
            "",
            "inline bool operationMetadata(",
            "    const std::string& profile_id, const std::string& operation_id, OperationMetadata* out) {",
            "  if (out == nullptr) {",
            "    return false;",
            "  }",
            "  std::size_t count = 0u;",
            "  const OperationMetadata* values = profileOperations(profile_id, &count);",
            "  for (std::size_t index = 0u; index < count; ++index) {",
            "    if (operation_id == values[index].operation_id) {",
            "      *out = values[index];",
            "      return true;",
            "    }",
            "  }",
            "  return false;",
            "}",
            "",
            "inline const ChannelMetadata* profileChannels(",
            "    const std::string& profile_id, std::size_t* count) {",
            "  if (count == nullptr) {",
            "    return nullptr;",
            "  }",
        ]
    )
    for profile_index, (profile_id, profile) in enumerate(profiles.items()):
        lines.append("  if (profile_id == {}) {{".format(cpp_string(profile_id)))
        for channel_index, channel in enumerate(profile["channels"]):
            prefix = "profile_{}_channel_{}".format(profile_index, channel_index)
            if channel["endpoints"]:
                lines.append(
                    "    static const EndpointMetadata {}_endpoints[] = {{".format(
                        prefix
                    )
                )
                for endpoint in channel["endpoints"]:
                    lines.append(
                        "      {{{}, {}, {}, {}, {}}},".format(
                            ENDPOINT_KIND_ENUM[endpoint["kind"]],
                            cpp_string(endpoint["role"]),
                            cpp_string(endpoint["name_template"]),
                            cpp_string(endpoint["ros_type"]),
                            ENDPOINT_SCOPE_ENUM[endpoint["scope"]],
                        )
                    )
                lines.append("    };")
            if channel["observes"]:
                lines.append(
                    "    static const char* const {}_observes[] = {{".format(prefix)
                )
                for observed in channel["observes"]:
                    lines.append("      {},".format(cpp_string(observed)))
                lines.append("    };")

            sorted_policy = sorted(channel["policy"].items())
            for policy_index, (_, value) in enumerate(sorted_policy):
                if isinstance(value, list):
                    lines.append(
                        "    static const char* const {}_policy_{}_strings[] = {{".format(
                            prefix, policy_index
                        )
                    )
                    for item in value:
                        lines.append("      {},".format(cpp_string(item)))
                    lines.append("    };")
            if sorted_policy:
                lines.append(
                    "    static const PolicyMetadata {}_policy[] = {{".format(prefix)
                )
                for policy_index, (key, value) in enumerate(sorted_policy):
                    kind = ""
                    string_value = '""'
                    integer_value = "0LL"
                    number_value = "0.0"
                    boolean_value = "false"
                    array_value = "nullptr"
                    array_size = "0u"
                    if isinstance(value, bool):
                        kind = "PolicyValueKind::kBoolean"
                        boolean_value = cpp_bool(value)
                    elif isinstance(value, int):
                        kind = "PolicyValueKind::kInteger"
                        integer_value = cpp_int64(value)
                    elif isinstance(value, float):
                        kind = "PolicyValueKind::kNumber"
                        number_value = repr(value)
                    elif isinstance(value, str):
                        kind = "PolicyValueKind::kString"
                        string_value = cpp_string(value)
                    elif isinstance(value, list) and all(
                        isinstance(item, str) for item in value
                    ):
                        kind = "PolicyValueKind::kStringArray"
                        array_value = "{}_policy_{}_strings".format(
                            prefix, policy_index
                        )
                        array_size = "sizeof({0}) / sizeof({0}[0])".format(
                            array_value
                        )
                    else:
                        raise ValueError(
                            "channel {} policy {} has an unsupported value".format(
                                channel["id"], key
                            )
                        )
                    lines.append(
                        "      {{{}, {}, {}, {}, {}, {}, {}, {}}},".format(
                            cpp_string(key),
                            kind,
                            string_value,
                            integer_value,
                            number_value,
                            boolean_value,
                            array_value,
                            array_size,
                        )
                    )
                lines.append("    };")

        lines.append("    static const ChannelMetadata channels[] = {")
        for channel_index, channel in enumerate(profile["channels"]):
            prefix = "profile_{}_channel_{}".format(profile_index, channel_index)
            endpoint_pointer = (
                "{}_endpoints".format(prefix) if channel["endpoints"] else "nullptr"
            )
            endpoint_count = (
                "sizeof({0}) / sizeof({0}[0])".format(endpoint_pointer)
                if channel["endpoints"]
                else "0u"
            )
            observes_pointer = (
                "{}_observes".format(prefix) if channel["observes"] else "nullptr"
            )
            observes_count = (
                "sizeof({0}) / sizeof({0}[0])".format(observes_pointer)
                if channel["observes"]
                else "0u"
            )
            policy_pointer = (
                "{}_policy".format(prefix) if channel["policy"] else "nullptr"
            )
            policy_count = (
                "sizeof({0}) / sizeof({0}[0])".format(policy_pointer)
                if channel["policy"]
                else "0u"
            )
            operation = channel["operation_contract"]
            lines.append(
                "      {{{}, {}, {}, {}, {}u, {}u, {}, {}u, {}u, {}, {}, {}, {}, {}, {}, {{{}, {}, {}, {}}}}},".format(
                    cpp_string(channel["id"]),
                    KIND_ENUM[channel["kind"]],
                    cpp_string(channel["processor"]),
                    cpp_string(channel["operation_id"]),
                    channel["input_message_id"],
                    channel["output_message_id"],
                    repr(channel["output_rate_hz"]),
                    channel["operation_timeout_millis"],
                    channel["stale_after_millis"],
                    endpoint_pointer,
                    endpoint_count,
                    observes_pointer,
                    observes_count,
                    policy_pointer,
                    policy_count,
                    cpp_string(operation.get("side_effect", "")),
                    cpp_string(operation.get("idempotency", "")),
                    cpp_bool(operation.get("cancellation_supported", False)),
                    cpp_bool(operation.get("deadline_required", False)),
                )
            )
        lines.extend(
            [
                "    };",
                "    *count = sizeof(channels) / sizeof(channels[0]);",
                "    return channels;",
                "  }",
            ]
        )
    lines.extend(
        [
            "  *count = 0u;",
            "  return nullptr;",
            "}",
            "",
            "inline bool channelMetadata(",
            "    const std::string& profile_id, const std::string& channel_id, ChannelMetadata* out) {",
            "  if (out == nullptr) {",
            "    return false;",
            "  }",
            "  std::size_t count = 0u;",
            "  const ChannelMetadata* channels = profileChannels(profile_id, &count);",
            "  for (std::size_t index = 0u; index < count; ++index) {",
            "    if (channel_id == channels[index].channel_id) {",
            "      *out = channels[index];",
            "      return true;",
            "    }",
            "  }",
            "  return false;",
            "}",
            "",
            "inline const EndpointMetadata* channelEndpoint(",
            "    const ChannelMetadata& channel, EndpointKind kind, const std::string& role) {",
            "  for (std::size_t index = 0u; index < channel.endpoint_count; ++index) {",
            "    const EndpointMetadata& endpoint = channel.endpoints[index];",
            "    if (endpoint.kind == kind && role == endpoint.role) {",
            "      return &endpoint;",
            "    }",
            "  }",
            "  return nullptr;",
            "}",
            "",
            "inline bool channelObserves(",
            "    const ChannelMetadata& channel, const std::string& channel_id) {",
            "  for (std::size_t index = 0u; index < channel.observes_count; ++index) {",
            "    if (channel_id == channel.observes[index]) {",
            "      return true;",
            "    }",
            "  }",
            "  return false;",
            "}",
            "",
            "inline const PolicyMetadata* channelPolicy(",
            "    const ChannelMetadata& channel, const std::string& key) {",
            "  for (std::size_t index = 0u; index < channel.policy_count; ++index) {",
            "    if (key == channel.policy[index].key) {",
            "      return &channel.policy[index];",
            "    }",
            "  }",
            "  return nullptr;",
            "}",
            "",
            "inline bool channelPolicyBoolean(",
            "    const ChannelMetadata& channel, const std::string& key, bool* out) {",
            "  const PolicyMetadata* value = channelPolicy(channel, key);",
            "  if (out == nullptr || value == nullptr || value->kind != PolicyValueKind::kBoolean) {",
            "    return false;",
            "  }",
            "  *out = value->boolean_value;",
            "  return true;",
            "}",
            "",
            "inline bool channelPolicyInteger(",
            "    const ChannelMetadata& channel, const std::string& key, std::int64_t* out) {",
            "  const PolicyMetadata* value = channelPolicy(channel, key);",
            "  if (out == nullptr || value == nullptr || value->kind != PolicyValueKind::kInteger) {",
            "    return false;",
            "  }",
            "  *out = value->integer_value;",
            "  return true;",
            "}",
            "",
            "inline bool channelPolicyNumber(",
            "    const ChannelMetadata& channel, const std::string& key, double* out) {",
            "  const PolicyMetadata* value = channelPolicy(channel, key);",
            "  if (out == nullptr || value == nullptr || value->kind != PolicyValueKind::kNumber) {",
            "    return false;",
            "  }",
            "  *out = value->number_value;",
            "  return true;",
            "}",
            "",
            "inline bool channelPolicyString(",
            "    const ChannelMetadata& channel, const std::string& key, const char** out) {",
            "  const PolicyMetadata* value = channelPolicy(channel, key);",
            "  if (out == nullptr || value == nullptr || value->kind != PolicyValueKind::kString) {",
            "    return false;",
            "  }",
            "  *out = value->string_value;",
            "  return true;",
            "}",
            "",
            "inline bool channelPolicyStringArray(",
            "    const ChannelMetadata& channel, const std::string& key,",
            "    const char* const** out, std::size_t* count) {",
            "  const PolicyMetadata* value = channelPolicy(channel, key);",
            "  if (out == nullptr || count == nullptr || value == nullptr ||",
            "      value->kind != PolicyValueKind::kStringArray) {",
            "    return false;",
            "  }",
            "  *out = value->string_array_value;",
            "  *count = value->string_array_size;",
            "  return true;",
            "}",
        ]
    )
    lines.extend(
        [
            "}  // namespace contract",
            "}}  // namespace {}".format(cpp_namespace),
            "",
        ]
    )
    return "\n".join(lines)


def main():
    args = parse_args()
    registry_fingerprint, messages = load_registry(Path(args.registry))
    profiles = load_profile(
        Path(args.profile_file), Path(args.profile_schema), messages
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        generate(
            registry_fingerprint,
            messages,
            profiles,
            args.definition_id,
            args.cpp_namespace,
        ),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
