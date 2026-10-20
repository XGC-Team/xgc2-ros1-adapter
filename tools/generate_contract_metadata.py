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


PROFILE_SCHEMA_ID = "xgc.robot.adapter-profile/v4"
PROFILE_CONTRACT_DIGEST_SCHEMA = "xgc.robot.profile-contract-digest/v4"
KINDS = {"stream_out", "operation"}
INPUT_KINDS = {"operation"}
OUTPUT_KINDS = {"stream_out", "operation"}
INPUT_ROLES = {"control", "request"}
OUTPUT_ROLES = {"telemetry", "diagnostic", "response"}
OPERATION_OUTPUT_ROLES = {"request", "response"}
ENDPOINT_PARAMETER = re.compile(r"\{([a-z][a-z0-9_]*)\}")
CPP_NAMESPACE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(::[A-Za-z_][A-Za-z0-9_]*)*$")
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
    "force-disarm": "xgc.semantic.aerial.v1.ForceDisarmRequest",
    "set-motion-intent": "xgc.semantic.common.v1.RemoteControlIntentRequest",
    "release-motion-intent": "xgc.semantic.common.v1.RemoteControlIntentRequest",
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
    parser.add_argument("--profile-file", required=True, action="append")
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
    elif operation_id in ("set-motion-intent", "release-motion-intent"):
        properties = {
            "gear": {"type": "integer", "minimum": 1, "maximum": 3},
            "longitudinal": {
                "type": "integer",
                "minimum": -1,
                "maximum": 1,
            },
            "lateral": {"type": "integer", "minimum": -1, "maximum": 1},
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
    if schema_marker != PROFILE_SCHEMA_ID:
        raise ValueError(
            "{}: profile schema does not declare {}".format(path, PROFILE_SCHEMA_ID)
        )
    return document


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


def validate_profile_document(profile_path, profile, schema, messages):
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

    if profile["schema"] != PROFILE_SCHEMA_ID:
        raise ValueError(
            "{}: unsupported profile schema {}".format(profile_path, profile["schema"])
        )
    if profile["native_protocol"] != "ros1":
        raise ValueError("{}: profile is not a ROS1 native mapping".format(profile_path))

    for parameter_name, definition in profile["parameters"].items():
        if "pattern" in definition:
            validate_portable_parameter_pattern(
                definition["pattern"],
                "{}: parameter {} pattern".format(profile_path, parameter_name),
            )

    seen_channels = set()
    channels_by_id = {}
    operation_ids = set()
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
            # Only the operation that releases a held motion intent may say so:
            # `safety` is what keeps an operation dispatchable after an unknown
            # result, so it must not be claimable by an arbitrary operation.
            safety = (operation_contract or {}).get("safety")
            if (safety == "release") != (operation_id == "release-motion-intent"):
                raise ValueError(
                    "{}: operation {} must declare safety {} (release-motion-intent is the only release operation)".format(
                        profile_path,
                        operation_id,
                        "release" if operation_id == "release-motion-intent" else "none",
                    )
                )
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
    schema = load_profile_schema(schema_path)
    channels = validate_profile_document(profile_path, profile, schema, messages)
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


def load_profiles(paths, schema_path, messages):
    if isinstance(paths, (str, Path)):
        paths = [paths]
    profiles = {}
    for path in paths:
        loaded = load_profile(Path(path), schema_path, messages)
        if profiles.keys() & loaded.keys():
            raise ValueError("duplicate robot profile identity")
        profiles.update(loaded)
    if not profiles or len({p["robot_kind"] for p in profiles.values()}) != 1:
        raise ValueError("a native Adapter serves profiles for one robot kind")
    return profiles


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


def policy_initializer(channel, prefix, lines):
    """Emit the typed policy table of one channel; returns (pointer, count)."""
    if not channel["policy"]:
        return "nullptr", "0u"
    sorted_policy = sorted(channel["policy"].items())
    for policy_index, (_, value) in enumerate(sorted_policy):
        if isinstance(value, list):
            lines.append(
                "  static const char* const {}_policy_{}_strings[] = {{".format(
                    prefix, policy_index
                )
            )
            for item in value:
                lines.append("      {},".format(cpp_string(item)))
            lines.append("  };")
    lines.append("  static const PolicyMetadata {}_policy[] = {{".format(prefix))
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
        elif isinstance(value, list) and all(isinstance(item, str) for item in value):
            kind = "PolicyValueKind::kStringArray"
            array_value = "{}_policy_{}_strings".format(prefix, policy_index)
            array_size = "sizeof({0}) / sizeof({0}[0])".format(array_value)
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
    lines.append("  };")
    return "{}_policy".format(prefix), "sizeof({0}_policy) / sizeof({0}_policy[0])".format(prefix)


def generate(
    registry_fingerprint,
    messages,
    profiles,
    provider_definition_id,
    cpp_namespace,
):
    """Emit the tables of one provider's contract.

    The header contains data only. The types and the lookups over them are
    xgc2_ros1_robot_adapter/contract.hpp, shared by every provider.
    """
    if not CPP_NAMESPACE.fullmatch(cpp_namespace):
        raise ValueError("invalid C++ namespace: {}".format(cpp_namespace))
    if not profiles:
        raise ValueError("a native Adapter requires a robot profile")
    required_messages = sorted(
        {1}
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
            "registry is missing robot messages {}".format(missing_runtime_messages)
        )
    profile_digests = {
        profile_id: profile_contract_digest(
            catalog_profile_body(profile, messages, provider_definition_id)
        )
        for profile_id, profile in profiles.items()
    }
    lines = [
        "// Generated by tools/generate_contract_metadata.py; do not edit.",
        "#pragma once",
        "",
        '#include "xgc2_ros1_robot_adapter/contract.hpp"',
        "",
        "namespace {} {{".format(cpp_namespace),
        "",
        "inline const xgc2_ros1_robot_adapter::contract::Contract &InstalledContract() {",
        "  using namespace xgc2_ros1_robot_adapter::contract;",
        "  static const MessageEntry messages[] = {",
    ]
    for message_id in required_messages:
        item = messages[message_id]
        lines.append(
            "      {{{}u, {{{}, {}u, {}ULL}}}},".format(
                message_id,
                cpp_string(item["type"]),
                item["version"],
                item["fingerprint"],
            )
        )
    lines.append("  };")
    profile_entries = []
    for profile_index, (profile_id, profile) in enumerate(profiles.items()):
        profile_prefix = "profile_{}".format(profile_index)
        parameters = sorted(profile["parameters"].items())
        parameters_pointer, parameters_count = "nullptr", "0u"
        if parameters:
            lines.append(
                "  static const ParameterMetadata {}_parameters[] = {{".format(
                    profile_prefix
                )
            )
            for name, definition in parameters:
                lines.append(
                    "      {{{}, {}, {}, {}, {}}},".format(
                        cpp_string(name),
                        PARAMETER_TYPE_ENUM[definition["type"]],
                        cpp_bool(definition["required"]),
                        cpp_string(definition.get("pattern", "")),
                        cpp_string(definition["description"]),
                    )
                )
            lines.append("  };")
            parameters_pointer = "{}_parameters".format(profile_prefix)
            parameters_count = "sizeof({0}) / sizeof({0}[0])".format(
                parameters_pointer
            )
        operations = catalog_semantics(profile)["operations"]
        operations_pointer, operations_count = "nullptr", "0u"
        if operations:
            lines.append(
                "  static const OperationMetadata {}_operations[] = {{".format(
                    profile_prefix
                )
            )
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
            lines.append("  };")
            operations_pointer = "{}_operations".format(profile_prefix)
            operations_count = "sizeof({0}) / sizeof({0}[0])".format(
                operations_pointer
            )
        channel_rows = []
        for channel_index, channel in enumerate(profile["channels"]):
            prefix = "{}_channel_{}".format(profile_prefix, channel_index)
            endpoint_pointer, endpoint_count = "nullptr", "0u"
            if channel["endpoints"]:
                lines.append(
                    "  static const EndpointMetadata {}_endpoints[] = {{".format(prefix)
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
                lines.append("  };")
                endpoint_pointer = "{}_endpoints".format(prefix)
                endpoint_count = "sizeof({0}) / sizeof({0}[0])".format(endpoint_pointer)
            observes_pointer, observes_count = "nullptr", "0u"
            if channel["observes"]:
                lines.append(
                    "  static const char* const {}_observes[] = {{".format(prefix)
                )
                for observed in channel["observes"]:
                    lines.append("      {},".format(cpp_string(observed)))
                lines.append("  };")
                observes_pointer = "{}_observes".format(prefix)
                observes_count = "sizeof({0}) / sizeof({0}[0])".format(observes_pointer)
            policy_pointer, policy_count = policy_initializer(channel, prefix, lines)
            operation = channel["operation_contract"]
            channel_rows.append(
                "      {{{}, {}, {}, {}, {}u, {}u, {}, {}u, {}u, {}, {}, {}, {}, {}, {}, {{{}, {}, {}, {}, {}}}}},".format(
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
                    cpp_string(operation.get("safety", "")),
                )
            )
        lines.append(
            "  static const ChannelMetadata {}_channels[] = {{".format(profile_prefix)
        )
        lines.extend(channel_rows)
        lines.append("  };")
        profile_entries.append(
            "      {{{}, {}, {}, {}, {}, {}, {}_channels, sizeof({}_channels) / sizeof({}_channels[0])}},".format(
                cpp_string(profile_id),
                cpp_string(profile_digests[profile_id]),
                parameters_pointer,
                parameters_count,
                operations_pointer,
                operations_count,
                profile_prefix,
                profile_prefix,
                profile_prefix,
            )
        )
    lines.append("  static const ProfileMetadata profiles[] = {")
    lines.extend(profile_entries)
    lines.append("  };")
    lines.extend(
        [
            "  static const Contract contract(",
            "      {}, {}ULL, messages,".format(
                cpp_string(provider_definition_id), registry_fingerprint
            ),
            "      sizeof(messages) / sizeof(messages[0]), profiles,",
            "      sizeof(profiles) / sizeof(profiles[0]));",
            "  return contract;",
            "}",
            "",
            "}  // namespace " + cpp_namespace,
            "",
        ]
    )
    return "\n".join(lines)


def main():
    args = parse_args()
    registry_fingerprint, messages = load_registry(Path(args.registry))
    profiles = load_profiles(
        args.profile_file, Path(args.profile_schema), messages
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
