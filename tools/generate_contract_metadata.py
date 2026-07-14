#!/usr/bin/env python3

"""Generate one adapter package's compile-time contract view.

The public protobuf product owns message IDs, fingerprints, and adapter
profiles. Each ROS package selects exactly one profile and fails its build if
that installed profile no longer matches the native endpoints it implements.
"""

import argparse
import hashlib
import json
from pathlib import Path

import yaml


PROFILE_FILES = {
    "px4.multirotor.ros1.v1": "ros1/px4-multirotor-ros1-v1.yaml",
    "scout-mini.ros1.v1": "ros1/scout-mini-ros1-v1.yaml",
}

EXPECTED_CHANNELS = {
    "px4.multirotor.ros1.v1": {
        "state.pose": ("stream_out", 0, 2001, 10.0),
        "state.velocity": ("stream_out", 0, 2002, 10.0),
        "state.imu": ("stream_out", 0, 2003, 10.0),
        "state.power": ("stream_out", 0, 2004, 1.0),
        "state.health": ("stream_out", 0, 2005, 2.0),
        "state.flight": ("stream_out", 0, 3001, 2.0),
        "diagnostic.channel-health": ("stream_out", 0, 2010, 1.0),
        "operation.arm": ("operation", 3201, 0, 0.0),
        "operation.mode": ("operation", 3202, 0, 0.0),
        "operation.autopilot-reboot": ("operation", 3203, 0, 0.0),
    },
    "scout-mini.ros1.v1": {
        "state.pose": ("stream_out", 0, 2001, 10.0),
        "state.velocity": ("stream_out", 0, 2002, 10.0),
        "state.imu": ("stream_out", 0, 2003, 10.0),
        "state.power": ("stream_out", 0, 2004, 2.0),
        "state.health": ("stream_out", 0, 2005, 2.0),
        "diagnostic.channel-health": ("stream_out", 0, 2010, 1.0),
    },
}

EXPECTED_PROCESSORS = {
    "px4.multirotor.ros1.v1": {
        "state.pose": {
            "processor": "px4.pose-estimate",
            "inputs": {"pose": {"name": "mavros/local_position/pose", "type": "geometry_msgs/PoseStamped"}},
        },
        "state.velocity": {
            "processor": "px4.velocity-estimate",
            "inputs": {"velocity": {"name": "mavros/local_position/velocity_local", "type": "geometry_msgs/TwistStamped"}},
        },
        "state.imu": {
            "processor": "px4.imu-estimate",
            "inputs": {"imu": {"name": "mavros/imu/data", "type": "sensor_msgs/Imu"}},
        },
        "state.power": {
            "processor": "px4.power-status",
            "inputs": {"battery": {"name": "mavros/battery", "type": "sensor_msgs/BatteryState"}},
        },
        "state.health": {
            "processor": "px4.vehicle-health",
            "inputs": {
                "state": {"name": "mavros/state", "type": "mavros_msgs/State"},
                "extended_state": {"name": "mavros/extended_state", "type": "mavros_msgs/ExtendedState"},
            },
        },
        "state.flight": {
            "processor": "px4.flight-status",
            "inputs": {
                "state": {"name": "mavros/state", "type": "mavros_msgs/State"},
                "extended_state": {"name": "mavros/extended_state", "type": "mavros_msgs/ExtendedState"},
            },
        },
        "diagnostic.channel-health": {
            "processor": "common.channel-health",
            "observes": ["state.pose", "state.velocity", "state.imu", "state.power", "state.health", "state.flight"],
        },
        "operation.arm": {
            "processor": "px4.arm",
            "service": {"name": "mavros/cmd/arming", "type": "mavros_msgs/CommandBool"},
            "policy": {"timeout_ms": 5000},
        },
        "operation.mode": {
            "processor": "px4.mode",
            "service": {"name": "mavros/set_mode", "type": "mavros_msgs/SetMode"},
            "policy": {"timeout_ms": 5000},
        },
        "operation.autopilot-reboot": {
            "processor": "px4.autopilot-reboot",
            "service": {"name": "mavros/cmd/command", "type": "mavros_msgs/CommandLong"},
            "policy": {"mav_command": 246, "normal_reboot_param1": 1, "timeout_ms": 5000},
        },
    },
    "scout-mini.ros1.v1": {
        "state.pose": {
            "processor": "scout-mini.pose-estimate",
            "inputs": {"odometry": {"name": "odom", "type": "nav_msgs/Odometry"}},
        },
        "state.velocity": {
            "processor": "scout-mini.velocity-estimate",
            "inputs": {"odometry": {"name": "odom", "type": "nav_msgs/Odometry"}},
        },
        "state.imu": {
            "processor": "scout-mini.imu-estimate",
            "inputs": {"imu": {"name": "imu/data_raw", "type": "sensor_msgs/Imu"}},
        },
        "state.power": {
            "processor": "scout-mini.power-status",
            "inputs": {"chassis_status": {"name": "scout_status", "type": "scout_msgs/ScoutStatus"}},
        },
        "state.health": {
            "processor": "scout-mini.vehicle-health",
            "inputs": {"chassis_status": {"name": "scout_status", "type": "scout_msgs/ScoutStatus"}},
        },
        "diagnostic.channel-health": {
            "processor": "common.channel-health",
            "observes": ["state.pose", "state.velocity", "state.imu", "state.power", "state.health"],
        },
    },
}

KIND_ENUM = {
    "stream_out": "::xgc::adapter::v1::CHANNEL_KIND_STREAM_OUT",
    "stream_in": "::xgc::adapter::v1::CHANNEL_KIND_STREAM_IN",
    "request_response": "::xgc::adapter::v1::CHANNEL_KIND_REQUEST_RESPONSE",
    "operation": "::xgc::adapter::v1::CHANNEL_KIND_OPERATION",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--profiles-dir", required=True)
    parser.add_argument("--profile-id", required=True, choices=sorted(PROFILE_FILES))
    parser.add_argument("--cpp-namespace", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def cpp_string(value):
    return json.dumps(str(value))


def load_registry(path, required_messages):
    document = json.loads(path.read_text(encoding="utf-8"))
    fingerprint = int(document["registry_fingerprint"])
    messages = {int(item["id"]): item for item in document["messages"]}
    missing = [message_id for message_id in required_messages if message_id not in messages]
    if missing:
        raise ValueError("installed registry is missing message IDs: {}".format(missing))
    return fingerprint, messages


def load_profile(root, profile_id):
    profiles = {}
    for selected_id, relative in PROFILE_FILES.items():
        if selected_id != profile_id:
            continue
        path = root / relative
        raw = path.read_bytes()
        profile = yaml.safe_load(raw)
        if profile.get("schema") != "xgc2.adapter-profile.v1":
            raise ValueError("{} has unsupported schema".format(path))
        if profile.get("profile_id") != profile_id:
            raise ValueError("{} profile_id does not match its installed path".format(path))
        if profile.get("native_protocol") != "ros1":
            raise ValueError("{} is not a ROS1 profile".format(path))
        if profile.get("namespace_parameter") != "namespace":
            raise ValueError("{} must use the namespace plan parameter".format(path))
        parameters = profile.get("parameters") or {}
        if set(parameters) != {"namespace"}:
            raise ValueError("{} has unsupported profile parameters: {}".format(path, sorted(parameters)))

        actual = {}
        ordered_channels = []
        for channel in profile.get("channels") or []:
            channel_id = str(channel["id"])
            if channel_id in actual:
                raise ValueError("{} repeats channel {}".format(path, channel_id))
            kind = str(channel["kind"])
            input_id = int(channel.get("input_message_id", 0))
            output_id = int(channel.get("output_message_id", 0))
            rate = float(channel.get("output_rate_hz", 0.0))
            actual[channel_id] = (kind, input_id, output_id, rate)
            processor_fields = dict(channel)
            for field in ("id", "kind", "input_message_id", "output_message_id", "output_rate_hz"):
                processor_fields.pop(field, None)
            if processor_fields != EXPECTED_PROCESSORS[profile_id].get(channel_id):
                raise ValueError(
                    "{} channel {} native processor/endpoints/policy do not match the implementation".format(
                        path, channel_id
                    )
                )
            ordered_channels.append(
                {
                    "id": channel_id,
                    "kind": kind,
                    "input": input_id,
                    "output": output_id,
                    "rate": rate,
                }
            )

        expected = EXPECTED_CHANNELS[profile_id]
        if actual != expected:
            missing_channels = sorted(set(expected) - set(actual))
            extra_channels = sorted(set(actual) - set(expected))
            mismatched = sorted(
                channel_id
                for channel_id in set(actual) & set(expected)
                if actual[channel_id] != expected[channel_id]
            )
            raise ValueError(
                "{} does not match the implemented adapter profile; missing={}, extra={}, mismatched={}".format(
                    path, missing_channels, extra_channels, mismatched
                )
            )
        if any(channel["kind"] in ("stream_in", "request_response") for channel in ordered_channels):
            raise ValueError("{} exposes an unsupported high-frequency/input channel".format(path))

        profiles[profile_id] = {
            "digest": hashlib.sha256(raw).hexdigest(),
            "channels": ordered_channels,
        }
    return profiles


def generate(registry_fingerprint, messages, profiles, cpp_namespace):
    required_messages = sorted(
        {
            message_id
            for profile in profiles.values()
            for channel in profile["channels"]
            for message_id in (channel["input"], channel["output"])
            if message_id
        }
    )
    lines = [
        "#pragma once",
        "",
        "#include <cstdint>",
        "#include <string>",
        "#include <vector>",
        "",
        '#include "xgc/adapter/v1/adapter.pb.h"',
        "",
        "namespace {} {{".format(cpp_namespace),
        "namespace contract {",
        "",
        "constexpr std::uint32_t kProtocolVersion = 1u;",
        "constexpr std::uint64_t kRegistryFingerprint = {}ULL;".format(registry_fingerprint),
        "",
        "struct MessageMetadata {",
        "  std::uint32_t version;",
        "  std::uint64_t fingerprint;",
        "};",
        "",
        "struct ChannelMetadata {",
        "  ::xgc::adapter::v1::ChannelKind kind;",
        "  std::uint32_t input_message_id;",
        "  std::uint32_t output_message_id;",
        "  double output_rate_hz;",
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
                "      *out = MessageMetadata{{{}u, {}ULL}};".format(
                    int(item["version"]), int(item["fingerprint"])
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
                cpp_string(profile_id), cpp_string(profile["digest"])
            )
        )
    lines.extend(["  return nullptr;", "}", "", "inline bool channelMetadata(",
                  "    const std::string& profile_id, const std::string& channel_id, ChannelMetadata* out) {",
                  "  if (out == nullptr) {", "    return false;", "  }"])
    for profile_id, profile in profiles.items():
        lines.append("  if (profile_id == {}) {{".format(cpp_string(profile_id)))
        for channel in profile["channels"]:
            lines.extend(
                [
                    "    if (channel_id == {}) {{".format(cpp_string(channel["id"])),
                    "      *out = ChannelMetadata{{{}, {}u, {}u, {}}};".format(
                        KIND_ENUM[channel["kind"]],
                        channel["input"],
                        channel["output"],
                        repr(channel["rate"]),
                    ),
                    "      return true;",
                    "    }",
                ]
            )
        lines.extend(["    return false;", "  }"])
    lines.extend(["  return false;", "}", "", "inline void addSupportedProfiles(",
                  "    std::vector<::xgc::adapter::v1::ProfileAdvertisement>* profiles) {"])
    for profile_id, profile in profiles.items():
        lines.extend(
            [
                "  {",
                "    profiles->emplace_back();",
                "    auto* profile = &profiles->back();",
                "    profile->set_profile_id({});".format(cpp_string(profile_id)),
                "    profile->set_profile_digest({});".format(cpp_string(profile["digest"])),
            ]
        )
        for channel in profile["channels"]:
            lines.extend(
                [
                    "    {",
                    "      auto* channel = profile->add_channels();",
                    "      channel->set_channel_id({});".format(cpp_string(channel["id"])),
                    "      channel->set_kind({});".format(KIND_ENUM[channel["kind"]]),
                    "      channel->set_input_message_id({}u);".format(channel["input"]),
                    "      channel->set_output_message_id({}u);".format(channel["output"]),
                    "    }",
                ]
            )
        lines.append("  }")
    lines.extend(
        [
            "}",
            "",
            "}  // namespace contract",
            "}}  // namespace {}".format(cpp_namespace),
            "",
        ]
    )
    return "\n".join(lines)


def main():
    args = parse_args()
    profiles = load_profile(Path(args.profiles_dir), args.profile_id)
    required_messages = sorted(
        {
            message_id
            for profile in profiles.values()
            for channel in profile["channels"]
            for message_id in (channel["input"], channel["output"])
            if message_id
        }
    )
    registry_fingerprint, messages = load_registry(
        Path(args.registry), required_messages
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        generate(registry_fingerprint, messages, profiles, args.cpp_namespace),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
