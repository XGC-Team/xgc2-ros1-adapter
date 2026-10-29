#pragma once

#include <cstdint>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include <google/protobuf/message.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc/v1/message.pb.h"

namespace xgc2_ros1_robot_adapter {

// Bounds one native telemetry envelope and the variable-size raw field kept
// for its selected latest snapshot. It is not a process RSS bound.
inline constexpr std::size_t kMaximumTelemetryItemBytes = 65536;

struct RobotChannelConfig {
  std::string channel_id;
  bool enabled = false;
};

struct RobotConfig {
  std::string robot_id;
  std::string profile_id;
  std::string profile_digest;
  std::map<std::string, std::string> parameters;
  std::vector<RobotChannelConfig> channels;
};

struct MessageSchema {
  std::uint32_t message_id = 0;
  std::string type_name;
  std::uint32_t version = 0;
  std::uint64_t fingerprint = 0;
};

struct RobotMessageContext {
  std::string robot_id;
  std::string channel_id;
  std::uint64_t sequence = 0;
  bool has_source_time = false;
  std::int64_t source_time_nanos = 0;
  xgc::v1::ClockDomain source_clock_domain =
      xgc::v1::CLOCK_DOMAIN_UNSPECIFIED;
  std::int64_t observed_unix_nanos = 0;
};

// Builds the domain-neutral Message and its robot routing wrapper. ROS callers
// must use CLOCK_DOMAIN_NATIVE for wall/native ROS time and
// CLOCK_DOMAIN_SIMULATION for simulated time.
// Type servers check immutable member identity and generated mappings at
// registration. Their hot path only encodes current values into the envelope.
bool ValidRobotIdentity(const std::string &robot_id);
bool EncodeRobotMessage(const RobotMessageContext &context,
                        const MessageSchema &schema,
                        const google::protobuf::Message &semantic_payload,
                        xgc::robot::v1::RobotMessage *output, std::string *error);

} // namespace xgc2_ros1_robot_adapter
