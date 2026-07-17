#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <google/protobuf/message.h>

#include "xgc/adapter/v1/adapter.pb.h"
#include "xgc/robot/v1/message.pb.h"
#include "xgc/v1/message.pb.h"

namespace xgc2_ros1_robot_adapter {

constexpr std::uint32_t kRobotAdapterSpecSchemaVersion = 2u;

struct InstanceSpecFence {
  std::string instance_id;
  std::uint64_t process_generation = 0;
  std::uint64_t revision = 0;
  std::string spec_digest;

  bool matches(const InstanceSpecFence &other) const;
};

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

struct RobotAdapterConfig {
  InstanceSpecFence fence;
  std::string scope_kind;
  std::string scope_key;
  std::map<std::string, std::string> scope_attributes;
  std::string asset_digest;
  std::vector<RobotConfig> robots;
};

struct MessageSchema {
  std::uint32_t message_id = 0;
  std::string type_name;
  std::uint32_t version = 0;
  std::uint64_t fingerprint = 0;
};

// Decodes the typed robot-domain configuration carried by one generic instance
// spec. The expected schema comes from the installed protobuf registry through
// the product-generated contract metadata; type name/version alone are not an
// adequate wire-compatibility fence. The output is replaced only after the
// complete document is valid.
bool DecodeRobotAdapterConfig(
    const xgc::adapter::v1::AdapterInstanceSpec &instance_spec,
    const MessageSchema &expected_configuration_schema,
    RobotAdapterConfig *output, std::string *error);

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
bool BuildRobotMessage(const RobotMessageContext &context,
                       const MessageSchema &schema,
                       const google::protobuf::Message &semantic_payload,
                       xgc::robot::v1::RobotMessage *output,
                       std::string *error);

bool SerializeRobotMessageItem(
    const RobotMessageContext &context, const MessageSchema &schema,
    const google::protobuf::Message &semantic_payload, std::string *output,
    std::string *error);

} // namespace xgc2_ros1_robot_adapter
