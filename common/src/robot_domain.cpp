#include "xgc2_ros1_robot_adapter/robot_domain.hpp"

#include <regex>
#include <set>
#include <utility>

namespace xgc2_ros1_robot_adapter {
namespace {

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

bool validRobotId(const std::string &value) {
  static const std::regex pattern("^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$");
  return std::regex_match(value, pattern);
}

bool validProfileId(const std::string &value) {
  static const std::regex pattern(
      "^[a-z0-9]+([.-][a-z0-9]+)*\\.v[1-9][0-9]*$");
  return std::regex_match(value, pattern);
}

bool validChannelId(const std::string &value) {
  static const std::regex pattern("^[a-z][a-z0-9]*([.-][a-z0-9]+)*$");
  return std::regex_match(value, pattern);
}

bool validRawSha256(const std::string &value) {
  static const std::regex pattern("^[0-9a-f]{64}$");
  return std::regex_match(value, pattern);
}

bool validCanonicalSha256(const std::string &value) {
  static const std::regex pattern("^sha256:[0-9a-f]{64}$");
  return std::regex_match(value, pattern);
}

bool validRosSourceClock(xgc::v1::ClockDomain clock_domain) {
  return clock_domain == xgc::v1::CLOCK_DOMAIN_NATIVE ||
         clock_domain == xgc::v1::CLOCK_DOMAIN_SIMULATION;
}

} // namespace

bool InstanceSpecFence::matches(const InstanceSpecFence &other) const {
  return instance_id == other.instance_id &&
         process_generation == other.process_generation &&
         revision == other.revision && spec_digest == other.spec_digest;
}

bool DecodeRobotAdapterConfig(
    const xgc::adapter::v1::AdapterInstanceSpec &instance_spec,
    const MessageSchema &expected_configuration_schema,
    RobotAdapterConfig *output, std::string *error) {
  if (output == nullptr)
    return fail(error, "RobotAdapterConfig output must not be null");
  if (instance_spec.instance_id().empty())
    return fail(error, "instance spec is missing instance_id");
  if (instance_spec.process_generation() == 0)
    return fail(error, "instance spec process_generation must be positive");
  if (instance_spec.revision() == 0)
    return fail(error, "instance spec revision must be positive");
  if (!validCanonicalSha256(instance_spec.spec_digest())) {
    return fail(error,
                "instance spec digest must use sha256:<lowercase-hex>");
  }
  if (instance_spec.scope().kind().empty() ||
      instance_spec.scope().key().empty()) {
    return fail(error, "instance spec must contain a complete scope");
  }
  if (instance_spec.scope().kind() != "robot-group" ||
      instance_spec.scope().attributes_size() != 3 ||
      instance_spec.scope().attributes().find("target-id") ==
          instance_spec.scope().attributes().end() ||
      instance_spec.scope().attributes().find("run-id") ==
          instance_spec.scope().attributes().end() ||
      instance_spec.scope().attributes().find("provider") ==
          instance_spec.scope().attributes().end()) {
    return fail(error,
                "robot Adapter scope must be robot-group with exactly "
                "target-id, run-id, and provider");
  }
  if (!instance_spec.has_configuration() ||
      !instance_spec.configuration().has_schema()) {
    return fail(error, "instance spec is missing typed configuration");
  }

  const auto &configuration = instance_spec.configuration();
  const auto &schema = configuration.schema();
  const std::string expected_type =
      xgc::robot::v1::RobotAdapterSpec::descriptor()->full_name();
  if (expected_configuration_schema.message_id == 0 ||
      expected_configuration_schema.type_name != expected_type ||
      expected_configuration_schema.version != kRobotAdapterSpecSchemaVersion ||
      expected_configuration_schema.fingerprint == 0) {
    return fail(error, "installed RobotAdapterSpec schema metadata is invalid");
  }
  if (schema.message_id() != expected_configuration_schema.message_id ||
      schema.type_name() != expected_configuration_schema.type_name ||
      schema.schema_version() != expected_configuration_schema.version ||
      schema.schema_fingerprint() !=
          expected_configuration_schema.fingerprint) {
    return fail(error,
                "instance configuration schema does not match the installed "
                "xgc.robot.v1.RobotAdapterSpec registry entry");
  }
  if (configuration.encoding() != xgc::v1::PAYLOAD_ENCODING_PROTOBUF) {
    return fail(error, "RobotAdapterSpec configuration must use protobuf encoding");
  }

  xgc::robot::v1::RobotAdapterSpec robot_spec;
  if (!robot_spec.ParseFromString(configuration.value()))
    return fail(error, "RobotAdapterSpec protobuf payload is malformed");
  if (!validRawSha256(robot_spec.asset_digest())) {
    return fail(error,
                "RobotAdapterSpec asset_digest must be raw lowercase SHA-256");
  }
  if (robot_spec.robots().empty())
    return fail(error, "RobotAdapterSpec must contain at least one robot");

  RobotAdapterConfig candidate;
  candidate.fence.instance_id = instance_spec.instance_id();
  candidate.fence.process_generation = instance_spec.process_generation();
  candidate.fence.revision = instance_spec.revision();
  candidate.fence.spec_digest = instance_spec.spec_digest();
  candidate.scope_kind = instance_spec.scope().kind();
  candidate.scope_key = instance_spec.scope().key();
  candidate.scope_attributes.insert(instance_spec.scope().attributes().begin(),
                                    instance_spec.scope().attributes().end());
  candidate.asset_digest = robot_spec.asset_digest();
  candidate.robots.reserve(static_cast<std::size_t>(robot_spec.robots_size()));

  std::set<std::string> robot_ids;
  for (const auto &robot : robot_spec.robots()) {
    if (!validRobotId(robot.robot_id()))
      return fail(error, "RobotAdapterSpec contains an invalid robot_id: " +
                             robot.robot_id());
    if (!robot_ids.insert(robot.robot_id()).second)
      return fail(error, "RobotAdapterSpec repeats robot_id: " +
                             robot.robot_id());
    if (!validProfileId(robot.profile_id())) {
      return fail(error, "robot " + robot.robot_id() +
                             " contains an invalid profile_id");
    }
    if (!validRawSha256(robot.profile_digest())) {
      return fail(error, "robot " + robot.robot_id() +
                             " contains an invalid profile_digest");
    }

    RobotConfig local_robot;
    local_robot.robot_id = robot.robot_id();
    local_robot.profile_id = robot.profile_id();
    local_robot.profile_digest = robot.profile_digest();
    local_robot.parameters.insert(robot.parameters().begin(),
                                  robot.parameters().end());
    local_robot.channels.reserve(
        static_cast<std::size_t>(robot.channels_size()));

    std::set<std::string> channel_ids;
    for (const auto &channel : robot.channels()) {
      if (!validChannelId(channel.channel_id())) {
        return fail(error, "robot " + robot.robot_id() +
                               " contains an invalid channel_id: " +
                               channel.channel_id());
      }
      if (!channel_ids.insert(channel.channel_id()).second) {
        return fail(error, "robot " + robot.robot_id() +
                               " repeats channel: " + channel.channel_id());
      }
      RobotChannelConfig local_channel;
      local_channel.channel_id = channel.channel_id();
      local_channel.enabled = channel.enabled();
      local_channel.parameters.insert(channel.parameters().begin(),
                                      channel.parameters().end());
      local_robot.channels.push_back(std::move(local_channel));
    }
    candidate.robots.push_back(std::move(local_robot));
  }

  *output = std::move(candidate);
  if (error != nullptr)
    error->clear();
  return true;
}

bool BuildRobotMessage(const RobotMessageContext &context,
                       const MessageSchema &schema,
                       const google::protobuf::Message &semantic_payload,
                       xgc::robot::v1::RobotMessage *output,
                       std::string *error) {
  if (output == nullptr)
    return fail(error, "RobotMessage output must not be null");
  if (!validRobotId(context.robot_id))
    return fail(error, "RobotMessage contains an invalid robot_id");
  if (!validChannelId(context.channel_id))
    return fail(error, "RobotMessage contains an invalid channel_id");
  if (context.sequence == 0)
    return fail(error, "RobotMessage sequence must be positive");
  if (context.observed_unix_nanos <= 0)
    return fail(error, "RobotMessage observed_unix_nanos must be positive");
  if (context.has_source_time &&
      !validRosSourceClock(context.source_clock_domain)) {
    return fail(error,
                "ROS source time must use native or simulation clock domain");
  }
  if (schema.message_id == 0 || schema.type_name.empty() ||
      schema.version == 0 || schema.fingerprint == 0) {
    return fail(error, "semantic message schema metadata is incomplete");
  }
  if (semantic_payload.GetDescriptor()->full_name() != schema.type_name) {
    return fail(error, "semantic payload type does not match schema metadata");
  }

  std::string encoded_payload;
  if (!semantic_payload.SerializeToString(&encoded_payload))
    return fail(error, "failed to serialize semantic protobuf payload");

  xgc::robot::v1::RobotMessage candidate;
  candidate.set_robot_id(context.robot_id);
  candidate.set_channel_id(context.channel_id);
  auto *message = candidate.mutable_message();
  message->set_sequence(context.sequence);
  if (context.has_source_time) {
    message->mutable_source_time()->set_nanoseconds(context.source_time_nanos);
    message->mutable_source_time()->set_clock_domain(
        context.source_clock_domain);
  }
  message->set_observed_unix_nanos(context.observed_unix_nanos);
  auto *payload = message->mutable_payload();
  payload->mutable_schema()->set_message_id(schema.message_id);
  payload->mutable_schema()->set_type_name(schema.type_name);
  payload->mutable_schema()->set_schema_version(schema.version);
  payload->mutable_schema()->set_schema_fingerprint(schema.fingerprint);
  payload->set_encoding(xgc::v1::PAYLOAD_ENCODING_PROTOBUF);
  payload->set_value(std::move(encoded_payload));

  *output = std::move(candidate);
  if (error != nullptr)
    error->clear();
  return true;
}

bool SerializeRobotMessageItem(
    const RobotMessageContext &context, const MessageSchema &schema,
    const google::protobuf::Message &semantic_payload, std::string *output,
    std::string *error) {
  if (output == nullptr)
    return fail(error, "serialized RobotMessage output must not be null");
  xgc::robot::v1::RobotMessage message;
  if (!BuildRobotMessage(context, schema, semantic_payload, &message, error))
    return false;

  std::string candidate;
  if (!message.SerializeToString(&candidate))
    return fail(error, "failed to serialize RobotMessage stream item");
  *output = std::move(candidate);
  if (error != nullptr)
    error->clear();
  return true;
}

} // namespace xgc2_ros1_robot_adapter
