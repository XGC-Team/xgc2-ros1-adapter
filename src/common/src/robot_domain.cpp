#include "xgc2_ros1_robot_adapter/robot_domain.hpp"

#include <regex>
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

} // namespace

bool ValidRobotIdentity(const std::string &robot_id) { return validRobotId(robot_id); }
bool EncodeRobotMessage(const RobotMessageContext &context, const MessageSchema &schema,
                        const google::protobuf::Message &semantic_payload,
                        xgc::robot::v1::RobotMessage *output, std::string *error) {
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

} // namespace xgc2_ros1_robot_adapter
