#pragma once
#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include "xgc/semantic/common/v1/control.pb.h"
#include <algorithm>

namespace xgc2_ros1_robot_adapter {
inline bool Enabled(const RobotConfig &config, const std::string &channel) {
  for (const auto &entry : config.channels) if (entry.channel_id == channel) return entry.enabled;
  return false;
}
template<class Channel, class Metadata> bool ValidateCommand(
    const xgc::adapter::v1::OperationRequest &request, const RobotConfig &config,
    const Channel &channel, const Metadata &input, std::string *error) {
  const auto &payload = request.input();
  const auto &schema = payload.schema();
  if (!Enabled(config, channel.channel_id)) *error = "command channel disabled";
  else if (payload.encoding() != xgc::v1::PAYLOAD_ENCODING_PROTOBUF ||
           schema.message_id() != channel.input_message_id || schema.type_name() != input.type_name ||
           schema.schema_version() != input.version || schema.schema_fingerprint() != input.fingerprint)
    *error = "command schema does not match installed operation";
  else return true;
  return false;
}
inline bool ValidMotionIntent(const xgc::semantic::common::v1::RemoteControlIntentRequest &intent) {
  return intent.gear() >= 1 && intent.gear() <= 3 &&
    intent.longitudinal() >= -1 && intent.longitudinal() <= 1 &&
    intent.lateral() >= -1 && intent.lateral() <= 1 && intent.yaw() >= -1 && intent.yaw() <= 1;
}
template<class Channel, class Metadata> xgc::adapter::v1::OperationEvent CommandSuccess(
    const xgc::adapter::v1::OperationRequest &request, const Channel &channel, const Metadata &output) {
  auto result = CommandError(request, xgc::adapter::v1::ERROR_CLASS_UNSPECIFIED, "", "");
  result.clear_error(); result.set_phase(xgc::adapter::v1::OPERATION_PHASE_SUCCEEDED);
  auto *payload = result.mutable_output(); payload->set_encoding(xgc::v1::PAYLOAD_ENCODING_PROTOBUF);
  auto *schema = payload->mutable_schema(); schema->set_message_id(channel.output_message_id);
  schema->set_type_name(output.type_name); schema->set_schema_version(output.version); schema->set_schema_fingerprint(output.fingerprint);
  return result;
}
inline AsyncRosServices::Clock::time_point NativeDeadline(const xgc::adapter::v1::OperationRequest &request,
                                                         std::uint32_t timeout_ms) {
  const auto now = std::chrono::system_clock::now();
  const auto absolute = std::chrono::system_clock::time_point(std::chrono::nanoseconds(request.context().deadline().deadline_unix_nanos()));
  return AsyncRosServices::Clock::now() + std::chrono::duration_cast<AsyncRosServices::Clock::duration>(
      std::min(absolute - now, std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::milliseconds(timeout_ms))));
}
} // namespace xgc2_ros1_robot_adapter
