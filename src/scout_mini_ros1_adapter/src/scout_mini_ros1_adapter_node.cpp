#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"
#include "xgc_scout_mini_ros1_adapter/robot_runtime.hpp"
#include "xgc_scout_mini_ros1_adapter/motion_command.hpp"
#include "xgc2_ros1_robot_adapter/native_command.hpp"
#include "xgc/semantic/common/v1/control.pb.h"

namespace xgc_scout_mini_ros1_adapter {
namespace shared = xgc2_ros1_robot_adapter;
namespace wire = xgc::adapter::v1;
class Native : public shared::NativeRobot {
public:
  shared::RobotConfig config;
  std::shared_ptr<RobotRuntime> runtime;
  std::shared_ptr<MotionCommandPublisher> motion;
  ~Native() override { Stop(); }
  void Stop() override { if (runtime) runtime->Stop(); if (motion) motion->Stop(); }
  void Periodic(const ros::WallTime &now) override { runtime->emitPeriodic(now); if (motion) motion->PublishPeriodic(); }
  std::uint64_t Execute(const wire::OperationRequest &request, shared::AsyncRosServices &, shared::CommandCompletion complete) override {
    std::size_t count = 0;
    const auto *channels = contract::profileChannels(config.profile_id, &count);
    const contract::ChannelMetadata *channel = nullptr;
    for (std::size_t i = 0; i < count; ++i)
      if (channels[i].kind == contract::ChannelKind::kOperation && request.context().endpoint_id() == channels[i].operation_id) channel = &channels[i];
    contract::MessageMetadata input{}, output{};
    std::string error;
    if (!channel || !contract::messageMetadata(channel->input_message_id, &input) || !contract::messageMetadata(channel->output_message_id, &output) ||
        !shared::ValidateCommand(request, config, *channel, input, &error)) {
      complete(shared::CommandError(request, wire::ERROR_CLASS_REJECTED, "invalid-command", error.empty() ? "unknown command" : error)); return 0;
    }
    xgc::semantic::common::v1::RemoteControlIntentRequest intent;
    const std::string processor(channel->processor);
    if (!motion || !intent.ParseFromString(request.input().value()) || !shared::ValidMotionIntent(intent) ||
        (processor != "scout-mini.set-motion-intent" && processor != "scout-mini.release-motion-intent")) {
      complete(shared::CommandError(request, wire::ERROR_CLASS_REJECTED, "invalid-command-input", "motion command unavailable or malformed")); return 0;
    }
    const bool success = processor == "scout-mini.release-motion-intent" ? motion->Release(&error) :
      motion->SetIntent(intent.gear(), intent.longitudinal(), intent.lateral(), intent.yaw(), &error);
    complete(success ? shared::CommandSuccess(request, *channel, output) :
      shared::CommandError(request, wire::ERROR_CLASS_TRANSIENT, "motion-command-publication-failed", error));
    return 0;
  }
};
std::shared_ptr<shared::NativeRobot> create(ros::NodeHandle node, const shared::RobotConfig &config, shared::TelemetryEmitter emit, std::string *error) {
  auto native = std::make_shared<Native>(); native->config = config;
  if (!contract::profileDigest(config.profile_id) || config.profile_digest != contract::profileDigest(config.profile_id)) {
    *error = "profile does not match installed native mapping"; return nullptr;
  }
  native->runtime = RobotRuntime::Create(node, config, 1, std::move(emit), error);
  if (!native->runtime) return nullptr;
  if (shared::Enabled(config, "operation.motion-intent") || shared::Enabled(config, "operation.motion-intent-release")) {
    std::string topic;
    if (!resolveMotionCommandTopic(config, &topic, error)) return nullptr;
    native->motion = MotionCommandPublisher::Create(node, topic, error);
    if (!native->motion) return nullptr;
  }
  return native;
}
} // namespace xgc_scout_mini_ros1_adapter
int main(int argc, char **argv) {
  return xgc2_ros1_robot_adapter::RunRobotServer(argc, argv, "xgc_scout_mini_ros1_adapter", "xgc2-scout-mini-ros1-adapter", xgc_scout_mini_ros1_adapter::create);
}
