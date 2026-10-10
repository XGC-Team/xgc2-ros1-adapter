#include "xgc/semantic/common/v1/control.pb.h"
#include "xgc2_ros1_robot_adapter/native_command.hpp"
#include "xgc2_ros1_robot_adapter/ros1_binding.hpp"
#include "xgc_ugv_ros1_adapter/motion_command.hpp"
#include "xgc_ugv_ros1_adapter/robot_runtime.hpp"
#include "xgc_ugv_ros1_adapter/variant.hpp"

namespace xgc_ugv_ros1_adapter {
namespace shared = xgc2_ros1_robot_adapter;
namespace wire = xgc::adapter::v1;
class Native : public shared::NativeRobot {
public:
  explicit Native(const UgvVariant &robot_type) : variant(robot_type) {}
  const UgvVariant &variant;
  shared::RobotConfig config;
  std::shared_ptr<RobotRuntime> runtime;
  std::shared_ptr<MotionCommandPublisher> motion;
  ~Native() override { Stop(); }
  void Stop() override { if (runtime) runtime->Stop(); if (motion) motion->Stop(); }
  void Periodic() override { runtime->emitPeriodic(ros::WallTime::now()); if (motion) motion->PublishPeriodic(); }
  std::uint64_t Execute(const wire::OperationRequest &request, shared::CommandIo &, shared::CommandCompletion complete) override {
    std::size_t count = 0;
    const auto *channels = variant.contract.profileChannels(config.profile_id, &count);
    const contract::ChannelMetadata *channel = nullptr;
    for (std::size_t i = 0; i < count; ++i)
      if (channels[i].kind == contract::ChannelKind::kOperation && request.context().endpoint_id() == channels[i].operation_id) channel = &channels[i];
    contract::MessageMetadata input{}, output{};
    std::string error;
    if (!channel || !variant.contract.messageMetadata(channel->input_message_id, &input) || !variant.contract.messageMetadata(channel->output_message_id, &output) ||
        !shared::ValidateCommand(request, config, *channel, input, &error)) {
      complete(shared::CommandError(request, wire::ERROR_CLASS_REJECTED, "invalid-command", error.empty() ? "unknown command" : error)); return 0;
    }
    xgc::semantic::common::v1::RemoteControlIntentRequest intent;
    const std::string processor(channel->processor);
    const std::string prefix = variant.processor_prefix;
    if (!motion || !intent.ParseFromString(request.input().value()) || !shared::ValidMotionIntent(intent) ||
        (processor != prefix + ".set-motion-intent" && processor != prefix + ".release-motion-intent")) {
      complete(shared::CommandError(request, wire::ERROR_CLASS_REJECTED, "invalid-command-input", "motion command unavailable or malformed")); return 0;
    }
    const bool success = processor == prefix + ".release-motion-intent" ? motion->Release(&error) :
      motion->SetIntent(intent.gear(), intent.longitudinal(), intent.lateral(), intent.yaw(), &error);
    complete(success ? shared::CommandSuccess(request, *channel, output) :
      shared::CommandError(request, wire::ERROR_CLASS_TRANSIENT, "motion-command-publication-failed", error));
    return 0;
  }
};
// The factory of one robot type.
shared::RobotFactory factoryFor(const UgvVariant &variant) {
  return [&variant](shared::RosNode &ros_node, const shared::RobotConfig &config, shared::TelemetryEmitter emit, std::string *error) -> std::shared_ptr<shared::NativeRobot> {
    ros::NodeHandle node = shared::Ros1Handle(ros_node);
    auto native = std::make_shared<Native>(variant); native->config = config;
    if (!variant.contract.profileDigest(config.profile_id) || config.profile_digest != variant.contract.profileDigest(config.profile_id)) {
      *error = "profile does not match installed native mapping"; return nullptr;
    }
    native->runtime = RobotRuntime::Create(variant, node, config, std::move(emit), error);
    if (!native->runtime) return nullptr;
    if (shared::Enabled(config, "operation.motion-intent") || shared::Enabled(config, "operation.motion-intent-release")) {
      std::string topic;
      if (!resolveMotionCommandTopic(variant, config, &topic, error)) return nullptr;
      native->motion = MotionCommandPublisher::Create(node, topic, variant.motion, error);
      if (!native->motion) return nullptr;
    }
    return native;
  };
}
} // namespace xgc_ugv_ros1_adapter

int main(int argc, char **argv) {
  using namespace xgc_ugv_ros1_adapter;
  return xgc2_ros1_robot_adapter::RunRos1TypeServer(
      argc, argv,
      {{scoutMini().provider, factoryFor(scoutMini())},
       {mecanumUgv().provider, factoryFor(mecanumUgv())}});
}
