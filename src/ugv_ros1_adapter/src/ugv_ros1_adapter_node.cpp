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
  std::chrono::milliseconds control_silence{0};
  ~Native() override { Stop(); }
  void Stop() override { if (runtime) runtime->Stop(); if (motion) motion->Stop(); }
  void Periodic() override { runtime->emitPeriodic(ros::WallTime::now()); if (motion) motion->PublishPeriodic(); }
  void SetStatusDemand(const std::set<std::string> &channels) override { runtime->SetStatusDemand(channels); }
  std::optional<shared::NativeHealth> DescribeHealth() override {
    const auto value = runtime->DescribeHealth();
    return shared::NativeHealth{value.first, value.second};
  }
  bool CoreSilent(std::chrono::milliseconds silent_for) override { return motion && motion->CoreSilent(silent_for); }
  bool RequiresControl(const wire::OperationRequest &request) const override {
    std::size_t count = 0;
    const auto *channels = variant.contract.profileChannels(config.profile_id, &count);
    for (std::size_t i = 0; i < count; ++i)
      if (channels[i].kind == contract::ChannelKind::kOperation &&
          request.context().endpoint_id() == channels[i].operation_id)
        return std::string(channels[i].channel_id) == "operation.motion-intent";
    return false;
  }
  bool IsControlRelease(const wire::OperationRequest &request) const override {
    std::size_t count = 0;
    const auto *channels = variant.contract.profileChannels(config.profile_id, &count);
    for (std::size_t i = 0; i < count; ++i)
      if (channels[i].kind == contract::ChannelKind::kOperation &&
          request.context().endpoint_id() == channels[i].operation_id)
        return std::string(channels[i].channel_id) == "operation.motion-intent-release";
    return false;
  }
  bool RetireControl(std::string *error) override { return !motion || motion->Release(error); }
  std::chrono::milliseconds ControlSilenceLimit() const override { return control_silence; }
  bool IsSafetyOperation(const wire::OperationRequest &request) const override {
    std::size_t count = 0;
    const auto *channels = variant.contract.profileChannels(config.profile_id, &count);
    for (std::size_t i = 0; i < count; ++i)
      if (channels[i].kind == contract::ChannelKind::kOperation && request.context().endpoint_id() == channels[i].operation_id)
        return contract::SafetyOperation(channels[i]);
    return false;
  }
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
      contract::ChannelMetadata motion_channel{};
      std::int64_t silence_ms = 0;
      if (!variant.contract.channelMetadata(config.profile_id, "operation.motion-intent", &motion_channel) ||
          !contract::channelPolicyInteger(motion_channel, "core_silence_ms", &silence_ms) || silence_ms < 0 || silence_ms > 60000) {
        *error = "motion-intent policy core_silence_ms is missing or out of range"; return nullptr;
      }
      native->motion = MotionCommandPublisher::Create(node, topic, variant.motion, std::chrono::milliseconds(silence_ms), error);
      if (!native->motion) return nullptr;
      native->control_silence = std::chrono::milliseconds(silence_ms);
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
