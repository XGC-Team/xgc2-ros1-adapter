#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc_px4_multirotor_ros1_adapter/px4_operations.hpp"
#include "xgc_px4_multirotor_ros1_adapter/remote_control.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"
#include "xgc2_ros1_robot_adapter/native_command.hpp"
#include "xgc/semantic/aerial/v1/control.pb.h"
#include "xgc/semantic/common/v1/control.pb.h"
#include <ros/serialization.h>
#include <ros/service_traits.h>
#include <ros/master.h>

namespace xgc_px4_multirotor_ros1_adapter {
namespace shared = xgc2_ros1_robot_adapter;
namespace wire = xgc::adapter::v1;
class Native : public shared::NativeRobot {
public:
  shared::RobotConfig config;
  NativeProfileConfig profile;
  std::shared_ptr<RobotRuntime> runtime;
  std::shared_ptr<RemoteControlPublisher> remote;
  struct State { std::mutex mutex; Px4StateSnapshot snapshot; };
  std::shared_ptr<State> state = std::make_shared<State>();
  ros::Subscriber state_subscriber;
  ~Native() override { Stop(); }
  void Stop() override { if (runtime) runtime->Stop(); if (remote) remote->Stop(); state_subscriber.shutdown(); }
  void Periodic(const ros::WallTime &now) override { runtime->emitPeriodic(now); if (remote) remote->PublishPeriodic(); }
  template<class Service, class Interpret> std::uint64_t call(Service command, const std::string &endpoint,
      const wire::OperationRequest &request, const contract::ChannelMetadata &channel, const contract::MessageMetadata &output,
      shared::AsyncRosServices &services, shared::CommandCompletion complete, Interpret interpret) {
    shared::AsyncRosServices::Request native;
    native.master_uri = ros::master::getURI(); native.caller_id = ros::this_node::getName();
    native.service = endpoint; native.type = ros::service_traits::DataType<Service>::value();
    native.md5 = ros::service_traits::MD5Sum<Service>::value(); native.deadline = shared::NativeDeadline(request, channel.operation_timeout_millis);
    native.bytes.resize(ros::serialization::serializationLength(command.request));
    ros::serialization::OStream stream(reinterpret_cast<std::uint8_t *>(&native.bytes[0]), native.bytes.size());
    ros::serialization::serialize(stream, command.request);
    return services.Start(std::move(native), [request, channel, output, complete, interpret](shared::AsyncRosServices::Result result) {
      if (result.outcome != shared::AsyncRosServices::Outcome::Response) {
        auto error = result.outcome == shared::AsyncRosServices::Outcome::Unknown ? wire::ERROR_CLASS_UNCERTAIN :
          result.outcome == shared::AsyncRosServices::Outcome::Deadline ? wire::ERROR_CLASS_DEADLINE :
          result.outcome == shared::AsyncRosServices::Outcome::Cancelled ? wire::ERROR_CLASS_CANCELLED : wire::ERROR_CLASS_TRANSIENT;
        const char *code = error == wire::ERROR_CLASS_UNCERTAIN ? "px4-result-uncertain" :
          error == wire::ERROR_CLASS_DEADLINE ? "px4-deadline-exceeded" :
          error == wire::ERROR_CLASS_CANCELLED ? "operation-cancelled" : "ros-transport-error";
        complete(shared::CommandError(request, error, code, result.detail)); return;
      }
      try {
        typename Service::Response response;
        ros::serialization::IStream stream(reinterpret_cast<std::uint8_t *>(&result.response[0]), result.response.size());
        ros::serialization::deserialize(stream, response);
        if (stream.getLength()) throw std::runtime_error("unexpected trailing service bytes");
        const auto interpreted = interpret(response);
        if (interpreted.succeeded()) { complete(shared::CommandSuccess(request, channel, output)); return; }
        auto error = interpreted.outcome == OperationOutcome::kUncertain ? wire::ERROR_CLASS_UNCERTAIN :
          interpreted.outcome == OperationOutcome::kNotReady ? wire::ERROR_CLASS_TRANSIENT : wire::ERROR_CLASS_REJECTED;
        const char *code = interpreted.outcome == OperationOutcome::kUncertain ? "px4-result-uncertain" :
          interpreted.outcome == OperationOutcome::kNotReady ? "px4-not-ready" :
          interpreted.outcome == OperationOutcome::kUnsupported ? "px4-unsupported" : "px4-rejected";
        auto event = shared::CommandError(request, error, code, interpreted.detail);
        if (interpreted.has_native_result) event.set_native_code(interpreted.native_result);
        complete(std::move(event));
      } catch (const std::exception &error) { complete(shared::CommandError(request, wire::ERROR_CLASS_UNCERTAIN, "px4-result-uncertain", error.what())); }
    });
  }
  std::uint64_t Execute(const wire::OperationRequest &request, shared::AsyncRosServices &services, shared::CommandCompletion complete) override {
    std::size_t count = 0;
    const auto *channels = contract::profileChannels(config.profile_id, &count);
    const contract::ChannelMetadata *channel = nullptr;
    for (std::size_t i = 0; i < count; ++i)
      if (channels[i].kind == contract::ChannelKind::kOperation && request.context().endpoint_id() == channels[i].operation_id) channel = &channels[i];
    contract::MessageMetadata input{}, output{}; std::string error;
    if (!channel || !contract::messageMetadata(channel->input_message_id, &input) || !contract::messageMetadata(channel->output_message_id, &output) ||
        !shared::ValidateCommand(request, config, *channel, input, &error)) {
      complete(shared::CommandError(request, wire::ERROR_CLASS_REJECTED, "invalid-command", error.empty() ? "unknown command" : error)); return 0;
    }
    const std::string processor(channel->processor);
    if (processor == "px4.arm") {
      xgc::semantic::aerial::v1::ArmRequest arm;
      if (!arm.ParseFromString(request.input().value())) error = "ArmRequest malformed";
      else return call(makeArmCommand(arm.armed()), profile.arm_service_endpoint, request, *channel, output, services, complete,
                       [arm](const mavros_msgs::CommandBool::Response &response) { return interpretArmResponse(arm.armed(), response); });
    } else if (processor == "px4.mode") {
      xgc::semantic::aerial::v1::ModeRequest mode;
      if (!mode.ParseFromString(request.input().value()) || !isAllowedPx4Mode(mode.mode(), profile.allowed_modes)) error = "PX4 mode malformed or not allowed";
      else return call(makeModeCommand(mode.mode()), profile.mode_service_endpoint, request, *channel, output, services, complete,
                       [mode](const mavros_msgs::SetMode::Response &response) { return interpretModeResponse(mode.mode(), response); });
    } else if (processor == "px4.autopilot-reboot") {
      xgc::semantic::aerial::v1::AutopilotRebootRequest reboot;
      if (!reboot.ParseFromString(request.input().value())) error = "AutopilotRebootRequest malformed";
      else {
        Px4StateSnapshot snapshot; { std::lock_guard<std::mutex> lock(state->mutex); snapshot = state->snapshot; }
        const auto ready = evaluatePx4RebootReadiness(snapshot, ros::WallTime::now(), profile.reboot_state_timeout_seconds);
        if (ready != Px4RebootReadiness::kReady) {
          complete(shared::CommandError(request, ready == Px4RebootReadiness::kArmed ? wire::ERROR_CLASS_REJECTED : wire::ERROR_CLASS_TRANSIENT,
                                        "px4-not-ready", px4RebootReadinessDetail(ready))); return 0;
        }
        return call(makeAutopilotRebootCommand(), profile.reboot_service_endpoint, request, *channel, output, services, complete, interpretAutopilotRebootResponse);
      }
    } else if (processor == "px4.force-disarm") {
      xgc::semantic::aerial::v1::ForceDisarmRequest force;
      if (!force.ParseFromString(request.input().value())) error = "ForceDisarmRequest malformed";
      else return call(makeForceDisarmCommand(), profile.reboot_service_endpoint, request, *channel, output, services, complete, interpretForceDisarmResponse);
    } else if (processor == "px4.set-motion-intent" || processor == "px4.release-motion-intent") {
      xgc::semantic::common::v1::RemoteControlIntentRequest intent;
      if (!remote || !intent.ParseFromString(request.input().value()) || !shared::ValidMotionIntent(intent)) error = "motion intent unavailable or malformed";
      else {
        const bool success = processor == "px4.release-motion-intent" ? remote->Release(&error) :
          remote->SetIntent(intent.gear(), intent.longitudinal(), intent.lateral(), intent.yaw(), &error);
        complete(success ? shared::CommandSuccess(request, *channel, output) :
          shared::CommandError(request, wire::ERROR_CLASS_TRANSIENT, "motion-command-publication-failed", error)); return 0;
      }
    } else error = "operation processor unavailable";
    complete(shared::CommandError(request, wire::ERROR_CLASS_REJECTED, "invalid-command-input", error)); return 0;
  }
};
std::shared_ptr<shared::NativeRobot> create(ros::NodeHandle node, const shared::RobotConfig &config, shared::TelemetryEmitter emit, std::string *error) {
  auto native = std::make_shared<Native>(); native->config = config;
  if (!contract::profileDigest(config.profile_id) || config.profile_digest != contract::profileDigest(config.profile_id)) {
    *error = "profile does not match installed native mapping"; return nullptr;
  }
  if (!BuildNativeProfileConfig(config, &native->profile, error)) return nullptr;
  native->runtime = RobotRuntime::Create(node, config, 1, std::move(emit), error);
  if (!native->runtime) return nullptr;
  if (shared::Enabled(config, "operation.motion-intent") || shared::Enabled(config, "operation.motion-intent-release")) {
    native->remote = RemoteControlPublisher::Create(node, native->profile.remote_control_endpoint,
        native->profile.remote_control_altitude_meters, native->profile.remote_control_maximum_linear_velocity_mps,
        native->profile.remote_control_maximum_yaw_rate_rps, error);
    if (!native->remote) return nullptr;
  }
  if (shared::Enabled(config, "operation.autopilot-reboot")) {
    const auto state = native->state;
    native->state_subscriber = node.subscribe<mavros_msgs::State>(native->profile.state_endpoint, 1,
      [state](const mavros_msgs::State::ConstPtr &message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->snapshot.known = true; state->snapshot.connected = message->connected; state->snapshot.armed = message->armed;
        state->snapshot.observed_at = ros::WallTime::now();
      }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    if (!native->state_subscriber) { *error = "PX4 state subscription failed"; return nullptr; }
  }
  native->runtime->Activate(); return native;
}
} // namespace xgc_px4_multirotor_ros1_adapter
int main(int argc, char **argv) {
  return xgc2_ros1_robot_adapter::RunRobotServer(argc, argv, "xgc_px4_multirotor_ros1_adapter", "xgc2-px4-multirotor-ros1-adapter", xgc_px4_multirotor_ros1_adapter::create);
}
