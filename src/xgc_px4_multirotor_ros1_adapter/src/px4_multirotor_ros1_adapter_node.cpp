#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include "xgc2/adapter_link/client.hpp"
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"
#include "xgc_px4_multirotor_ros1_adapter/shutdown_signal.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

constexpr const char *kProfile = "px4.multirotor.ros1.v2";
constexpr const char *kSoftwareVersion = "0.4.0";

std::int64_t wallUnixNanos() {
  return static_cast<std::int64_t>(ros::WallTime::now().toNSec());
}

std::int64_t
operationDeadline(const xgc::adapter::v1::OperationRequest &request) {
  std::int64_t deadline = request.deadline_unix_nanos();
  if (request.ttl_ms() > 0 && request.issued_unix_nanos() > 0) {
    const std::int64_t ttl_nanos =
        static_cast<std::int64_t>(request.ttl_ms()) * 1000000LL;
    const std::int64_t issued = request.issued_unix_nanos();
    const std::int64_t ttl_deadline =
        issued > std::numeric_limits<std::int64_t>::max() - ttl_nanos
            ? std::numeric_limits<std::int64_t>::max()
            : issued + ttl_nanos;
    if (deadline <= 0 || ttl_deadline < deadline) {
      deadline = ttl_deadline;
    }
  }
  return deadline;
}

void logFromClient(xgc2::adapter_link::LogLevel level,
                   const std::string &message) {
  switch (level) {
  case xgc2::adapter_link::LogLevel::kDebug:
    ROS_DEBUG_STREAM(message);
    break;
  case xgc2::adapter_link::LogLevel::kInfo:
    ROS_INFO_STREAM(message);
    break;
  case xgc2::adapter_link::LogLevel::kWarning:
    ROS_WARN_STREAM(message);
    break;
  case xgc2::adapter_link::LogLevel::kError:
    ROS_ERROR_STREAM(message);
    break;
  }
}

} // namespace

class Px4MultirotorRos1AdapterNode {
public:
  Px4MultirotorRos1AdapterNode(ros::NodeHandle node_handle,
                               ros::NodeHandle private_node_handle)
      : node_handle_(std::move(node_handle)),
        private_node_handle_(std::move(private_node_handle)) {
    xgc2::adapter_link::ClientConfig config;
    private_node_handle_.param<std::string>("adapter_id", config.adapter_id,
                                            "px4-multirotor-ros1-adapter");
    private_node_handle_.param<std::string>(
        "socket_path", config.socket_path,
        "/run/xgc2/adapter/adapter-link.sock");
    private_node_handle_.param<std::string>("bootstrap_token_file",
                                            config.bootstrap_token_file, "");
    config.native_protocol = "ros1";
    config.software_version = kSoftwareVersion;
    config.supported_protocol_versions.push_back(contract::kProtocolVersion);
    config.registry_fingerprint = contract::kRegistryFingerprint;
    contract::addSupportedProfiles(&config.supported_profiles);

    xgc2::adapter_link::ClientCallbacks callbacks;
    callbacks.validate_and_apply_plan =
        [this](const xgc::adapter::v1::AdapterPlan &plan, std::string *error) {
          return applyPlan(plan, error);
        };
    callbacks.clear_plan = [this] { clearPlan(); };
    callbacks.handle_operation =
        [this](const xgc::adapter::v1::OperationRequest &request) {
          return handleOperation(request);
        };
    callbacks.log = logFromClient;

    client_.reset(new xgc2::adapter_link::Client(std::move(config),
                                                 std::move(callbacks)));
    std::string error;
    if (!client_->Start(&error)) {
      throw std::runtime_error("AdapterLink startup failed: " + error);
    }

    periodic_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(0.1), &Px4MultirotorRos1AdapterNode::periodicTimer,
        this);
  }

  ~Px4MultirotorRos1AdapterNode() {
    periodic_timer_.stop();
    if (client_) {
      client_->Stop();
    }
  }

  Px4MultirotorRos1AdapterNode(const Px4MultirotorRos1AdapterNode &) = delete;
  Px4MultirotorRos1AdapterNode &
  operator=(const Px4MultirotorRos1AdapterNode &) = delete;

private:
  bool applyPlan(const xgc::adapter::v1::AdapterPlan &plan,
                 std::string *error) {
    static const std::regex robot_id_pattern(
        "^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$");
    if (!validateNonEmptyAdapterPlan(plan, error)) {
      return false;
    }
    if (plan.asset_digest().empty()) {
      *error = "non-empty robot plan omitted its asset_digest";
      return false;
    }

    std::map<std::string, std::shared_ptr<RobotRuntime>> new_robots;
    std::set<std::string> namespaces;
    for (const auto &robot_plan : plan.robots()) {
      if (!std::regex_match(robot_plan.robot_id(), robot_id_pattern)) {
        *error = "invalid or empty robot_id: " + robot_plan.robot_id();
        return false;
      }
      if (robot_plan.profile_id() != kProfile) {
        *error = "PX4 adapter received unsupported profile for robot " +
                 robot_plan.robot_id() + ": " + robot_plan.profile_id();
        return false;
      }
      if (new_robots.count(robot_plan.robot_id()) != 0) {
        *error = "duplicate robot_id: " + robot_plan.robot_id();
        return false;
      }
      const char *digest = contract::profileDigest(robot_plan.profile_id());
      if (digest == nullptr || robot_plan.profile_digest() != digest) {
        *error = "profile digest mismatch for robot " + robot_plan.robot_id();
        return false;
      }
      if (robot_plan.parameters().size() != 2 ||
          robot_plan.parameters().find("namespace") ==
              robot_plan.parameters().end() ||
          robot_plan.parameters().find("mocap_rigid_body") ==
              robot_plan.parameters().end()) {
        *error = "robot " + robot_plan.robot_id() +
                 " must contain exactly namespace and mocap_rigid_body profile parameters";
        return false;
      }
      const std::string &robot_namespace =
          robot_plan.parameters().find("namespace")->second;
      std::string namespace_error;
      if (!validRobotNamespace(robot_namespace, &namespace_error)) {
        *error = "robot " + robot_plan.robot_id() +
                 " has invalid namespace: " + namespace_error;
        return false;
      }
      if (!namespaces.insert(robot_namespace).second) {
        *error = "multiple robots share ROS namespace " + robot_namespace;
        return false;
      }

      std::set<std::string> channel_ids;
      for (const auto &channel : robot_plan.channels()) {
        if (!channel_ids.insert(channel.channel_id()).second) {
          *error = "robot " + robot_plan.robot_id() + " repeats channel " +
                   channel.channel_id();
          return false;
        }
        contract::ChannelMetadata metadata;
        if (!contract::channelMetadata(robot_plan.profile_id(),
                                       channel.channel_id(), &metadata)) {
          *error = "robot " + robot_plan.robot_id() +
                   " contains unknown channel " + channel.channel_id();
          return false;
        }
        if (!channel.parameters().empty()) {
          *error = "channel parameters are not defined by profile " +
                   robot_plan.profile_id() + ": " + channel.channel_id();
          return false;
        }
        if (metadata.kind == xgc::adapter::v1::CHANNEL_KIND_STREAM_IN ||
            metadata.kind == xgc::adapter::v1::CHANNEL_KIND_REQUEST_RESPONSE) {
          *error = "high-frequency/input channels are not implemented by "
                   "the PX4 ROS1 adapter";
          return false;
        }
      }

      std::string runtime_error;
      auto runtime = RobotRuntime::Create(
          node_handle_, robot_plan, plan.revision(),
          [this](std::uint64_t revision, xgc::v1::Message message) {
            if (client_) {
              client_->Publish(revision, std::move(message));
            }
          },
          &runtime_error);
      if (!runtime) {
        *error = "failed to build PX4 robot " + robot_plan.robot_id() + ": " +
                 runtime_error;
        return false;
      }
      new_robots.emplace(robot_plan.robot_id(), std::move(runtime));
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      robots_.swap(new_robots);
    }
    return true;
  }

  void clearPlan() {
    std::lock_guard<std::mutex> lock(mutex_);
    robots_.clear();
  }

  xgc2::adapter_link::OperationExecutionResult
  handleOperation(const xgc::adapter::v1::OperationRequest &request) {
    const auto &message = request.message();
    if (message.robot_id().empty() || message.channel_id().empty()) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "operation message must identify robot_id and channel_id",
          xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT);
    }

    std::shared_ptr<RobotRuntime> runtime;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = robots_.find(message.robot_id());
      if (it == robots_.end()) {
        return xgc2::adapter_link::OperationExecutionResult::Rejected(
            "operation robot does not exist in the applied PX4 plan",
            xgc::adapter::v1::RESULT_CODE_NOT_FOUND);
      }
      runtime = it->second;
    }
    if (request.plan_revision() != runtime->planRevision()) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "operation plan_revision does not match the applied plan",
          xgc::adapter::v1::RESULT_CODE_REJECTED);
    }
    if (!runtime->channelEnabled(message.channel_id())) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "operation channel is disabled in the applied plan",
          xgc::adapter::v1::RESULT_CODE_UNSUPPORTED);
    }

    contract::ChannelMetadata channel;
    if (!contract::channelMetadata(runtime->profileId(), message.channel_id(),
                                   &channel) ||
        channel.kind != xgc::adapter::v1::CHANNEL_KIND_OPERATION) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "message channel is not a PX4 operation",
          xgc::adapter::v1::RESULT_CODE_UNSUPPORTED);
    }
    if (message.message_id() != channel.input_message_id) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "operation message_id does not match the profile channel",
          xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT);
    }
    contract::MessageMetadata metadata;
    if (!contract::messageMetadata(message.message_id(), &metadata)) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "operation message_id does not exist in the installed registry",
          xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT);
    }
    if (message.payload().size() > 64 * 1024) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "operation payload exceeds 64 KiB",
          xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT);
    }
    if (request.ttl_ms() > 0 && request.issued_unix_nanos() <= 0) {
      return xgc2::adapter_link::OperationExecutionResult::Rejected(
          "ttl_ms requires a positive issued_unix_nanos",
          xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT);
    }

    double timeout_seconds = 5.0;
    const std::int64_t deadline = operationDeadline(request);
    if (deadline > 0) {
      const std::int64_t now = wallUnixNanos();
      if (now >= deadline) {
        auto result = xgc2::adapter_link::OperationExecutionResult::Rejected(
            "operation deadline or TTL has expired",
            xgc::adapter::v1::RESULT_CODE_TIMEOUT);
        result.phase = xgc::adapter::v1::OPERATION_PHASE_EXPIRED;
        return result;
      }
      timeout_seconds =
          std::min(timeout_seconds, static_cast<double>(deadline - now) / 1e9);
    }
    return runtime->executeOperation(request, timeout_seconds);
  }

  void periodicTimer(const ros::WallTimerEvent &) {
    std::vector<std::shared_ptr<RobotRuntime>> robots;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &entry : robots_) {
        robots.push_back(entry.second);
      }
    }
    const ros::WallTime now = ros::WallTime::now();
    for (const auto &robot : robots) {
      robot->emitPeriodic(now);
    }
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::WallTimer periodic_timer_;
  std::unique_ptr<xgc2::adapter_link::Client> client_;

  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<RobotRuntime>> robots_;
};

} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  ros::init(argc, argv, "xgc_px4_multirotor_ros1_adapter",
            ros::init_options::NoSigintHandler);
  try {
    xgc_px4_multirotor_ros1_adapter::ShutdownSignalHandler shutdown_signals;
    xgc_px4_multirotor_ros1_adapter::Px4MultirotorRos1AdapterNode node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok() && !shutdown_signals.requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (shutdown_signals.requested() && ros::ok()) {
      ROS_INFO_STREAM("PX4 multirotor ROS1 adapter received shutdown signal "
                      << shutdown_signals.signalNumber());
      ros::shutdown();
    }
    ros::waitForShutdown();
    spinner.stop();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM(
        "PX4 multirotor ROS1 adapter startup failed: " << error.what());
    return 1;
  }
  return 0;
}
