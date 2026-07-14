#include <chrono>
#include <cstdint>
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
#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"
#include "xgc_scout_mini_ros1_adapter/robot_runtime.hpp"
#include "xgc_scout_mini_ros1_adapter/shutdown_signal.hpp"

namespace xgc_scout_mini_ros1_adapter {
namespace {

constexpr const char *kProfile = "scout-mini.ros1.v1";
constexpr const char *kSoftwareVersion = "0.3.0";

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

class ScoutMiniRos1AdapterNode {
public:
  ScoutMiniRos1AdapterNode(ros::NodeHandle node_handle,
                           ros::NodeHandle private_node_handle)
      : node_handle_(std::move(node_handle)),
        private_node_handle_(std::move(private_node_handle)) {
    xgc2::adapter_link::ClientConfig config;
    private_node_handle_.param<std::string>("adapter_id", config.adapter_id,
                                            "scout-mini-ros1-adapter");
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
        [](const xgc::adapter::v1::OperationRequest &) {
          return xgc2::adapter_link::OperationExecutionResult::Rejected(
              "Scout Mini ROS1 profile exposes no operation channels",
              xgc::adapter::v1::RESULT_CODE_UNSUPPORTED);
        };
    callbacks.log = logFromClient;

    client_.reset(new xgc2::adapter_link::Client(std::move(config),
                                                 std::move(callbacks)));
    std::string error;
    if (!client_->Start(&error)) {
      throw std::runtime_error("AdapterLink startup failed: " + error);
    }

    periodic_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(0.1), &ScoutMiniRos1AdapterNode::periodicTimer, this);
  }

  ~ScoutMiniRos1AdapterNode() {
    periodic_timer_.stop();
    if (client_) {
      client_->Stop();
    }
  }

  ScoutMiniRos1AdapterNode(const ScoutMiniRos1AdapterNode &) = delete;
  ScoutMiniRos1AdapterNode &
  operator=(const ScoutMiniRos1AdapterNode &) = delete;

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
        *error = "Scout Mini adapter received unsupported profile for robot " +
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
      if (robot_plan.parameters().size() != 1 ||
          robot_plan.parameters().find("namespace") ==
              robot_plan.parameters().end()) {
        *error = "robot " + robot_plan.robot_id() +
                 " must contain exactly the namespace profile parameter";
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
        if (metadata.kind != xgc::adapter::v1::CHANNEL_KIND_STREAM_OUT) {
          *error = "Scout Mini adapter supports only stream_out channels";
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
        *error = "failed to build Scout Mini robot " + robot_plan.robot_id() +
                 ": " + runtime_error;
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

} // namespace xgc_scout_mini_ros1_adapter

int main(int argc, char **argv) {
  ros::init(argc, argv, "xgc_scout_mini_ros1_adapter",
            ros::init_options::NoSigintHandler);
  try {
    xgc_scout_mini_ros1_adapter::ShutdownSignalHandler shutdown_signals;
    xgc_scout_mini_ros1_adapter::ScoutMiniRos1AdapterNode node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok() && !shutdown_signals.requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (shutdown_signals.requested() && ros::ok()) {
      ROS_INFO_STREAM("Scout Mini ROS1 adapter received shutdown signal "
                      << shutdown_signals.signalNumber());
      ros::shutdown();
    }
    ros::waitForShutdown();
    spinner.stop();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM(
        "Scout Mini ROS1 adapter startup failed: " << error.what());
    return 1;
  }
  return 0;
}
