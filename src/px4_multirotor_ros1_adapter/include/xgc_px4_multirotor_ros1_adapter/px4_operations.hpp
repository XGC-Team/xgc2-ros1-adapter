#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>


namespace xgc_px4_multirotor_ros1_adapter {

constexpr double kDefaultPx4StateTimeoutSeconds = 1.0;
constexpr std::uint16_t kPx4RebootMavCommand = 246u;
constexpr float kPx4NormalRebootParam1 = 1.0F;
constexpr std::uint16_t kPx4ForceDisarmMavCommand = 400u;
constexpr float kPx4ForceDisarmParam2 = 21196.0F;

enum class OperationOutcome {
  kSucceeded,
  kNotReady,
  kRejected,
  kUnsupported,
  kUncertain,
  kTransportError,
};

struct OperationResult {
  OperationResult();
  OperationResult(OperationOutcome outcome_value, std::string detail_value);

  bool succeeded() const;

  OperationOutcome outcome;
  std::string detail;
  // True once native request bytes may have reached the service. Any non-success outcome
  // with this flag set is indeterminate and must not be retried blindly.
  bool dispatched = false;
  bool has_native_result = false;
  std::uint8_t native_result = 0;
};

bool isAllowedPx4Mode(const std::string &mode,
                      const std::vector<std::string> &allowed_modes);
const char *mavResultName(std::uint8_t result);

mavros_msgs::CommandBool makeArmCommand(bool armed);
mavros_msgs::SetMode makeModeCommand(const std::string &mode);
mavros_msgs::CommandLong makeAutopilotRebootCommand();
mavros_msgs::CommandLong makeForceDisarmCommand();

OperationResult
interpretArmResponse(bool requested_armed,
                     const mavros_msgs::CommandBool::Response &response);
OperationResult
interpretModeResponse(const std::string &requested_mode,
                      const mavros_msgs::SetMode::Response &response);
OperationResult interpretAutopilotRebootResponse(
    const mavros_msgs::CommandLong::Response &response);
OperationResult interpretForceDisarmResponse(
    const mavros_msgs::CommandLong::Response &response);

struct Px4StateSnapshot {
  bool known = false;
  bool connected = false;
  bool armed = false;
  ros::WallTime observed_at;
};

enum class Px4RebootReadiness {
  kReady,
  kStateUnknown,
  kStateStale,
  kDisconnected,
  kArmed,
};

Px4RebootReadiness evaluatePx4RebootReadiness(
    const Px4StateSnapshot &state, const ros::WallTime &now,
    double state_timeout_seconds = kDefaultPx4StateTimeoutSeconds);
const char *px4RebootReadinessDetail(Px4RebootReadiness readiness);

} // namespace xgc_px4_multirotor_ros1_adapter
