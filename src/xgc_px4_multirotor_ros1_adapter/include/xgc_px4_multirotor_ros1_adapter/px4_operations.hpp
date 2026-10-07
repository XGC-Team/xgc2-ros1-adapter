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

constexpr double kDefaultOperationTimeoutSeconds = 5.0;
constexpr double kDefaultPx4StateTimeoutSeconds = 1.0;
constexpr std::uint16_t kPx4RebootMavCommand = 246u;
constexpr float kPx4NormalRebootParam1 = 1.0F;
constexpr std::uint16_t kPx4ForceDisarmMavCommand = 400u;
constexpr float kPx4ForceDisarmParam2 = 21196.0F;

enum class OperationOutcome {
  kSucceeded,
  kInvalidArgument,
  kNotReady,
  kRejected,
  kUnsupported,
  kUncertain,
  kTransportError,
  kTimedOut,
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

struct OperationTiming {
  explicit OperationTiming(
      double timeout_seconds_value = kDefaultOperationTimeoutSeconds,
      ros::WallTime deadline_value = ros::WallTime());

  double timeout_seconds;
  // Optional absolute ROS wall-time cutoff. Once a command is dispatched, a
  // response received before cancellation remains authoritative; otherwise
  // the outcome is explicitly uncertain.
  ros::WallTime deadline;
};

enum class OperationWindowState {
  kReady,
  kInvalid,
  kExpired,
};

struct OperationWindow {
  bool ready() const;

  OperationWindowState state = OperationWindowState::kInvalid;
  ros::WallTime started_at;
  ros::WallTime deadline;
  std::string detail;
};

OperationWindow makeOperationWindow(
    const OperationTiming &timing, const ros::WallTime &started_at,
    double maximum_timeout_seconds = kDefaultOperationTimeoutSeconds);
ros::WallDuration operationTimeRemaining(const OperationWindow &window,
                                         const ros::WallTime &now);

bool isAllowedPx4Mode(const std::string &mode,
                      const std::vector<std::string> &allowed_modes);
ros::WallTime operationDeadlineFromUnixNanos(std::int64_t unix_nanos);
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
