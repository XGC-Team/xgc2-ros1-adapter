#include "xgc_px4_multirotor_ros1_adapter/px4_operations.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <mavros_msgs/CommandCode.h>

namespace xgc_px4_multirotor_ros1_adapter {
namespace {
constexpr std::uint8_t kMavResultAccepted = 0u;
constexpr std::uint8_t kMavResultTemporarilyRejected = 1u;
constexpr std::uint8_t kMavResultDenied = 2u;
constexpr std::uint8_t kMavResultUnsupported = 3u;
constexpr std::uint8_t kMavResultFailed = 4u;
constexpr std::uint8_t kMavResultInProgress = 5u;
constexpr std::uint8_t kMavResultCancelled = 6u;
OperationOutcome nativeFailureOutcome(std::uint8_t result) {
  switch (result) {
  case kMavResultTemporarilyRejected:
    return OperationOutcome::kNotReady;
  case kMavResultInProgress:
    return OperationOutcome::kUncertain;
  case kMavResultUnsupported:
    return OperationOutcome::kUnsupported;
  case kMavResultAccepted:
    return OperationOutcome::kUncertain;
  case kMavResultDenied:
  case kMavResultFailed:
  case kMavResultCancelled:
    return OperationOutcome::kRejected;
  default:
    return OperationOutcome::kUncertain;
  }
}

std::string nativeResultSuffix(std::uint8_t result) {
  std::ostringstream stream;
  stream << mavResultName(result) << "=" << static_cast<unsigned int>(result);
  return stream.str();
}

OperationResult nativeCommandResult(bool success, std::uint8_t native_result,
                                    const std::string &success_detail,
                                    const std::string &failure_detail) {
  const bool accepted = success && native_result == kMavResultAccepted;
  OperationResult result(accepted ? OperationOutcome::kSucceeded
                                  : nativeFailureOutcome(native_result),
                         (accepted ? success_detail : failure_detail) + " (" +
                             nativeResultSuffix(native_result) + ")");
  result.dispatched = true;
  result.has_native_result = true;
  result.native_result = native_result;
  return result;
}

} // namespace

OperationResult::OperationResult()
    : outcome(OperationOutcome::kTransportError) {}

OperationResult::OperationResult(OperationOutcome outcome_value,
                                 std::string detail_value)
    : outcome(outcome_value), detail(std::move(detail_value)) {}

bool OperationResult::succeeded() const {
  return outcome == OperationOutcome::kSucceeded;
}

bool isAllowedPx4Mode(const std::string &mode,
                      const std::vector<std::string> &allowed_modes) {
  return std::find(allowed_modes.begin(), allowed_modes.end(), mode) !=
         allowed_modes.end();
}

const char *mavResultName(std::uint8_t result) {
  switch (result) {
  case kMavResultAccepted:
    return "MAV_RESULT_ACCEPTED";
  case kMavResultTemporarilyRejected:
    return "MAV_RESULT_TEMPORARILY_REJECTED";
  case kMavResultDenied:
    return "MAV_RESULT_DENIED";
  case kMavResultUnsupported:
    return "MAV_RESULT_UNSUPPORTED";
  case kMavResultFailed:
    return "MAV_RESULT_FAILED";
  case kMavResultInProgress:
    return "MAV_RESULT_IN_PROGRESS";
  case kMavResultCancelled:
    return "MAV_RESULT_CANCELLED";
  default:
    return "MAV_RESULT_UNKNOWN";
  }
}

mavros_msgs::CommandBool makeArmCommand(bool armed) {
  mavros_msgs::CommandBool command;
  command.request.value = armed;
  return command;
}

mavros_msgs::SetMode makeModeCommand(const std::string &mode) {
  mavros_msgs::SetMode command;
  command.request.base_mode = 0u;
  command.request.custom_mode = mode;
  return command;
}

mavros_msgs::CommandLong makeAutopilotRebootCommand() {
  static_assert(mavros_msgs::CommandCode::PREFLIGHT_REBOOT_SHUTDOWN ==
                    kPx4RebootMavCommand,
                "MAVROS reboot command code changed");
  mavros_msgs::CommandLong command;
  command.request.broadcast = false;
  command.request.command = kPx4RebootMavCommand;
  command.request.confirmation = 0u;
  command.request.param1 = kPx4NormalRebootParam1;
  return command;
}

mavros_msgs::CommandLong makeForceDisarmCommand() {
  mavros_msgs::CommandLong command;
  command.request.broadcast = false;
  command.request.command = kPx4ForceDisarmMavCommand;
  command.request.confirmation = 0u;
  command.request.param1 = 0.0F;
  command.request.param2 = kPx4ForceDisarmParam2;
  return command;
}

OperationResult
interpretArmResponse(bool requested_armed,
                     const mavros_msgs::CommandBool::Response &response) {
  return nativeCommandResult(
      response.success, response.result,
      requested_armed ? "autopilot accepted arm command"
                      : "autopilot accepted disarm command",
      requested_armed ? "autopilot rejected arm command"
                      : "autopilot rejected disarm command");
}

OperationResult
interpretModeResponse(const std::string &requested_mode,
                      const mavros_msgs::SetMode::Response &response) {
  OperationResult result(response.mode_sent ? OperationOutcome::kSucceeded
                                            : OperationOutcome::kRejected,
                         response.mode_sent
                             ? "MAVROS sent PX4 mode " + requested_mode
                             : "MAVROS rejected PX4 mode " + requested_mode);
  result.dispatched = true;
  return result;
}

OperationResult interpretAutopilotRebootResponse(
    const mavros_msgs::CommandLong::Response &response) {
  return nativeCommandResult(response.success, response.result,
                             "autopilot accepted reboot command",
                             "autopilot rejected reboot command");
}

OperationResult interpretForceDisarmResponse(
    const mavros_msgs::CommandLong::Response &response) {
  return nativeCommandResult(response.success, response.result,
                             "autopilot accepted force-disarm command",
                             "autopilot rejected force-disarm command");
}

Px4RebootReadiness evaluatePx4RebootReadiness(const Px4StateSnapshot &state,
                                              const ros::WallTime &now,
                                              double state_timeout_seconds) {
  if (!state.known || state.observed_at.isZero())
    return Px4RebootReadiness::kStateUnknown;
  if (!std::isfinite(state_timeout_seconds) || state_timeout_seconds <= 0.0 ||
      now < state.observed_at ||
      (now - state.observed_at).toSec() > state_timeout_seconds) {
    return Px4RebootReadiness::kStateStale;
  }
  if (!state.connected)
    return Px4RebootReadiness::kDisconnected;
  if (state.armed)
    return Px4RebootReadiness::kArmed;
  return Px4RebootReadiness::kReady;
}

const char *px4RebootReadinessDetail(Px4RebootReadiness readiness) {
  switch (readiness) {
  case Px4RebootReadiness::kReady:
    return "autopilot state is fresh, connected, and disarmed";
  case Px4RebootReadiness::kStateUnknown:
    return "autopilot state is unknown";
  case Px4RebootReadiness::kStateStale:
    return "autopilot state is stale";
  case Px4RebootReadiness::kDisconnected:
    return "autopilot is disconnected";
  case Px4RebootReadiness::kArmed:
    return "armed autopilot cannot be rebooted";
  }
  return "autopilot reboot readiness is unknown";
}

} // namespace xgc_px4_multirotor_ros1_adapter
