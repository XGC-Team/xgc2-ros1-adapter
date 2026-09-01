#include "xgc_px4_multirotor_ros1_adapter/px4_operations.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>

#include <mavros_msgs/CommandCode.h>
#include <ros/names.h>

extern char **environ;

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

constexpr std::uint8_t kMavResultAccepted = 0u;
constexpr std::uint8_t kMavResultTemporarilyRejected = 1u;
constexpr std::uint8_t kMavResultDenied = 2u;
constexpr std::uint8_t kMavResultUnsupported = 3u;
constexpr std::uint8_t kMavResultFailed = 4u;
constexpr std::uint8_t kMavResultInProgress = 5u;
constexpr std::uint8_t kMavResultCancelled = 6u;
constexpr int kHelperSocketDescriptor = 198;
constexpr char kServiceHelperExecutable[] =
    "xgc_px4_multirotor_ros1_adapter_service_helper";

using SteadyClock = std::chrono::steady_clock;

SteadyClock::time_point steadyDeadline(const OperationWindow &window,
                                       SteadyClock::time_point started_at) {
  const std::chrono::duration<double> duration(
      (window.deadline - window.started_at).toSec());
  return started_at +
         std::chrono::duration_cast<SteadyClock::duration>(duration);
}

std::chrono::nanoseconds
steadyTimeRemaining(const SteadyClock::time_point &deadline) {
  const SteadyClock::time_point now = SteadyClock::now();
  if (now >= deadline)
    return std::chrono::nanoseconds::zero();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
}

int pollUntil(int descriptor, short events,
              const SteadyClock::time_point &deadline, short *revents) {
  for (;;) {
    const std::chrono::nanoseconds remaining = steadyTimeRemaining(deadline);
    if (remaining.count() <= 0)
      return 0;
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    long long timeout = milliseconds.count();
    if (milliseconds < remaining)
      ++timeout;
    timeout = std::max<long long>(1LL, std::min<long long>(timeout, INT_MAX));

    pollfd poll_descriptor{};
    poll_descriptor.fd = descriptor;
    poll_descriptor.events = events;
    const int result = poll(&poll_descriptor, 1u, static_cast<int>(timeout));
    if (result < 0 && errno == EINTR)
      continue;
    if (result > 0 && revents != nullptr)
      *revents = poll_descriptor.revents;
    return result;
  }
}

bool currentExecutableSibling(const std::string &sibling, std::string *path,
                              std::string *error) {
  std::array<char, PATH_MAX + 1u> executable{};
  const ssize_t size = readlink("/proc/self/exe", executable.data(), PATH_MAX);
  if (size < 0) {
    if (error != nullptr)
      *error =
          std::string("cannot resolve /proc/self/exe: ") + std::strerror(errno);
    return false;
  }
  if (size == PATH_MAX) {
    if (error != nullptr)
      *error = "resolved executable path exceeds PATH_MAX";
    return false;
  }
  executable[static_cast<std::size_t>(size)] = '\0';
  const std::string current(executable.data());
  const std::size_t separator = current.rfind('/');
  if (separator == std::string::npos) {
    if (error != nullptr)
      *error = "resolved executable has no parent directory";
    return false;
  }
  *path = current.substr(0u, separator + 1u) + sibling;
  return true;
}

bool validAbsoluteRosEndpoint(const std::string &value,
                              const std::string &description,
                              std::string *error) {
  if (value.empty() || value == "/") {
    if (error != nullptr)
      *error = description + " must not be empty or root";
    return false;
  }
  if (value.front() != '/') {
    if (error != nullptr)
      *error = description + " must be absolute";
    return false;
  }
  if (value.back() == '/') {
    if (error != nullptr)
      *error = description + " must not have a trailing slash";
    return false;
  }
  if (value.find("//") != std::string::npos) {
    if (error != nullptr)
      *error = description + " must not contain repeated slashes";
    return false;
  }
  std::string ros_error;
  if (!ros::names::validate(value, ros_error)) {
    if (error != nullptr)
      *error = description + " is invalid: " + ros_error;
    return false;
  }
  return true;
}

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

OperationResult operationWindowFailure(const OperationWindow &window) {
  if (window.state == OperationWindowState::kExpired) {
    return OperationResult(OperationOutcome::kTimedOut, window.detail);
  }
  return OperationResult(OperationOutcome::kInvalidArgument, window.detail);
}

} // namespace

class Px4OperationExecutor::NativeServiceHelper {
public:
  enum class CallState {
    kResponse,
    kUndispatchedTimedOut,
    kUndispatchedTransportError,
    kDispatchedUncertain,
  };

  struct CallResult {
    CallState state = CallState::kUndispatchedTransportError;
    Px4ServiceResponseFrame response{};
    std::string detail;
  };

  NativeServiceHelper(std::string arm_service_endpoint,
                      std::string mode_service_endpoint,
                      std::string reboot_service_endpoint)
      : arm_service_endpoint_(std::move(arm_service_endpoint)),
        mode_service_endpoint_(std::move(mode_service_endpoint)),
        reboot_service_endpoint_(std::move(reboot_service_endpoint)) {}

  ~NativeServiceHelper() { terminateAndReap(); }

  CallResult call(const Px4ServiceRequestFrame &request,
                  const SteadyClock::time_point &deadline) {
    CallResult result;
    if (!validatePx4ServiceRequest(request)) {
      result.detail = "refused to send an invalid native helper request";
      return result;
    }
    if (steadyTimeRemaining(deadline).count() <= 0) {
      result.state = CallState::kUndispatchedTimedOut;
      result.detail = "native helper request was not sent before the deadline";
      return result;
    }
    if (!ensureRunning(deadline, &result))
      return result;

    for (;;) {
      if (steadyTimeRemaining(deadline).count() <= 0) {
        result.state = CallState::kUndispatchedTimedOut;
        result.detail =
            "native helper request was not sent before the deadline";
        return result;
      }
      const ssize_t sent =
          send(socket_fd_, &request, sizeof(request), MSG_NOSIGNAL);
      if (sent == static_cast<ssize_t>(sizeof(request))) {
        break;
      }
      if (sent > 0) {
        terminateAndReap();
        result.state = CallState::kDispatchedUncertain;
        result.detail = "native helper accepted a malformed partial frame";
        return result;
      }
      if (sent < 0 && errno == EINTR)
        continue;
      if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        const std::string reason = std::strerror(errno);
        terminateAndReap();
        result.state = CallState::kUndispatchedTransportError;
        result.detail = "native helper request could not be sent: " + reason;
        return result;
      }

      short revents = 0;
      const int poll_result =
          pollUntil(socket_fd_, POLLOUT, deadline, &revents);
      if (poll_result == 0) {
        result.state = CallState::kUndispatchedTimedOut;
        result.detail =
            "native helper request was not sent before the deadline";
        return result;
      }
      if (poll_result < 0 || (revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        const std::string reason =
            poll_result < 0 ? std::strerror(errno) : "helper disconnected";
        terminateAndReap();
        result.state = CallState::kUndispatchedTransportError;
        result.detail = "native helper request could not be sent: " + reason;
        return result;
      }
    }

    for (;;) {
      short revents = 0;
      const int poll_result = pollUntil(socket_fd_, POLLIN, deadline, &revents);
      if (poll_result == 0) {
        terminateAndReap();
        result.state = CallState::kDispatchedUncertain;
        result.detail = "native helper did not return before the deadline";
        return result;
      }
      if (poll_result < 0) {
        const std::string reason = std::strerror(errno);
        terminateAndReap();
        result.state = CallState::kDispatchedUncertain;
        result.detail = "native helper response failed: " + reason;
        return result;
      }
      // The first parent observation of socket readability defines whether the
      // response met the operation deadline. The protocol does not trust a
      // response merely because it remained queued after cancellation.
      if (SteadyClock::now() >= deadline) {
        terminateAndReap();
        result.state = CallState::kDispatchedUncertain;
        result.detail =
            "native helper response became observable after the deadline";
        return result;
      }

      Px4ServiceResponseFrame response{};
      const ssize_t received = recv(socket_fd_, &response, sizeof(response),
                                    MSG_DONTWAIT | MSG_TRUNC);
      if (received < 0 && errno == EINTR)
        continue;
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        if ((revents & (POLLERR | POLLHUP | POLLNVAL)) == 0)
          continue;
      } else if (received == static_cast<ssize_t>(sizeof(response)) &&
                 validatePx4ServiceResponse(response, request)) {
        const auto status =
            static_cast<Px4ServiceResponseStatus>(response.status);
        if (status == Px4ServiceResponseStatus::kCompleted) {
          result.state = CallState::kResponse;
          result.response = response;
          result.detail = "native helper returned a complete response";
          return result;
        }
        terminateAndReap();
        result.state = CallState::kDispatchedUncertain;
        result.detail = status == Px4ServiceResponseStatus::kCallFailed
                            ? "native ROS service call ended without a response"
                            : "native helper rejected the request protocol";
        return result;
      }

      terminateAndReap();
      result.state = CallState::kDispatchedUncertain;
      result.detail = "native helper returned an invalid response frame";
      return result;
    }
  }

private:
  bool ensureRunning(const SteadyClock::time_point &deadline,
                     CallResult *failure) {
    if (process_id_ > 0) {
      int status = 0;
      pid_t waited;
      do {
        waited = waitpid(process_id_, &status, WNOHANG);
      } while (waited < 0 && errno == EINTR);
      if (waited == 0)
        return true;
      if (waited < 0 && errno != ECHILD) {
        terminateAndReap();
      }
      if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
      }
      process_id_ = -1;
    }

    if (steadyTimeRemaining(deadline).count() <= 0) {
      failure->state = CallState::kUndispatchedTimedOut;
      failure->detail = "native helper was not started before the deadline";
      return false;
    }

    std::string helper_path;
    if (!currentExecutableSibling(kServiceHelperExecutable, &helper_path,
                                  &failure->detail)) {
      failure->state = CallState::kUndispatchedTransportError;
      return false;
    }

    int sockets[2] = {-1, -1};
    if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0,
                   sockets) != 0) {
      failure->state = CallState::kUndispatchedTransportError;
      failure->detail = std::string("cannot create native helper socket: ") +
                        std::strerror(errno);
      return false;
    }
    for (int index = 0; index < 2; ++index) {
      if (sockets[index] == kHelperSocketDescriptor) {
        const int replacement =
            fcntl(sockets[index], F_DUPFD_CLOEXEC, kHelperSocketDescriptor + 1);
        if (replacement < 0) {
          const std::string reason = std::strerror(errno);
          close(sockets[0]);
          close(sockets[1]);
          failure->state = CallState::kUndispatchedTransportError;
          failure->detail = "cannot relocate native helper socket: " + reason;
          return false;
        }
        close(sockets[index]);
        sockets[index] = replacement;
      }
    }

    posix_spawn_file_actions_t actions;
    int spawn_error = posix_spawn_file_actions_init(&actions);
    const bool actions_initialized = spawn_error == 0;
    if (spawn_error == 0)
      spawn_error = posix_spawn_file_actions_adddup2(&actions, sockets[1],
                                                     kHelperSocketDescriptor);
    if (spawn_error == 0)
      spawn_error = posix_spawn_file_actions_addclose(&actions, sockets[0]);
    if (spawn_error == 0)
      spawn_error = posix_spawn_file_actions_addclose(&actions, sockets[1]);

    pid_t child = -1;
    if (spawn_error == 0) {
      const std::string descriptor = std::to_string(kHelperSocketDescriptor);
      std::array<char *, 6u> arguments{
          const_cast<char *>(helper_path.c_str()),
          const_cast<char *>(descriptor.c_str()),
          const_cast<char *>(arm_service_endpoint_.c_str()),
          const_cast<char *>(mode_service_endpoint_.c_str()),
          const_cast<char *>(reboot_service_endpoint_.c_str()),
          nullptr};
      spawn_error = posix_spawn(&child, helper_path.c_str(), &actions, nullptr,
                                arguments.data(), environ);
    }
    if (actions_initialized)
      posix_spawn_file_actions_destroy(&actions);
    close(sockets[1]);

    if (spawn_error != 0) {
      close(sockets[0]);
      failure->state = CallState::kUndispatchedTransportError;
      failure->detail = "cannot start native service helper: " +
                        std::string(std::strerror(spawn_error));
      return false;
    }

    process_id_ = child;
    socket_fd_ = sockets[0];
    if (steadyTimeRemaining(deadline).count() <= 0) {
      terminateAndReap();
      failure->state = CallState::kUndispatchedTimedOut;
      failure->detail = "native helper was not started before the deadline";
      return false;
    }
    return true;
  }

  void terminateAndReap() {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
    if (process_id_ <= 0)
      return;

    int kill_result;
    do {
      kill_result = kill(process_id_, SIGKILL);
    } while (kill_result < 0 && errno == EINTR);

    int status = 0;
    pid_t waited;
    do {
      waited = waitpid(process_id_, &status, 0);
    } while (waited < 0 && errno == EINTR);
    process_id_ = -1;
  }

  const std::string arm_service_endpoint_;
  const std::string mode_service_endpoint_;
  const std::string reboot_service_endpoint_;
  pid_t process_id_ = -1;
  int socket_fd_ = -1;
};

OperationResult::OperationResult()
    : outcome(OperationOutcome::kTransportError) {}

OperationResult::OperationResult(OperationOutcome outcome_value,
                                 std::string detail_value)
    : outcome(outcome_value), detail(std::move(detail_value)) {}

bool OperationResult::succeeded() const {
  return outcome == OperationOutcome::kSucceeded;
}

OperationTiming::OperationTiming(double timeout_seconds_value,
                                 ros::WallTime deadline_value)
    : timeout_seconds(timeout_seconds_value), deadline(deadline_value) {}

bool OperationWindow::ready() const {
  return state == OperationWindowState::kReady;
}

OperationWindow makeOperationWindow(const OperationTiming &timing,
                                    const ros::WallTime &started_at,
                                    double maximum_timeout_seconds) {
  OperationWindow window;
  window.started_at = started_at;

  if (!std::isfinite(timing.timeout_seconds) || timing.timeout_seconds <= 0.0) {
    window.detail = "operation timeout must be finite and positive";
    return window;
  }
  if (!std::isfinite(maximum_timeout_seconds) ||
      maximum_timeout_seconds <= 0.0) {
    window.detail = "maximum operation timeout must be finite and positive";
    return window;
  }
  if (!timing.deadline.isZero() && timing.deadline <= started_at) {
    window.state = OperationWindowState::kExpired;
    window.deadline = timing.deadline;
    window.detail = "operation deadline has expired";
    return window;
  }

  const double bounded_timeout =
      std::min(timing.timeout_seconds, maximum_timeout_seconds);
  try {
    window.deadline = started_at + ros::WallDuration(bounded_timeout);
  } catch (const std::runtime_error &) {
    window.detail = "operation timeout exceeds the ROS wall-time range";
    return window;
  }
  if (!timing.deadline.isZero() && timing.deadline < window.deadline) {
    window.deadline = timing.deadline;
  }
  window.state = OperationWindowState::kReady;
  window.detail = "operation may be dispatched";
  return window;
}

ros::WallDuration operationTimeRemaining(const OperationWindow &window,
                                         const ros::WallTime &now) {
  if (!window.ready() || now < window.started_at || now >= window.deadline) {
    return ros::WallDuration(0.0);
  }
  return window.deadline - now;
}

bool isAllowedPx4Mode(const std::string &mode,
                      const std::vector<std::string> &allowed_modes) {
  return std::find(allowed_modes.begin(), allowed_modes.end(), mode) !=
         allowed_modes.end();
}

ros::WallTime operationDeadlineFromUnixNanos(std::int64_t unix_nanos) {
  if (unix_nanos <= 0)
    return ros::WallTime();

  constexpr std::uint64_t kNanosPerSecond = 1000000000ULL;
  const std::uint64_t value = static_cast<std::uint64_t>(unix_nanos);
  const std::uint64_t seconds = value / kNanosPerSecond;
  if (seconds > std::numeric_limits<std::uint32_t>::max()) {
    return ros::WallTime(std::numeric_limits<std::uint32_t>::max(),
                         static_cast<std::uint32_t>(kNanosPerSecond - 1u));
  }
  return ros::WallTime(static_cast<std::uint32_t>(seconds),
                       static_cast<std::uint32_t>(value % kNanosPerSecond));
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

std::unique_ptr<Px4OperationExecutor>
Px4OperationExecutor::Create(ros::NodeHandle node_handle, Config config,
                             std::string *error) {
  const std::array<std::pair<const std::string *, const char *>, 3u> endpoints{{
      {&config.arm_service_endpoint, "PX4 arming service endpoint"},
      {&config.mode_service_endpoint, "PX4 mode service endpoint"},
      {&config.reboot_service_endpoint, "PX4 reboot service endpoint"},
  }};
  for (const auto &endpoint : endpoints) {
    if (!validAbsoluteRosEndpoint(*endpoint.first, endpoint.second, error))
      return nullptr;
  }
  if (config.require_state &&
      !validAbsoluteRosEndpoint(config.state_endpoint, "PX4 state endpoint",
                                error)) {
    return nullptr;
  }
  if (config.arm_service_endpoint == config.mode_service_endpoint ||
      config.arm_service_endpoint == config.reboot_service_endpoint ||
      config.mode_service_endpoint == config.reboot_service_endpoint) {
    if (error != nullptr)
      *error = "PX4 operation service endpoints must be distinct";
    return nullptr;
  }
  if (config.allowed_modes.empty()) {
    if (error != nullptr)
      *error = "PX4 mode allowlist must not be empty";
    return nullptr;
  }
  for (std::size_t index = 0u; index < config.allowed_modes.size(); ++index) {
    const std::string &mode = config.allowed_modes[index];
    Px4ServiceRequestFrame frame{};
    if (mode.empty() || !makePx4SetModeRequest(1u, mode, &frame)) {
      if (error != nullptr)
        *error = "PX4 mode allowlist contains an invalid helper mode";
      return nullptr;
    }
    if (std::find(config.allowed_modes.begin(),
                  config.allowed_modes.begin() + index,
                  mode) != config.allowed_modes.begin() + index) {
      if (error != nullptr)
        *error = "PX4 mode allowlist contains a duplicate mode";
      return nullptr;
    }
  }
  if (!std::isfinite(config.state_timeout_seconds) ||
      config.state_timeout_seconds <= 0.0) {
    if (error != nullptr)
      *error = "state timeout must be finite and positive";
    return nullptr;
  }
  if (config.state_timeout_seconds > kDefaultPx4StateTimeoutSeconds) {
    if (error != nullptr)
      *error = "state timeout exceeds the PX4 profile safety limit";
    return nullptr;
  }
  if (!std::isfinite(config.maximum_operation_timeout_seconds) ||
      config.maximum_operation_timeout_seconds <= 0.0) {
    if (error != nullptr)
      *error = "maximum operation timeout must be finite and positive";
    return nullptr;
  }
  if (config.maximum_operation_timeout_seconds >
      kDefaultOperationTimeoutSeconds) {
    if (error != nullptr)
      *error = "operation timeout exceeds the PX4 profile safety limit";
    return nullptr;
  }
  const bool require_state = config.require_state;
  std::unique_ptr<Px4OperationExecutor> executor(
      new Px4OperationExecutor(std::move(node_handle), std::move(config)));
  if (require_state && !executor->state_subscriber_) {
    if (error != nullptr) {
      *error = "ROS master did not accept PX4 state subscription";
    }
    return nullptr;
  }
  return executor;
}

Px4OperationExecutor::Px4OperationExecutor(ros::NodeHandle node_handle,
                                           Config config)
    : node_handle_(std::move(node_handle)),
      state_timeout_seconds_(config.state_timeout_seconds),
      maximum_operation_timeout_seconds_(
          config.maximum_operation_timeout_seconds),
      allowed_modes_(std::move(config.allowed_modes)),
      state_(std::make_shared<StateStore>()),
      native_service_helper_(new NativeServiceHelper(
          config.arm_service_endpoint, config.mode_service_endpoint,
          config.reboot_service_endpoint)) {
  if (config.require_state) {
    const std::shared_ptr<StateStore> state = state_;
    state_subscriber_ = node_handle_.subscribe<mavros_msgs::State>(
        config.state_endpoint, 1,
        [state](const mavros_msgs::State::ConstPtr &message) {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->snapshot.known = true;
          state->snapshot.connected = message->connected;
          state->snapshot.armed = message->armed;
          state->snapshot.observed_at = ros::WallTime::now();
        },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  }
}

Px4OperationExecutor::~Px4OperationExecutor() = default;

Px4StateSnapshot Px4OperationExecutor::stateSnapshot() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->snapshot;
}

OperationResult
Px4OperationExecutor::callNativeService(const Px4ServiceRequestFrame &request,
                                        const SteadyClock::time_point &deadline,
                                        const std::string &service_description,
                                        Px4ServiceResponseFrame *response) {
  const NativeServiceHelper::CallResult helper_result =
      native_service_helper_->call(request, deadline);
  switch (helper_result.state) {
  case NativeServiceHelper::CallState::kResponse:
    *response = helper_result.response;
    return OperationResult(OperationOutcome::kSucceeded,
                           service_description + " returned a response");
  case NativeServiceHelper::CallState::kUndispatchedTimedOut:
    return OperationResult(OperationOutcome::kTimedOut,
                           service_description +
                               " was not dispatched: " + helper_result.detail);
  case NativeServiceHelper::CallState::kUndispatchedTransportError:
    return OperationResult(OperationOutcome::kTransportError,
                           service_description +
                               " was not dispatched: " + helper_result.detail);
  case NativeServiceHelper::CallState::kDispatchedUncertain: {
    OperationResult result(
        OperationOutcome::kUncertain,
        service_description +
            " has an uncertain native outcome: " + helper_result.detail);
    result.dispatched = true;
    return result;
  }
  }
  return OperationResult(OperationOutcome::kTransportError,
                         service_description + " helper state is invalid");
}

OperationResult Px4OperationExecutor::setArmed(bool armed,
                                               const OperationTiming &timing) {
  const SteadyClock::time_point steady_started_at = SteadyClock::now();
  const OperationWindow window = makeOperationWindow(
      timing, ros::WallTime::now(), maximum_operation_timeout_seconds_);
  if (!window.ready())
    return operationWindowFailure(window);
  const SteadyClock::time_point deadline =
      steadyDeadline(window, steady_started_at);
  std::unique_lock<std::timed_mutex> operation_lock(operation_mutex_,
                                                    std::defer_lock);
  if (!operation_lock.try_lock_until(deadline)) {
    return OperationResult(
        OperationOutcome::kTimedOut,
        "MAVROS arming service was not dispatched before the deadline while "
        "waiting for another native operation");
  }

  if (next_request_id_ == 0u)
    next_request_id_ = 1u;
  const Px4ServiceRequestFrame request =
      makePx4SetArmedRequest(next_request_id_++, armed);
  Px4ServiceResponseFrame response{};
  OperationResult failure =
      callNativeService(request, deadline, "MAVROS arming service", &response);
  if (!failure.succeeded())
    return failure;
  mavros_msgs::CommandBool::Response native_response;
  native_response.success = response.logical_success != 0u;
  native_response.result = response.native_result;
  return interpretArmResponse(armed, native_response);
}

OperationResult Px4OperationExecutor::setMode(const std::string &mode,
                                              const OperationTiming &timing) {
  const SteadyClock::time_point steady_started_at = SteadyClock::now();
  const OperationWindow window = makeOperationWindow(
      timing, ros::WallTime::now(), maximum_operation_timeout_seconds_);
  if (!window.ready())
    return operationWindowFailure(window);
  if (!isAllowedPx4Mode(mode, allowed_modes_)) {
    return OperationResult(OperationOutcome::kInvalidArgument,
                           "PX4 mode is not allowed by the installed profile");
  }

  const SteadyClock::time_point deadline =
      steadyDeadline(window, steady_started_at);
  std::unique_lock<std::timed_mutex> operation_lock(operation_mutex_,
                                                    std::defer_lock);
  if (!operation_lock.try_lock_until(deadline)) {
    return OperationResult(
        OperationOutcome::kTimedOut,
        "MAVROS set-mode service was not dispatched before the deadline while "
        "waiting for another native operation");
  }

  if (next_request_id_ == 0u)
    next_request_id_ = 1u;
  Px4ServiceRequestFrame request{};
  if (!makePx4SetModeRequest(next_request_id_++, mode, &request)) {
    return OperationResult(OperationOutcome::kInvalidArgument,
                           "PX4 mode cannot be represented by the helper "
                           "protocol");
  }
  Px4ServiceResponseFrame response{};
  OperationResult failure = callNativeService(
      request, deadline, "MAVROS set-mode service", &response);
  if (!failure.succeeded())
    return failure;
  mavros_msgs::SetMode::Response native_response;
  native_response.mode_sent = response.logical_success != 0u;
  return interpretModeResponse(mode, native_response);
}

OperationResult
Px4OperationExecutor::rebootAutopilot(const OperationTiming &timing) {
  const SteadyClock::time_point steady_started_at = SteadyClock::now();
  const OperationWindow window = makeOperationWindow(
      timing, ros::WallTime::now(), maximum_operation_timeout_seconds_);
  if (!window.ready())
    return operationWindowFailure(window);
  const SteadyClock::time_point deadline =
      steadyDeadline(window, steady_started_at);
  std::unique_lock<std::timed_mutex> operation_lock(operation_mutex_,
                                                    std::defer_lock);
  if (!operation_lock.try_lock_until(deadline)) {
    return OperationResult(
        OperationOutcome::kTimedOut,
        "MAVROS reboot command was not dispatched before the deadline while "
        "waiting for another native operation");
  }

  const ros::WallTime readiness_checked_at = ros::WallTime::now();
  const Px4RebootReadiness readiness = evaluatePx4RebootReadiness(
      stateSnapshot(), readiness_checked_at, state_timeout_seconds_);
  if (readiness != Px4RebootReadiness::kReady) {
    const OperationOutcome outcome = readiness == Px4RebootReadiness::kArmed
                                         ? OperationOutcome::kRejected
                                         : OperationOutcome::kNotReady;
    return OperationResult(outcome, px4RebootReadinessDetail(readiness));
  }
  if (steadyTimeRemaining(deadline).count() <= 0) {
    return OperationResult(
        OperationOutcome::kTimedOut,
        "MAVROS reboot command was not dispatched before the deadline");
  }

  if (next_request_id_ == 0u)
    next_request_id_ = 1u;
  const Px4ServiceRequestFrame request =
      makePx4RebootRequest(next_request_id_++);
  Px4ServiceResponseFrame response{};
  OperationResult failure = callNativeService(
      request, deadline, "MAVROS reboot command service", &response);
  if (!failure.succeeded())
    return failure;
  mavros_msgs::CommandLong::Response native_response;
  native_response.success = response.logical_success != 0u;
  native_response.result = response.native_result;
  return interpretAutopilotRebootResponse(native_response);
}

OperationResult
Px4OperationExecutor::forceDisarm(const OperationTiming &timing) {
  const SteadyClock::time_point steady_started_at = SteadyClock::now();
  const OperationWindow window = makeOperationWindow(
      timing, ros::WallTime::now(), maximum_operation_timeout_seconds_);
  if (!window.ready())
    return operationWindowFailure(window);
  const SteadyClock::time_point deadline =
      steadyDeadline(window, steady_started_at);
  std::unique_lock<std::timed_mutex> operation_lock(operation_mutex_,
                                                    std::defer_lock);
  if (!operation_lock.try_lock_until(deadline)) {
    return OperationResult(
        OperationOutcome::kTimedOut,
        "MAVROS force-disarm command was not dispatched before the deadline "
        "while waiting for another native operation");
  }

  if (next_request_id_ == 0u)
    next_request_id_ = 1u;
  const Px4ServiceRequestFrame request =
      makePx4ForceDisarmRequest(next_request_id_++);
  Px4ServiceResponseFrame response{};
  OperationResult failure = callNativeService(
      request, deadline, "MAVROS force-disarm command service", &response);
  if (!failure.succeeded())
    return failure;
  mavros_msgs::CommandLong::Response native_response;
  native_response.success = response.logical_success != 0u;
  native_response.result = response.native_result;
  return interpretForceDisarmResponse(native_response);
}

} // namespace xgc_px4_multirotor_ros1_adapter
