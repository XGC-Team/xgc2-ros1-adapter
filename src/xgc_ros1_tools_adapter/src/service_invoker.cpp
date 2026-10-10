#include "xgc_ros1_tools_adapter/service_invoker.hpp"

#include <fcntl.h>
#include <poll.h>
#include <ros_babel_fish/generation/message_creation.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "xgc_ros1_tools_adapter/error.hpp"
#include "xgc_ros1_tools_adapter/service_helper_protocol.hpp"

extern char** environ;

namespace xgc_ros1_tools_adapter {
namespace {

constexpr int kHelperSocketDescriptor = 198;
constexpr char kServiceHelperExecutable[] =
    "xgc_ros1_tools_adapter_service_helper";
constexpr auto kCancellationPollInterval = std::chrono::milliseconds(5);

using SteadyClock = std::chrono::steady_clock;

bool deadlineExpired(std::int64_t deadline_unix_nanos) {
  if (deadline_unix_nanos <= 0) {
    return false;
  }
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto now_nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return now_nanos >= deadline_unix_nanos;
}

bool cancellationRequested(const ServiceCallRequest& request) {
  return request.cancellation_requested && request.cancellation_requested();
}

void requirePreDispatchAllowed(const ServiceCallRequest& request) {
  if (cancellationRequested(request)) {
    cancelledError("service_call_cancelled",
                   "ROS1 service call was cancelled before dispatch");
  }
  if (deadlineExpired(request.deadline_unix_nanos)) {
    deadlineError("service_deadline_elapsed",
                  "ROS1 service deadline elapsed before dispatch");
  }
}

SteadyClock::time_point runtimeDeadline(std::int64_t deadline_unix_nanos) {
  if (deadline_unix_nanos <= 0) {
    return SteadyClock::time_point::max();
  }
  const auto system_now = std::chrono::system_clock::now().time_since_epoch();
  const auto system_now_nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(system_now).count();
  const auto steady_now = SteadyClock::now();
  if (deadline_unix_nanos <= system_now_nanos) {
    return steady_now;
  }
  const std::chrono::nanoseconds remaining(deadline_unix_nanos -
                                           system_now_nanos);
  const auto maximum_remaining =
      SteadyClock::time_point::max().time_since_epoch() -
      steady_now.time_since_epoch();
  if (remaining >= maximum_remaining) {
    return SteadyClock::time_point::max();
  }
  return steady_now +
         std::chrono::duration_cast<SteadyClock::duration>(remaining);
}

int pollTimeout(const SteadyClock::time_point& deadline) {
  if (deadline == SteadyClock::time_point::max()) {
    return static_cast<int>(kCancellationPollInterval.count());
  }
  const auto now = SteadyClock::now();
  if (now >= deadline) {
    return 0;
  }
  const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
  const auto bounded =
      std::max(std::chrono::milliseconds(1),
               std::min(kCancellationPollInterval,
                        remaining + std::chrono::milliseconds(1)));
  return static_cast<int>(bounded.count());
}

bool currentExecutableSibling(const std::string& sibling, std::string* path,
                              std::string* error) {
  std::array<char, PATH_MAX + 1u> executable{};
  const ssize_t size = readlink("/proc/self/exe", executable.data(), PATH_MAX);
  if (size < 0) {
    if (error != nullptr) {
      *error =
          std::string("cannot resolve /proc/self/exe: ") + std::strerror(errno);
    }
    return false;
  }
  if (size == PATH_MAX) {
    if (error != nullptr) {
      *error = "resolved executable path exceeds PATH_MAX";
    }
    return false;
  }
  executable[static_cast<std::size_t>(size)] = '\0';
  const std::string current(executable.data());
  const std::size_t separator = current.rfind('/');
  if (separator == std::string::npos) {
    if (error != nullptr) {
      *error = "resolved executable has no parent directory";
    }
    return false;
  }
  *path = current.substr(0u, separator + 1u) + sibling;
  return true;
}

class HelperProcess {
 public:
  HelperProcess() = default;
  ~HelperProcess() { terminateAndReap(); }

  HelperProcess(const HelperProcess&) = delete;
  HelperProcess& operator=(const HelperProcess&) = delete;

  bool start(std::string* error) {
    std::string helper_path;
    if (!currentExecutableSibling(kServiceHelperExecutable, &helper_path,
                                  error)) {
      return false;
    }

    int sockets[2] = {-1, -1};
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0,
                   sockets) != 0) {
      if (error != nullptr) {
        *error = std::string("cannot create service helper socket: ") +
                 std::strerror(errno);
      }
      return false;
    }
    for (int index = 0; index < 2; ++index) {
      if (sockets[index] != kHelperSocketDescriptor) {
        continue;
      }
      const int replacement =
          fcntl(sockets[index], F_DUPFD_CLOEXEC, kHelperSocketDescriptor + 1);
      if (replacement < 0) {
        const std::string reason = std::strerror(errno);
        close(sockets[0]);
        close(sockets[1]);
        if (error != nullptr) {
          *error = "cannot relocate service helper socket: " + reason;
        }
        return false;
      }
      close(sockets[index]);
      sockets[index] = replacement;
    }

    posix_spawn_file_actions_t actions;
    int spawn_error = posix_spawn_file_actions_init(&actions);
    const bool actions_initialized = spawn_error == 0;
    if (spawn_error == 0) {
      spawn_error = posix_spawn_file_actions_adddup2(&actions, sockets[1],
                                                     kHelperSocketDescriptor);
    }
    if (spawn_error == 0) {
      spawn_error = posix_spawn_file_actions_addclose(&actions, sockets[0]);
    }
    if (spawn_error == 0) {
      spawn_error = posix_spawn_file_actions_addclose(&actions, sockets[1]);
    }

    pid_t child = -1;
    if (spawn_error == 0) {
      const std::string descriptor = std::to_string(kHelperSocketDescriptor);
      std::array<char*, 3u> arguments{const_cast<char*>(helper_path.c_str()),
                                      const_cast<char*>(descriptor.c_str()),
                                      nullptr};
      spawn_error = posix_spawn(&child, helper_path.c_str(), &actions, nullptr,
                                arguments.data(), environ);
    }
    if (actions_initialized) {
      posix_spawn_file_actions_destroy(&actions);
    }
    close(sockets[1]);

    if (spawn_error != 0) {
      close(sockets[0]);
      if (error != nullptr) {
        *error = "cannot start service helper: " +
                 std::string(std::strerror(spawn_error));
      }
      return false;
    }

    process_id_ = child;
    socket_fd_ = sockets[0];
    return true;
  }

  int socketFd() const noexcept { return socket_fd_; }

  void terminateAndReap() noexcept {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
    if (process_id_ <= 0) {
      return;
    }
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

 private:
  pid_t process_id_ = -1;
  int socket_fd_ = -1;
};

class InFlightPermit {
 public:
  explicit InFlightPermit(std::shared_ptr<std::atomic<std::uint32_t>> in_flight)
      : in_flight_(std::move(in_flight)) {}

  ~InFlightPermit() {
    if (in_flight_) {
      in_flight_->fetch_sub(1u, std::memory_order_acq_rel);
    }
  }

  InFlightPermit(const InFlightPermit&) = delete;
  InFlightPermit& operator=(const InFlightPermit&) = delete;

 private:
  std::shared_ptr<std::atomic<std::uint32_t>> in_flight_;
};

bool acquireInFlight(const std::shared_ptr<std::atomic<std::uint32_t>>& state,
                     std::uint32_t maximum) {
  std::uint32_t current = state->load(std::memory_order_relaxed);
  while (true) {
    if (current >= maximum) {
      return false;
    }
    if (state->compare_exchange_weak(current, current + 1u,
                                     std::memory_order_acq_rel,
                                     std::memory_order_relaxed)) {
      return true;
    }
  }
}

enum class PreCommitIoResult {
  kComplete,
  kCancelled,
  kDeadline,
  kTransportError,
};

PreCommitIoResult waitPreCommitEvent(int socket_fd, short events,
                                     const ServiceCallRequest& request,
                                     const SteadyClock::time_point& deadline,
                                     std::string* error) {
  for (;;) {
    if (cancellationRequested(request)) {
      return PreCommitIoResult::kCancelled;
    }
    if (deadlineExpired(request.deadline_unix_nanos) ||
        SteadyClock::now() >= deadline) {
      return PreCommitIoResult::kDeadline;
    }
    pollfd descriptor{};
    descriptor.fd = socket_fd;
    descriptor.events = events;
    const int result = poll(&descriptor, 1u, pollTimeout(deadline));
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0) {
      if (error != nullptr) {
        *error = std::strerror(errno);
      }
      return PreCommitIoResult::kTransportError;
    }
    if (result == 0) {
      continue;
    }
    if ((descriptor.revents & events) != 0) {
      return PreCommitIoResult::kComplete;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      if (error != nullptr) {
        *error = "service helper disconnected";
      }
      return PreCommitIoResult::kTransportError;
    }
  }
}

PreCommitIoResult writePreCommit(int socket_fd, const void* input,
                                 std::size_t size,
                                 const ServiceCallRequest& request,
                                 const SteadyClock::time_point& deadline,
                                 std::string* error) {
  const auto* cursor = static_cast<const std::uint8_t*>(input);
  while (size > 0u) {
    if (cancellationRequested(request)) {
      return PreCommitIoResult::kCancelled;
    }
    if (deadlineExpired(request.deadline_unix_nanos) ||
        SteadyClock::now() >= deadline) {
      return PreCommitIoResult::kDeadline;
    }
    const ssize_t sent =
        send(socket_fd, cursor, size, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent > 0) {
      cursor += sent;
      size -= static_cast<std::size_t>(sent);
      continue;
    }
    if (sent < 0 && errno == EINTR) {
      continue;
    }
    if (sent >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
      if (error != nullptr) {
        *error =
            sent == 0 ? "service helper socket closed" : std::strerror(errno);
      }
      return PreCommitIoResult::kTransportError;
    }
    const PreCommitIoResult waited =
        waitPreCommitEvent(socket_fd, POLLOUT, request, deadline, error);
    if (waited != PreCommitIoResult::kComplete) {
      return waited;
    }
  }
  return PreCommitIoResult::kComplete;
}

PreCommitIoResult readPreCommit(int socket_fd, void* output, std::size_t size,
                                const ServiceCallRequest& request,
                                const SteadyClock::time_point& deadline,
                                std::string* error) {
  auto* cursor = static_cast<std::uint8_t*>(output);
  while (size > 0u) {
    if (cancellationRequested(request)) {
      return PreCommitIoResult::kCancelled;
    }
    if (deadlineExpired(request.deadline_unix_nanos) ||
        SteadyClock::now() >= deadline) {
      return PreCommitIoResult::kDeadline;
    }
    const ssize_t received = recv(socket_fd, cursor, size, MSG_DONTWAIT);
    if (received > 0) {
      cursor += received;
      size -= static_cast<std::size_t>(received);
      continue;
    }
    if (received == 0) {
      if (error != nullptr) {
        *error = "service helper disconnected";
      }
      return PreCommitIoResult::kTransportError;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      if (error != nullptr) {
        *error = std::strerror(errno);
      }
      return PreCommitIoResult::kTransportError;
    }
    const PreCommitIoResult waited =
        waitPreCommitEvent(socket_fd, POLLIN, request, deadline, error);
    if (waited != PreCommitIoResult::kComplete) {
      return waited;
    }
  }
  return PreCommitIoResult::kComplete;
}

[[noreturn]] void failPreCommit(PreCommitIoResult result,
                                const std::string& transport_error) {
  switch (result) {
    case PreCommitIoResult::kCancelled:
      cancelledError("service_call_cancelled",
                     "ROS1 service call was cancelled before dispatch");
    case PreCommitIoResult::kDeadline:
      deadlineError("service_deadline_elapsed",
                    "ROS1 service deadline elapsed before dispatch");
    case PreCommitIoResult::kTransportError:
      transientError(
          "service_helper_unavailable",
          "ROS1 service helper failed before dispatch: " + transport_error);
    case PreCommitIoResult::kComplete:
      break;
  }
  transientError("service_helper_unavailable",
                 "ROS1 service helper failed before dispatch");
}

enum class PostCommitIoResult {
  kComplete,
  kCancelled,
  kRuntimeDeadline,
  kCallTimeout,
  kTransportError,
};

PostCommitIoResult readPostCommit(
    int socket_fd, void* output, std::size_t size,
    const ServiceCallRequest& request,
    const SteadyClock::time_point& runtime_deadline,
    const SteadyClock::time_point& call_deadline, std::string* error) {
  auto* cursor = static_cast<std::uint8_t*>(output);
  while (size > 0u) {
    if (cancellationRequested(request)) {
      return PostCommitIoResult::kCancelled;
    }
    const auto now = SteadyClock::now();
    if (deadlineExpired(request.deadline_unix_nanos) ||
        now >= runtime_deadline) {
      return PostCommitIoResult::kRuntimeDeadline;
    }
    if (now >= call_deadline) {
      return PostCommitIoResult::kCallTimeout;
    }

    const ssize_t received = recv(socket_fd, cursor, size, MSG_DONTWAIT);
    if (received > 0) {
      cursor += received;
      size -= static_cast<std::size_t>(received);
      continue;
    }
    if (received == 0) {
      if (error != nullptr) {
        *error = "service helper disconnected";
      }
      return PostCommitIoResult::kTransportError;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      if (error != nullptr) {
        *error = std::strerror(errno);
      }
      return PostCommitIoResult::kTransportError;
    }

    pollfd descriptor{};
    descriptor.fd = socket_fd;
    descriptor.events = POLLIN;
    const auto effective_deadline = std::min(runtime_deadline, call_deadline);
    const int result = poll(&descriptor, 1u, pollTimeout(effective_deadline));
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0) {
      if (error != nullptr) {
        *error = std::strerror(errno);
      }
      return PostCommitIoResult::kTransportError;
    }
    if (result == 0) {
      continue;
    }
    if ((descriptor.revents & POLLIN) != 0) {
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      if (error != nullptr) {
        *error = "service helper disconnected";
      }
      return PostCommitIoResult::kTransportError;
    }
  }

  if (cancellationRequested(request)) {
    return PostCommitIoResult::kCancelled;
  }
  const auto now = SteadyClock::now();
  if (deadlineExpired(request.deadline_unix_nanos) || now >= runtime_deadline) {
    return PostCommitIoResult::kRuntimeDeadline;
  }
  if (now >= call_deadline) {
    return PostCommitIoResult::kCallTimeout;
  }
  return PostCommitIoResult::kComplete;
}

[[noreturn]] void failPostCommit(PostCommitIoResult result,
                                 const std::string& service,
                                 const std::string& transport_error) {
  switch (result) {
    case PostCommitIoResult::kCancelled:
      uncertainError("service_call_cancelled",
                     "ROS1 service call to '" + service +
                         "' was cancelled after commit; the server may have "
                         "executed the request");
    case PostCommitIoResult::kRuntimeDeadline:
      uncertainError("service_deadline_elapsed",
                     "ROS1 service call to '" + service +
                         "' crossed its Runtime deadline after commit; the "
                         "server may have executed the request");
    case PostCommitIoResult::kCallTimeout:
      uncertainError(
          "service_call_timeout",
          "ROS1 service call to '" + service +
              "' exceeded callTimeoutMs after commit; the server may "
              "have executed the request");
    case PostCommitIoResult::kTransportError:
      uncertainError("service_call_failed",
                     "ROS1 service helper failed after commit for '" + service +
                         "': " + transport_error);
    case PostCommitIoResult::kComplete:
      break;
  }
  uncertainError(
      "service_call_failed",
      "ROS1 service helper failed after commit for '" + service + "'");
}

std::vector<std::uint8_t> serializeServiceRequest(
    const ros_babel_fish::Message& message) {
  const std::size_t payload_size = message._sizeInBytes();
  if (payload_size > kMaximumServicePayloadBytes ||
      payload_size > std::numeric_limits<std::uint32_t>::max()) {
    permanentError("service_request_too_large",
                   "serialized ROS1 service request exceeds the helper limit");
  }
  std::vector<std::uint8_t> payload(payload_size);
  if (!payload.empty()) {
    const std::size_t written = message.writeToStream(payload.data());
    if (written != payload.size()) {
      permanentError("service_request_serialization_failed",
                     "serialized ROS1 service request length changed");
    }
  }
  return payload;
}

std::uint64_t nextRequestId() {
  static std::atomic<std::uint64_t> next{1u};
  for (;;) {
    const std::uint64_t selected =
        next.fetch_add(1u, std::memory_order_relaxed);
    if (selected != 0u) {
      return selected;
    }
  }
}

}  // namespace

ServiceInvoker::ServiceInvoker(TypeRegistry& types, const JsonCodec& codec,
                               std::uint32_t maximum_in_flight)
    : types_(types),
      codec_(codec),
      maximum_in_flight_(maximum_in_flight),
      in_flight_(std::make_shared<std::atomic<std::uint32_t>>(0)) {
  if (maximum_in_flight_ == 0) {
    permanentError("invalid_configuration",
                   "maximum in-flight service calls must be greater than zero");
  }
}

ServiceCallResult ServiceInvoker::call(const ServiceCallRequest& request) {
  const auto started_at = SteadyClock::now();
  requirePreDispatchAllowed(request);

  const auto description = types_.resolveService(request.service_type);
  auto dynamic_request = types_.createServiceRequest(description);
  auto dynamic_response_template = types_.createServiceResponse(description);
  if (!dynamic_response_template) {
    permanentError("invalid_service_description",
                   "ROS1 service response description is incomplete");
  }
  codec_.decode(request.request, *dynamic_request);
  const std::vector<std::uint8_t> request_payload =
      serializeServiceRequest(*dynamic_request);
  requirePreDispatchAllowed(request);

  if (!description || !validServiceMd5(description->md5)) {
    permanentError("invalid_service_description",
                   "ROS1 service description has an invalid MD5 digest");
  }
  if (request.service.empty() ||
      request.service.size() > kMaximumServiceNameBytes) {
    permanentError("invalid_service_name",
                   "ROS1 service name exceeds the helper protocol limit");
  }

  if (!acquireInFlight(in_flight_, maximum_in_flight_)) {
    resourceExhaustedError("service_call_busy",
                           "maximum concurrent ROS1 service calls reached");
  }
  InFlightPermit permit(in_flight_);

  HelperProcess helper;
  std::string transport_error;
  if (!helper.start(&transport_error)) {
    requirePreDispatchAllowed(request);
    transientError("service_helper_unavailable",
                   "unable to start ROS1 service helper: " + transport_error);
  }

  const auto runtime_deadline = runtimeDeadline(request.deadline_unix_nanos);
  const std::uint64_t request_id = nextRequestId();
  const ServiceHelperFrame request_frame = makeServiceHelperRequest(
      request_id, request.service.size(), description->md5.size(),
      request_payload.size(), request.wait_for_service_ms);

  const auto send_precommit = [&](const void* data, std::size_t size) {
    const PreCommitIoResult result =
        writePreCommit(helper.socketFd(), data, size, request, runtime_deadline,
                       &transport_error);
    if (result != PreCommitIoResult::kComplete) {
      helper.terminateAndReap();
      failPreCommit(result, transport_error);
    }
  };
  send_precommit(&request_frame, sizeof(request_frame));
  send_precommit(request.service.data(), request.service.size());
  send_precommit(description->md5.data(), description->md5.size());
  if (!request_payload.empty()) {
    send_precommit(request_payload.data(), request_payload.size());
  }

  ServiceHelperFrame ready{};
  const PreCommitIoResult ready_result =
      readPreCommit(helper.socketFd(), &ready, sizeof(ready), request,
                    runtime_deadline, &transport_error);
  if (ready_result != PreCommitIoResult::kComplete) {
    helper.terminateAndReap();
    failPreCommit(ready_result, transport_error);
  }
  if (!validateServiceHelperReady(ready, request_id)) {
    helper.terminateAndReap();
    transientError("service_helper_protocol_error",
                   "ROS1 service helper returned an invalid pre-commit frame");
  }
  const auto ready_status = static_cast<ServiceHelperReadyStatus>(ready.status);
  if (ready_status == ServiceHelperReadyStatus::kServiceUnavailable) {
    helper.terminateAndReap();
    transientError("service_unavailable",
                   "ROS1 service '" + request.service +
                       "' was not available before waitForServiceMs elapsed");
  }
  if (ready_status != ServiceHelperReadyStatus::kReady) {
    helper.terminateAndReap();
    transientError("service_helper_protocol_error",
                   "ROS1 service helper rejected the pre-commit request");
  }

  requirePreDispatchAllowed(request);
  const ServiceHelperFrame commit = makeServiceHelperCommit(request_id);
  const PreCommitIoResult commit_result =
      writePreCommit(helper.socketFd(), &commit, sizeof(commit), request,
                     runtime_deadline, &transport_error);
  if (commit_result != PreCommitIoResult::kComplete) {
    helper.terminateAndReap();
    failPreCommit(commit_result, transport_error);
  }
  // Once every commit byte is accepted by the socket, the helper may enter
  // roscpp call(). All later cancellation, timeout, transport, and decode
  // failures therefore preserve an explicitly uncertain terminal.

  try {
    const auto call_deadline =
        SteadyClock::now() + std::chrono::milliseconds(request.call_timeout_ms);
    ServiceHelperFrame response{};
    PostCommitIoResult response_result =
        readPostCommit(helper.socketFd(), &response, sizeof(response), request,
                       runtime_deadline, call_deadline, &transport_error);
    if (response_result != PostCommitIoResult::kComplete) {
      helper.terminateAndReap();
      failPostCommit(response_result, request.service, transport_error);
    }
    if (!validateServiceHelperResponse(response, request_id)) {
      helper.terminateAndReap();
      uncertainError(
          "invalid_service_response",
          "ROS1 service helper returned an invalid post-commit frame");
    }

    std::vector<std::uint8_t> response_payload;
    response_payload.resize(response.payload_bytes);
    if (!response_payload.empty()) {
      response_result = readPostCommit(
          helper.socketFd(), response_payload.data(), response_payload.size(),
          request, runtime_deadline, call_deadline, &transport_error);
      if (response_result != PostCommitIoResult::kComplete) {
        helper.terminateAndReap();
        failPostCommit(response_result, request.service, transport_error);
      }
    }
    helper.terminateAndReap();

    const auto response_status =
        static_cast<ServiceHelperResponseStatus>(response.status);
    if (response_status == ServiceHelperResponseStatus::kCallFailed) {
      uncertainError(
          "service_call_failed",
          "ROS1 service call to '" + request.service + "' failed after commit");
    }
    if (response_status == ServiceHelperResponseStatus::kResponseTooLarge) {
      uncertainError("service_response_too_large",
                     "ROS1 service response exceeded the helper limit after "
                     "commit");
    }
    if (response_status != ServiceHelperResponseStatus::kCompleted) {
      uncertainError("invalid_service_response",
                     "ROS1 service helper failed after commit");
    }

    Json::Value decoded_response;
    std::size_t bytes_read = 0u;
    std::uint8_t empty_payload = 0u;
    const std::uint8_t* response_data =
        response_payload.empty() ? &empty_payload : response_payload.data();
    auto dynamic_response = ros_babel_fish::createMessageFromTemplate(
        description->response->message_template, response_data,
        response_payload.size(), bytes_read);
    if (!dynamic_response || bytes_read != response_payload.size()) {
      uncertainError("invalid_service_response",
                     "ROS1 service response length did not match its type");
    }
    decoded_response = codec_.encode(*dynamic_response);

    ServiceCallResult result;
    result.service = request.service;
    result.service_type = request.service_type;
    result.response = std::move(decoded_response);
    result.duration_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            SteadyClock::now() - started_at)
            .count());
    return result;
  } catch (const Ros1ToolsError& error) {
    if (error.errorClass() == "uncertain") {
      throw;
    }
    uncertainError("service_result_failed",
                   "ROS1 service processing failed after commit: " +
                       std::string(error.what()));
  } catch (const std::exception& exception) {
    uncertainError("service_result_failed",
                   "ROS1 service processing failed after commit: " +
                       std::string(exception.what()));
  } catch (...) {
    uncertainError("service_result_failed",
                   "ROS1 service processing failed after commit");
  }
}

}  // namespace xgc_ros1_tools_adapter
