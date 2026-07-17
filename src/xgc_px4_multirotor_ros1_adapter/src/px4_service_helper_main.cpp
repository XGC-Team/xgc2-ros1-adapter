#include "xgc_px4_multirotor_ros1_adapter/px4_service_protocol.hpp"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <dirent.h>
#include <string>

#include <poll.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/names.h>
#include <ros/ros.h>

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

constexpr std::uint16_t kAutopilotRebootCommand = 246u;
constexpr float kNormalAutopilotReboot = 1.0F;
constexpr int kHelperUsageExit = 64;
constexpr int kHelperLifecycleExit = 70;

static_assert(mavros_msgs::CommandCode::PREFLIGHT_REBOOT_SHUTDOWN ==
                  kAutopilotRebootCommand,
              "MAVROS reboot command code changed");

bool parseFileDescriptor(const char *value, int *descriptor) {
  if (value == nullptr || descriptor == nullptr || *value == '\0')
    return false;
  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed < 0L ||
      parsed > INT_MAX) {
    return false;
  }
  *descriptor = static_cast<int>(parsed);
  return true;
}

bool installParentDeathFence(int helper_socket) {
  ucred peer{};
  socklen_t peer_size = sizeof(peer);
  if (getsockopt(helper_socket, SOL_SOCKET, SO_PEERCRED, &peer, &peer_size) !=
          0 ||
      peer_size != sizeof(peer) || peer.pid <= 1 || getppid() != peer.pid) {
    return false;
  }
  if (prctl(PR_SET_PDEATHSIG, SIGKILL) != 0)
    return false;
  // SO_PEERCRED retains the PID that created the socket pair, so this catches
  // reparenting even when a process supervisor is a child subreaper. The parent
  // may also exit between the first check and prctl(); after prctl() succeeds,
  // any later exit is fenced by the kernel.
  return getppid() == peer.pid;
}

bool closeInheritedFileDescriptors(int helper_socket) {
  DIR *directory = opendir("/proc/self/fd");
  if (directory == nullptr)
    return false;
  const int directory_fd = dirfd(directory);
  if (directory_fd < 0) {
    closedir(directory);
    return false;
  }

  bool complete = true;
  for (;;) {
    errno = 0;
    const dirent *entry = readdir(directory);
    if (entry == nullptr) {
      complete = errno == 0;
      break;
    }

    errno = 0;
    char *end = nullptr;
    const long parsed = std::strtol(entry->d_name, &end, 10);
    if (errno != 0 || end == entry->d_name || *end != '\0' || parsed < 0L ||
        parsed > INT_MAX) {
      continue;
    }
    const int descriptor = static_cast<int>(parsed);
    if (descriptor == STDIN_FILENO || descriptor == STDOUT_FILENO ||
        descriptor == STDERR_FILENO || descriptor == helper_socket ||
        descriptor == directory_fd) {
      continue;
    }
    // On Linux close() releases the descriptor even when interrupted. No other
    // thread exists before ros::init(), so descriptors cannot be concurrently
    // recycled while this directory is scanned.
    close(descriptor);
  }

  if (closedir(directory) != 0)
    complete = false;
  return complete;
}

bool validHelperSocket(int descriptor) {
  if (descriptor <= STDERR_FILENO)
    return false;
  int socket_type = 0;
  socklen_t size = sizeof(socket_type);
  return getsockopt(descriptor, SOL_SOCKET, SO_TYPE, &socket_type, &size) ==
             0 &&
         size == sizeof(socket_type) && socket_type == SOCK_SEQPACKET;
}

bool sendResponse(int socket_fd, const Px4ServiceResponseFrame &response) {
  for (;;) {
    const ssize_t sent =
        send(socket_fd, &response, sizeof(response), MSG_NOSIGNAL);
    if (sent == static_cast<ssize_t>(sizeof(response)))
      return true;
    if (sent >= 0)
      return false;
    if (errno == EINTR)
      continue;
    if (errno != EAGAIN && errno != EWOULDBLOCK)
      return false;

    pollfd descriptor{};
    descriptor.fd = socket_fd;
    descriptor.events = POLLOUT;
    int poll_result;
    do {
      poll_result = poll(&descriptor, 1u, -1);
    } while (poll_result < 0 && errno == EINTR);
    if (poll_result <= 0 ||
        (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return false;
    }
  }
}

bool receiveRequest(int socket_fd, Px4ServiceRequestFrame *request) {
  for (;;) {
    const ssize_t received =
        recv(socket_fd, request, sizeof(*request), MSG_DONTWAIT | MSG_TRUNC);
    if (received == static_cast<ssize_t>(sizeof(*request)))
      return true;
    if (received >= 0)
      return false;
    if (errno == EINTR)
      continue;
    if (errno != EAGAIN && errno != EWOULDBLOCK)
      return false;

    pollfd descriptor{};
    descriptor.fd = socket_fd;
    descriptor.events = POLLIN;
    int poll_result;
    do {
      poll_result = poll(&descriptor, 1u, -1);
    } while (poll_result < 0 && errno == EINTR);
    if (poll_result <= 0 ||
        (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return false;
    }
  }
}

bool validEndpoint(const std::string &value) {
  std::string error;
  return !value.empty() && value != "/" && value.front() == '/' &&
         value.back() != '/' && value.find("//") == std::string::npos &&
         ros::names::validate(value, error);
}

int runHelper(int socket_fd, const std::string &arm_endpoint,
              const std::string &mode_endpoint,
              const std::string &reboot_endpoint) {
  ros::NodeHandle node_handle;
  ros::ServiceClient arm_client =
      node_handle.serviceClient<mavros_msgs::CommandBool>(arm_endpoint, true);
  ros::ServiceClient mode_client =
      node_handle.serviceClient<mavros_msgs::SetMode>(mode_endpoint, true);
  ros::ServiceClient command_client =
      node_handle.serviceClient<mavros_msgs::CommandLong>(reboot_endpoint,
                                                          true);

  for (;;) {
    Px4ServiceRequestFrame request{};
    if (!receiveRequest(socket_fd, &request))
      return 0;
    if (!validatePx4ServiceRequest(request)) {
      if (!sendResponse(
              socket_fd,
              makePx4ServiceResponse(
                  request, Px4ServiceResponseStatus::kProtocolError))) {
        return 0;
      }
      continue;
    }

    Px4ServiceResponseFrame response{};
    const auto operation = static_cast<Px4ServiceOperation>(request.operation);
    switch (operation) {
    case Px4ServiceOperation::kSetArmed: {
      mavros_msgs::CommandBool command;
      command.request.value = request.armed != 0u;
      if (!arm_client.call(command)) {
        response = makePx4ServiceResponse(
            request, Px4ServiceResponseStatus::kCallFailed);
      } else {
        response = makePx4ServiceResponse(
            request, Px4ServiceResponseStatus::kCompleted,
            command.response.success, true, command.response.result);
      }
      break;
    }
    case Px4ServiceOperation::kSetMode: {
      mavros_msgs::SetMode command;
      command.request.base_mode = 0u;
      command.request.custom_mode = px4ServiceRequestMode(request);
      if (!mode_client.call(command)) {
        response = makePx4ServiceResponse(
            request, Px4ServiceResponseStatus::kCallFailed);
      } else {
        response = makePx4ServiceResponse(request,
                                          Px4ServiceResponseStatus::kCompleted,
                                          command.response.mode_sent);
      }
      break;
    }
    case Px4ServiceOperation::kRebootAutopilot: {
      mavros_msgs::CommandLong command;
      command.request.broadcast = false;
      command.request.command = kAutopilotRebootCommand;
      command.request.confirmation = 0u;
      command.request.param1 = kNormalAutopilotReboot;
      if (!command_client.call(command)) {
        response = makePx4ServiceResponse(
            request, Px4ServiceResponseStatus::kCallFailed);
      } else {
        response = makePx4ServiceResponse(
            request, Px4ServiceResponseStatus::kCompleted,
            command.response.success, true, command.response.result);
      }
      break;
    }
    }
    if (!sendResponse(socket_fd, response))
      return 0;
  }
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  if (argc != 5) {
    return xgc_px4_multirotor_ros1_adapter::kHelperUsageExit;
  }
  int socket_fd = -1;
  if (!xgc_px4_multirotor_ros1_adapter::parseFileDescriptor(argv[1],
                                                            &socket_fd) ||
      !xgc_px4_multirotor_ros1_adapter::validHelperSocket(socket_fd))
    return xgc_px4_multirotor_ros1_adapter::kHelperUsageExit;
  if (!xgc_px4_multirotor_ros1_adapter::installParentDeathFence(socket_fd) ||
      !xgc_px4_multirotor_ros1_adapter::closeInheritedFileDescriptors(
          socket_fd))
    return xgc_px4_multirotor_ros1_adapter::kHelperLifecycleExit;

  const std::string arm_endpoint(argv[2]);
  const std::string mode_endpoint(argv[3]);
  const std::string reboot_endpoint(argv[4]);
  if (!xgc_px4_multirotor_ros1_adapter::validEndpoint(arm_endpoint) ||
      !xgc_px4_multirotor_ros1_adapter::validEndpoint(mode_endpoint) ||
      !xgc_px4_multirotor_ros1_adapter::validEndpoint(reboot_endpoint) ||
      arm_endpoint == mode_endpoint || arm_endpoint == reboot_endpoint ||
      mode_endpoint == reboot_endpoint) {
    return xgc_px4_multirotor_ros1_adapter::kHelperUsageExit;
  }

  ros::init(argc, argv, "xgc_px4_service_helper",
            ros::init_options::NoSigintHandler |
                ros::init_options::AnonymousName | ros::init_options::NoRosout);
  return xgc_px4_multirotor_ros1_adapter::runHelper(
      socket_fd, arm_endpoint, mode_endpoint, reboot_endpoint);
}
