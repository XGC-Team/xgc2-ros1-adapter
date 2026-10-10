#include <dirent.h>
#include <poll.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/service.h>
#include <ros/service_client.h>
#include <ros/service_client_options.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "xgc_ros1_tools_adapter/service_helper_protocol.hpp"

namespace xgc_ros1_tools_adapter {
namespace {

constexpr int kHelperUsageExit = 64;
constexpr int kHelperLifecycleExit = 70;

bool parseFileDescriptor(const char* value, int* descriptor) {
  if (value == nullptr || descriptor == nullptr || *value == '\0') {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed < 0L ||
      parsed > INT_MAX) {
    return false;
  }
  *descriptor = static_cast<int>(parsed);
  return true;
}

bool validHelperSocket(int descriptor) {
  if (descriptor <= STDERR_FILENO) {
    return false;
  }
  int socket_type = 0;
  socklen_t size = sizeof(socket_type);
  return getsockopt(descriptor, SOL_SOCKET, SO_TYPE, &socket_type, &size) ==
             0 &&
         size == sizeof(socket_type) && socket_type == SOCK_STREAM;
}

bool installParentDeathFence(int helper_socket) {
  ucred peer{};
  socklen_t peer_size = sizeof(peer);
  if (getsockopt(helper_socket, SOL_SOCKET, SO_PEERCRED, &peer, &peer_size) !=
          0 ||
      peer_size != sizeof(peer) || peer.pid <= 1 || getppid() != peer.pid) {
    return false;
  }
  if (prctl(PR_SET_PDEATHSIG, SIGKILL) != 0) {
    return false;
  }
  // SO_PEERCRED retains the socketpair creator. Rechecking after prctl closes
  // the race where the creator exits before the kernel death signal is armed,
  // including when a subreaper would otherwise conceal the reparenting.
  return getppid() == peer.pid;
}

bool closeInheritedFileDescriptors(int helper_socket) {
  DIR* directory = opendir("/proc/self/fd");
  if (directory == nullptr) {
    return false;
  }
  const int directory_fd = dirfd(directory);
  if (directory_fd < 0) {
    closedir(directory);
    return false;
  }

  bool complete = true;
  for (;;) {
    errno = 0;
    const dirent* entry = readdir(directory);
    if (entry == nullptr) {
      complete = errno == 0;
      break;
    }
    errno = 0;
    char* end = nullptr;
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
    close(descriptor);
  }
  if (closedir(directory) != 0) {
    complete = false;
  }
  return complete;
}

bool waitForSocket(int socket_fd, short events) {
  pollfd descriptor{};
  descriptor.fd = socket_fd;
  descriptor.events = events;
  for (;;) {
    const int result = poll(&descriptor, 1u, -1);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    return result > 0 &&
           (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) == 0 &&
           (descriptor.revents & events) != 0;
  }
}

bool readExact(int socket_fd, void* output, std::size_t size) {
  auto* cursor = static_cast<std::uint8_t*>(output);
  while (size > 0u) {
    const ssize_t received = recv(socket_fd, cursor, size, MSG_DONTWAIT);
    if (received > 0) {
      cursor += received;
      size -= static_cast<std::size_t>(received);
      continue;
    }
    if (received == 0) {
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      return false;
    }
    if (!waitForSocket(socket_fd, POLLIN)) {
      return false;
    }
  }
  return true;
}

bool writeExact(int socket_fd, const void* input, std::size_t size) {
  const auto* cursor = static_cast<const std::uint8_t*>(input);
  while (size > 0u) {
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
      return false;
    }
    if (!waitForSocket(socket_fd, POLLOUT)) {
      return false;
    }
  }
  return true;
}

bool validAbsoluteServiceName(const std::string& service) {
  std::string error;
  return !service.empty() && service.front() == '/' && service != "/" &&
         ros::names::validate(service, error);
}

ros::SerializedMessage serviceRequest(
    const std::vector<std::uint8_t>& payload) {
  ros::SerializedMessage request;
  request.num_bytes = payload.size() + 4u;
  request.buf.reset(new std::uint8_t[request.num_bytes]);
  ros::serialization::OStream stream(
      request.buf.get(), static_cast<std::uint32_t>(request.num_bytes));
  stream.next(static_cast<std::uint32_t>(payload.size()));
  request.message_start = stream.getData();
  if (!payload.empty()) {
    std::memcpy(request.message_start, payload.data(), payload.size());
  }
  return request;
}

bool responsePayload(const ros::SerializedMessage& response,
                     const std::uint8_t** data, std::size_t* size) {
  if (data == nullptr || size == nullptr) {
    return false;
  }
  if (response.num_bytes == 0u) {
    *data = nullptr;
    *size = 0u;
    return true;
  }
  if (!response.buf || response.message_start == nullptr ||
      response.message_start < response.buf.get()) {
    return false;
  }
  const std::size_t offset =
      static_cast<std::size_t>(response.message_start - response.buf.get());
  if (offset > response.num_bytes) {
    return false;
  }
  *data = response.message_start;
  *size = response.num_bytes - offset;
  return true;
}

bool serviceAvailable(const std::string& service,
                      std::uint32_t wait_for_service_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(wait_for_service_ms);
  for (;;) {
    if (ros::service::exists(service, false)) {
      return true;
    }
    if (!ros::ok() || wait_for_service_ms == 0u ||
        std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

int runHelper(int socket_fd) {
  ServiceHelperFrame request_frame{};
  if (!readExact(socket_fd, &request_frame, sizeof(request_frame)) ||
      !validateServiceHelperRequest(request_frame)) {
    return 0;
  }

  std::string service(request_frame.service_name_bytes, '\0');
  std::string md5(request_frame.service_md5_bytes, '\0');
  std::vector<std::uint8_t> request_payload(request_frame.payload_bytes);
  if (!readExact(socket_fd, service.data(), service.size()) ||
      !readExact(socket_fd, md5.data(), md5.size()) ||
      (!request_payload.empty() &&
       !readExact(socket_fd, request_payload.data(), request_payload.size()))) {
    return 0;
  }

  if (!validAbsoluteServiceName(service) || !validServiceMd5(md5)) {
    const ServiceHelperFrame response = makeServiceHelperReady(
        request_frame.request_id, ServiceHelperReadyStatus::kProtocolError);
    writeExact(socket_fd, &response, sizeof(response));
    return 0;
  }

  ros::NodeHandle node_handle;
  if (!serviceAvailable(service, request_frame.wait_for_service_ms)) {
    const ServiceHelperFrame response =
        makeServiceHelperReady(request_frame.request_id,
                               ServiceHelperReadyStatus::kServiceUnavailable);
    writeExact(socket_fd, &response, sizeof(response));
    return 0;
  }

  ros::SerializedMessage serialized_request;
  ros::ServiceClient client;
  try {
    serialized_request = serviceRequest(request_payload);
    ros::ServiceClientOptions options(service, md5, false, ros::M_string{});
    client = node_handle.serviceClient(options);
  } catch (...) {
    const ServiceHelperFrame response = makeServiceHelperReady(
        request_frame.request_id, ServiceHelperReadyStatus::kProtocolError);
    writeExact(socket_fd, &response, sizeof(response));
    return 0;
  }

  const ServiceHelperFrame ready = makeServiceHelperReady(
      request_frame.request_id, ServiceHelperReadyStatus::kReady);
  if (!writeExact(socket_fd, &ready, sizeof(ready))) {
    return 0;
  }

  // This is the native dispatch fence. The helper cannot enter roscpp call()
  // until the parent has sent one complete, matching commit frame.
  ServiceHelperFrame commit{};
  if (!readExact(socket_fd, &commit, sizeof(commit)) ||
      !validateServiceHelperCommit(commit, request_frame.request_id)) {
    return 0;
  }

  ros::SerializedMessage serialized_response;
  bool called = false;
  try {
    called = client.call(serialized_request, serialized_response, md5);
  } catch (...) {
    called = false;
  }
  if (!called) {
    const ServiceHelperFrame response = makeServiceHelperResponse(
        request_frame.request_id, ServiceHelperResponseStatus::kCallFailed);
    writeExact(socket_fd, &response, sizeof(response));
    return 0;
  }

  const std::uint8_t* response_data = nullptr;
  std::size_t response_size = 0u;
  if (!responsePayload(serialized_response, &response_data, &response_size)) {
    const ServiceHelperFrame response = makeServiceHelperResponse(
        request_frame.request_id, ServiceHelperResponseStatus::kProtocolError);
    writeExact(socket_fd, &response, sizeof(response));
    return 0;
  }
  if (response_size > kMaximumServicePayloadBytes) {
    const ServiceHelperFrame response = makeServiceHelperResponse(
        request_frame.request_id,
        ServiceHelperResponseStatus::kResponseTooLarge);
    writeExact(socket_fd, &response, sizeof(response));
    return 0;
  }

  const ServiceHelperFrame response = makeServiceHelperResponse(
      request_frame.request_id, ServiceHelperResponseStatus::kCompleted,
      response_size);
  if (!writeExact(socket_fd, &response, sizeof(response)) ||
      (response_size > 0u &&
       !writeExact(socket_fd, response_data, response_size))) {
    return 0;
  }
  return 0;
}

}  // namespace
}  // namespace xgc_ros1_tools_adapter

int main(int argc, char** argv) {
  if (argc != 2) {
    return xgc_ros1_tools_adapter::kHelperUsageExit;
  }
  int socket_fd = -1;
  if (!xgc_ros1_tools_adapter::parseFileDescriptor(argv[1], &socket_fd) ||
      !xgc_ros1_tools_adapter::validHelperSocket(socket_fd)) {
    return xgc_ros1_tools_adapter::kHelperUsageExit;
  }
  if (!xgc_ros1_tools_adapter::installParentDeathFence(socket_fd) ||
      !xgc_ros1_tools_adapter::closeInheritedFileDescriptors(socket_fd)) {
    return xgc_ros1_tools_adapter::kHelperLifecycleExit;
  }

  ros::init(argc, argv, "xgc_ros1_tools_service_helper",
            ros::init_options::NoSigintHandler |
                ros::init_options::AnonymousName | ros::init_options::NoRosout);
  return xgc_ros1_tools_adapter::runHelper(socket_fd);
}
