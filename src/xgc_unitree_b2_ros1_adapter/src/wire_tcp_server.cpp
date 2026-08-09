#include "xgc_unitree_b2_ros1_adapter/wire_tcp_server.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <set>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace xgc_unitree_b2_ros1_adapter {
namespace {
constexpr std::uint32_t kMaximumKeyBytes = 512u;
constexpr std::uint32_t kMaximumPayloadBytes = 1024u * 1024u;
}

WireTcpServer::WireTcpServer(std::string host, std::uint16_t port,
                             Handler handler)
    : host_(std::move(host)), port_(port), handler_(std::move(handler)) {}

WireTcpServer::~WireTcpServer() { Stop(); }

bool WireTcpServer::Start(std::string *error) {
  if (running_.load()) return true;
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    if (error) *error = std::string("cannot create TCP socket: ") + std::strerror(errno);
    return false;
  }
  int reuse = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1) {
    if (error) *error = "wire_host must be an IPv4 listen address";
    ::close(listen_fd_); listen_fd_ = -1; return false;
  }
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      ::listen(listen_fd_, 8) != 0) {
    if (error) *error = std::string("cannot listen on ") + host_ + ":" +
                        std::to_string(port_) + ": " + std::strerror(errno);
    ::close(listen_fd_); listen_fd_ = -1; return false;
  }
  stopping_.store(false);
  running_.store(true);
  accept_thread_ = std::thread(&WireTcpServer::acceptLoop, this);
  return true;
}

void WireTcpServer::Stop() {
  if (!running_.exchange(false) && listen_fd_ < 0) return;
  stopping_.store(true);
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  std::vector<int> clients;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients = client_fds_;
  }
  for (const int fd : clients) ::shutdown(fd, SHUT_RDWR);
  if (accept_thread_.joinable()) accept_thread_.join();
  for (auto &thread : client_threads_)
    if (thread.joinable()) thread.join();
  client_threads_.clear();
  std::lock_guard<std::mutex> lock(clients_mutex_);
  client_fds_.clear();
}

void WireTcpServer::acceptLoop() {
  while (!stopping_.load()) {
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) {
      if (errno == EINTR) continue;
      break;
    }
    ++accepted_;
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_fds_.push_back(fd);
    client_threads_.emplace_back(&WireTcpServer::clientLoop, this, fd);
  }
}

bool WireTcpServer::readExact(int socket_fd, void *output, std::size_t size) {
  auto *cursor = static_cast<unsigned char *>(output);
  std::size_t received = 0;
  while (received < size && !stopping_.load()) {
    const ssize_t count = ::recv(socket_fd, cursor + received, size - received, 0);
    if (count == 0) return false;
    if (count < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    received += static_cast<std::size_t>(count);
  }
  return received == size;
}

void WireTcpServer::clientLoop(int socket_fd) {
  while (!stopping_.load()) {
    std::uint32_t network_key_size = 0;
    if (!readExact(socket_fd, &network_key_size, sizeof(network_key_size))) break;
    const std::uint32_t key_size = ntohl(network_key_size);
    if (key_size == 0 || key_size > kMaximumKeyBytes) { ++malformed_; break; }
    std::string key(key_size, '\0');
    if (!readExact(socket_fd, &key[0], key.size())) break;
    std::uint32_t network_payload_size = 0;
    if (!readExact(socket_fd, &network_payload_size, sizeof(network_payload_size))) break;
    const std::uint32_t payload_size = ntohl(network_payload_size);
    if (payload_size == 0 || payload_size > kMaximumPayloadBytes) { ++malformed_; break; }
    std::string payload(payload_size, '\0');
    if (!readExact(socket_fd, &payload[0], payload.size())) break;
    try { handler_(key, payload); } catch (...) { ++malformed_; }
  }
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_fds_.erase(std::remove(client_fds_.begin(), client_fds_.end(), socket_fd),
                      client_fds_.end());
  }
  ::close(socket_fd);
}

bool validWireKey(const std::string &robot_id, const std::string &key,
                  std::string *channel) {
  const std::string prefix = "xgc2/" + robot_id + "/up/";
  if (key.compare(0, prefix.size(), prefix) != 0) return false;
  const std::string value = key.substr(prefix.size());
  static const std::set<std::string> allowed{
      "odom", "joint_states", "power_summary", "driver_status",
      "forwarder_hb", "arm_slave_status", "arm_forwarder_hb"};
  if (allowed.count(value) == 0) return false;
  if (channel) *channel = value;
  return true;
}

}  // namespace xgc_unitree_b2_ros1_adapter
