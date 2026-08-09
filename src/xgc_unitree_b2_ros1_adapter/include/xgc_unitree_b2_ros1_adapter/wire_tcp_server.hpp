#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace xgc_unitree_b2_ros1_adapter {

class WireTcpServer {
 public:
  using Handler = std::function<void(const std::string &, const std::string &)>;
  WireTcpServer(std::string host, std::uint16_t port, Handler handler);
  ~WireTcpServer();
  bool Start(std::string *error);
  void Stop();
  bool running() const noexcept { return running_.load(); }
  std::uint64_t acceptedConnections() const noexcept { return accepted_.load(); }
  std::uint64_t malformedFrames() const noexcept { return malformed_.load(); }

 private:
  void acceptLoop();
  void clientLoop(int socket_fd);
  bool readExact(int socket_fd, void *output, std::size_t size);

  const std::string host_;
  const std::uint16_t port_;
  const Handler handler_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> accepted_{0};
  std::atomic<std::uint64_t> malformed_{0};
  int listen_fd_ = -1;
  std::thread accept_thread_;
  mutable std::mutex clients_mutex_;
  std::vector<int> client_fds_;
  std::vector<std::thread> client_threads_;
};

bool validWireKey(const std::string &robot_id, const std::string &key,
                  std::string *channel);

}  // namespace xgc_unitree_b2_ros1_adapter
