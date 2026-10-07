#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace xgc2_ros1_robot_adapter {

// Narrow TCPROS client for generated MAVROS service request/response bytes.
// All methods, including callbacks, run on the one command event loop. Pump
// multiplexes network waits; it never calls roscpp ServiceClient::call or DNS
// getaddrinfo. Each absolute deadline includes discovery and DNS.
class AsyncRosServices {
public:
  using Clock = std::chrono::steady_clock;
  enum class Outcome { Response, NotSent, Deadline, Cancelled, Unknown };
  struct Result {
    Outcome outcome = Outcome::NotSent;
    std::string response;
    std::string detail;
  };
  struct Request {
    std::string master_uri;
    std::string caller_id;
    std::string service;
    std::string type;
    std::string md5;
    std::string bytes;
    // Internal roscpp master registration bridge; empty for native services.
    std::string master_xml;
    Clock::time_point deadline;
  };
  using Completion = std::function<void(Result)>;

  explicit AsyncRosServices(std::string resolver_servers = {});
  ~AsyncRosServices();
  std::uint64_t Start(Request request, Completion completion);
  void Cancel(std::uint64_t id);
  void Pump(std::chrono::milliseconds maximum_wait);
  std::size_t pending() const;
  AsyncRosServices(const AsyncRosServices &) = delete;
  AsyncRosServices &operator=(const AsyncRosServices &) = delete;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace xgc2_ros1_robot_adapter
