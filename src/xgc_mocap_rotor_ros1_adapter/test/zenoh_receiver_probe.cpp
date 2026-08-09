#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <set>
#include <string>

#include "xgc_mocap_rotor_ros1_adapter/wire_contract.hpp"
#include "xgc_mocap_rotor_ros1_adapter/zenoh_subscriber.hpp"

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

const std::set<std::string> kRequiredLeaves{
    "flight_state", "forwarder_hb",   "imu",
    "local_pose",   "local_velocity", "power"};

struct ProbeState {
  std::mutex mutex;
  std::condition_variable changed;
  std::set<std::string> received;
  std::string error;
};

int run(const std::string &endpoint, const std::string &robot_id) {
  ProbeState state;
  ZenohSubscriber subscriber([&](std::string key, std::string payload) {
    ParsedWireKey parsed;
    std::string parse_error;
    if (!ParseWireKey(key, &parsed, &parse_error)) {
      std::lock_guard<std::mutex> lock(state.mutex);
      state.error = std::move(parse_error);
      state.changed.notify_all();
      return;
    }
    if (parsed.robot_id != robot_id)
      return;
    const std::string leaf = WireChannelLeaf(parsed.channel);
    std::lock_guard<std::mutex> lock(state.mutex);
    if (payload.empty() ||
        payload.size() > MaximumPayloadBytes(parsed.channel) ||
        payload.find("\"v\":1") == std::string::npos) {
      state.error = "received an empty, oversized, or non-v1 wire payload";
    } else if (state.received.insert(leaf).second) {
      std::cout << "received=" << leaf << std::endl;
    }
    state.changed.notify_all();
  });

  std::string error;
  if (!subscriber.Start(endpoint, &error)) {
    std::cerr << error << std::endl;
    return 2;
  }
  std::cout << "listening=" << endpoint << " robot=" << robot_id << std::endl;

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(30);
  std::unique_lock<std::mutex> lock(state.mutex);
  while (state.error.empty() && state.received != kRequiredLeaves &&
         std::chrono::steady_clock::now() < deadline) {
    state.changed.wait_until(lock, deadline);
  }
  const std::string final_error = state.error;
  const auto final_received = state.received;
  lock.unlock();
  subscriber.Stop();

  if (!final_error.empty()) {
    std::cerr << final_error << std::endl;
    return 3;
  }
  if (final_received != kRequiredLeaves) {
    std::cerr << "timed out before all six Mocap Rotor uplink leaves arrived"
              << std::endl;
    return 4;
  }
  std::cout << "result=ready channels=" << final_received.size() << std::endl;
  return 0;
}

} // namespace
} // namespace xgc_mocap_rotor_ros1_adapter

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: zenoh_receiver_probe tcp/host:port robot-id"
              << std::endl;
    return 2;
  }
  return xgc_mocap_rotor_ros1_adapter::run(argv[1], argv[2]);
}
