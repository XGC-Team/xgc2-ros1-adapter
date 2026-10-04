#include "ros_edge.hpp"

namespace xgc_ros_edge {

std::atomic<int>& output_gate() {
  static std::atomic<int> gate{kGateUnclaimed};
  return gate;
}

std::atomic<int>& suppress_backlog() {
  static std::atomic<int> flag{0};
  return flag;
}

bool output_allowed() {
  const int gate = output_gate().load(std::memory_order_acquire);
  return gate == kGateUnclaimed || gate == static_cast<int>(XGC_CLOCK_GATE_OPEN);
}

bool take_suppress_backlog() { return suppress_backlog().exchange(0, std::memory_order_acq_rel) == 1; }

void set_output_gate(uint32_t gate) {
  if (gate != XGC_CLOCK_GATE_OPEN) suppress_backlog().store(1, std::memory_order_release);
  output_gate().store(static_cast<int>(gate), std::memory_order_release);
}

std::mutex& ros_init_mu() {
  static std::mutex mu;
  return mu;
}

std::mutex& publisher_lifecycle_mutex() {
  static std::mutex mu;
  return mu;
}

std::string& frozen_node_name() {
  static std::string name;
  return name;
}

std::string bare_node_name(std::string name) {
  if (!name.empty() && name.front() == '/') name.erase(name.begin());
  return name;
}

RosInit ensure_ros(const std::string& requested_bare) {
  if (requested_bare.empty() || requested_bare.front() == '/' || requested_bare.find('/') != std::string::npos) {
    return {false, "node_name must be one bare ROS name"};
  }
  std::lock_guard<std::mutex> lock(ros_init_mu());
  try {
    if (!ros::isInitialized()) {
      ros::M_string remappings;
      ros::init(remappings, requested_bare, ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
      frozen_node_name() = requested_bare;
      return {true, ""};
    }
  } catch (...) {
    return {false, "ros::init failed"};
  }
  std::string current = frozen_node_name();
  if (current.empty()) {
    current = bare_node_name(ros::this_node::getName());
    frozen_node_name() = current;
  }
  if (current != requested_bare) return {false, "ROS node name does not match the frozen node"};
  return {true, ""};
}

}  // namespace xgc_ros_edge
