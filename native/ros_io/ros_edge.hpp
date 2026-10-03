#pragma once
// Shared by the clock-source service, generic edge and optional domain edges.
// Link XgcRosIo::Edge once; these declarations never create a second gate/name
// state in a caller DSO. Callback queues remain owned by each edge.
#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include "xgc_clock_source.h"

namespace xgc_ros_edge {

inline constexpr int kGateUnclaimed = -1;

__attribute__((visibility("default"))) std::atomic<int>& output_gate();

__attribute__((visibility("default"))) std::atomic<int>& suppress_backlog();

__attribute__((visibility("default"))) bool output_allowed();

__attribute__((visibility("default"))) bool take_suppress_backlog();

__attribute__((visibility("default"))) void set_output_gate(uint32_t gate);

struct RosInit {
  bool ok{false};
  const char* error{""};
};

__attribute__((visibility("default"))) std::mutex& ros_init_mu();

__attribute__((visibility("default"))) std::string& frozen_node_name();

__attribute__((visibility("default"))) std::string bare_node_name(std::string name);

// Initialize ROS at most once. A later caller must repeat the same bare node
// name; the master and ROS_IP stay those of the process environment.
__attribute__((visibility("default"))) RosInit ensure_ros(const std::string& requested_bare);

}  // namespace xgc_ros_edge
