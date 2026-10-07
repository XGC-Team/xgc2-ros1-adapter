#pragma once

#include "xgc/robot/v1/server.grpc.pb.h"
#include "xgc2_ros1_robot_adapter/async_ros_services.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include <functional>
#include <memory>
#include <ros/ros.h>

namespace xgc2_ros1_robot_adapter {
// Bound a complete registration/cleanup sequence, including roscpp's master
// calls. No detached synchronous call survives this scope's deadline.
class RosMasterDeadline {
public:
  explicit RosMasterDeadline(std::chrono::milliseconds budget);
  ~RosMasterDeadline();
  bool failed() const;
private:
  std::chrono::steady_clock::time_point previous_;
  bool previous_failed_;
};

using CommandCompletion = std::function<void(xgc::adapter::v1::OperationEvent)>;
using TelemetryEmitter = std::function<void(xgc::robot::v1::RobotMessage)>;

// One data object per native ROS resource, with the original type-specific
// processors. Called on the registration thread (Create/Stop), or the shared
// command scheduler (Periodic/Execute); it owns no thread or executor.
class NativeRobot {
public:
  virtual ~NativeRobot() = default;
  virtual void Stop() = 0;
  virtual void Periodic(const ros::WallTime &) = 0;
  virtual std::uint64_t Execute(const xgc::adapter::v1::OperationRequest &,
                                AsyncRosServices &, CommandCompletion) = 0;
};
using RobotFactory = std::function<std::shared_ptr<NativeRobot>(
    ros::NodeHandle, const RobotConfig &, TelemetryEmitter, std::string *)>;

// Main thread owns gRPC's one CQ, two fixed ROS callback threads, one command
// event loop, and one registration/cleanup thread: five application threads.
int RunRobotServer(int argc, char **argv, const std::string &node_name,
                   const std::string &provider, RobotFactory factory);

xgc::adapter::v1::OperationEvent CommandError(
    const xgc::adapter::v1::OperationRequest &, xgc::adapter::v1::ErrorClass,
    const std::string &code, const std::string &detail);
} // namespace xgc2_ros1_robot_adapter
