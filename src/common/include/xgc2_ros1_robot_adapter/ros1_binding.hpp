#pragma once

#include "xgc2_ros1_robot_adapter/async_ros_services.hpp"
#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include <array>
#include <memory>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <string>
#include <vector>

// The ROS1 side of the RosBinding seam: everything the type server core needs
// from roscpp, and nothing else in the server uses it.
namespace xgc2_ros1_robot_adapter {

// Bounds every ROS master call made by the calling thread while it is alive,
// including roscpp's own registrations. A duration bounds the whole scope;
// nullopt tracks failures without an additional scope-wide deadline.
// A lost reply stops further forward calls in the scope; unregister and
// unsubscribe calls retain their remaining per-call/scope budget for cleanup.
class RosMasterDeadline final : public MasterBudget {
public:
  explicit RosMasterDeadline(
      std::optional<std::chrono::milliseconds> budget);
  ~RosMasterDeadline() override;
  bool failed() const override;

private:
  std::chrono::steady_clock::time_point previous_;
  bool previous_failed_;
  bool previous_active_;
};

// The node a ROS1 robot factory receives: a handle whose callbacks run on one
// of the server's fixed callback lanes.
class Ros1Node final : public RosNode {
public:
  explicit Ros1Node(ros::NodeHandle handle) : handle_(std::move(handle)) {}
  ros::NodeHandle &handle() { return handle_; }

private:
  ros::NodeHandle handle_;
};
// The ROS1 handle behind a factory's node. Throws std::logic_error when the
// node was not created by Ros1Binding.
ros::NodeHandle &Ros1Handle(RosNode &node);

class Ros1Binding final : public RosBinding {
public:
  void Init(const ServerOptions &options, const std::string &node_name) override;
  void Connect() override;
  bool Running() const override;
  bool ProbeMaster(MasterIdentity *identity) override;
  std::string MasterRunId() const override;
  void Shutdown() override;
  std::size_t Lanes() const override { return queues_.size(); }
  void PumpLane(std::size_t lane, std::chrono::milliseconds wait) override;
  void DisableLanes() override;
  std::unique_ptr<RosNode> CreateNode(std::size_t lane) override;
  std::unique_ptr<CommandIo> CreateCommandIo() override;
  std::unique_ptr<MasterBudget>
  BoundMasterCalls(
      std::optional<std::chrono::milliseconds> budget) override;
  void FlushPublications(std::chrono::milliseconds budget) override;

private:
  std::array<ros::CallbackQueue, 2> queues_;
  std::unique_ptr<ros::NodeHandle> node_;
  std::string node_name_;
  bool run_id_cache_ready_ = false;
};

// A robot type served by one ROS1 executable.
struct Ros1Provider {
  std::string provider;
  RobotFactory factory;
};

// main() of every ROS1 type server: parses the command line, serves until the
// Run ends the process, and returns the exit status (see ExitCode).
int RunRos1TypeServer(int argc, char **argv,
                      const std::vector<Ros1Provider> &providers);

} // namespace xgc2_ros1_robot_adapter
