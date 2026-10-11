#include "xgc2_ros1_robot_adapter/ros1_binding.hpp"
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <ros/master.h>
#include <ros/param.h>
#include <ros/poll_manager.h>
#include <stdexcept>
#include <xmlrpcpp/XmlRpcValue.h>

namespace xgc2_ros1_robot_adapter {
ros::NodeHandle &Ros1Handle(RosNode &node) {
  auto *ros1 = dynamic_cast<Ros1Node *>(&node);
  if (ros1 == nullptr)
    throw std::logic_error("robot factory received a node of another binding");
  return ros1->handle();
}

void Ros1Binding::Init(const ServerOptions &options,
                       const std::string &node_name) {
  node_name_ = node_name;
  // The master is bound once, here: roscpp reads these at ros::init and the
  // process never rebinds to another master.
  setenv("ROS_MASTER_URI", options.ros_master_uri.c_str(), 1);
  if (!options.ros_ip.empty()) {
    setenv("ROS_IP", options.ros_ip.c_str(), 1);
    unsetenv("ROS_HOSTNAME");
  }
  ros::M_string remappings;
  ros::init(remappings, node_name, ros::init_options::NoSigintHandler);
}

void Ros1Binding::Connect() {
  node_.reset(new ros::NodeHandle());
  node_->setCallbackQueue(&queues_[0]);
}

bool Ros1Binding::Running() const { return !ros::isStarted() || ros::ok(); }

// getPid and getUri are the cheapest calls that name a master incarnation: a
// restarted master has another pid even when it listens on the same URI. Both
// run under the bounded master execution, so a silent master fails the probe
// within seconds.
bool Ros1Binding::ProbeMaster(MasterIdentity *identity) {
  XmlRpc::XmlRpcValue request, response, payload;
  request.setSize(1);
  request[0] = "/" + node_name_;
  if (!ros::master::execute("getPid", request, response, payload, false) ||
      payload.getType() != XmlRpc::XmlRpcValue::TypeInt)
    return false;
  const long pid = static_cast<int>(payload);
  if (!ros::master::execute("getUri", request, response, payload, false) ||
      payload.getType() != XmlRpc::XmlRpcValue::TypeString)
    return false;
  identity->pid = pid;
  identity->uri = static_cast<std::string>(payload);
  // roscore/roslaunch publish the id of the master run as /run_id; a bare
  // rosmaster has none. It is read once, with the first successful probe.
  if (!master_run_id_read_) {
    std::string run_id;
    if (ros::param::get("/run_id", run_id))
      master_run_id_ = run_id;
    master_run_id_read_ = true;
  }
  identity->run_id = master_run_id_;
  return true;
}

void Ros1Binding::Shutdown() {
  node_.reset();
  ros::shutdown();
}

void Ros1Binding::PumpLane(std::size_t lane, std::chrono::milliseconds wait) {
  queues_.at(lane).callOne(ros::WallDuration(wait.count() / 1000.0));
}

void Ros1Binding::DisableLanes() {
  for (auto &queue : queues_)
    queue.disable();
}

std::unique_ptr<RosNode> Ros1Binding::CreateNode(std::size_t lane) {
  ros::NodeHandle handle(*node_);
  handle.setCallbackQueue(&queues_.at(lane));
  return std::unique_ptr<RosNode>(new Ros1Node(std::move(handle)));
}

std::unique_ptr<CommandIo> Ros1Binding::CreateCommandIo() {
  return std::unique_ptr<CommandIo>(new AsyncRosServices());
}

std::unique_ptr<MasterBudget>
Ros1Binding::BoundMasterCalls(std::chrono::milliseconds budget) {
  return std::unique_ptr<MasterBudget>(new RosMasterDeadline(budget));
}

// roscpp queues a published message for its transport thread, and a publisher
// destroyed before that thread has sent the message drops it. The thread runs
// its listeners at the start of each pass and sends what is ready during the
// pass. A message published before this call is therefore on its socket once
// the thread has started a second pass after the call.
void Ros1Binding::FlushPublications(std::chrono::milliseconds budget) {
  struct Passes {
    std::mutex mutex;
    std::condition_variable started;
    int count = 0;
  } passes;
  const ros::PollManagerPtr &transport = ros::PollManager::instance();
  const auto listener = transport->addPollThreadListener([&passes] {
    std::lock_guard<std::mutex> lock(passes.mutex);
    ++passes.count;
    passes.started.notify_all();
  });
  transport->getPollSet().signal();
  {
    std::unique_lock<std::mutex> lock(passes.mutex);
    passes.started.wait_for(lock, budget,
                            [&passes] { return passes.count >= 2; });
  }
  // Waits for a listener call in progress, so none runs after this returns.
  transport->removePollThreadListener(listener);
}
} // namespace xgc2_ros1_robot_adapter
