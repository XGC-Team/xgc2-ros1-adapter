#pragma once

#include "xgc/adapter/v1/adapter.pb.h"
#include "xgc/robot/v1/message.pb.h"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace xgc2_ros1_robot_adapter {

// One type server serves one Run: the Run starts it, binds it to one ROS master
// and one node name, and the process ends with the Run. Every field is a
// command-line flag of the same name.
struct ServerOptions {
  std::string socket_path;    // Unix socket the gRPC service listens on
  std::string run_id;         // owning Run; every member must carry it
  std::string node_name;      // ROS node base name; contains the run id
  std::string provider;       // robot type, e.g. xgc2-scout-mini-ros1-adapter
  std::string target_id;      // XRPC target the service is published on
  std::string ros_master_uri; // the one master this process binds to
  std::string ros_ip;         // optional ROS_IP of this host
};

// 1-128 characters of [A-Za-z0-9._:-]: the grammar of every identifier the
// service exchanges (run, target, robot).
bool ValidReferenceId(const std::string &value);

// Parses and validates the command line. Throws std::invalid_argument with a
// message that names the offending flag. `providers` lists the robot types
// the executable can serve.
ServerOptions ParseServerOptions(int argc, char **argv,
                                 const std::vector<std::string> &providers);

// One line per flag, for the message that follows a usage error.
std::string ServerUsage(const char *program);

// The ROS graph name derived from --node-name: characters that are not legal
// in a ROS name become underscores, so a Run identifier (UUID, dashes) can be
// part of the node name.
std::string RosNodeName(const std::string &node_name);

// Process exit status. Anything but kExitOk after a requested stop tells the
// supervisor why the server ended without being asked to.
enum ExitCode : int {
  kExitOk = 0,                // SIGTERM/SIGINT or Shutdown RPC, drained cleanly
  kExitFailure = 1,           // startup failure or internal worker failure
  kExitUsage = 2,             // invalid command line
  kExitMasterChanged = 3,     // the ROS master was replaced
  kExitMasterUnreachable = 4, // the ROS master stayed unreachable
  kExitRosShutdown = 5,       // ROS shut this node down (rosnode kill)
};

// Identity of the ROS master as the master itself reports it. A restarted
// master has another pid even when it listens on the same URI.
struct MasterIdentity {
  std::string uri; // URI fixed by Init
  long pid = 0;    // getPid
  std::string run_id; // parameter /run_id when the master has one
  bool sameMaster(const MasterIdentity &other) const {
    return uri == other.uri && pid == other.pid &&
           (run_id.empty() || other.run_id.empty() || run_id == other.run_id);
  }
};

// Bounds every master call made on the calling thread while it is alive, and
// reports whether one of them did not complete within the budget.
class MasterBudget {
public:
  virtual ~MasterBudget() = default;
  virtual bool failed() const = 0;
};

// The node of the robot middleware handed to a robot factory. The binding
// creates it; only code written for that binding looks inside.
class RosNode {
public:
  virtual ~RosNode() = default;
};

// Command I/O owned by the command scheduler thread; all calls and completions
// run there. Robots start calls on the concrete type of their binding.
class CommandIo {
public:
  virtual ~CommandIo() = default;
  // Completes a call returned by NativeRobot::Execute as cancelled.
  virtual void Cancel(std::uint64_t id) = 0;
  // Runs pending I/O and completions for at most maximum_wait.
  virtual void Pump(std::chrono::milliseconds maximum_wait) = 0;
};

// Everything the server core needs from the robot middleware (ROS1 today).
// The core never includes middleware headers; a ROS2 binding replaces only
// this interface and the robot factories.
class RosBinding {
public:
  virtual ~RosBinding() = default;
  // Process start, main thread: bind the process to options.ros_master_uri
  // and options.ros_ip and initialize the node. Contacts nothing.
  virtual void Init(const ServerOptions &options,
                    const std::string &node_name) = 0;
  // Registration thread: start the node and register it with the master.
  virtual void Connect() = 0;
  // True while the middleware wants to keep running.
  virtual bool Running() const = 0;
  // Reads the identity of the bound master within the master-call bound.
  virtual bool ProbeMaster(MasterIdentity *identity) = 0;
  // Local parameter cache, populated once during Connect and updated by ROS.
  virtual std::string MasterRunId() const = 0;
  // Unregisters the node. Callable once, after every robot is stopped.
  virtual void Shutdown() = 0;

  // Fixed callback lanes; each lane is pumped by exactly one core thread.
  virtual std::size_t Lanes() const = 0;
  virtual void PumpLane(std::size_t lane, std::chrono::milliseconds wait) = 0;
  virtual void DisableLanes() = 0;
  virtual std::unique_ptr<RosNode> CreateNode(std::size_t lane) = 0;

  virtual std::unique_ptr<CommandIo> CreateCommandIo() = 0;
  // A duration bounds the entire scope. nullopt only tracks failures; each
  // native master call retains its independent finite transport bound.
  virtual std::unique_ptr<MasterBudget>
  BoundMasterCalls(
      std::optional<std::chrono::milliseconds> budget) = 0;

  // Returns once everything the robots published so far has left the process,
  // or after the budget. Middleware hands a published message to a transport
  // thread; a publisher destroyed before that thread has sent the message
  // drops it. The core calls this between the final commands of stopped
  // robots and the destruction of the robots.
  virtual void FlushPublications(std::chrono::milliseconds budget) = 0;
};

using CommandCompletion = std::function<void(xgc::adapter::v1::OperationEvent)>;
using TelemetryEmitter = std::function<void(xgc::robot::v1::RobotMessage)>;

// One data object per robot with the original type-specific processors. Called
// on the registration thread (create/Stop) and the command scheduler thread
// (Periodic/Execute); it owns no thread of its own.
class NativeRobot {
public:
  virtual ~NativeRobot() = default;
  // Final zero command if a motion intent is active, and no callback after it
  // returns. The publishers that carried that command stay registered until
  // the robot is destroyed, which the core does after FlushPublications.
  virtual void Stop() = 0;
  virtual void Periodic() = 0;
  virtual std::uint64_t Execute(const xgc::adapter::v1::OperationRequest &,
                                CommandIo &, CommandCompletion) = 0;
};
using RobotFactory = std::function<std::shared_ptr<NativeRobot>(
    RosNode &, const RobotConfig &, TelemetryEmitter, std::string *)>;

// Time budgets of the lifecycle; the defaults are the production values.
struct ServerTimings {
  std::chrono::milliseconds master_probe_interval{1000};
  std::chrono::milliseconds master_unreachable_after{5000};
  std::chrono::milliseconds member_removal{10000};
  std::chrono::milliseconds command_drain{2000};
  std::chrono::milliseconds member_drain{15000};
  std::chrono::milliseconds status_flush{500};
  std::chrono::milliseconds publication_flush{500};
  std::chrono::milliseconds rpc_drain{5000};
};

class RobotServer {
public:
  RobotServer(ServerOptions options, RosBinding &binding, RobotFactory factory,
              ServerTimings timings = {});
  ~RobotServer();
  RobotServer(const RobotServer &) = delete;
  RobotServer &operator=(const RobotServer &) = delete;

  // Serves until the Run ends the server (Shutdown RPC, SIGTERM) or the
  // environment fails (ROS master replaced or unreachable), drains, and
  // returns the process exit status.
  int Run();
  // Asks Run() to drain and return. Safe from a signal handler.
  void RequestStop() noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

xgc::adapter::v1::OperationEvent
CommandError(const xgc::adapter::v1::OperationRequest &,
             xgc::adapter::v1::ErrorClass, const std::string &code,
             const std::string &detail);
} // namespace xgc2_ros1_robot_adapter
