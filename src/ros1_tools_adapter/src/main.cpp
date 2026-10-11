// ROS1 Tools: typed topic publishing and service calls over XRPC http.v1.
//
// A Run starts this process for its ROS master and ends it with the Run. It
// waits for that one master, reports readiness through GET /v1/describe
// and exits, with a status that names the reason, when the master is replaced
// or stays unreachable.
#include <ros/ros.h>
#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/unix.hpp>
#include <xgc2_ros1_robot_adapter/ros1_binding.hpp>
#include <xgc2_ros1_robot_adapter/master_retry.hpp>
#include "xgc_ros1_tools_adapter/error.hpp"
#include "xgc_ros1_tools_adapter/tools.hpp"
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace {
using namespace xgc2::xrpc;
using namespace xgc_ros1_tools_adapter;
namespace adapter = xgc2_ros1_robot_adapter;

// Process exit status, shared with the ROS1 type servers.
constexpr int kExitOk = 0;
constexpr int kExitFailure = 1;
constexpr int kExitUsage = 2;
constexpr int kExitMasterChanged = 3;
constexpr int kExitMasterUnreachable = 4;
constexpr int kExitRosShutdown = 5;

// After binding, an unreachable master ends the process after this budget.
constexpr std::chrono::milliseconds kProbeInterval{1000};
constexpr std::chrono::milliseconds kUnreachableAfter{5000};
constexpr std::uint32_t kMaximumReadyWaitMs = 30000;

std::atomic<bool> stopping{false};
std::condition_variable stop_changed;
std::mutex stop_mutex;
std::atomic<int> exit_status{kExitOk};
std::string stop_reason = "stop signal";

void signalStop(int) { stopping.store(true); }

// Records why the process ends. The first reason wins.
void requestStop(int status, const std::string &reason) {
  {
    std::lock_guard<std::mutex> lock(stop_mutex);
    if (!stopping.exchange(true)) {
      exit_status.store(status);
      stop_reason = reason;
    }
  }
  stop_changed.notify_all();
}

std::string json(const Json::Value &value) {
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return Json::writeString(writer, value);
}

// ROS names allow letters, digits and underscores; a Run identifier may carry
// more, so a node name keeps only those.
std::string rosNameCharacters(const std::string &name) {
  std::string result;
  for (const unsigned char character : name)
    result += std::isalnum(character) || character == '_'
                  ? static_cast<char>(character)
                  : '_';
  return result;
}

std::string rosNodeName(const std::string &name) {
  std::string result = rosNameCharacters(name);
  if (result.empty() || !std::isalpha(static_cast<unsigned char>(result[0])))
    result.insert(result.begin(), 'n');
  return result;
}

bool validIdentifier(const std::string &value) {
  if (value.empty() || value.size() > 128)
    return false;
  for (const unsigned char character : value)
    if (!(std::isalnum(character) || character == '.' || character == '_' ||
          character == ':' || character == '-'))
      return false;
  return true;
}

// Readiness (GET /v1/describe): the process is bound to its ROS master. A
// request with wait_ready_ms is held, without polling, until that is true or
// the budget ends.
class Describe {
public:
  Describe(std::string instance, std::string target, std::string run_id,
           std::string node, const NativeContext &context)
      : instance_(std::move(instance)), target_(std::move(target)),
        run_id_(std::move(run_id)), node_(std::move(node)), context_(context),
        timer_([this] { expireLoop(); }) {}

  ~Describe() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      timer_stopping_ = true;
    }
    changed_.notify_all();
    timer_.join();
  }

  void probe() { ++probe_attempts_; }

  void ready(long master_pid, const std::string &master_run_id) {
    std::vector<HttpReply> released;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = true;
      master_pid_ = master_pid;
      master_run_id_ = master_run_id;
      for (auto &waiter : waiters_)
        released.push_back(std::move(waiter.reply));
      waiters_.clear();
    }
    changed_.notify_all();
    for (auto &reply : released)
      answer(reply);
  }

  // Parses wait_ready_ms from the query of a /v1/describe target.
  static bool parseWait(const std::string &target, std::uint32_t *wait) {
    *wait = 0;
    const auto query = target.find('?');
    if (query == std::string::npos)
      return true;
    const std::string parameter = target.substr(query + 1);
    const std::string key = "wait_ready_ms=";
    if (parameter.compare(0, key.size(), key) != 0)
      return false;
    const std::string digits = parameter.substr(key.size());
    if (digits.empty() || digits.size() > 5 ||
        digits.find_first_not_of("0123456789") != std::string::npos)
      return false;
    *wait = static_cast<std::uint32_t>(std::stoul(digits));
    return *wait <= kMaximumReadyWaitMs;
  }

  void handle(const HttpRequest &request, HttpReply reply) {
    std::uint32_t wait = 0;
    if (!parseWait(request.target, &wait)) {
      reply.complete(http_error(
          400, "invalid_argument",
          "the only query is wait_ready_ms between 0 and 30000"));
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (ready_ || wait == 0) {
      lock.unlock();
      answer(reply);
      return;
    }
    // Answer before the caller's own deadline so it receives the state.
    const auto until = std::min(Clock::now() + std::chrono::milliseconds(wait),
                                request.deadline - std::chrono::milliseconds(250));
    waiters_.push_back({std::move(reply), until});
    changed_.notify_all();
  }

  // Ends the held requests with the current (not ready) state.
  void release() {
    std::vector<HttpReply> released;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto &waiter : waiters_)
        released.push_back(std::move(waiter.reply));
      waiters_.clear();
    }
    changed_.notify_all();
    for (auto &reply : released)
      answer(reply);
  }

private:
  // Wake on a new deadline, readiness, shutdown, or the earliest deadline.
  // HttpServer admission bounds the number of retained replies.
  void expireLoop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!timer_stopping_) {
      if (waiters_.empty()) {
        changed_.wait(lock, [this] { return timer_stopping_ || !waiters_.empty(); });
        continue;
      }
      auto until = waiters_.front().until;
      for (const auto &waiter : waiters_)
        until = std::min(until, waiter.until);
      if (changed_.wait_until(lock, until) != std::cv_status::timeout)
        continue;
      std::vector<HttpReply> expired;
      for (auto waiter = waiters_.begin(); waiter != waiters_.end();) {
        if (Clock::now() >= waiter->until) {
          expired.push_back(std::move(waiter->reply));
          waiter = waiters_.erase(waiter);
        } else {
          ++waiter;
        }
      }
      lock.unlock();
      for (auto &reply : expired)
        answer(reply);
      lock.lock();
    }
  }

  struct Waiter {
    HttpReply reply;
    Clock::time_point until;
  };
  void answer(HttpReply &reply) {
    Json::Value body(Json::objectValue);
    bool ready = false;
    long master_pid = 0;
    std::string master_run_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready = ready_ && !stopping.load();
      master_pid = master_pid_;
      master_run_id = master_run_id_;
    }
    body["service"] = "ros1.tools";
    body["api_version"] = "1";
    body["instance_id"] = instance_;
    body["ready"] = ready;
    Json::Value &facts = body["facts"] = Json::Value(Json::objectValue);
    facts["ros_master_uri"] = context_.ros_master_uri;
    facts["ros_master_run_id"] = master_run_id;
    facts["ros_master_pid"] = static_cast<Json::Int64>(master_pid);
    facts["master_probe_attempts"] = static_cast<Json::UInt64>(probe_attempts_.load());
    facts["master_bindings"] = static_cast<Json::UInt64>(master_pid > 0 ? 1 : 0);
    facts["ros_ip"] = context_.ros_ip;
    facts["ros_hostname"] = context_.ros_hostname;
    facts["ros_node"] = "/" + node_;
    facts["run_id"] = run_id_;
    facts["target_id"] = target_;
    facts["state"] = stopping.load() ? "stopped" : ready ? "ready" : "starting";
    facts["reason"] = !ready && !stopping.load()
                          ? "waiting for ROS master binding"
                          : "";
    reply.complete({200, {{"Content-Type", "application/json"}}, json(body)});
  }

  const std::string instance_, target_, run_id_, node_;
  const NativeContext context_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::atomic<std::uint64_t> probe_attempts_{0};
  bool timer_stopping_ = false;
  bool ready_ = false;
  long master_pid_ = 0;
  std::string master_run_id_;
  std::deque<Waiter> waiters_;
  std::thread timer_;
};

struct Work {
  HttpRequest request;
  HttpReply reply;
};

class Workers {
public:
  explicit Workers(Tools &tools) : tools_(tools) {
    try {
      for (int i = 0; i < 16; ++i)
        threads_.emplace_back([this] { run(); });
    } catch (...) {
      stop();
      throw;
    }
  }
  ~Workers() { stop(); }
  void push(HttpRequest request, HttpReply reply) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || queue_.size() >= 16) {
      reply.complete(
          http_error(503, "resource_exhausted", "ROS work capacity exhausted"));
      return;
    }
    queue_.push_back({std::move(request), std::move(reply)});
    changed_.notify_one();
  }
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    changed_.notify_all();
    for (auto &thread : threads_)
      if (thread.joinable())
        thread.join();
  }

private:
  void run() {
    for (;;) {
      Work work;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock, [this] { return closed_ || !queue_.empty(); });
        if (queue_.empty())
          return;
        work = std::move(queue_.front());
        queue_.pop_front();
      }
      if (work.reply.cancelled() || stopping.load())
        continue;
      try {
        const auto remaining = work.request.deadline - Clock::now();
        const auto deadline =
            std::chrono::system_clock::now().time_since_epoch() + remaining;
        CallContext call{
            std::chrono::duration_cast<std::chrono::nanoseconds>(deadline).count(),
            [&work] { return stopping.load() || work.reply.cancelled(); }};
        auto result = work.request.target == "/api/v1/publish"
                          ? tools_.Publish(work.request.body, call)
                          : tools_.CallService(work.request.body, call);
        work.reply.complete(
            {200, {{"Content-Type", "application/json"}}, json(result)});
      } catch (const Ros1ToolsError &error) {
        Json::Value body(Json::objectValue);
        body["error"]["class"] = error.errorClass();
        body["error"]["code"] = error.code();
        body["error"]["message"] = error.what();
        work.reply.complete(
            {200, {{"Content-Type", "application/json"}}, json(body)});
      } catch (const std::exception &) {
        // A transport/client must never replay a request whose native outcome
        // is unknown.
        work.reply.complete(http_error(
            500, "outcome_unknown",
            "ROS operation did not produce a known terminal result"));
      }
    }
  }
  Tools &tools_;
  std::mutex mutex_;
  std::condition_variable changed_;
  bool closed_ = false;
  std::deque<Work> queue_;
  std::vector<std::thread> threads_;
};

// Waits for the master before starting ROS work, then watches its identity.
// The socket thread remains available throughout startup.
class MasterWatch {
public:
  MasterWatch(adapter::Ros1Binding &binding, const NativeContext &context,
              Describe &describe)
      : binding_(binding), context_(context), describe_(describe),
        thread_([this] { run(); }) {}
  ~MasterWatch() {
    stopping.store(true);
    stop_changed.notify_all();
    if (thread_.joinable())
      thread_.join();
    auto budget = binding_.BoundMasterCalls(std::chrono::milliseconds(3000));
    workers_.reset();
    tools_.reset();
  }
  void push(HttpRequest request, HttpReply reply) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!workers_) {
      reply.complete(http_error(503, "not_ready", "waiting for ROS master binding"));
      return;
    }
    workers_->push(std::move(request), std::move(reply));
  }

private:
  void run() {
    try {
      adapter::MasterIdentity master;
      adapter::MasterRetry retry;
      for (;;) {
        if (stopping.load())
          return;
        {
          auto budget = binding_.BoundMasterCalls(std::chrono::milliseconds(3000));
          describe_.probe();
          if (binding_.ProbeMaster(&master) && !budget->failed()) {
            binding_.Connect();
            if (!budget->failed()) {
              master.run_id = binding_.MasterRunId();
              break;
            }
          }
        }
        const auto delay = retry.failed();
        std::unique_lock<std::mutex> lock(stop_mutex);
        stop_changed.wait_for(lock, delay, [] { return stopping.load(); });
      }
      if (stopping.load())
        return;
      ros::NodeHandle node;
      ros::AsyncSpinner spinner(4);
      spinner.start();
      observed_master_pid_.store(master.pid);
      tools_.reset(new Tools(node, context_, master.pid, observed_master_pid_));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        workers_.reset(new Workers(*tools_));
      }
      describe_.ready(master.pid, master.run_id);
      watch(master);
    } catch (const std::exception &error) {
      requestStop(kExitFailure, error.what());
    }
  }
  void watch(adapter::MasterIdentity master) {
    bool unreachable = false;
    Clock::time_point since;
    adapter::MasterRetry retry;
    while (!stopping.load() && ros::ok()) {
      adapter::MasterIdentity current;
      bool answered;
      {
        auto budget = binding_.BoundMasterCalls(std::chrono::milliseconds(3000));
        describe_.probe();
        answered = binding_.ProbeMaster(&current) && !budget->failed();
      }
      if (answered) {
        unreachable = false;
        retry.reset();
        if (!master.sameMaster(current)) {
          observed_master_pid_.store(0);
          requestStop(kExitMasterChanged, "the ROS master was replaced (pid " +
                          std::to_string(current.pid) + ")");
          return;
        }
        observed_master_pid_.store(current.pid);
        if (master.run_id.empty() && !current.run_id.empty()) {
          master.run_id = current.run_id;
          describe_.ready(master.pid, master.run_id);
        }
      } else {
        observed_master_pid_.store(0);
        if (!unreachable)
          since = Clock::now();
        unreachable = true;
        if (Clock::now() - since >= kUnreachableAfter) {
          requestStop(kExitMasterUnreachable,
                      "the ROS master has not answered for " +
                          std::to_string(kUnreachableAfter.count()) + " ms");
          return;
        }
      }
      const auto delay = answered ? kProbeInterval
                                 : std::min(retry.failed(),
                                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                                since + kUnreachableAfter - Clock::now()));
      std::unique_lock<std::mutex> lock(stop_mutex);
      stop_changed.wait_for(lock, delay,
                            [] { return stopping.load(); });
    }
    if (!stopping.load() && !ros::ok())
      requestStop(kExitRosShutdown, "ROS shut this node down");
  }
  adapter::Ros1Binding &binding_;
  const NativeContext context_;
  Describe &describe_;
  std::mutex mutex_;
  std::atomic<long> observed_master_pid_{0};
  std::unique_ptr<Tools> tools_;
  std::unique_ptr<Workers> workers_;
  std::thread thread_;
};

int usage() {
  std::cerr << "usage: xgc_ros1_tools_adapter_node --socket-path PATH "
               "--target-id ID --run-id ID --node-name NAME --ros-master-uri URI "
               "--ros-ip IP --ros-hostname HOST\n";
  return kExitUsage;
}
} // namespace

int main(int argc, char **argv) {
  std::map<std::string, std::string> args;
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      usage();
      return kExitOk;
    }
    for (int i = 1; i < argc; i += 2)
      if (i + 1 >= argc || !args.emplace(argv[i], argv[i + 1]).second)
        throw std::invalid_argument("invalid or duplicate argument");
    const std::set<std::string> flags{"--socket-path", "--target-id",
                                      "--run-id",      "--node-name",
                                      "--ros-master-uri", "--ros-ip",
                                      "--ros-hostname"};
    for (const auto &entry : args)
      if (!flags.count(entry.first))
        throw std::invalid_argument("unknown argument " + entry.first);
    if (args.size() != flags.size())
      throw std::invalid_argument("every argument is required");
    if (!validIdentifier(args.at("--target-id")) ||
        !validIdentifier(args.at("--run-id")) || args.at("--node-name").empty() ||
        args.at("--node-name").size() > 128)
      throw std::invalid_argument("invalid target, run or node identity");
    // Two Runs on one ROS master must never share a node name: the master
    // would let the newer node displace the older one.
    if (rosNameCharacters(args.at("--node-name"))
            .find(rosNameCharacters(args.at("--run-id"))) == std::string::npos)
      throw std::invalid_argument("--node-name must include the --run-id");
  } catch (const std::exception &error) {
    std::cerr << "ros1.tools: " << error.what() << '\n';
    return usage();
  }

  try {
    Json::Value config(Json::objectValue);
    config["rosMasterUri"] = args.at("--ros-master-uri");
    config["rosIp"] = args.at("--ros-ip");
    config["rosHostname"] = args.at("--ros-hostname");
    const auto context = NativeContext::FromJson(json(config));
    context.ApplyEnvironment();
    std::signal(SIGINT, signalStop);
    std::signal(SIGTERM, signalStop);
    const auto node_name = rosNodeName(args.at("--node-name"));
    HttpLimits limits;
    limits.connections = 32;
    limits.inflight = 16;
    limits.request_bytes = 1 << 20;
    limits.response_bytes = 1 << 20;
    limits.request_timeout = std::chrono::milliseconds(300000);
    std::cerr << "ros1.tools: run=" << args.at("--run-id") << " node=/"
              << node_name << " master=" << context.ros_master_uri << " socket="
              << args.at("--socket-path") << std::endl;
    ros::M_string remappings;
    ros::init(remappings, node_name,
              ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    adapter::ServerOptions options;
    options.ros_master_uri = context.ros_master_uri;
    options.ros_ip = context.ros_ip;
    adapter::Ros1Binding binding;
    binding.Init(options, node_name);
    const auto instance = new_instance_id();
    Describe describe(instance, args.at("--target-id"), args.at("--run-id"),
                      node_name, context);
    std::unique_ptr<MasterWatch> watch;
    HttpServer server(
        UnixOptions{args.at("--socket-path")},
        [&](HttpRequest request, HttpReply reply) {
          if (request.method == "GET" &&
              request.target.compare(0, 12, "/v1/describe") == 0) {
            describe.handle(request, std::move(reply));
            return;
          }
          if (request.method != "POST" ||
              (request.target != "/api/v1/publish" &&
               request.target != "/api/v1/call")) {
            reply.complete(
                http_error(404, "not_found", "unknown ROS1 Tools operation"));
            return;
          }
          if (stopping.load()) {
            reply.complete(http_error(503, "not_ready", "ROS1 Tools is stopping"));
            return;
          }
          watch->push(std::move(request), std::move(reply));
        },
        limits, HttpIdentity{instance, {"/v1/describe"}});
    watch.reset(new MasterWatch(binding, context, describe));
    while (!stopping.load()) {
      server.poll(std::chrono::milliseconds(100));
    }
    stopping.store(true);
    stop_changed.notify_all();
    watch.reset();
    describe.release();
    server.drain_until(Clock::now() + std::chrono::seconds(2));
    {
      auto budget = binding.BoundMasterCalls(std::chrono::milliseconds(3000));
      binding.Shutdown();
    }
    std::cerr << "ros1.tools: exit " << exit_status.load() << " ("
              << stop_reason << ")" << std::endl;
    return exit_status.load();
  } catch (const std::exception &error) {
    std::cerr << "ros1.tools: " << error.what() << '\n';
    if (ros::isInitialized()) {
      adapter::RosMasterDeadline budget(std::chrono::milliseconds(3000));
      ros::shutdown();
    }
    return kExitFailure;
  }
}
