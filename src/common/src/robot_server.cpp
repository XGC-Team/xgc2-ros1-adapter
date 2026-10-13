#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include "xgc2_ros1_robot_adapter/master_retry.hpp"
#include "xgc/robot/v1/server.grpc.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc2/xrpc/grpc.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <exception>
#include <future>
#include <google/protobuf/util/message_differencer.h>
#include <grpcpp/grpcpp.h>
#include <iostream>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <poll.h>
#include <set>
#include <sys/eventfd.h>
#include <thread>
#include <unistd.h>

namespace xgc2_ros1_robot_adapter {
namespace {
namespace wire = xgc::robot::v1;
namespace operation = xgc::adapter::v1;
using Steady = std::chrono::steady_clock;
using Milliseconds = std::chrono::milliseconds;
constexpr std::size_t kMaximumRobots = 256;
constexpr std::size_t kMaximumBatchMembers = 256, kMaximumCommandsPerRobot = 32;
constexpr std::size_t kMaximumRegistrationJobs = 32, kMaximumCommandJobs = 32;
constexpr std::size_t kMaximumChannelsPerRobot = 32;
constexpr std::size_t kMaximumNativeMessageBytes = 65536;
constexpr std::size_t kMaximumStatusSubscribers = 4, kMaximumBlockingCalls = 24;
constexpr std::size_t kMaximumReadinessWaits = 4;
constexpr std::uint32_t kMaximumReadyWaitMs = 30000;
// A held Describe answers this long before the caller's deadline so the
// caller receives the current (not ready) state instead of a timeout.
constexpr Milliseconds kReadyAnswerMargin{250};

struct BoundedWaiter {
  std::atomic<std::size_t> &count;
  bool accepted;
  BoundedWaiter(std::atomic<std::size_t> &value, std::size_t maximum)
      : count(value), accepted(count.fetch_add(1) < maximum) {
    if (!accepted)
      count.fetch_sub(1);
  }
  ~BoundedWaiter() {
    if (accepted)
      count.fetch_sub(1);
  }
};
bool validIdentity(const wire::MemberIdentity &id) {
  return ValidRobotIdentity(id.robot_id()) &&
         ValidReferenceId(id.target_id()) &&
         ValidReferenceId(id.run_id()) && id.connection_epoch();
}
RobotConfig configFrom(const wire::RobotResource &wire) {
  RobotConfig result;
  result.robot_id = wire.robot_id();
  result.profile_id = wire.profile_id();
  result.profile_digest = wire.profile_digest();
  result.parameters.insert(wire.parameters().begin(), wire.parameters().end());
  for (const auto &channel : wire.channels())
    result.channels.push_back({channel.channel_id(), channel.enabled()});
  return result;
}
// Native methods use the shared fixed gRPC worker pool. A retained domain
// permit survives caller cancellation until the admitted work really finishes.
template <class Request, class Response> struct Unary {
  Request request;
  Response response;
  std::atomic<bool> cancelled{false};
  xgc2::xrpc::GrpcWorkPermit work;
  std::mutex mutex;
  std::condition_variable changed;
  bool finished = false;
  grpc::Status status;
  void finish(Response result, grpc::Status result_status = grpc::Status::OK) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (finished)
        return;
      response = std::move(result);
      status = std::move(result_status);
      finished = true;
    }
    changed.notify_all();
  }
};
using ExecuteCall = Unary<wire::ExecuteRequest, wire::ExecuteResponse>;
void finishExecute(const std::shared_ptr<ExecuteCall> &call,
                   operation::OperationEvent result) {
  wire::ExecuteResponse response;
  *response.mutable_event() = std::move(result);
  call->finish(std::move(response));
}
void log(const std::string &line) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  std::cerr << "robot server: " << line << std::endl;
}
} // namespace

class RobotServer::Impl {
  enum class Phase { Starting, Ready, Draining, Drained, Stopped };
  enum class StopReason {
    None,
    Signal,
    ShutdownRpc,
    MasterChanged,
    MasterUnreachable,
    RosShutdown,
    Failure
  };
  // One robot of this Run. Registered by ApplyMembers, ended by RemoveMembers
  // or the drain; owns the type-specific native object.
  struct Member {
    wire::MemberIdentity identity;
    wire::RobotResource configuration;
    std::shared_ptr<NativeRobot> native;
    std::mutex state_mutex;
    std::map<std::string, std::pair<std::uint64_t, wire::RobotMessage>> latest;
    std::uint64_t state_version = 0;
    std::size_t lane = 0;
    std::atomic<bool> closing{true};
    // Owned exclusively by the command scheduler; member changes are posted
    // to it.
    std::deque<std::shared_ptr<ExecuteCall>> commands;
    std::atomic<bool> in_flight{false};
    bool uncertain = false;
    std::uint64_t native_call = 0;
    std::shared_ptr<ExecuteCall> active_call;
    std::mutex native_mutex; // only short non-network native access
  };
  struct Stream {
    wire::SubscribeStatusResponse sending;
    std::map<std::string, std::map<std::string, std::uint64_t>> seen;
    std::map<std::string, wire::MemberIdentity> identities;
  };
  struct Job {
    std::function<void()> run;
    std::function<void(bool)> abort;
  };
  class Service final : public wire::RobotAdapterServerService::Service {
  public:
    explicit Service(Impl &owner) : owner_(owner) {}
    grpc::Status Describe(grpc::ServerContext *context,
                          const wire::DescribeRequest *request,
                          wire::DescribeResponse *response) override {
      auto scope = owner_.admission_.begin(*context, false, true);
      if (!scope)
        return scope.status();
      if (request->wait_ready_ms() > kMaximumReadyWaitMs)
        return scope.application_error(
            grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                         "wait_ready_ms must be at most 30000"));
      if (request->wait_ready_ms()) {
        BoundedWaiter waiter(owner_.readiness_waits_, kMaximumReadinessWaits);
        if (!waiter.accepted)
          return scope.application_error(
              grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                           "readiness wait capacity exhausted"));
        owner_.awaitReady(scope, Milliseconds(request->wait_ready_ms()));
      }
      owner_.describe(response);
      return grpc::Status::OK;
    }
    grpc::Status ApplyMembers(grpc::ServerContext *context,
                              const wire::ApplyMembersRequest *request,
                              wire::ApplyMembersResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      if (const auto refusal = owner_.requireReady(scope))
        return *refusal;
      if (request->members_size() > static_cast<int>(kMaximumBatchMembers))
        return scope.application_error(
            grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                         "member batch exceeds 256"));
      return owner_.invoke(
          std::move(scope), *request, response, [this](auto call) {
            return owner_.postRegistration(
                {[this, call] {
                   call->finish(owner_.apply(call->request, call->work));
                 },
                 [call](bool started) {
                   call->finish(
                       {},
                       grpc::Status(
                           started ? grpc::StatusCode::UNKNOWN
                                   : grpc::StatusCode::CANCELLED,
                           started
                               ? "native registration interrupted; reconcile "
                                 "outcome"
                               : "registration did not start: server stopped"));
                 }});
          });
    }
    grpc::Status RemoveMembers(grpc::ServerContext *context,
                               const wire::RemoveMembersRequest *request,
                               wire::RemoveMembersResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      if (const auto refusal = owner_.requireStarted(scope))
        return *refusal;
      if (request->members_size() > static_cast<int>(kMaximumBatchMembers))
        return scope.application_error(
            grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                         "member batch exceeds 256"));
      return owner_.invoke(
          std::move(scope), *request, response, [this](auto call) {
            return owner_.postRegistration(
                {[this, call] {
                   call->finish(owner_.remove(call->request, call->work));
                 },
                 [call](bool started) {
                   call->finish(
                       {},
                       grpc::Status(
                           started ? grpc::StatusCode::UNKNOWN
                                   : grpc::StatusCode::CANCELLED,
                           started
                               ? "native removal interrupted; reconcile outcome"
                               : "removal did not start: server stopped"));
                 }});
          });
    }
    grpc::Status Execute(grpc::ServerContext *context,
                         const wire::ExecuteRequest *request,
                         wire::ExecuteResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      if (const auto refusal = owner_.requireReady(scope))
        return *refusal;
      return owner_.invoke(
          std::move(scope), *request, response, [this](auto call) {
            return owner_.postCommand(
                {[this, call] { owner_.enqueue(call); },
                 [call](bool started) {
                   finishExecute(
                       call,
                       CommandError(call->request.operation(),
                                    started ? operation::ERROR_CLASS_UNCERTAIN
                                            : operation::ERROR_CLASS_CANCELLED,
                                    started ? "native-result-unknown"
                                            : "server-stopped",
                                    started ? "command admission interrupted; "
                                              "reconcile outcome"
                                            : "not executed: server shutdown"));
                 }});
          });
    }
    grpc::Status SubscribeStatus(
        grpc::ServerContext *context, const wire::SubscribeStatusRequest *,
        grpc::ServerWriter<wire::SubscribeStatusResponse> *writer) override {
      auto scope = owner_.admission_.begin_stream(*context, *writer);
      if (!scope)
        return scope.status();
      BoundedWaiter waiter(owner_.status_subscribers_,
                           kMaximumStatusSubscribers);
      if (!waiter.accepted)
        return scope.application_error(
            grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                         "status subscriber capacity exhausted"));
      return owner_.subscribe(writer, scope);
    }
    grpc::Status Health(grpc::ServerContext *context,
                        const wire::HealthRequest *,
                        wire::HealthResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      response->set_serving(owner_.phase() == Phase::Ready);
      response->set_instance_id(owner_.reference_.instance_id());
      {
        std::lock_guard<std::mutex> lock(owner_.table_mutex_);
        response->set_robots(owner_.members_.size());
        response->set_members(owner_.members_.size());
      }
      return grpc::Status::OK;
    }
    // Ends this server: stops new work, finishes or cancels commands, sends
    // the final zero commands, removes the members, and answers once that is
    // done. The process exits right after the answer.
    grpc::Status Shutdown(grpc::ServerContext *context,
                          const wire::ShutdownRequest *,
                          wire::ShutdownResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      owner_.requestStop(StopReason::ShutdownRpc);
      if (!owner_.awaitDrained(scope))
        return scope.application_error(grpc::Status(
            grpc::StatusCode::DEADLINE_EXCEEDED,
            "the drain continues; the server exits when it is done"));
      response->set_serving(false);
      return grpc::Status::OK;
    }

  private:
    Impl &owner_;
  };

public:
  Impl(ServerOptions options, RosBinding &binding, RobotFactory factory,
       ServerTimings timings)
      : options_(std::move(options)), binding_(binding),
        factory_(std::move(factory)), timings_(timings),
        admission_(xgc2::xrpc::new_instance_id(), grpcLimits()),
        service_(*this) {
    reference_.set_target_id(options_.target_id);
    reference_.set_service("xgc.robot.v1.RobotAdapterServerService");
    reference_.set_api_version("v1");
    reference_.set_instance_id(admission_.instance_id());
    reference_.set_profile("grpc.v1");
    reference_.mutable_endpoint()->set_kind("unix");
    reference_.mutable_endpoint()->set_address(options_.socket_path);
    ros_node_name_ = RosNodeName(options_.node_name);
    command_jobs_.reserve(kMaximumCommandJobs);
    wake_ = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (wake_ < 0)
      throw std::runtime_error("cannot create the server wake descriptor");
  }
  ~Impl() { ::close(wake_); }

  int Run() {
    log("run=" + options_.run_id + " provider=" + options_.provider +
        " node=/" + ros_node_name_ + " master=" + options_.ros_master_uri +
        " socket=" + options_.socket_path);
    try {
      binding_.Init(options_, ros_node_name_);
      // The socket exists before ROS is up; Describe holds readiness callers.
      server_ = std::make_unique<xgc2::xrpc::GrpcUnixServer>(
          xgc2::xrpc::UnixOptions{options_.socket_path}, admission_,
          std::vector<grpc::Service *>{&service_});
      startWorkers();
      serve();
    } catch (...) {
      workerFailed("startup", std::current_exception());
    }
    return finish();
  }

  // Async-signal-safe.
  void requestSignalStop() noexcept { requestStop(StopReason::Signal); }

private:
  void requestStop(StopReason reason) noexcept {
    StopReason expected = StopReason::None;
    stop_reason_.compare_exchange_strong(expected, reason);
    wakeMain();
  }
  static xgc2::xrpc::GrpcLimits grpcLimits() {
    xgc2::xrpc::GrpcLimits limits;
    // A fixed native pool serves the whole type host. The bounded domain
    // waits support a complete sixteen-robot fleet while leaving native
    // control room.
    limits.native_threads = 32;
    limits.native_memory_bytes = 32 * 1048576;
    return limits;
  }
  void wakeMain() noexcept {
    const std::uint64_t one = 1;
    const auto ignored = ::write(wake_, &one, sizeof(one));
    (void)ignored;
  }
  Phase phase() const {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    return phase_;
  }
  void setPhase(Phase phase) {
    {
      std::lock_guard<std::mutex> lock(phase_mutex_);
      phase_ = phase;
    }
    phase_changed_.notify_all();
  }

  // ---- lifecycle ---------------------------------------------------------
  void startWorkers() {
    registration_ = std::thread([this] {
      try {
        registrationLoop();
      } catch (...) {
        workerFailed("registration", std::current_exception());
      }
    });
    scheduler_ = std::thread([this] {
      try {
        commandLoop();
      } catch (...) {
        workerFailed("commands", std::current_exception());
      }
      scheduler_alive_.store(false, std::memory_order_release);
    });
    for (std::size_t lane = 0; lane < callbacks_.size(); ++lane)
      callbacks_[lane] = std::thread([this, lane] {
        try {
          pthread_setname_np(pthread_self(), "robot-callback");
          workerReady();
          while (!callbacks_stopping_)
            binding_.PumpLane(lane, Milliseconds(100));
        } catch (...) {
          workerFailed("callbacks", std::current_exception());
        }
      });
  }
  void workerReady() {
    {
      std::lock_guard<std::mutex> lock(startup_mutex_);
      ++ready_workers_;
    }
    wakeMain();
  }
  bool startupComplete() {
    std::lock_guard<std::mutex> lock(startup_mutex_);
    return ready_workers_ == 2 + callbacks_.size() && master_pinned_;
  }
  // Main thread: serves until the Run ends the server or the environment
  // fails; then drains.
  void serve() {
    for (;;) {
      pollfd wake{wake_, POLLIN, 0};
      if (::poll(&wake, 1, -1) < 0 && errno != EINTR)
        throw std::runtime_error("server wake descriptor failed");
      std::uint64_t counter = 0;
      const auto ignored = ::read(wake_, &counter, sizeof(counter));
      (void)ignored;
      if (stop_reason_.load() != StopReason::None)
        break;
      if (phase() == Phase::Starting && startupComplete()) {
        watch_ = std::thread([this] { watchLoop(); });
        setPhase(Phase::Ready);
        log("ready");
      }
    }
    if (phase() == Phase::Ready) {
      log("draining: " + reasonText(stop_reason_.load()));
      drain();
    }
  }
  // Finishes the stop: ends streams, stops every owner and closes the socket.
  int finish() {
    shutdown_ = true;
    notifyStatus();
    stopWatch();
    {
      std::lock_guard<std::mutex> lock(phase_mutex_);
      if (phase_ < Phase::Drained)
        phase_ = Phase::Stopped;
    }
    phase_changed_.notify_all();
    // Streams and the Shutdown reply leave first; then the endpoint closes.
    bool quiesced = server_ ? server_->drain_until(
                                  Steady::now() + timings_.rpc_drain)
                            : true;
    management_stopping_ = true;
    jobs_ready_.notify_all();
    if (registration_.joinable())
      registration_.join();
    stopping_ = true;
    if (scheduler_.joinable())
      scheduler_.join();
    callbacks_stopping_ = true;
    binding_.DisableLanes();
    for (auto &worker : callbacks_)
      if (worker.joinable())
        worker.join();
    // Failed construction may leave jobs queued, but none may run here.
    abortQueuedJobs();
    // NativeRobot objects outlive scheduler I/O and every callback worker.
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      members_.clear();
      namespaces_.clear();
    }
    while (!quiesced) {
      log("admitted RPCs did not finish within the drain budget; retaining "
          "endpoint ownership");
      quiesced = server_->shutdown_until(Steady::now() + timings_.rpc_drain);
    }
    server_.reset();
    {
      auto budget = binding_.BoundMasterCalls(Milliseconds(3000));
      binding_.Shutdown();
    }
    const auto code = exitCode();
    log("exit " + std::to_string(code) + " (" + reasonText(stop_reason_.load()) +
        ")");
    return code;
  }
  int exitCode() const {
    switch (stop_reason_.load()) {
    case StopReason::MasterChanged:
      return kExitMasterChanged;
    case StopReason::MasterUnreachable:
      return kExitMasterUnreachable;
    case StopReason::RosShutdown:
      return kExitRosShutdown;
    case StopReason::Failure:
      return kExitFailure;
    default:
      return fatal_ ? kExitFailure : kExitOk;
    }
  }
  std::string reasonText(StopReason reason) const {
    switch (reason) {
    case StopReason::Signal:
      return "stop signal";
    case StopReason::ShutdownRpc:
      return "Shutdown RPC";
    case StopReason::MasterChanged:
      return "ROS master was replaced: " + failure_detail();
    case StopReason::MasterUnreachable:
      return "ROS master unreachable: " + failure_detail();
    case StopReason::RosShutdown:
      return "ROS shut this node down";
    case StopReason::Failure:
      return "failure: " + failure_detail();
    default:
      return fatal_ ? "failure" : "stopped";
    }
  }
  std::string failure_detail() const {
    std::lock_guard<std::mutex> lock(detail_mutex_);
    return detail_;
  }
  void setDetail(const std::string &detail) {
    std::lock_guard<std::mutex> lock(detail_mutex_);
    if (detail_.empty())
      detail_ = detail;
  }
  void workerFailed(const char *owner, std::exception_ptr failure) noexcept {
    fatal_ = true;
    try {
      std::rethrow_exception(failure);
    } catch (const std::exception &error) {
      setDetail(std::string(owner) + ": " + error.what());
    } catch (...) {
      setDetail(std::string(owner) + ": unknown native failure");
    }
    log(failure_detail());
    jobs_ready_.notify_all();
    requestStop(StopReason::Failure);
  }

  // ---- ROS master watch ----------------------------------------------------
  void watchLoop() {
    pthread_setname_np(pthread_self(), "master-watch");
    bool unreachable = false;
    Steady::time_point unreachable_since;
    MasterRetry retry(std::min(timings_.master_probe_interval, Milliseconds(250)));
    std::unique_lock<std::mutex> lock(watch_mutex_);
    while (!watch_stop_) {
      lock.unlock();
      if (!binding_.Running()) {
        requestStop(StopReason::RosShutdown);
        return;
      }
      MasterIdentity now;
      bool known;
      {
        auto budget = binding_.BoundMasterCalls(Milliseconds(3000));
        ++master_probe_attempts_;
        known = binding_.ProbeMaster(&now) && !budget->failed();
      }
      lock.lock();
      if (watch_stop_)
        break;
      if (known) {
        unreachable = false;
        retry.reset();
        if (!now.sameMaster(pinned_)) {
          setDetail("pinned pid " + std::to_string(pinned_.pid) + " at " +
                    pinned_.uri + ", now pid " + std::to_string(now.pid) +
                    " at " + now.uri);
          log(failure_detail());
          requestStop(StopReason::MasterChanged);
          return;
        }
        // roscore may publish /run_id after its master socket opens.
        if (pinned_.run_id.empty() && !now.run_id.empty()) {
          std::lock_guard<std::mutex> startup_lock(startup_mutex_);
          pinned_.run_id = now.run_id;
        }
      } else {
        if (!unreachable)
          unreachable_since = Steady::now();
        unreachable = true;
        if (Steady::now() - unreachable_since >=
            timings_.master_unreachable_after) {
          setDetail(options_.ros_master_uri + " has not answered for " +
                    std::to_string(timings_.master_unreachable_after.count()) +
                    " ms");
          log(failure_detail());
          requestStop(StopReason::MasterUnreachable);
          return;
        }
      }
      const auto delay = known ? timings_.master_probe_interval
                               : std::min(retry.failed(),
                                          std::chrono::duration_cast<Milliseconds>(
                                              unreachable_since + timings_.master_unreachable_after -
                                              Steady::now()));
      watch_changed_.wait_for(lock, delay,
                              [this] { return watch_stop_; });
    }
  }
  void stopWatch() {
    {
      std::lock_guard<std::mutex> lock(watch_mutex_);
      watch_stop_ = true;
    }
    watch_changed_.notify_all();
    if (watch_.joinable())
      watch_.join();
  }

  // ---- drain -----------------------------------------------------------------
  void drain() {
    setPhase(Phase::Draining);
    notifyStatus();
    // 1. No new commands; let those in flight finish within the budget.
    commands_closed_ = true;
    {
      std::unique_lock<std::mutex> lock(command_mutex_);
      idle_changed_.wait_for(lock, timings_.command_drain,
                             [this] { return noCommandInFlight(); });
    }
    // 2. Cancel the rest, send the final zero commands and remove every
    // member; the registration thread owns that sequence.
    auto done = std::make_shared<std::promise<void>>();
    auto finished = done->get_future();
    if (postRegistration({[this, done] {
                            closeAllMembers();
                            done->set_value();
                          },
                          [done](bool) { done->set_value(); }},
                         true)) {
      if (finished.wait_for(timings_.member_drain) != std::future_status::ready)
        log("members were not all stopped within the drain budget");
    }
    // 3. Let status subscribers read the removals.
    awaitStatusDelivered(timings_.status_flush);
    log("drained");
    setPhase(Phase::Drained);
  }
  bool noCommandInFlight() {
    std::lock_guard<std::mutex> lock(table_mutex_);
    for (const auto &entry : members_)
      if (entry.second->in_flight)
        return false;
    return true;
  }
  void closeAllMembers() {
    std::vector<std::shared_ptr<Member>> members;
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      for (auto &entry : members_) {
        entry.second->closing = true;
        members.push_back(entry.second);
      }
      members_.clear();
      namespaces_.clear();
      ++members_revision_;
    }
    notifyStatus();
    auto budget = binding_.BoundMasterCalls(timings_.member_removal);
    for (auto &member : members) {
      try {
        fenceCommands(member);
      } catch (...) {
        workerFailed("native fence", std::current_exception());
      }
      try {
        if (member->native)
          member->native->Stop();
      } catch (...) {
        workerFailed("native stop", std::current_exception());
      }
    }
    // The final commands must leave the process before the robots, and with
    // them their publishers, are destroyed.
    if (!members.empty())
      binding_.FlushPublications(timings_.publication_flush);
  }
  void awaitReady(const xgc2::xrpc::GrpcCallScope &scope, Milliseconds wait) {
    const auto until =
        std::min(Steady::now() + wait, scope.deadline() - kReadyAnswerMargin);
    std::unique_lock<std::mutex> lock(phase_mutex_);
    phase_changed_.wait_until(lock, until, [this] {
      return phase_ != Phase::Starting;
    });
  }
  bool awaitDrained(const xgc2::xrpc::GrpcCallScope &scope) {
    std::unique_lock<std::mutex> lock(phase_mutex_);
    while (phase_ < Phase::Drained) {
      if (scope.cancelled() || Steady::now() >= scope.deadline())
        return false;
      phase_changed_.wait_until(
          lock, std::min(scope.deadline(), Steady::now() + Milliseconds(100)));
    }
    return true;
  }
  std::optional<grpc::Status>
  requireStarted(const xgc2::xrpc::GrpcCallScope &scope) const {
    if (phase() == Phase::Starting)
      return scope.application_error(grpc::Status(
          grpc::StatusCode::FAILED_PRECONDITION, "server is not ready"));
    return std::nullopt;
  }
  std::optional<grpc::Status>
  requireReady(const xgc2::xrpc::GrpcCallScope &scope) const {
    switch (phase()) {
    case Phase::Ready:
      return std::nullopt;
    case Phase::Starting:
      return scope.application_error(grpc::Status(
          grpc::StatusCode::FAILED_PRECONDITION, "server is not ready"));
    default:
      return scope.application_error(grpc::Status(
          grpc::StatusCode::FAILED_PRECONDITION, "server is draining"));
    }
  }

  // ---- describe --------------------------------------------------------------
  void describe(wire::DescribeResponse *response) {
    *response->mutable_service_ref() = reference_;
    response->set_provider_definition_id(options_.provider);
    response->set_maximum_robots(kMaximumRobots);
    response->set_maximum_members(kMaximumRobots);
    response->set_maximum_batch_members(kMaximumBatchMembers);
    response->set_maximum_commands_per_robot(kMaximumCommandsPerRobot);
    const auto current = phase();
    response->set_ready(current == Phase::Ready);
    response->set_facts_json(facts(current));
  }
  std::string facts(Phase current) {
    nlohmann::json facts;
    MasterIdentity master;
    bool pinned = false;
    {
      std::lock_guard<std::mutex> lock(startup_mutex_);
      master = pinned_;
      pinned = master_pinned_;
    }
    facts["ros_master_uri"] = options_.ros_master_uri;
    facts["ros_master_run_id"] = pinned ? master.run_id : "";
    facts["ros_master_pid"] = pinned ? master.pid : 0;
    facts["master_probe_attempts"] = master_probe_attempts_.load();
    facts["master_bindings"] = master_bindings_.load();
    facts["ros_node"] = "/" + ros_node_name_;
    facts["run_id"] = options_.run_id;
    facts["provider"] = options_.provider;
    std::vector<std::shared_ptr<Member>> members;
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      facts["members"] = members_.size();
      facts["members_revision"] = members_revision_;
      for (const auto &entry : members_)
        members.push_back(entry.second);
    }
    facts["robots"] = nlohmann::json::object();
    for (const auto &member : members) {
      xgc::semantic::common::v1::VehicleHealth health;
      {
        std::lock_guard<std::mutex> lock(member->state_mutex);
        const auto found = member->latest.find("state.health");
        if (found != member->latest.end())
          health.ParseFromString(found->second.second.message().payload().value());
      }
      auto &robot = facts["robots"][member->identity.robot_id()];
      robot["state"] = health.online() ? "online" : "waiting";
      robot["reason"] = health.online() ? ""
                         : health.summary().empty() ? "waiting for robot telemetry"
                                                    : health.summary();
    }
    facts["state"] = current == Phase::Starting   ? "starting"
                     : current == Phase::Ready    ? "ready"
                     : current == Phase::Draining ? "draining"
                                                  : "stopped";
    facts["reason"] = current == Phase::Starting && !pinned
                          ? "waiting for ROS master binding"
                          : "";
    return facts.dump();
  }

  // ---- jobs --------------------------------------------------------------------
  static void abortJob(Job &job, bool started) noexcept {
    if (!job.abort)
      return;
    try {
      job.abort(started);
    } catch (...) {
      log("failed to construct interrupted-work reply");
    }
  }
  void abortQueuedJobs() {
    std::deque<Job> registration;
    std::vector<Job> commands;
    Job fence;
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      registration.swap(registration_jobs_);
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      commands.swap(command_jobs_);
      fence = std::move(fence_job_);
      fence_job_ = {};
    }
    for (auto &job : registration)
      abortJob(job, false);
    for (auto &job : commands)
      abortJob(job, false);
    abortJob(fence, false);
  }
  bool postRegistration(Job job, bool privileged = false) {
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      if (management_stopping_ || shutdown_ ||
          (!privileged &&
           registration_jobs_.size() >= kMaximumRegistrationJobs))
        return false;
      if (privileged)
        registration_jobs_.push_front(std::move(job));
      else
        registration_jobs_.push_back(std::move(job));
    }
    jobs_ready_.notify_one();
    return true;
  }
  bool postCommand(Job job) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (stopping_ || shutdown_ || command_jobs_.size() >= kMaximumCommandJobs)
      return false;
    command_jobs_.push_back(std::move(job));
    return true;
  }
  void postFence(Job job) {
    // The single registration owner waits for each barrier before posting the
    // next one. Its reserved slot cannot be consumed by mutation admission.
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!scheduler_alive_.load(std::memory_order_acquire))
      throw std::runtime_error("native scheduler exited");
    if (fence_job_.run)
      throw std::logic_error("overlapping native lifecycle barriers");
    fence_job_ = std::move(job);
  }

  // ---- registration thread --------------------------------------------------------
  void registrationLoop() {
    pthread_setname_np(pthread_self(), "ros-library");
    MasterIdentity identity;
    MasterRetry retry(std::min(timings_.master_probe_interval, Milliseconds(250)));
    for (;;) {
      if (shutdown_ || stop_reason_.load() != StopReason::None)
        return;
      {
        auto budget = binding_.BoundMasterCalls(Milliseconds(3000));
        // Do not start roscpp until its master exists. Each attempt is
        // bounded, but waiting for the first binding has no deadline.
        ++master_probe_attempts_;
        if (binding_.ProbeMaster(&identity) && !budget->failed()) {
          binding_.Connect();
          if (!budget->failed()) {
            identity.run_id = binding_.MasterRunId();
            ++master_bindings_;
            break;
          }
        }
      }
      const auto delay = retry.failed();
      std::unique_lock<std::mutex> lock(watch_mutex_);
      watch_changed_.wait_for(lock, delay, [this] {
        return shutdown_ || stop_reason_.load() != StopReason::None;
      });
    }
    {
      std::lock_guard<std::mutex> lock(startup_mutex_);
      pinned_ = identity;
      master_pinned_ = true;
    }
    pthread_setname_np(pthread_self(), "robot-members");
    workerReady();
    for (;;) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(jobs_mutex_);
        jobs_ready_.wait(lock, [this] {
          return management_stopping_ || shutdown_ || !registration_jobs_.empty();
        });
        if (management_stopping_ || shutdown_)
          break;
        job = std::move(registration_jobs_.front());
        registration_jobs_.pop_front();
      }
      try {
        job.run();
      } catch (...) {
        abortJob(job, true);
        throw;
      }
    }
  }

  // ---- command scheduler ------------------------------------------------------------
  void commandLoop() {
    pthread_setname_np(pthread_self(), "robot-commands");
    std::exception_ptr failure;
    std::vector<Job> jobs;
    jobs.reserve(kMaximumCommandJobs);
    std::vector<std::shared_ptr<Member>> members;
    {
      std::unique_ptr<CommandIo> io = binding_.CreateCommandIo();
      io_ = io.get();
      scheduler_alive_.store(true, std::memory_order_release);
      workerReady();
      auto next_periodic = Steady::now();
      std::uint64_t revision = UINT64_MAX;
      try {
        while (!stopping_) {
          jobs.clear();
          Job fence;
          {
            std::lock_guard<std::mutex> lock(command_mutex_);
            jobs.swap(command_jobs_);
            fence = std::move(fence_job_);
            fence_job_ = {};
          }
          for (auto &job : jobs) {
            if (shutdown_)
              abortJob(job, false);
            else
              try {
                job.run();
              } catch (...) {
                abortJob(job, true);
                job = {};
                throw;
              }
            job = {};
          }
          if (fence.run)
            try {
              fence.run();
            } catch (...) {
              abortJob(fence, true);
              throw;
            }
          {
            std::lock_guard<std::mutex> lock(table_mutex_);
            if (revision != members_revision_) {
              members.clear();
              members.reserve(members_.size());
              for (auto &entry : members_)
                members.push_back(entry.second);
              revision = members_revision_;
            }
          }
          const bool periodic = Steady::now() >= next_periodic;
          if (periodic)
            next_periodic = Steady::now() + Milliseconds(100);
          for (auto &member : members) {
            if (member->closing) {
              if (member->native_call)
                io->Cancel(member->native_call);
              continue;
            }
            if (member->native_call && member->active_call &&
                (member->active_call->cancelled ||
                 member->active_call->work.cancelled()))
              io->Cancel(member->native_call);
            if (periodic && !shutdown_) {
              std::lock_guard<std::mutex> lock(member->native_mutex);
              if (!member->closing)
                member->native->Periodic();
            }
            if (!shutdown_ && !member->in_flight && !member->commands.empty())
              dispatch(member, *io);
          }
          io->Pump(members.empty() ? Milliseconds(100) : Milliseconds(2));
        }
      } catch (...) {
        failure = std::current_exception();
        workerFailed("commands", failure);
      }
      for (auto &job : jobs)
        abortJob(job, false);
      jobs.clear();
      // Keep the last scheduler snapshot as well as current members. Closing
      // native I/O must precede releasing an active call's retained permit.
      {
        std::lock_guard<std::mutex> lock(table_mutex_);
        for (auto &entry : members_)
          if (std::find(members.begin(), members.end(), entry.second) ==
              members.end())
            members.push_back(entry.second);
      }
      for (auto &member : members) {
        if (member->native_call)
          io->Cancel(member->native_call);
        for (auto &call : member->commands)
          finishExecute(call, CommandError(call->request.operation(),
                                           operation::ERROR_CLASS_CANCELLED,
                                           "server-stopped",
                                           "not executed: server shutdown"));
        member->commands.clear();
      }
      try {
        io->Pump(Milliseconds(0));
      } catch (...) {
        failure = std::current_exception();
        workerFailed("native completion", failure);
      }
      io_ = nullptr;
    }
    for (auto &member : members) {
      if (member->active_call)
        finishExecute(
            member->active_call,
            CommandError(
                member->active_call->request.operation(),
                operation::ERROR_CLASS_UNCERTAIN, "native-result-unknown",
                "native owner exited without a result; reconcile outcome"));
      member->active_call.reset();
      member->native_call = 0;
      member->in_flight = false;
    }
  }
  // Runs on the scheduler: admits a command to its member's queue.
  void enqueue(const std::shared_ptr<ExecuteCall> &call) {
    std::shared_ptr<Member> member;
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      const auto found = members_.find(call->request.identity().robot_id());
      if (found != members_.end() &&
          found->second->identity.run_id() == call->request.identity().run_id() &&
          found->second->identity.connection_epoch() ==
              call->request.identity().connection_epoch())
        member = found->second;
    }
    if (!member || member->closing || commands_closed_) {
      finishExecute(call, CommandError(call->request.operation(),
                                       operation::ERROR_CLASS_REJECTED,
                                       commands_closed_ ? "server-draining"
                                                        : "member-unavailable",
                                       commands_closed_
                                           ? "not executed: server is draining"
                                           : "not executed: active member required"));
      return;
    }
    if (member->commands.size() >= kMaximumCommandsPerRobot) {
      finishExecute(call, CommandError(call->request.operation(),
                                       operation::ERROR_CLASS_RESOURCE_EXHAUSTED,
                                       "robot-queue-full",
                                       "not executed: native command queue is full"));
      return;
    }
    member->commands.push_back(call);
  }
  void dispatch(const std::shared_ptr<Member> &member, CommandIo &io) {
    auto call = std::move(member->commands.front());
    member->commands.pop_front();
    auto &request = call->request.operation();
    if (commands_closed_) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_CANCELLED,
                                       "server-draining",
                                       "not executed: server is draining"));
      return;
    }
    if (member->uncertain) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_REJECTED,
                                       "previous-result-unknown",
                                       "not executed: previous native result is unknown"));
      return;
    }
    if (call->cancelled || call->work.cancelled()) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_CANCELLED,
                                       "operation-cancelled",
                                       "not executed: call cancelled"));
      return;
    }
    if (request.context().deadline().deadline_unix_nanos() <=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count()) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_DEADLINE,
                                       "deadline-exceeded",
                                       "not executed: operation deadline elapsed"));
      return;
    }
    member->in_flight = true;
    member->active_call = call;
    std::lock_guard<std::mutex> lock(member->native_mutex);
    member->native_call = member->native->Execute(
        request, io, [this, member, call](operation::OperationEvent result) {
          member->in_flight = false;
          member->native_call = 0;
          member->active_call.reset();
          if (result.phase() == operation::OPERATION_PHASE_UNCERTAIN)
            member->uncertain = true;
          finishExecute(call, std::move(result));
          { std::lock_guard<std::mutex> lock(command_mutex_); }
          idle_changed_.notify_all();
        });
  }
  template <class Request, class Response, class Submit>
  grpc::Status invoke(xgc2::xrpc::GrpcCallScope scope, const Request &request,
                      Response *response, Submit submit) {
    BoundedWaiter waiter(blocking_calls_, kMaximumBlockingCalls);
    if (!waiter.accepted)
      return scope.application_error(
          grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                       "native blocking call capacity exhausted"));
    auto call = std::make_shared<Unary<Request, Response>>();
    call->request = request;
    call->work = scope.retain_work();
    if (!submit(call))
      return scope.application_error(
          grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                       "native dispatch queue is full"));
    std::unique_lock<std::mutex> lock(call->mutex);
    while (!call->finished) {
      if (scope.cancelled()) {
        call->cancelled = true;
        return grpc::Status(
            Steady::now() >= scope.deadline()
                ? grpc::StatusCode::DEADLINE_EXCEEDED
                : grpc::StatusCode::CANCELLED,
            "native call cancelled or expired; reconcile its outcome");
      }
      call->changed.wait_until(
          lock, std::min(scope.deadline(), Steady::now() + Milliseconds(20)),
          [&] { return call->finished; });
    }
    *response = std::move(call->response);
    return call->status;
  }

  // ---- status stream ------------------------------------------------------------------
  void notifyStatus() {
    status_version_.fetch_add(1, std::memory_order_release);
    status_changed_.notify_all();
  }
  struct Subscriber {
    std::atomic<std::uint64_t> delivered{0};
  };
  void awaitStatusDelivered(Milliseconds budget) {
    const auto target = status_version_.load(std::memory_order_acquire);
    std::unique_lock<std::mutex> lock(subscribers_mutex_);
    subscribers_changed_.wait_for(lock, budget, [&] {
      for (const auto *subscriber : subscribers_)
        if (subscriber->delivered.load() < target)
          return false;
      return true;
    });
  }
  grpc::Status
  subscribe(grpc::ServerWriter<wire::SubscribeStatusResponse> *writer,
            const xgc2::xrpc::GrpcCallScope &scope) {
    const auto deadline = scope.deadline();
    Stream stream;
    Subscriber subscriber;
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      subscribers_.push_back(&subscriber);
    }
    struct Registration {
      Impl &owner;
      Subscriber &subscriber;
      ~Registration() {
        std::lock_guard<std::mutex> lock(owner.subscribers_mutex_);
        owner.subscribers_.erase(std::find(owner.subscribers_.begin(),
                                           owner.subscribers_.end(),
                                           &subscriber));
        owner.subscribers_changed_.notify_all();
      }
    } registration{*this, subscriber};
    const auto frame_limit = admission_.limits().response_bytes;
    std::uint64_t observed = UINT64_MAX;
    while (!shutdown_ && !scope.cancelled()) {
      const auto version = status_version_.load(std::memory_order_acquire);
      if (version == observed) {
        std::unique_lock<std::mutex> lock(status_mutex_);
        status_changed_.wait_until(
            lock, std::min(deadline, Steady::now() + Milliseconds(20)), [&] {
              return shutdown_ || scope.cancelled() ||
                     status_version_.load(std::memory_order_acquire) != observed;
            });
        continue;
      }
      observed = version;
      std::map<std::string, std::shared_ptr<Member>> members;
      {
        std::lock_guard<std::mutex> lock(table_mutex_);
        members = members_;
      }
      stream.sending.Clear();
      auto flush = [&] {
        if (!stream.sending.members_size())
          return true;
        const auto success = writer->Write(stream.sending);
        stream.sending.Clear();
        return success;
      };
      for (auto i = stream.identities.begin(); i != stream.identities.end();) {
        const auto current = members.find(i->first);
        if (current == members.end() ||
            current->second->identity.connection_epoch() !=
                i->second.connection_epoch()) {
          wire::MemberStatus removal;
          *removal.mutable_identity() = i->second;
          removal.set_removed(true);
          if (removal.ByteSizeLong() + 16 > frame_limit)
            return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                "member identity exceeds response policy");
          if (stream.sending.ByteSizeLong() + removal.ByteSizeLong() + 16 >
                  frame_limit &&
              !flush())
            return grpc::Status::CANCELLED;
          *stream.sending.add_members() = std::move(removal);
          stream.seen.erase(i->first);
          i = stream.identities.erase(i);
        } else
          ++i;
      }
      for (const auto &entry : members) {
        const auto &member = entry.second;
        stream.identities[entry.first] = member->identity;
        std::vector<std::pair<std::string,
                              std::pair<std::uint64_t, wire::RobotMessage>>>
            updates;
        updates.reserve(kMaximumChannelsPerRobot);
        {
          std::lock_guard<std::mutex> lock(member->state_mutex);
          for (const auto &channel : member->latest)
            if (stream.seen[entry.first][channel.first] != channel.second.first)
              updates.push_back(channel);
        }
        // Keep each member's channels together within the response policy.
        // No native state mutex is held across a slow peer's blocking Write.
        wire::MemberStatus update;
        *update.mutable_identity() = member->identity;
        auto append_update = [&] {
          if (!update.messages_size())
            return true;
          if (stream.sending.ByteSizeLong() + update.ByteSizeLong() + 16 >
                  frame_limit &&
              !flush())
            return false;
          *stream.sending.add_members() = std::move(update);
          return true;
        };
        for (const auto &channel : updates) {
          const auto channel_bytes = channel.second.second.ByteSizeLong() + 16;
          if (member->identity.ByteSizeLong() + channel_bytes + 16 > frame_limit)
            return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                "native state exceeds response policy");
          if (update.ByteSizeLong() + channel_bytes + 16 > frame_limit) {
            if (!append_update() || !flush())
              return grpc::Status::CANCELLED;
            update.Clear();
            *update.mutable_identity() = member->identity;
          }
          *update.add_messages() = channel.second.second;
          stream.seen[entry.first][channel.first] = channel.second.first;
        }
        if (!append_update())
          return grpc::Status::CANCELLED;
      }
      if (!flush())
        return grpc::Status::CANCELLED;
      subscriber.delivered.store(observed, std::memory_order_release);
      { std::lock_guard<std::mutex> lock(subscribers_mutex_); }
      subscribers_changed_.notify_all();
    }
    return scope.cancelled()
               ? grpc::Status(Steady::now() >= deadline
                                  ? grpc::StatusCode::DEADLINE_EXCEEDED
                                  : grpc::StatusCode::CANCELLED,
                              "status stream cancelled or expired")
               : grpc::Status::OK;
  }

  // ---- members ----------------------------------------------------------------------------
  wire::ApplyMembersResponse apply(const wire::ApplyMembersRequest &request,
                                   const xgc2::xrpc::GrpcWorkPermit &work) {
    wire::ApplyMembersResponse result;
    for (const auto &registration : request.members()) {
      auto *item = result.add_members();
      *item->mutable_identity() = registration.identity();
      std::string error;
      auto error_class = operation::ERROR_CLASS_REJECTED;
      std::string code = "member-registration-failed";
      const auto &identity = registration.identity();
      const auto config = configFrom(registration.configuration());
      const auto namespace_entry = config.parameters.find("namespace");
      if (work.cancelled() || shutdown_ || phase() != Phase::Ready)
        error = "member registration cancelled";
      else if (!validIdentity(identity) ||
               identity.target_id() != options_.target_id ||
               config.robot_id != identity.robot_id())
        error = "invalid or cross-target member identity";
      else if (identity.run_id() != options_.run_id)
        error = "member belongs to another Run";
      else if (registration.configuration().channels_size() >
               static_cast<int>(kMaximumChannelsPerRobot))
        error = "native channel limit exceeded";
      else if (namespace_entry == config.parameters.end() ||
               namespace_entry->second.empty())
        error = "native namespace required";
      for (const auto &name : {std::make_pair("ros_master_uri", &options_.ros_master_uri),
                               std::make_pair("ros_ip", &options_.ros_ip)}) {
        const auto value = config.parameters.find(name.first);
        if (error.empty() && value != config.parameters.end() &&
            !value->second.empty() && value->second != *name.second)
          error = "member crosses server ROS environment";
      }
      std::shared_ptr<Member> member;
      if (error.empty()) {
        std::lock_guard<std::mutex> lock(table_mutex_);
        const auto existing = members_.find(identity.robot_id());
        const auto owner = namespaces_.find(namespace_entry->second);
        if (existing != members_.end()) {
          if (existing->second->identity.connection_epoch() !=
              identity.connection_epoch()) {
            error = "active member connection epoch conflicts";
            code = "member-epoch-conflict";
          } else if (!google::protobuf::util::MessageDifferencer::Equals(
                         existing->second->configuration,
                         registration.configuration())) {
            error = "member is registered with another configuration; remove "
                    "it first";
            code = "member-configuration-conflict";
          } else {
            continue; // already applied: idempotent
          }
        } else if (owner != namespaces_.end()) {
          error = "ROS namespace is used by robot " + owner->second;
          code = "namespace-conflict";
        } else if (members_.size() >= kMaximumRobots) {
          error = "native robot table capacity exhausted";
          error_class = operation::ERROR_CLASS_RESOURCE_EXHAUSTED;
        } else {
          member = std::make_shared<Member>();
          member->lane = next_lane_++ % binding_.Lanes();
        }
      }
      if (member && error.empty()) {
        try {
          auto node = binding_.CreateNode(member->lane);
          auto budget = binding_.BoundMasterCalls(timings_.member_registration);
          const std::weak_ptr<Member> weak = member;
          auto native = factory_(
              *node, config,
              [this, weak](wire::RobotMessage message) {
                auto member = weak.lock();
                if (!member || member->closing)
                  return;
                if (message.ByteSizeLong() > kMaximumNativeMessageBytes)
                  return;
                bool declared = false;
                for (const auto &channel : member->configuration.channels())
                  if (channel.enabled() &&
                      channel.channel_id() == message.channel_id()) {
                    declared = true;
                    break;
                  }
                if (!declared)
                  return;
                std::lock_guard<std::mutex> lock(member->state_mutex);
                auto &slot = member->latest[message.channel_id()];
                slot.first = ++member->state_version;
                slot.second = std::move(message);
                notifyStatus();
              },
              &error);
          if (budget->failed() && error.empty())
            error = "ROS master registration did not complete";
          if (!native && error.empty())
            error = "native resource creation failed";
          if (!error.empty() && native) {
            try {
              native->Stop();
            } catch (...) {
              member->native = native;
              retained_.push_back(member);
              workerFailed("failed factory cleanup", std::current_exception());
              error_class = operation::ERROR_CLASS_UNCERTAIN;
            }
          }
          if (error.empty()) {
            member->identity = identity;
            member->configuration = registration.configuration();
            member->native = std::move(native);
          }
        } catch (const std::exception &failure) {
          error = std::string("native resource creation interrupted: ") +
                  failure.what();
          error_class = operation::ERROR_CLASS_UNCERTAIN;
        } catch (...) {
          error =
              "native resource creation interrupted by an unknown exception";
          error_class = operation::ERROR_CLASS_UNCERTAIN;
        }
      }
      if (member && error.empty()) {
        // The member is complete; publish it. The draining flag is checked
        // under the table lock so no member appears after the drain began.
        std::lock_guard<std::mutex> lock(table_mutex_);
        if (phase() != Phase::Ready) {
          error = "member registration cancelled";
        } else {
          member->closing = false;
          members_[identity.robot_id()] = member;
          namespaces_[namespace_entry->second] = identity.robot_id();
          ++members_revision_;
          notifyStatus();
        }
      }
      if (!error.empty()) {
        if (member && member->native && phase() != Phase::Ready) {
          try {
            member->native->Stop();
          } catch (...) {
            workerFailed("late member cleanup", std::current_exception());
          }
        }
        item->mutable_error()->set_class_(error_class);
        item->mutable_error()->set_code(code);
        item->mutable_error()->set_message(error);
      }
    }
    return result;
  }
  wire::RemoveMembersResponse remove(const wire::RemoveMembersRequest &request,
                                     const xgc2::xrpc::GrpcWorkPermit &work) {
    auto budget = binding_.BoundMasterCalls(timings_.member_removal);
    wire::RemoveMembersResponse result;
    std::vector<std::shared_ptr<Member>> stopped;
    for (const auto &identity : request.members()) {
      auto *item = result.add_members();
      *item->mutable_identity() = identity;
      if (work.cancelled()) {
        item->mutable_error()->set_class_(operation::ERROR_CLASS_CANCELLED);
        item->mutable_error()->set_code("member-removal-cancelled");
        continue;
      }
      std::shared_ptr<Member> released;
      {
        std::lock_guard<std::mutex> lock(table_mutex_);
        const auto found = members_.find(identity.robot_id());
        if (found == members_.end() ||
            found->second->identity.run_id() != identity.run_id())
          continue; // not registered: removal is idempotent
        if (found->second->identity.connection_epoch() !=
            identity.connection_epoch()) {
          item->mutable_error()->set_class_(operation::ERROR_CLASS_REJECTED);
          item->mutable_error()->set_code("member-epoch-conflict");
          continue;
        }
        released = found->second;
        released->closing = true;
        namespaces_.erase(released->configuration.parameters().at("namespace"));
        members_.erase(found);
        ++members_revision_;
        notifyStatus();
      }
      // Stop fences native callbacks before the removal is acknowledged. The
      // scheduler sees closing and cannot publish or submit another side
      // effect.
      fenceCommands(released);
      released->native->Stop();
      stopped.push_back(std::move(released));
    }
    // Once for the whole batch, before the stopped robots are destroyed.
    if (!stopped.empty())
      binding_.FlushPublications(timings_.publication_flush);
    return result;
  }
  // Cancels the member's native call and fails its queued commands; returns
  // once the scheduler has done so.
  void fenceCommands(const std::shared_ptr<Member> &member) {
    auto done = std::make_shared<std::promise<void>>();
    auto ready = done->get_future();
    if (!scheduler_alive_.load(std::memory_order_acquire)) {
      // The scheduler flag changes only after its native services destruct.
      if (member->active_call)
        finishExecute(member->active_call,
                      CommandError(member->active_call->request.operation(),
                                   operation::ERROR_CLASS_UNCERTAIN,
                                   "native-result-unknown",
                                   "native scheduler exited; reconcile outcome"));
      member->active_call.reset();
      member->native_call = 0;
      member->in_flight = false;
      for (auto &call : member->commands)
        finishExecute(call, CommandError(call->request.operation(),
                                         operation::ERROR_CLASS_CANCELLED,
                                         "server-stopped",
                                         "not executed: native scheduler exited"));
      member->commands.clear();
      return;
    }
    postFence(
        {[this, member, done] {
           if (member->native_call && io_)
             io_->Cancel(member->native_call);
           for (auto &call : member->commands)
             finishExecute(call, CommandError(call->request.operation(),
                                              operation::ERROR_CLASS_CANCELLED,
                                              "member-removed",
                                              "not executed: member removed"));
           member->commands.clear();
           if (io_)
             io_->Pump(Milliseconds(0));
           done->set_value();
         },
         [done](bool) {
           done->set_exception(std::make_exception_ptr(
               std::runtime_error("native lifecycle barrier interrupted")));
         }});
    while (ready.wait_for(Milliseconds(20)) != std::future_status::ready) {
      if (!scheduler_alive_.load(std::memory_order_acquire)) {
        Job pending;
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          pending = std::move(fence_job_);
          fence_job_ = {};
        }
        abortJob(pending, false);
        fenceCommands(member);
        return;
      }
    }
    ready.get();
  }

  const ServerOptions options_;
  RosBinding &binding_;
  RobotFactory factory_;
  const ServerTimings timings_;
  xgc2::xrpc::GrpcAdmission admission_;
  Service service_;
  operation::RuntimeServiceReference reference_;
  std::string ros_node_name_;
  std::unique_ptr<xgc2::xrpc::GrpcUnixServer> server_;
  int wake_ = -1;

  mutable std::mutex phase_mutex_;
  std::condition_variable phase_changed_;
  Phase phase_ = Phase::Starting;
  std::atomic<StopReason> stop_reason_{StopReason::None};
  mutable std::mutex detail_mutex_;
  std::string detail_;
  std::atomic<bool> fatal_{false};

  std::mutex startup_mutex_;
  std::size_t ready_workers_ = 0;
  bool master_pinned_ = false;
  MasterIdentity pinned_;

  std::mutex watch_mutex_;
  std::condition_variable watch_changed_;
  bool watch_stop_ = false;
  std::atomic<std::uint64_t> master_probe_attempts_{0}, master_bindings_{0};
  std::thread watch_;

  std::mutex status_mutex_;
  std::condition_variable status_changed_;
  std::atomic<std::uint64_t> status_version_{0};
  std::mutex subscribers_mutex_;
  std::condition_variable subscribers_changed_;
  std::vector<Subscriber *> subscribers_;
  std::atomic<std::size_t> status_subscribers_{0}, blocking_calls_{0},
      readiness_waits_{0};

  std::mutex table_mutex_, jobs_mutex_, command_mutex_;
  std::map<std::string, std::shared_ptr<Member>> members_;
  std::map<std::string, std::string> namespaces_;
  std::uint64_t members_revision_ = 0;
  std::size_t next_lane_ = 0;
  std::vector<std::shared_ptr<Member>> retained_;
  std::deque<Job> registration_jobs_;
  std::vector<Job> command_jobs_;
  Job fence_job_;
  std::condition_variable jobs_ready_, idle_changed_;
  std::thread registration_, scheduler_;
  std::array<std::thread, 2> callbacks_;
  CommandIo *io_ = nullptr;
  std::atomic<bool> scheduler_alive_{false}, commands_closed_{false};
  std::atomic<bool> shutdown_{false}, stopping_{false},
      management_stopping_{false}, callbacks_stopping_{false};
};

RobotServer::RobotServer(ServerOptions options, RosBinding &binding,
                         RobotFactory factory, ServerTimings timings)
    : impl_(new Impl(std::move(options), binding, std::move(factory),
                     timings)) {}
RobotServer::~RobotServer() = default;
int RobotServer::Run() { return impl_->Run(); }
void RobotServer::RequestStop() noexcept { impl_->requestSignalStop(); }

operation::OperationEvent
CommandError(const operation::OperationRequest &request,
             operation::ErrorClass error_class, const std::string &code,
             const std::string &detail) {
  operation::OperationEvent event;
  event.set_work_id(request.context().work_id());
  event.set_occurred_unix_nanos(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  event.set_phase(error_class == operation::ERROR_CLASS_UNCERTAIN
                      ? operation::OPERATION_PHASE_UNCERTAIN
                      : error_class == operation::ERROR_CLASS_REJECTED
                            ? operation::OPERATION_PHASE_REJECTED
                            : error_class == operation::ERROR_CLASS_CANCELLED
                                  ? operation::OPERATION_PHASE_CANCELLED
                                  : error_class ==
                                            operation::ERROR_CLASS_DEADLINE
                                        ? operation::OPERATION_PHASE_EXPIRED
                                        : operation::OPERATION_PHASE_FAILED);
  auto *error = event.mutable_error();
  error->set_class_(error_class);
  error->set_code(code);
  error->set_message(detail);
  return event;
}
} // namespace xgc2_ros1_robot_adapter
