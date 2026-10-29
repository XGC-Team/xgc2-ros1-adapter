#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include "xgc2_ros1_robot_adapter/master_retry.hpp"
#include "xgc/robot/v1/server.grpc.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc2/xrpc/bootstrap.hpp"
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
#include <grpcpp/alarm.h>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <openssl/crypto.h>
#include <poll.h>
#include <set>
#include <sys/eventfd.h>
#include <sys/stat.h>
#include <thread>
#include <type_traits>
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
// These lanes fit inside the existing 32-call XRPC host: 18 ordinary work
// owners, two retirement waiters, one independent safety owner, one member
// mutation owner, four status streams and four readiness waits leave two
// admissions for Shutdown and short calls. The owner retains each work lane
// until the actual native work joins, even if the caller cancels.
constexpr std::size_t kMaximumStatusSubscribers = 4, kMaximumBlockingCalls = 18;
constexpr std::size_t kMaximumControlWaits = 2;
constexpr std::size_t kMaximumSafetyWaits = 1, kMaximumMembershipWaits = 1;
constexpr std::size_t kMaximumReadinessWaits = 4;
constexpr std::uint32_t kMaximumReadyWaitMs = 30000;
// A held Describe answers this long before the caller's deadline so the
// caller receives the current (not ready) state instead of a timeout.
constexpr Milliseconds kReadyAnswerMargin{250};
// The control watchdog runs on the existing 100 ms native scheduler. A
// profile must leave at least three such intervals for its owned heartbeat.
constexpr Milliseconds kMinimumControlHeartbeat{100};
constexpr Milliseconds kMinimumControlRenewInterval = kMinimumControlHeartbeat / 2;
constexpr Milliseconds kMaximumControlHeartbeat{10000};

std::array<unsigned char, 32> readProcessOwner() {
  const char *value = std::getenv("XGC_PROCESS_OWNER_FD");
  if (!value || !*value)
    throw std::invalid_argument("private process owner descriptor is required");
  std::string descriptor(value);
  ::unsetenv("XGC_PROCESS_OWNER_FD");
  if (descriptor.size() > 9 ||
      descriptor.find_first_not_of("0123456789") != std::string::npos)
    throw std::invalid_argument("invalid private process owner descriptor");
  const auto number = std::stoul(descriptor);
  if (number < 3 || number > static_cast<unsigned long>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("invalid private process owner descriptor");
  const int fd = static_cast<int>(number);
  struct Close { int fd; ~Close() { ::close(fd); } } close{fd};
  struct stat info{};
  const int flags = ::fcntl(fd, F_GETFL);
  if (::fstat(fd, &info) != 0 || !S_ISFIFO(info.st_mode) || flags < 0 ||
      ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0)
    throw std::invalid_argument("invalid private process owner input");
  std::array<unsigned char, 32> owner{};
  std::size_t size = 0;
  while (size < owner.size()) {
    const auto count = ::read(fd, owner.data() + size, owner.size() - size);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      OPENSSL_cleanse(owner.data(), owner.size());
      throw std::invalid_argument("incomplete private process owner input");
    }
    size += static_cast<std::size_t>(count);
  }
  unsigned char extra;
  ssize_t count;
  do { count = ::read(fd, &extra, 1); } while (count < 0 && errno == EINTR);
  if (count != 0) {
    OPENSSL_cleanse(owner.data(), owner.size());
    throw std::invalid_argument("invalid private process owner input length");
  }
  return owner;
}

bool secretEqual(const std::string &value, const unsigned char *expected,
                 std::size_t size) {
  return value.size() == size && CRYPTO_memcmp(value.data(), expected, size) == 0;
}

struct BoundedWaiter {
  std::atomic<std::size_t> *count = nullptr;
  bool accepted = false;
  static bool take(std::atomic<std::size_t> &value, std::size_t maximum) {
    if (value.fetch_add(1) < maximum) return true;
    value.fetch_sub(1);
    return false;
  }
  BoundedWaiter(std::atomic<std::size_t> &value, std::size_t maximum) {
    if (take(value, maximum)) {
      count = &value;
      accepted = true;
    }
  }
  BoundedWaiter(const BoundedWaiter &) = delete;
  BoundedWaiter &operator=(const BoundedWaiter &) = delete;
  ~BoundedWaiter() {
    if (accepted)
      count->fetch_sub(1);
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
  // This charge follows the existing actual work permit through cancellation.
  std::unique_ptr<BoundedWaiter> domain_permit;
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
  struct StatusCall;
  enum class Phase { Starting, Ready, Draining, Drained, Stopped };
  enum class WaitClass { SafetyCandidate, Membership };
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
    std::mutex status_mutex;
    std::set<std::string> status_channels;
    std::size_t lane = 0;
    std::atomic<bool> closing{true};
    // Owned exclusively by the command scheduler; member changes are posted
    // to it.
    std::deque<std::shared_ptr<ExecuteCall>> commands;
    std::atomic<bool> in_flight{false};
    bool uncertain = false;
    std::uint64_t native_call = 0;
    bool native_cancel_requested = false;
    std::shared_ptr<ExecuteCall> active_call;
    std::mutex native_mutex; // native execution/release/stop serialization
    bool native_stopped = false; // guarded by native_mutex
    // Guarded by controller_mutex_. Control is a member incarnation's own
    // resource, independent of every optional status subscriber.
    std::uint64_t control_owner = 0, control_generation = 0, retired_control_generation = 0;
    std::size_t control_work = 0;
    bool control_held = false, control_retiring = false, control_finishing = false;
    bool control_release_waiting = false;
    std::atomic<bool> potential_motion{false};
    Steady::time_point silent_since = Steady::now();
    Steady::time_point control_heartbeat{};
  };
  struct Stream {
    wire::SubscribeStatusResponse sending;
    std::map<std::string, std::map<std::string, std::uint64_t>> seen;
    std::map<std::string, wire::MemberIdentity> identities;
  };
  struct StatusSelection {
    struct SelectedMember {
      wire::MemberIdentity identity;
      std::set<std::string> channels;
    };
    bool all = false;
    bool control = false;
    std::map<std::string, SelectedMember> members;
  };
  struct Job {
    std::function<void()> run;
    std::function<void(bool)> abort;
  };
  class Service final : public wire::RobotAdapterServerService::WithAsyncMethod_SubscribeStatus<wire::RobotAdapterServerService::Service> {
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
          std::move(scope), *request, response, WaitClass::Membership, [this](auto call) {
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
          std::move(scope), *request, response, WaitClass::Membership, [this](auto call) {
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
          std::move(scope), *request, response, WaitClass::SafetyCandidate, [this](auto call) {
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
    grpc::Status InstallController(grpc::ServerContext *context,
                                   const wire::InstallControllerRequest *request,
                                   wire::InstallControllerResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope) return scope.status();
      if (const auto refusal = owner_.requireReady(scope)) return *refusal;
      return scope.application_error(owner_.installController(*context, *request, response));
    }
    grpc::Status RetireController(grpc::ServerContext *context,
                                  const wire::RetireControllerRequest *request,
                                  wire::RetireControllerResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope) return scope.status();
      if (const auto refusal = owner_.requireStarted(scope)) return *refusal;
      BoundedWaiter waiter(owner_.control_blocking_calls_, kMaximumControlWaits);
      if (!waiter.accepted)
        return scope.application_error(grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                                     "controller retirement capacity exhausted"));
      return scope.application_error(owner_.retireController(*context, scope, *request, response));
    }
    grpc::Status RetireStatus(grpc::ServerContext *context,
                              const wire::RetireStatusRequest *request,
                              wire::RetireStatusResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope) return scope.status();
      if (const auto refusal = owner_.requireStarted(scope)) return *refusal;
      BoundedWaiter waiter(owner_.control_blocking_calls_, kMaximumControlWaits);
      if (!waiter.accepted)
        return scope.application_error(grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                                     "status retirement capacity exhausted"));
      return scope.application_error(owner_.retireStatus(scope, *request, response));
    }
    grpc::Status AcquireControl(grpc::ServerContext *context,
                                const wire::AcquireControlRequest *request,
                                wire::AcquireControlResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope) return scope.status();
      if (const auto refusal = owner_.requireReady(scope)) return *refusal;
      return scope.application_error(owner_.acquireControl(scope, *request, response));
    }
    grpc::Status ReleaseControl(grpc::ServerContext *context,
                                const wire::ReleaseControlRequest *request,
                                wire::ReleaseControlResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope) return scope.status();
      if (const auto refusal = owner_.requireStarted(scope)) return *refusal;
      BoundedWaiter waiter(owner_.control_blocking_calls_, kMaximumControlWaits);
      std::unique_ptr<BoundedWaiter> safety;
      if (!waiter.accepted) {
        if (const auto refusal = owner_.requireIndependentRelease(scope, *request))
          return scope.application_error(*refusal);
        safety = std::make_unique<BoundedWaiter>(owner_.safety_blocking_calls_, kMaximumSafetyWaits);
        if (!safety->accepted)
          return scope.application_error(grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                                       "control retirement capacity exhausted"));
      }
      return scope.application_error(owner_.releaseControl(scope, *request, response));
    }
    grpc::Status RenewControl(grpc::ServerContext *context,
                              const wire::RenewControlRequest *request,
                              wire::RenewControlResponse *response) override {
      owner_.control_renewal_request_bytes_.fetch_add(request->ByteSizeLong(), std::memory_order_relaxed);
      auto scope = owner_.admission_.begin(*context);
      if (!scope) {
        owner_.control_renewal_rejections_.fetch_add(1, std::memory_order_relaxed);
        return scope.status();
      }
      if (const auto refusal = owner_.requireReady(scope)) {
        owner_.control_renewal_rejections_.fetch_add(1, std::memory_order_relaxed);
        return *refusal;
      }
      const auto result = owner_.renewControl(scope, *request, response);
      if (result.ok())
        owner_.control_renewal_response_bytes_.fetch_add(response->ByteSizeLong(), std::memory_order_relaxed);
      (result.ok() ? owner_.control_renewals_ : owner_.control_renewal_rejections_)
          .fetch_add(1, std::memory_order_relaxed);
      return scope.application_error(result);
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
      if (!owner_.awaitDrained(scope)) {
        if (owner_.fatal_.load())
          return scope.application_error(grpc::Status(
              grpc::StatusCode::UNKNOWN, "native drain failed; cleanup was not confirmed"));
        return scope.application_error(grpc::Status(
            grpc::StatusCode::DEADLINE_EXCEEDED,
            "the drain continues; the server exits when it is done"));
      }
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
    reference_.set_api_version("v2");
    reference_.set_instance_id(admission_.instance_id());
    reference_.set_profile("grpc.v1");
    reference_.mutable_endpoint()->set_kind("unix");
    reference_.mutable_endpoint()->set_address(options_.socket_path);
    if (!options_.network_bootstrap.empty()) {
      network_bootstrap_ = std::make_unique<xgc2::xrpc::BootstrapInput>(
          xgc2::xrpc::loadBootstrapInput(options_.network_bootstrap));
      const auto &binding = network_bootstrap_->binding();
      if (binding.target_id() != options_.target_id ||
          binding.service() != reference_.service() ||
          binding.api_version() != reference_.api_version() ||
          binding.profile() != reference_.profile() ||
          binding.endpoint().kind != "tls")
        throw std::invalid_argument("network bootstrap does not identify this service");
      network_reference_ = reference_;
      network_reference_->mutable_endpoint()->set_kind(binding.endpoint().kind);
      network_reference_->mutable_endpoint()->set_address(binding.endpoint().address);
    }
    process_owner_ = readProcessOwner();
    ros_node_name_ = RosNodeName(options_.node_name);
    command_jobs_.reserve(kMaximumCommandJobs);
    wake_ = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (wake_ < 0)
      throw std::runtime_error("cannot create the server wake descriptor");
  }
  ~Impl() {
    OPENSSL_cleanse(process_owner_.data(), process_owner_.size());
    if (!controller_grant_.empty())
      OPENSSL_cleanse(&controller_grant_[0], controller_grant_.size());
    ::close(wake_);
  }

  int Run() {
    log("run=" + options_.run_id + " provider=" + options_.provider +
        " node=/" + ros_node_name_ + " master=" + options_.ros_master_uri +
        " socket=" + options_.socket_path);
    try {
      binding_.Init(options_, ros_node_name_);
      // The socket exists before ROS is up; Describe holds readiness callers.
      if (network_bootstrap_)
        server_ = std::make_unique<xgc2::xrpc::GrpcUnixServer>(
            xgc2::xrpc::UnixOptions{options_.socket_path}, admission_,
            std::vector<grpc::Service *>{&service_}, *network_bootstrap_,
            status_queue_);
      else
        server_ = std::make_unique<xgc2::xrpc::GrpcUnixServer>(
            xgc2::xrpc::UnixOptions{options_.socket_path}, admission_,
            std::vector<grpc::Service *>{&service_}, status_queue_);
      startStatusCompletion();
      startWorkers();
      serve();
    } catch (...) {
      workerFailed("startup", std::current_exception());
    }
    return finish();
  }

  // Async-signal-safe.
  void requestSignalStop() noexcept { requestStop(StopReason::Signal); }
  StatusWork statusWork() const noexcept {
    return {status_snapshots_.load(), status_writes_started_.load(),
            status_writes_joined_.load(), status_protobuf_bytes_.load(),
            status_subscribers_joined_.load(), status_subscribers_.load(),
            control_renewals_.load(), control_renewal_rejections_.load(),
            control_renewal_request_bytes_.load(), control_renewal_response_bytes_.load(),
            blocking_calls_.load(), control_blocking_calls_.load(),
            safety_blocking_calls_.load(), membership_blocking_calls_.load()};
  }

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
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      registration_alive_ = true;
    }
    registration_ = std::thread([this] {
      try {
        registrationLoop();
      } catch (...) {
        workerFailed("registration", std::current_exception());
      }
      std::deque<Job> abandoned;
      {
        std::lock_guard<std::mutex> lock(jobs_mutex_);
        registration_alive_ = false;
        abandoned.swap(registration_jobs_);
      }
      for (auto &job : abandoned) abortJob(job, false);
    });
    scheduler_ = std::thread([this] {
      try {
        commandLoop();
      } catch (...) {
        workerFailed("commands", std::current_exception());
      }
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        scheduler_alive_.store(false, std::memory_order_release);
      }
      idle_changed_.notify_all();
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
    {
      std::map<StatusCall *, std::shared_ptr<StatusCall>> calls;
      {
        std::lock_guard<std::mutex> lock(controller_mutex_);
        markControlDownLocked();
        calls = controller_calls_;
      }
      for (const auto &entry : calls) entry.second->context.TryCancel();
    }
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
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      stopping_ = true;
      command_ready_.notify_all();
    }
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
    stopStatusCompletion();
    server_.reset();
    status_queue_.reset();
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
  void stopMember(const std::shared_ptr<Member> &member) {
    {
      std::lock_guard<std::mutex> lock(member->native_mutex);
      if (!member->native_stopped && member->native) {
        member->native->Stop();
        member->native_stopped = true;
      }
    }
    std::lock_guard<std::mutex> lock(controller_mutex_);
    member->potential_motion = false;
    member->control_held = false;
    member->retired_control_generation = member->control_generation;
    control_members_.erase(member.get());
    controller_changed_.notify_all();
  }
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
    auto done = std::make_shared<std::promise<bool>>();
    auto finished = done->get_future();
    if (!postRegistration({[this, done] {
                             closeAllMembers();
                             done->set_value(true);
                           },
                           [done](bool) { done->set_value(false); }},
                          true)) {
      workerFailed("native drain", std::make_exception_ptr(std::runtime_error("member cleanup was not admitted")));
      setPhase(Phase::Stopped);
      return;
    }
    if (finished.wait_for(timings_.member_drain) != std::future_status::ready) {
      log("members were not all stopped within the drain budget; retaining their owner");
      // A caller can time out, but the existing owner remains responsible for
      // the actual native Stop and flush. A timer is not a successful join.
      finished.wait();
    }
    if (!finished.get() || fatal_.load()) {
      setPhase(Phase::Stopped);
      return;
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
    try {
      fenceCommands(members);
    } catch (...) {
      workerFailed("native fence", std::current_exception());
    }
    for (auto &member : members) {
      try {
        stopMember(member);
      } catch (...) {
        workerFailed("native stop", std::current_exception());
      }
    }
    // The final commands must leave the process before the robots, and with
    // them their publishers, are destroyed.
    if (!members.empty() && !binding_.FlushPublications(timings_.publication_flush))
      workerFailed("publication flush", std::make_exception_ptr(
          std::runtime_error("native publication flush did not complete")));
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
    while (phase_ != Phase::Drained && phase_ != Phase::Stopped) {
      if (scope.cancelled() || Steady::now() >= scope.deadline())
        return false;
      phase_changed_.wait_until(
          lock, std::min(scope.deadline(), Steady::now() + Milliseconds(100)));
    }
    return phase_ == Phase::Drained && !fatal_.load();
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
    if (network_reference_) {
      const auto &ref = *network_reference_;
      facts["direct_service_ref"] = {
          {"target_id", ref.target_id()}, {"service", ref.service()},
          {"api_version", ref.api_version()}, {"instance_id", ref.instance_id()},
          {"profile", ref.profile()},
          {"endpoint", {{"kind", ref.endpoint().kind()},
                        {"address", ref.endpoint().address()}}}};
    }
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
      const auto health = member->closing ? std::optional<NativeHealth>{}
                                          : member->native->DescribeHealth();
      auto &robot = facts["robots"][member->identity.robot_id()];
      robot["state"] = health && health->online ? "online" : "waiting";
      robot["reason"] = health && health->online ? ""
                         : !health || health->summary.empty() ? "waiting for robot telemetry"
                                                              : health->summary;
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
      if (!registration_alive_ || management_stopping_ || shutdown_ ||
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
    ++command_revision_;
    command_ready_.notify_one();
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
    ++command_revision_;
    command_ready_.notify_one();
  }
  void notifyCommandMemberChange() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    ++command_revision_;
    command_ready_.notify_one();
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
          std::uint64_t command_revision;
          {
            std::lock_guard<std::mutex> lock(command_mutex_);
            jobs.swap(command_jobs_);
            fence = std::move(fence_job_);
            fence_job_ = {};
            command_revision = command_revision_;
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
              std::vector<std::shared_ptr<Member>> previous;
              previous.swap(members);
              members.reserve(members_.size());
              for (auto &entry : members_)
                members.push_back(entry.second);
              // A removed member remains owned by this scheduler until its
              // cancelled native completion has actually returned.
              for (auto &member : previous)
                if (member->closing && member->in_flight &&
                    std::find(members.begin(), members.end(), member) == members.end())
                  members.push_back(std::move(member));
              revision = members_revision_;
            }
          }
          members.erase(std::remove_if(members.begin(), members.end(), [](const auto &member) {
                          return member->closing && !member->in_flight;
                        }),
                        members.end());
          const bool periodic = Steady::now() >= next_periodic;
          if (periodic)
            next_periodic = Steady::now() + Milliseconds(100);
          bool active_native_io = false;
          bool ready_command = false;
          for (auto &member : members) {
            if (member->closing) {
              if (member->native_call) {
                cancelNative(member, *io);
                active_native_io = true;
              }
              continue;
            }
            if (member->native_call && member->active_call &&
                (member->active_call->cancelled ||
                 member->active_call->work.cancelled()))
              cancelNative(member, *io);
            if (periodic && !shutdown_) {
              std::unique_lock<std::mutex> lock(member->native_mutex, std::try_to_lock);
              if (lock.owns_lock() && !member->closing) {
                if (member->potential_motion.load(std::memory_order_relaxed)) {
                  const auto silent_for = memberSilence(member);
                  if (silent_for > Milliseconds(0) && member->native->CoreSilent(silent_for)) {
                    std::lock_guard<std::mutex> owner(controller_mutex_);
                    if (member->control_work == 0) member->potential_motion = false;
                  }
                }
                member->native->Periodic();
              }
            }
            bool native_busy = false;
            if (!shutdown_ && !member->in_flight && !member->commands.empty())
              native_busy = !dispatch(member, *io);
            active_native_io = active_native_io || member->native_call != 0;
            ready_command = ready_command ||
                            (!native_busy && !shutdown_ && !member->in_flight && !member->commands.empty());
          }
          if (ready_command)
            continue;
          if (active_native_io) {
            io->Pump(Milliseconds(2));
          } else {
            // Queue and member publication wake the scheduler. Only a live
            // member's existing safety/periodic deadline needs a timer.
            std::unique_lock<std::mutex> lock(command_mutex_);
            const auto changed = [&] {
              return stopping_ || command_revision_ != command_revision;
            };
            if (members.empty())
              command_ready_.wait(lock, changed);
            else
              command_ready_.wait_until(lock, next_periodic, changed);
          }
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
          cancelNative(member, *io);
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
  // Runs on the scheduler; cancellation is requested once per native call.
  void cancelNative(const std::shared_ptr<Member> &member, CommandIo &io) {
    if (!member->native_call || member->native_cancel_requested)
      return;
    member->native_cancel_requested = true;
    io.Cancel(member->native_call);
  }
  // Runs on the scheduler: admits a command to its member's queue.
  void enqueue(const std::shared_ptr<ExecuteCall> &call) {
    std::shared_ptr<Member> member;
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      const auto found = members_.find(call->request.identity().robot_id());
      if (found != members_.end() && sameIdentity(found->second->identity, call->request.identity()))
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
  bool dispatch(const std::shared_ptr<Member> &member, CommandIo &io) {
    std::unique_lock<std::mutex> native(member->native_mutex, std::try_to_lock);
    if (!native.owns_lock())
      return false;
    auto call = std::move(member->commands.front());
    member->commands.pop_front();
    auto &request = call->request.operation();
    if (commands_closed_) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_CANCELLED,
                                       "server-draining",
                                       "not executed: server is draining"));
      return true;
    }
    // An unknown result blocks further commands of this robot until it is
    // registered again, except the ones that make it safe: the operator must
    // still be able to release a held motion intent.
    if (member->uncertain && !member->native->IsSafetyOperation(request)) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_REJECTED,
                                       "previous-result-unknown",
                                       "not executed: previous native result is unknown"));
      return true;
    }
    if (call->cancelled || call->work.cancelled()) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_CANCELLED,
                                       "operation-cancelled",
                                       "not executed: call cancelled"));
      return true;
    }
    if (request.context().deadline().deadline_unix_nanos() <=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count()) {
      finishExecute(call, CommandError(request, operation::ERROR_CLASS_DEADLINE,
                                       "deadline-exceeded",
                                       "not executed: operation deadline elapsed"));
      return true;
    }
    const bool motion = member->native->RequiresControl(request);
    const bool release = member->native->IsControlRelease(request);
    bool control_work = false;
    {
      std::lock_guard<std::mutex> owner(controller_mutex_);
      const auto watchdog = member->native->ControlSilenceLimit();
      const bool stale_heartbeat = watchdog > Milliseconds(0) &&
          Steady::now() - member->control_heartbeat >= watchdog;
      if (motion && (!controllerGrantLocked(call->request.controller_grant()) ||
                     !control_status_.active || !control_status_.ready || control_status_.retiring ||
                     !member->control_held || member->control_retiring || member->closing ||
                     stale_heartbeat ||
                     member->control_owner != controller_generation_ ||
                     member->control_generation != call->request.control_generation())) {
        finishExecute(call, CommandError(request, operation::ERROR_CLASS_REJECTED,
                                         "control-unavailable",
                                         "not executed: admitted member control is required"));
        return true;
      }
      if (release && member->control_owner == controller_generation_ && controller_generation_ &&
          (member->control_finishing || member->control_retiring)) {
        finishExecute(call, CommandError(request, operation::ERROR_CLASS_REJECTED,
                                         "control-retiring",
                                         "not executed: native control is retiring"));
        return true;
      }
      control_work = motion || (release && member->control_owner == controller_generation_ &&
                                controller_generation_ && member->control_held);
      if (control_work) {
        ++controller_work_;
        ++member->control_work;
        if (motion) member->potential_motion = true;
      }
    }
    member->in_flight = true;
    member->active_call = call;
    member->native_cancel_requested = false;
    member->native_call = member->native->Execute(
        request, io, [this, member, call, control_work, release](operation::OperationEvent result) {
          member->native_call = 0;
          member->active_call.reset();
          if (result.phase() == operation::OPERATION_PHASE_UNCERTAIN)
            member->uncertain = true;
          const bool succeeded = result.phase() == operation::OPERATION_PHASE_SUCCEEDED && !result.has_error();
          finishExecute(call, std::move(result));
          if (control_work) {
            std::lock_guard<std::mutex> owner(controller_mutex_);
            if (release && succeeded) member->potential_motion = false;
            --member->control_work;
            --controller_work_;
            controller_changed_.notify_all();
          }
          {
            std::lock_guard<std::mutex> lock(command_mutex_);
            member->in_flight = false;
          }
          idle_changed_.notify_all();
        });
    return true;
  }
  template <class Request, class Response, class Submit>
  grpc::Status invoke(xgc2::xrpc::GrpcCallScope scope, const Request &request,
                      Response *response, WaitClass lane, Submit submit) {
    auto waiter = std::make_unique<BoundedWaiter>(blocking_calls_, kMaximumBlockingCalls);
    if (!waiter->accepted && lane == WaitClass::Membership)
      waiter = std::make_unique<BoundedWaiter>(membership_blocking_calls_, kMaximumMembershipWaits);
    if (!waiter->accepted && lane == WaitClass::SafetyCandidate) {
      if constexpr (std::is_same_v<Request, wire::ExecuteRequest>) {
        if (isSafetyRequest(request))
          waiter = std::make_unique<BoundedWaiter>(safety_blocking_calls_, kMaximumSafetyWaits);
      }
    }
    if (!waiter->accepted)
      return scope.application_error(
          grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                       "native blocking call capacity exhausted"));
    auto call = std::make_shared<Unary<Request, Response>>();
    call->domain_permit = std::move(waiter);
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
  static bool sameIdentity(const wire::MemberIdentity &a,
                           const wire::MemberIdentity &b) {
    return a.target_id() == b.target_id() && a.run_id() == b.run_id() &&
           a.robot_id() == b.robot_id() &&
           a.connection_epoch() == b.connection_epoch();
  }
  grpc::Status validateStatusSelection(const wire::SubscribeStatusRequest &request,
                                       StatusSelection *selection) const {
    if (request.selection_case() == wire::SubscribeStatusRequest::kControl) {
      if (!request.control() || !request.has_owner())
        return {grpc::StatusCode::INVALID_ARGUMENT, "control status needs an owner"};
      selection->control = true;
      return grpc::Status::OK;
    }
    if (request.selection_case() == wire::SubscribeStatusRequest::kAll) {
      if (!request.all())
        return {grpc::StatusCode::INVALID_ARGUMENT, "all must be true"};
      selection->all = true;
      return grpc::Status::OK;
    }
    if (request.selection_case() != wire::SubscribeStatusRequest::kSelected ||
        request.selected().members().empty() ||
        request.selected().members_size() > static_cast<int>(kMaximumBatchMembers))
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid status member selection"};
    for (const auto &requested : request.selected().members()) {
      const auto &identity = requested.identity();
      if (!requested.has_identity() || !validIdentity(identity) ||
          identity.target_id() != options_.target_id ||
          identity.run_id() != options_.run_id ||
          requested.channel_ids().empty() ||
          requested.channel_ids_size() > static_cast<int>(kMaximumChannelsPerRobot))
        return {grpc::StatusCode::INVALID_ARGUMENT, "invalid status member or channels"};
      StatusSelection::SelectedMember member;
      member.identity = identity;
      for (const auto &channel : requested.channel_ids())
        if (channel.empty() || !member.channels.insert(channel).second)
          return {grpc::StatusCode::INVALID_ARGUMENT, "invalid or duplicate status channel"};
      if (!selection->members.emplace(identity.robot_id(), std::move(member)).second)
        return {grpc::StatusCode::INVALID_ARGUMENT, "duplicate status member"};
    }
    return grpc::Status::OK;
  }
  struct Subscriber {
    explicit Subscriber(StatusSelection value) : selection(std::move(value)) {}
    const StatusSelection selection;
    std::condition_variable changed;
    std::uint64_t revision = 1;
    std::uint64_t delivered = 0;
  };
  void reconcileMemberStatus(const std::shared_ptr<Member> &member) {
    std::lock_guard<std::mutex> owner(member->status_mutex);
    if (member->closing)
      return;
    std::set<std::string> wanted;
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      for (const auto &subscriber : subscribers_) {
        if (subscriber->selection.all) {
          for (const auto &channel : member->configuration.channels())
            if (channel.enabled())
              wanted.insert(channel.channel_id());
          continue;
        }
        const auto found = subscriber->selection.members.find(member->identity.robot_id());
        if (found == subscriber->selection.members.end() ||
            !sameIdentity(found->second.identity, member->identity))
          continue;
        for (const auto &channel : member->configuration.channels())
          if (channel.enabled() && found->second.channels.count(channel.channel_id()))
            wanted.insert(channel.channel_id());
      }
    }
    if (wanted == member->status_channels)
      return;
    {
      std::lock_guard<std::mutex> lock(member->state_mutex);
      for (const auto &channel : wanted)
        if (!member->status_channels.count(channel))
          member->latest.erase(channel);
    }
    member->native->SetStatusDemand(wanted);
    {
      std::lock_guard<std::mutex> lock(member->state_mutex);
      for (const auto &channel : member->status_channels)
        if (!wanted.count(channel))
          member->latest.erase(channel);
    }
    member->status_channels = std::move(wanted);
  }
  void reconcileSubscriberMembers(const StatusSelection &selection) {
    std::vector<std::shared_ptr<Member>> selected;
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      if (selection.all) {
        for (const auto &member : members_)
          selected.push_back(member.second);
      } else {
        for (const auto &entry : selection.members) {
          const auto member = members_.find(entry.first);
          if (member != members_.end() && sameIdentity(member->second->identity, entry.second.identity))
            selected.push_back(member->second);
        }
      }
    }
    for (const auto &member : selected)
      reconcileMemberStatus(member);
  }
  struct StatusTag {
    enum class Kind { Accept, Done, IO, Join } kind;
    StatusCall *call;
  };
  struct StatusCall {
    explicit StatusCall(Impl &server)
        : owner(server), writer(&context), accept{StatusTag::Kind::Accept, this},
          done{StatusTag::Kind::Done, this}, io{StatusTag::Kind::IO, this},
          join{StatusTag::Kind::Join, this} {}
    Impl &owner;
    grpc::ServerContext context;
    wire::SubscribeStatusRequest request;
    grpc::ServerAsyncWriter<wire::SubscribeStatusResponse> writer;
    StatusTag accept, done, io, join;
    grpc::Alarm joined;
    std::optional<xgc2::xrpc::GrpcCallScope> scope;
    StatusSelection selection;
    grpc::Status result;
    std::atomic<bool> cancelled{false};
    bool counted = false;
    bool optional = false;
    bool owned = false;
    bool control = false;
    std::shared_ptr<Subscriber> subscriber; // guarded by subscribers_mutex_
    std::mutex io_mutex;
    std::condition_variable io_changed;
    bool io_done = false, io_ok = false, done_received = false;
    bool direct_finish = false;
    std::thread worker;

    bool metadata() {
      std::unique_lock<std::mutex> lock(io_mutex);
      io_done = false;
      writer.SendInitialMetadata(&io);
      io_changed.wait(lock, [this] { return io_done; });
      return io_ok;
    }
    bool write(const wire::SubscribeStatusResponse &frame) {
      std::unique_lock<std::mutex> lock(io_mutex);
      if (cancelled.load(std::memory_order_acquire))
        return false;
      io_done = false;
      owner.status_writes_started_.fetch_add(1, std::memory_order_relaxed);
      owner.status_protobuf_bytes_.fetch_add(frame.ByteSizeLong(), std::memory_order_relaxed);
      writer.Write(frame, &io);
      io_changed.wait(lock, [this] { return io_done; });
      owner.status_writes_joined_.fetch_add(1, std::memory_order_relaxed);
      return io_ok;
    }
    void finish(const grpc::Status &status) {
      std::unique_lock<std::mutex> lock(io_mutex);
      io_done = false;
      writer.Finish(status, &io);
      io_changed.wait(lock, [this] { return io_done; });
      io_changed.wait(lock, [this] { return done_received; });
    }
  };

  struct OwnedStatusSlot {
    StatusCall *active = nullptr;
    std::uint64_t generation = 0, retired_generation = 0;
    std::string open_id, retired_open_id;
    bool retiring = false, ready = false;
  };

  bool controllerCredentialLocked(const std::string &grant) const {
    return controller_generation_ &&
           secretEqual(grant, reinterpret_cast<const unsigned char *>(controller_grant_.data()), 32);
  }
  bool controllerGrantLocked(const std::string &grant) const {
    return !controller_retiring_ && controllerCredentialLocked(grant);
  }
  static bool validStatusOwner(const wire::StatusOwner &owner) {
    return owner.controller_grant().size() == 32 && owner.generation() &&
           owner.open_id().size() == 16 &&
           ((owner.predecessor_generation() == 0 && owner.predecessor_open_id().empty()) ||
            (owner.predecessor_generation() != 0 && owner.predecessor_open_id().size() == 16));
  }
  static bool matchesPredecessor(const wire::StatusOwner &owner,
                                 const OwnedStatusSlot &slot) {
    return owner.predecessor_generation() == slot.retired_generation &&
           owner.predecessor_open_id() == slot.retired_open_id;
  }
  void markControlDownLocked() {
    control_status_.ready = false;
  }
  std::shared_ptr<Member> selectedMember(const wire::MemberIdentity &identity) {
    std::lock_guard<std::mutex> lock(table_mutex_);
    const auto found = members_.find(identity.robot_id());
    if (found == members_.end() || !sameIdentity(found->second->identity, identity)) return {};
    return found->second;
  }
  bool isSafetyRequest(const wire::ExecuteRequest &request) {
    if (!request.has_identity() || !request.has_operation()) return false;
    const auto member = selectedMember(request.identity());
    return member && !member->closing && member->native &&
           member->native->IsSafetyOperation(request.operation());
  }
  std::optional<grpc::Status>
  requireIndependentRelease(const xgc2::xrpc::GrpcCallScope &scope,
                            const wire::ReleaseControlRequest &request) {
    if (!request.has_identity() || !validIdentity(request.identity()) || !request.generation())
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "invalid control retirement");
    if (scope.cancelled())
      return grpc::Status(grpc::StatusCode::CANCELLED, "control retirement was not admitted");
    {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      if (!controllerCredentialLocked(request.controller_grant()))
        return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable");
    }
    const auto member = selectedMember(request.identity());
    if (!member)
      return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "member is not current");
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (!controllerCredentialLocked(request.controller_grant()) || member->control_owner != controller_generation_)
      return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable");
    if (!member->control_held && request.generation() == member->retired_control_generation)
      return std::nullopt;
    if (!member->control_held || request.generation() != member->control_generation)
      return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "member control does not match");
    if (member->control_release_waiting || member->control_finishing)
      return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "control retirement is already joining");
    return std::nullopt;
  }
  grpc::Status acquireControl(const xgc2::xrpc::GrpcCallScope &scope,
                              const wire::AcquireControlRequest &request,
                              wire::AcquireControlResponse *response) {
    if (!request.has_identity() || !validIdentity(request.identity()) || !request.generation())
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid control admission"};
    {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      if (!controllerGrantLocked(request.controller_grant()))
        return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
      if (scope.cancelled() || !control_status_.active || !control_status_.ready || control_status_.retiring)
        return {grpc::StatusCode::FAILED_PRECONDITION, "required control stream is unavailable"};
    }
    const auto member = selectedMember(request.identity());
    if (!member) return {grpc::StatusCode::FAILED_PRECONDITION, "member is not current"};
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (!controllerGrantLocked(request.controller_grant()))
      return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
    if (scope.cancelled() || member->closing || !control_status_.active || !control_status_.ready ||
        control_status_.retiring)
      return {grpc::StatusCode::FAILED_PRECONDITION, "required control stream is unavailable"};
    const auto silence = member->native->ControlSilenceLimit();
    if (silence > Milliseconds(0) && silence < 3 * kMinimumControlHeartbeat)
      return {grpc::StatusCode::FAILED_PRECONDITION, "control watchdog is shorter than the scheduler budget"};
    if (member->control_owner != controller_generation_) {
      if (member->control_work || member->control_held || member->potential_motion)
        return {grpc::StatusCode::FAILED_PRECONDITION, "previous control owner is not retired"};
      member->control_owner = controller_generation_;
      member->control_generation = member->retired_control_generation = 0;
      member->control_retiring = false;
    }
    if (member->control_held) {
      if (member->control_retiring || request.generation() != member->control_generation)
        return {grpc::StatusCode::FAILED_PRECONDITION, "member control is already held"};
    } else {
      if (request.generation() <= member->retired_control_generation)
        return {grpc::StatusCode::FAILED_PRECONDITION, "member control generation is retired"};
      if (control_members_.size() >= kMaximumRobots && !control_members_.count(member.get()))
        return {grpc::StatusCode::RESOURCE_EXHAUSTED, "member control capacity exhausted"};
      member->control_generation = request.generation();
      member->control_held = true;
      member->control_retiring = false;
      member->control_heartbeat = Steady::now();
      control_members_[member.get()] = member;
    }
    *response->mutable_identity() = member->identity;
    response->set_generation(member->control_generation);
    response->set_heartbeat_interval_ms(silence <= Milliseconds(0) ? 0 :
        static_cast<std::uint32_t>(std::min(silence / 3, kMaximumControlHeartbeat).count()));
    return grpc::Status::OK;
  }
  grpc::Status renewControl(const xgc2::xrpc::GrpcCallScope &scope,
                            const wire::RenewControlRequest &request,
                            wire::RenewControlResponse *response) {
    if (!request.status_generation() || request.status_open_id().size() != 16 ||
        request.members().empty() || request.members_size() > static_cast<int>(kMaximumRobots))
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid control renewal"};
    {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      if (!controllerGrantLocked(request.controller_grant()))
        return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
      if (scope.cancelled() || !control_status_.active || !control_status_.ready || control_status_.retiring ||
          control_status_.generation != request.status_generation() || control_status_.open_id != request.status_open_id())
        return {grpc::StatusCode::FAILED_PRECONDITION, "required control stream is unavailable"};
      const auto now = Steady::now();
      if (last_control_renewal_ != Steady::time_point{} &&
          now - last_control_renewal_ < kMinimumControlRenewInterval)
        return {grpc::StatusCode::RESOURCE_EXHAUSTED, "control renewal is faster than its scheduler policy"};
      // Reserve the per-owner rate edge before looking up any members. A
      // canceled or malformed admitted batch still consumes this edge.
      last_control_renewal_ = now;
    }
    std::vector<std::shared_ptr<Member>> members;
    std::set<std::string> ids;
    members.reserve(request.members_size());
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      for (const auto &renewal : request.members()) {
        if (!renewal.has_identity() || !validIdentity(renewal.identity()) || !renewal.generation() ||
            !ids.insert(renewal.identity().robot_id()).second)
          return {grpc::StatusCode::INVALID_ARGUMENT, "invalid control renewal member"};
        const auto found = members_.find(renewal.identity().robot_id());
        members.push_back(found != members_.end() && sameIdentity(found->second->identity, renewal.identity())
                              ? found->second : nullptr);
      }
    }
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (!controllerGrantLocked(request.controller_grant()))
      return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
    if (scope.cancelled() || !control_status_.active || !control_status_.ready || control_status_.retiring ||
        control_status_.generation != request.status_generation() || control_status_.open_id != request.status_open_id())
      return {grpc::StatusCode::FAILED_PRECONDITION, "required control stream is unavailable"};
    const auto now = Steady::now();
    for (int index = 0; index < request.members_size(); ++index) {
      const auto &member = members[static_cast<std::size_t>(index)];
      const auto &renewal = request.members(index);
      const bool current = member && !member->closing && member->control_owner == controller_generation_ &&
                           member->control_held && !member->control_retiring &&
                           member->control_generation == renewal.generation() &&
                           member->native->ControlSilenceLimit() > Milliseconds(0);
      auto *result = response->add_members();
      *result->mutable_identity() = renewal.identity();
      result->set_generation(renewal.generation());
      result->set_renewed(current);
      if (current) member->control_heartbeat = now;
    }
    return grpc::Status::OK;
  }
  bool retireNativeControl(const std::shared_ptr<Member> &member, std::string *error) {
    auto wake = [this] {
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        ++command_revision_;
      }
      command_ready_.notify_one();
    };
    try {
      bool result;
      {
        std::lock_guard<std::mutex> lock(member->native_mutex);
        result = member->native_stopped || member->native->RetireControl(error);
      }
      wake();
      return result;
    } catch (...) {
      wake();
      throw;
    }
  }
  grpc::Status releaseControl(const xgc2::xrpc::GrpcCallScope &scope,
                              const wire::ReleaseControlRequest &request,
                              wire::ReleaseControlResponse *response) {
    if (!request.has_identity() || !validIdentity(request.identity()) || !request.generation())
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid control retirement"};
    {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      if (!controllerCredentialLocked(request.controller_grant()))
        return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
    }
    const auto member = selectedMember(request.identity());
    if (!member) return {grpc::StatusCode::FAILED_PRECONDITION, "member is not current"};
    std::unique_lock<std::mutex> lock(controller_mutex_);
    if (!controllerCredentialLocked(request.controller_grant()) || member->control_owner != controller_generation_)
      return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
    if (!member->control_held && request.generation() == member->retired_control_generation) {
      *response->mutable_identity() = member->identity;
      response->set_generation(request.generation());
      response->set_joined(true);
      return grpc::Status::OK;
    }
    if (!member->control_held || request.generation() != member->control_generation)
      return {grpc::StatusCode::FAILED_PRECONDITION, "member control does not match"};
    if (scope.cancelled())
      return {grpc::StatusCode::CANCELLED, "control retirement was not admitted"};
    // Concurrent retries for the same member cannot occupy the independent
    // safety lane. A later exact retry can continue after this waiter exits.
    if (member->control_release_waiting || member->control_finishing)
      return {grpc::StatusCode::FAILED_PRECONDITION, "control retirement is already joining"};
    member->control_retiring = true;
    member->control_release_waiting = true;
    ++controller_work_;
    struct ReleaseWait {
      Impl &owner;
      Member &member;
      std::unique_lock<std::mutex> &lock;
      ~ReleaseWait() {
        if (!lock.owns_lock()) lock.lock();
        member.control_release_waiting = false;
        --owner.controller_work_;
        owner.controller_changed_.notify_all();
      }
    } waiting{*this, *member, lock};
    member->silent_since = Steady::now();
    if (!controller_changed_.wait_until(lock, scope.deadline(), [&] {
          return !member->control_held ||
                 (member->control_work == 0 && !member->control_finishing);
        }))
      return {grpc::StatusCode::DEADLINE_EXCEEDED, "member control is still joining"};
    if (!member->control_held) {
      if (request.generation() != member->retired_control_generation)
        return {grpc::StatusCode::FAILED_PRECONDITION, "member control changed"};
    } else {
      member->control_finishing = true;
      const bool release = member->potential_motion;
      lock.unlock();
      std::string error;
      bool success = !release;
      try { if (release) success = retireNativeControl(member, &error); }
      catch (...) { success = false; }
      lock.lock();
      member->control_finishing = false;
      if (!success) {
        controller_changed_.notify_all();
        return {grpc::StatusCode::UNKNOWN, "native control release did not complete"};
      }
      member->potential_motion = false;
      member->control_held = false;
      member->retired_control_generation = member->control_generation;
      member->control_retiring = false;
      control_members_.erase(member.get());
      controller_changed_.notify_all();
    }
    *response->mutable_identity() = member->identity;
    response->set_generation(request.generation());
    response->set_joined(true);
    return grpc::Status::OK;
  }
  Milliseconds memberSilence(const std::shared_ptr<Member> &member) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (!member->potential_motion) return Milliseconds(0);
    const auto now = Steady::now();
    if (member->control_held && member->control_owner == controller_generation_ &&
        !member->control_retiring && !controller_retiring_) {
      const auto limit = member->native->ControlSilenceLimit();
      if (limit <= Milliseconds(0)) return Milliseconds(0);
      const auto age = std::chrono::duration_cast<Milliseconds>(now - member->control_heartbeat);
      return age < limit ? Milliseconds(0) : age;
    }
    return std::chrono::duration_cast<Milliseconds>(now - member->silent_since);
  }
  grpc::Status admitOwnedStatus(StatusCall *call) {
    if (!call->request.has_owner())
      return call->control ? grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "control owner is required")
                           : grpc::Status::OK;
    const auto &owner = call->request.owner();
    if (!validStatusOwner(owner))
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid status owner"};
    std::shared_ptr<StatusCall> retained;
    {
      std::lock_guard<std::mutex> lock(status_calls_mutex_);
      const auto found = status_calls_.find(call);
      if (found == status_calls_.end())
        return {grpc::StatusCode::INTERNAL, "status admission ended"};
      retained = found->second;
    }
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (!controllerGrantLocked(owner.controller_grant()))
      return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
    auto &slot = call->control ? control_status_ : optional_status_;
    if (slot.active)
      return {grpc::StatusCode::FAILED_PRECONDITION, "prior status owner is not joined"};
    if (!matchesPredecessor(owner, slot) || owner.generation() <= slot.retired_generation)
      return {grpc::StatusCode::FAILED_PRECONDITION, "status predecessor is retired or mismatched"};
    slot.active = call;
    slot.generation = owner.generation();
    slot.open_id = owner.open_id();
    slot.retiring = false;
    slot.ready = false;
    call->owned = true;
    controller_calls_.emplace(call, std::move(retained));
    return grpc::Status::OK;
  }
  void releaseStatus(StatusCall *call) {
    if (call->counted) {
      status_subscribers_.fetch_sub(1);
      if (call->optional) optional_status_subscribers_.fetch_sub(1);
      status_subscribers_joined_.fetch_add(1);
      call->counted = false;
    }
    if (call->owned) {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      auto &slot = call->control ? control_status_ : optional_status_;
      if (slot.active == call) {
        if (call->control) markControlDownLocked();
        slot.retired_generation = slot.generation;
        slot.retired_open_id = std::move(slot.open_id);
        slot.active = nullptr;
        slot.generation = 0;
        slot.retiring = false;
        slot.ready = false;
      }
      controller_calls_.erase(call);
      call->owned = false;
      controller_changed_.notify_all();
    }
  }
  grpc::Status retireStatus(const xgc2::xrpc::GrpcCallScope &scope,
                            const wire::RetireStatusRequest &request,
                            wire::RetireStatusResponse *response) {
    if (!request.has_owner() || !validStatusOwner(request.owner()))
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid status retirement"};
    const auto &owner = request.owner();
    std::unique_lock<std::mutex> lock(controller_mutex_);
    if (!controllerGrantLocked(owner.controller_grant()))
      return {grpc::StatusCode::PERMISSION_DENIED, "controller grant is unavailable"};
    auto &slot = request.control() ? control_status_ : optional_status_;
    if (slot.active && slot.generation == owner.generation() && slot.open_id == owner.open_id() &&
        matchesPredecessor(owner, slot)) {
      slot.retiring = true;
      if (request.control()) markControlDownLocked();
      else slot.ready = false;
      auto call = controller_calls_.at(slot.active);
      lock.unlock();
      call->context.TryCancel();
      lock.lock();
      if (!controller_changed_.wait_until(lock, scope.deadline(), [&] {
            return !slot.active || slot.generation != owner.generation() || slot.open_id != owner.open_id();
          }))
        return {grpc::StatusCode::DEADLINE_EXCEEDED, "status retirement is still joining"};
    } else if (!slot.active && owner.generation() > slot.retired_generation && matchesPredecessor(owner, slot)) {
      // Fence an exact unadmitted/unknown open before a late Accept can run.
      slot.retired_generation = owner.generation();
      slot.retired_open_id = owner.open_id();
    }
    if (slot.retired_generation != owner.generation() || slot.retired_open_id != owner.open_id())
      return {grpc::StatusCode::FAILED_PRECONDITION, "status owner does not match"};
    response->set_generation(owner.generation());
    response->set_open_id(owner.open_id());
    response->set_joined(true);
    return grpc::Status::OK;
  }

  bool ownerAuthorized(const grpc::ServerContext &context,
                       const std::string &presented) const {
    const auto peer = context.peer();
    // XRPC hands accepted Unix sockets to the pinned native gRPC server by
    // descriptor; that transport reports "fd:N", whereas its separate TLS
    // listener reports an IP peer. Neither form is caller-supplied metadata.
    const bool unix_fd = peer.size() > 3 && peer.compare(0, 3, "fd:") == 0 &&
                         peer.find_first_not_of("0123456789", 3) == std::string::npos;
    return unix_fd &&
           secretEqual(presented, process_owner_.data(), process_owner_.size());
  }
  grpc::Status installController(const grpc::ServerContext &context,
                                 const wire::InstallControllerRequest &request,
                                 wire::InstallControllerResponse *response) {
    if (!ownerAuthorized(context, request.process_owner()))
      return {grpc::StatusCode::PERMISSION_DENIED, "private process owner is required"};
    const auto &binding = request.binding();
    const bool local = binding.agent_id().empty() && binding.agent_epoch_id().empty();
    const bool remote = ValidReferenceId(binding.agent_id()) &&
                        ValidReferenceId(binding.agent_epoch_id()) &&
                        binding.agent_id() == options_.target_id;
    if (!request.has_binding() || binding.provider_instance_id() != reference_.instance_id() ||
        !ValidReferenceId(binding.core_id()) || !ValidReferenceId(binding.core_boot_id()) ||
        (!local && !remote) || !request.generation() || request.open_id().size() != 16 ||
        request.controller_grant().size() != 32)
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid controller installation"};
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (controller_generation_) {
      if (controller_retiring_ || request.generation() != controller_generation_ ||
          request.open_id() != controller_open_id_ ||
          !secretEqual(request.controller_grant(),
                       reinterpret_cast<const unsigned char *>(controller_grant_.data()), 32) ||
          !google::protobuf::util::MessageDifferencer::Equals(binding, controller_binding_))
        return {grpc::StatusCode::FAILED_PRECONDITION, "controller owner already exists"};
    } else {
      if (request.generation() <= retired_controller_generation_)
        return {grpc::StatusCode::FAILED_PRECONDITION, "controller generation is retired"};
      controller_generation_ = request.generation();
      controller_open_id_ = request.open_id();
      controller_grant_ = request.controller_grant();
      controller_binding_ = binding;
    }
    response->set_generation(controller_generation_);
    response->set_open_id(controller_open_id_);
    return grpc::Status::OK;
  }
  grpc::Status retireController(const grpc::ServerContext &context,
                                const xgc2::xrpc::GrpcCallScope &scope,
                                const wire::RetireControllerRequest &request,
                                wire::RetireControllerResponse *response) {
    if (!ownerAuthorized(context, request.process_owner()))
      return {grpc::StatusCode::PERMISSION_DENIED, "private process owner is required"};
    if (!request.generation() || request.open_id().size() != 16)
      return {grpc::StatusCode::INVALID_ARGUMENT, "invalid controller retirement"};
    std::unique_lock<std::mutex> lock(controller_mutex_);
    if (!controller_generation_ && request.generation() == retired_controller_generation_ &&
        request.open_id() == retired_controller_open_id_) {
      response->set_generation(request.generation());
      response->set_open_id(request.open_id());
      response->set_joined(true);
      return grpc::Status::OK;
    }
    if (request.generation() != controller_generation_ || request.open_id() != controller_open_id_)
      return {grpc::StatusCode::FAILED_PRECONDITION, "controller owner does not match"};
    controller_retiring_ = true;
    markControlDownLocked();
    controller_changed_.notify_all();
    auto calls = controller_calls_;
    lock.unlock();
    for (const auto &entry : calls) entry.second->context.TryCancel();
    lock.lock();
    if (!controller_changed_.wait_until(lock, scope.deadline(), [this] {
          return !controller_generation_ ||
                 (controller_calls_.empty() && controller_work_ == 0 && !controller_finishing_);
        }))
      return {grpc::StatusCode::DEADLINE_EXCEEDED, "controller retirement is still joining"};
    // Another exact Retire may have completed while this caller was waiting.
    if (!controller_generation_) {
      if (request.generation() != retired_controller_generation_ ||
          request.open_id() != retired_controller_open_id_)
        return {grpc::StatusCode::FAILED_PRECONDITION, "controller owner changed"};
    } else if (request.generation() == controller_generation_ &&
               request.open_id() == controller_open_id_ && controller_retiring_) {
      controller_finishing_ = true;
      std::vector<std::shared_ptr<Member>> controls;
      for (const auto &entry : control_members_) controls.push_back(entry.second);
      for (const auto &member : controls) {
        if (!controller_changed_.wait_until(lock, scope.deadline(), [&] {
              return member->control_owner != request.generation() || !member->potential_motion ||
                     (!member->control_work && !member->control_release_waiting &&
                      !member->control_finishing);
            })) {
          controller_finishing_ = false;
          controller_changed_.notify_all();
          return {grpc::StatusCode::DEADLINE_EXCEEDED, "member control is still joining"};
        }
        if (member->control_owner != request.generation() || !member->potential_motion)
          continue;
        member->control_finishing = true;
        lock.unlock();
        std::string error;
        bool released;
        try { released = retireNativeControl(member, &error); }
        catch (...) { released = false; }
        lock.lock();
        member->control_finishing = false;
        controller_changed_.notify_all();
        if (!released) {
          controller_finishing_ = false;
          controller_changed_.notify_all();
          return {grpc::StatusCode::UNKNOWN, "native control release did not complete"};
        }
        member->potential_motion = false;
      }
      if (!controller_changed_.wait_until(lock, scope.deadline(), [this] {
            return controller_work_ == 0;
          })) {
        controller_finishing_ = false;
        controller_changed_.notify_all();
        return {grpc::StatusCode::DEADLINE_EXCEEDED, "independent control work is still joining"};
      }
      for (const auto &member : controls) {
        if (member->control_owner != request.generation()) continue;
        member->control_held = false;
        member->control_retiring = false;
        member->retired_control_generation = member->control_generation;
        member->silent_since = Steady::now();
        control_members_.erase(member.get());
      }
      retired_controller_generation_ = controller_generation_;
      retired_controller_open_id_ = controller_open_id_;
      controller_generation_ = 0;
      controller_open_id_.clear();
      OPENSSL_cleanse(&controller_grant_[0], controller_grant_.size());
      controller_grant_.clear();
      controller_binding_.Clear();
      controller_retiring_ = false;
      controller_finishing_ = false;
      optional_status_ = {};
      control_status_ = {};
      last_control_renewal_ = {};
      controller_changed_.notify_all();
    } else {
      return {grpc::StatusCode::FAILED_PRECONDITION, "controller owner changed"};
    }
    response->set_generation(request.generation());
    response->set_open_id(request.open_id());
    response->set_joined(true);
    return grpc::Status::OK;
  }

  void postStatusAccept() {
    auto call = std::make_shared<StatusCall>(*this);
    {
      std::lock_guard<std::mutex> lock(status_calls_mutex_);
      status_calls_.emplace(call.get(), call);
    }
    call->context.AsyncNotifyWhenDone(&call->done);
    service_.RequestSubscribeStatus(&call->context, &call->request, &call->writer,
                                    status_queue_.get(), status_queue_.get(),
                                    &call->accept);
  }
  void postStatusAcceptSafely() noexcept {
    try {
      postStatusAccept();
    } catch (...) {
      workerFailed("status admission", std::current_exception());
    }
  }
  void finishDirectStatusIfJoined(StatusCall *call) {
    if (!call->direct_finish || !call->io_done || !call->done_received)
      return;
    call->scope.reset();
    const bool counted = call->counted;
    std::shared_ptr<StatusCall> retained;
    {
      std::lock_guard<std::mutex> lock(status_calls_mutex_);
      retained = status_calls_.at(call);
      status_calls_.erase(call);
      status_calls_changed_.notify_all();
    }
    releaseStatus(call);
    if (!counted && !shutdown_)
      postStatusAcceptSafely();
  }
  void startStatusCompletion() {
    status_completion_ = std::thread([this] {
      void *value = nullptr;
      bool ok = false;
      while (status_queue_->Next(&value, &ok)) {
        auto *tag = static_cast<StatusTag *>(value);
        StatusCall *call = tag->call;
        switch (tag->kind) {
        case StatusTag::Kind::Accept:
          if (!ok) {
            std::lock_guard<std::mutex> lock(status_calls_mutex_);
            status_calls_.erase(call);
            status_calls_changed_.notify_all();
            break;
          }
          call->scope.emplace(admission_.begin(call->context, true));
          call->result = call->scope->status();
          if (call->result.ok()) {
            call->result = validateStatusSelection(call->request, &call->selection);
            if (!call->result.ok())
              call->result = call->scope->application_error(call->result);
          }
          if (call->result.ok()) {
            call->control = call->selection.control;
            call->optional = !call->control;
            if (call->optional &&
                optional_status_subscribers_.fetch_add(1) >= kMaximumStatusSubscribers - 1) {
              optional_status_subscribers_.fetch_sub(1);
              call->result = call->scope->application_error(
                  grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                               "optional status subscriber capacity exhausted"));
            } else {
              call->result = admitOwnedStatus(call);
              if (call->result.ok()) {
                call->counted = true;
                status_subscribers_.fetch_add(1);
              } else {
                if (call->optional) optional_status_subscribers_.fetch_sub(1);
                call->result = call->scope->application_error(call->result);
              }
            }
          }
          try {
            call->worker = std::thread([this, call] {
              auto result = call->result;
              try {
                if (result.ok()) {
                  if (call->metadata())
                    result = subscribe(*call, *call->scope, std::move(call->selection));
                  else
                    result = grpc::Status(grpc::StatusCode::CANCELLED, "status metadata cancelled");
                }
              } catch (...) {
                workerFailed("status subscriber", std::current_exception());
                result = grpc::Status(grpc::StatusCode::INTERNAL, "status subscriber failed");
              }
              call->finish(result);
              call->scope.reset(); // IsCancelled is safe after the Done tag.
              call->joined.Set(status_queue_.get(), std::chrono::system_clock::now(), &call->join);
            });
          } catch (...) {
            call->direct_finish = true;
            const grpc::Status exhausted(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                         "status worker capacity exhausted");
            call->writer.Finish(*call->scope ? call->scope->application_error(exhausted)
                                             : call->scope->status(), &call->io);
          }
          if (call->counted && !shutdown_)
            postStatusAcceptSafely();
          break;
        case StatusTag::Kind::Done:
          call->cancelled.store(call->context.IsCancelled(), std::memory_order_release);
          if (call->owned && call->control) {
            std::lock_guard<std::mutex> lock(controller_mutex_);
            if (control_status_.active == call) markControlDownLocked();
          }
          {
            std::lock_guard<std::mutex> lock(subscribers_mutex_);
            if (call->subscriber)
              call->subscriber->changed.notify_one();
          }
          {
            std::lock_guard<std::mutex> lock(call->io_mutex);
            call->done_received = true;
            call->io_changed.notify_all();
          }
          finishDirectStatusIfJoined(call);
          break;
        case StatusTag::Kind::IO:
          {
            std::lock_guard<std::mutex> lock(call->io_mutex);
            call->io_done = true;
            call->io_ok = ok;
            call->io_changed.notify_all();
          }
          finishDirectStatusIfJoined(call);
          break;
        case StatusTag::Kind::Join:
          call->worker.join();
          {
          const bool counted = call->counted;
          std::shared_ptr<StatusCall> retained;
          {
            std::lock_guard<std::mutex> lock(status_calls_mutex_);
            retained = status_calls_.at(call);
            status_calls_.erase(call);
            status_calls_changed_.notify_all();
          }
          releaseStatus(call);
          if (!counted && !shutdown_)
            postStatusAcceptSafely();
          }
          break;
        }
      }
    });
    postStatusAcceptSafely();
  }
  void stopStatusCompletion() {
    if (!status_queue_)
      return;
    {
      std::unique_lock<std::mutex> lock(status_calls_mutex_);
      status_calls_changed_.wait(lock, [this] { return status_calls_.empty(); });
    }
    status_queue_->Shutdown();
    if (status_completion_.joinable())
      status_completion_.join();
  }
  // Membership changes are relevant to the selected robot even when a newer
  // epoch replaced it. A channel change needs the complete selected identity.
  // The caller holds neither this lock across IO nor a subscriber's state lock.
  void notifyStatus(const wire::MemberIdentity *identity = nullptr,
                    const std::string *channel = nullptr) {
    if (status_subscribers_.load(std::memory_order_relaxed) == 0)
      return;
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    for (const auto &subscriber : subscribers_) {
      bool relevant = !identity || subscriber->selection.all;
      if (!relevant) {
        const auto selected = subscriber->selection.members.find(identity->robot_id());
        relevant = selected != subscriber->selection.members.end() &&
                   (!channel || (sameIdentity(selected->second.identity, *identity) &&
                                 selected->second.channels.count(*channel)));
      }
      if (relevant) {
        ++subscriber->revision;
        subscriber->changed.notify_one();
      }
    }
  }
  void awaitStatusDelivered(Milliseconds budget) {
    std::unique_lock<std::mutex> lock(subscribers_mutex_);
    std::vector<std::pair<std::shared_ptr<Subscriber>, std::uint64_t>> targets;
    targets.reserve(subscribers_.size());
    for (const auto &subscriber : subscribers_)
      targets.emplace_back(subscriber, subscriber->revision);
    subscribers_changed_.wait_for(lock, budget, [&] {
      for (const auto &target : targets)
        if (std::find(subscribers_.begin(), subscribers_.end(), target.first) !=
                subscribers_.end() && target.first->delivered < target.second)
          return false;
      return true;
    });
  }
  grpc::Status
  subscribe(StatusCall &call,
            const xgc2::xrpc::GrpcCallScope &scope, StatusSelection selection) {
    const auto deadline = scope.deadline();
    if (call.owned) {
      {
        std::lock_guard<std::mutex> lock(controller_mutex_);
        auto &slot = call.control ? control_status_ : optional_status_;
        if (slot.active != &call || slot.retiring || controller_retiring_)
          return grpc::Status::CANCELLED;
        slot.ready = true;
      }
      wire::SubscribeStatusResponse receipt;
      receipt.set_generation(call.request.owner().generation());
      receipt.set_open_id(call.request.owner().open_id());
      if (!call.write(receipt)) return grpc::Status::CANCELLED;
      if (call.control) {
        std::unique_lock<std::mutex> lock(call.io_mutex);
        call.io_changed.wait_until(lock, deadline, [&] {
          return call.done_received || call.cancelled.load(std::memory_order_acquire);
        });
        return grpc::Status::CANCELLED;
      }
    }
    Stream stream;
    auto subscriber = std::make_shared<Subscriber>(std::move(selection));
    for (const auto &entry : subscriber->selection.members)
      stream.identities.emplace(entry.first, entry.second.identity);
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      subscribers_.push_back(subscriber);
      call.subscriber = subscriber;
    }
    struct Registration {
      Impl &owner;
      StatusCall &call;
      std::shared_ptr<Subscriber> subscriber;
      ~Registration() {
        {
          std::lock_guard<std::mutex> lock(owner.subscribers_mutex_);
          owner.subscribers_.erase(std::find(owner.subscribers_.begin(),
                                             owner.subscribers_.end(),
                                             subscriber));
          call.subscriber.reset();
        }
        try {
          owner.reconcileSubscriberMembers(subscriber->selection);
        } catch (...) {
          owner.workerFailed("status source retirement", std::current_exception());
        }
        owner.subscribers_changed_.notify_all();
      }
    } registration{*this, call, subscriber};
    reconcileSubscriberMembers(subscriber->selection);
    const auto frame_limit = admission_.limits().response_bytes;
    std::uint64_t observed = 0;
    while (!shutdown_ && !call.cancelled.load(std::memory_order_acquire) &&
           Steady::now() < deadline) {
      {
        std::unique_lock<std::mutex> lock(subscribers_mutex_);
        if (subscriber->revision == observed) {
          subscriber->changed.wait_until(lock, deadline, [&] {
            return shutdown_ || call.cancelled.load(std::memory_order_acquire) ||
                   subscriber->revision != observed;
          });
          if (subscriber->revision == observed)
            continue;
        }
        observed = subscriber->revision;
      }
      status_snapshots_.fetch_add(1, std::memory_order_relaxed);
      std::map<std::string, std::shared_ptr<Member>> members;
      {
        std::lock_guard<std::mutex> lock(table_mutex_);
        if (subscriber->selection.all) {
          members = members_;
        } else {
          for (const auto &selected : subscriber->selection.members) {
            const auto found = members_.find(selected.first);
            if (found != members_.end() &&
                sameIdentity(found->second->identity, selected.second.identity))
              members.emplace(found->first, found->second);
          }
        }
      }
      stream.sending.Clear();
      auto flush = [&] {
        if (!stream.sending.members_size())
          return true;
        const auto success = call.write(stream.sending);
        stream.sending.Clear();
        return success;
      };
      for (auto i = stream.identities.begin(); i != stream.identities.end();) {
        const auto current = members.find(i->first);
        if (current == members.end() ||
            !sameIdentity(current->second->identity, i->second)) {
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
        updates.reserve(subscriber->selection.all
                            ? kMaximumChannelsPerRobot
                            : subscriber->selection.members.at(entry.first).channels.size());
        {
          std::lock_guard<std::mutex> lock(member->state_mutex);
          if (subscriber->selection.all) {
            for (const auto &channel : member->latest)
              if (stream.seen[entry.first][channel.first] != channel.second.first)
                updates.push_back(channel);
          } else {
            const auto &selected = subscriber->selection.members.at(entry.first);
            for (const auto &id : selected.channels) {
              const auto channel = member->latest.find(id);
              if (channel != member->latest.end() &&
                  stream.seen[entry.first][id] != channel->second.first)
                updates.push_back(*channel);
            }
          }
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
      {
        std::lock_guard<std::mutex> lock(subscribers_mutex_);
        subscriber->delivered = observed;
      }
      subscribers_changed_.notify_all();
    }
    return call.cancelled.load(std::memory_order_acquire) || Steady::now() >= deadline
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
          auto budget = binding_.BoundMasterCalls(std::nullopt);
          const std::weak_ptr<Member> weak = member;
          auto native = factory_(
              *node, config,
              [this, weak](wire::RobotMessage message) {
                auto member = weak.lock();
                if (!member || member->closing)
                  return;
                if (message.ByteSizeLong() > kMaximumTelemetryItemBytes)
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
                const auto channel = message.channel_id();
                {
                  std::lock_guard<std::mutex> lock(member->state_mutex);
                  auto &slot = member->latest[channel];
                  if (slot.first && message.message().sequence() <=
                                        slot.second.message().sequence())
                    return;
                  slot.first = ++member->state_version;
                  slot.second = std::move(message);
                }
                notifyStatus(&member->identity, &channel);
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
        bool published = false;
        {
          std::lock_guard<std::mutex> lock(table_mutex_);
          if (phase() != Phase::Ready) {
            error = "member registration cancelled";
          } else {
            member->closing = false;
            members_[identity.robot_id()] = member;
            namespaces_[namespace_entry->second] = identity.robot_id();
            ++members_revision_;
            published = true;
            notifyStatus(&member->identity);
          }
        }
        if (published) {
          notifyCommandMemberChange();
          reconcileMemberStatus(member);
        }
      }
      if (!error.empty()) {
        if (member && member->native && phase() != Phase::Ready) {
          try {
            stopMember(member);
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
        if (found->second->identity.target_id() != identity.target_id()) {
          item->mutable_error()->set_class_(operation::ERROR_CLASS_REJECTED);
          item->mutable_error()->set_code("member-target-conflict");
          continue;
        }
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
        notifyStatus(&released->identity);
      }
      // Stop fences native callbacks before the removal is acknowledged. The
      // scheduler sees closing and cannot publish or submit another side
      // effect.
      fenceCommands(released);
      stopMember(released);
      stopped.push_back(std::move(released));
    }
    // Once for the whole batch, before the stopped robots are destroyed.
    if (!stopped.empty() && !binding_.FlushPublications(timings_.publication_flush))
      throw std::runtime_error("native publication flush did not complete");
    return result;
  }
  // Cancels each member's native call and fails its queued commands; returns
  // once the scheduler has done so.
  void fenceCommands(const std::shared_ptr<Member> &member) {
    fenceCommands(std::vector<std::shared_ptr<Member>>{member});
  }
  void fenceCommands(const std::vector<std::shared_ptr<Member>> &members) {
    if (members.empty())
      return;
    auto done = std::make_shared<std::promise<void>>();
    auto ready = done->get_future();
    if (!scheduler_alive_.load(std::memory_order_acquire)) {
      // The scheduler flag changes only after its native services destruct.
      for (auto &member : members) {
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
      }
      return;
    }
    postFence(
        {[this, members, done] {
           for (auto &member : members) {
             if (io_)
               cancelNative(member, *io_);
             for (auto &call : member->commands)
               finishExecute(call, CommandError(call->request.operation(),
                                                operation::ERROR_CLASS_CANCELLED,
                                                "member-removed",
                                                "not executed: member removed"));
             member->commands.clear();
           }
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
        fenceCommands(members);
        return;
      }
    }
    ready.get();
    std::unique_lock<std::mutex> lock(command_mutex_);
    idle_changed_.wait(lock, [&] {
      return !scheduler_alive_.load(std::memory_order_acquire) ||
             std::all_of(members.begin(), members.end(),
                         [](const auto &member) { return !member->in_flight; });
    });
  }

  const ServerOptions options_;
  RosBinding &binding_;
  RobotFactory factory_;
  const ServerTimings timings_;
  xgc2::xrpc::GrpcAdmission admission_;
  Service service_;
  operation::RuntimeServiceReference reference_;
  std::optional<operation::RuntimeServiceReference> network_reference_;
  std::unique_ptr<xgc2::xrpc::BootstrapInput> network_bootstrap_;
  std::string ros_node_name_;
  std::unique_ptr<xgc2::xrpc::GrpcUnixServer> server_;
  std::unique_ptr<grpc::ServerCompletionQueue> status_queue_;
  std::thread status_completion_;
  std::mutex status_calls_mutex_;
  std::condition_variable status_calls_changed_;
  std::map<StatusCall *, std::shared_ptr<StatusCall>> status_calls_;
  std::array<unsigned char, 32> process_owner_{};
  std::mutex controller_mutex_;
  std::condition_variable controller_changed_;
  std::uint64_t controller_generation_ = 0, retired_controller_generation_ = 0;
  std::string controller_open_id_, retired_controller_open_id_, controller_grant_;
  wire::ControllerBinding controller_binding_;
  bool controller_retiring_ = false, controller_finishing_ = false;
  std::size_t controller_work_ = 0;
  std::map<Member *, std::shared_ptr<Member>> control_members_;
  Steady::time_point last_control_renewal_{};
  std::map<StatusCall *, std::shared_ptr<StatusCall>> controller_calls_;
  OwnedStatusSlot optional_status_, control_status_;
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

  std::mutex subscribers_mutex_;
  std::condition_variable subscribers_changed_;
  std::vector<std::shared_ptr<Subscriber>> subscribers_;
  std::atomic<std::size_t> status_subscribers_{0}, blocking_calls_{0},
      readiness_waits_{0}, optional_status_subscribers_{0}, control_blocking_calls_{0},
      safety_blocking_calls_{0}, membership_blocking_calls_{0};
  std::atomic<std::uint64_t> status_snapshots_{0}, status_writes_started_{0},
      status_writes_joined_{0}, status_protobuf_bytes_{0},
      status_subscribers_joined_{0}, control_renewals_{0},
      control_renewal_rejections_{0}, control_renewal_request_bytes_{0},
      control_renewal_response_bytes_{0};

  std::mutex table_mutex_, jobs_mutex_, command_mutex_;
  std::map<std::string, std::shared_ptr<Member>> members_;
  std::map<std::string, std::string> namespaces_;
  std::uint64_t members_revision_ = 0;
  std::size_t next_lane_ = 0;
  std::vector<std::shared_ptr<Member>> retained_;
  std::deque<Job> registration_jobs_;
  bool registration_alive_ = false; // guarded by jobs_mutex_
  std::vector<Job> command_jobs_;
  std::uint64_t command_revision_ = 0;
  Job fence_job_;
  std::condition_variable jobs_ready_, command_ready_, idle_changed_;
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
StatusWork RobotServer::SnapshotStatusWork() const noexcept { return impl_->statusWork(); }

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
