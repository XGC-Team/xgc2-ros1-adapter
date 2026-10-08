#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include "robot_server_transport.hpp"
#include "xgc2/xrpc/grpc.hpp"
#include "xgc2/xrpc/runtime_policy.hpp"
#include "xgc2_ros1_robot_adapter/robot_server_entry.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <exception>
#include <fstream>
#include <future>
#include <google/protobuf/util/message_differencer.h>
#include <grpcpp/grpcpp.h>
#include <iostream>
#include <map>
#include <mutex>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <set>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace xgc2_ros1_robot_adapter {
void ConfigureBoundedRosMaster();
namespace {
namespace wire = xgc::robot::v1;
namespace operation = xgc::adapter::v1;
using Steady = std::chrono::steady_clock;
constexpr std::size_t kMaximumRobots = 256, kMaximumMembers = 1024;
constexpr std::size_t kMaximumBatchMembers = 256, kMaximumCommandsPerRobot = 32;
constexpr std::size_t kMaximumRegistrationJobs = 32, kMaximumCommandJobs = 32;
constexpr std::size_t kMaximumChannelsPerRobot = 32;
constexpr std::size_t kMaximumNativeMessageBytes = 65536;
constexpr std::size_t kMaximumStatusSubscribers = 4, kMaximumBlockingCalls = 24;
volatile std::sig_atomic_t signal_stop = 0;
void stopSignal(int) { signal_stop = 1; }
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
std::string useKey(const wire::MemberIdentity &id) {
  return id.target_id() + '\0' + id.run_id() + '\0' + id.robot_id();
}
bool validIdentity(const wire::MemberIdentity &id) {
  return ValidRobotIdentity(id.robot_id()) &&
         ValidRuntimeReferenceId(id.target_id()) &&
         ValidRuntimeReferenceId(id.run_id()) && id.connection_epoch() &&
         id.target_id().find('\0') == std::string::npos &&
         id.run_id().find('\0') == std::string::npos &&
         id.robot_id().find('\0') == std::string::npos;
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
void finishExecute(
    const std::shared_ptr<Unary<wire::ExecuteRequest, wire::ExecuteResponse>>
        &call,
    operation::OperationEvent result) {
  wire::ExecuteResponse response;
  *response.mutable_event() = std::move(result);
  call->finish(std::move(response));
}
class Server {
  struct Context;
  struct Use {
    wire::MemberIdentity identity;
    std::shared_ptr<Context> robot;
  };
  struct Command {
    std::shared_ptr<Unary<wire::ExecuteRequest, wire::ExecuteResponse>> call;
    wire::MemberIdentity identity;
  };
  struct Context {
    wire::RobotResource configuration;
    std::shared_ptr<NativeRobot> native;
    std::mutex state_mutex;
    std::map<std::string, std::pair<std::uint64_t, wire::RobotMessage>> latest;
    std::uint64_t state_version = 0;
    std::size_t callback_worker = 0;
    std::atomic<bool> closing{false};
    // Owned exclusively by command scheduler; member changes are posted to it.
    std::deque<Command> commands;
    bool in_flight = false, uncertain = false;
    std::uint64_t native_call = 0;
    std::shared_ptr<Unary<wire::ExecuteRequest, wire::ExecuteResponse>>
        active_call;
    wire::MemberIdentity active_identity;
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
    explicit Service(Server &owner) : owner_(owner) {}
    grpc::Status Describe(grpc::ServerContext *context,
                          const wire::DescribeRequest *,
                          wire::DescribeResponse *response) override {
      auto scope = owner_.admission_.begin(*context, false, true);
      if (!scope)
        return scope.status();
      *response->mutable_service_ref() = owner_.reference_;
      response->set_provider_definition_id(
          owner_.bootstrap_.provider_definition_id());
      response->set_maximum_robots(kMaximumRobots);
      response->set_maximum_members(kMaximumMembers);
      response->set_maximum_batch_members(kMaximumBatchMembers);
      response->set_maximum_commands_per_robot(kMaximumCommandsPerRobot);
      *response->mutable_runtime_policy() = PolicyObservation(owner_.policy_);
      return grpc::Status::OK;
    }
    grpc::Status ApplyMembers(grpc::ServerContext *context,
                              const wire::ApplyMembersRequest *request,
                              wire::ApplyMembersResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      if (request->members_size() > static_cast<int>(kMaximumBatchMembers))
        return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                            "member batch exceeds 256");
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
      if (request->members_size() > static_cast<int>(kMaximumBatchMembers))
        return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                            "member batch exceeds 256");
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
      return owner_.invoke(
          std::move(scope), *request, response, [this](auto call) {
            return owner_.postCommand(
                {[this, call] {
                   std::shared_ptr<Context> robot;
                   {
                     std::lock_guard<std::mutex> lock(owner_.table_mutex_);
                     const auto use =
                         owner_.uses_.find(useKey(call->request.identity()));
                     if (use != owner_.uses_.end() &&
                         use->second->identity.connection_epoch() ==
                             call->request.identity().connection_epoch())
                       robot = use->second->robot;
                   }
                   if (!robot || robot->closing) {
                     finishExecute(
                         call,
                         CommandError(call->request.operation(),
                                      operation::ERROR_CLASS_REJECTED,
                                      "member-unavailable",
                                      "not executed: active member required"));
                     return;
                   }
                   if (robot->commands.size() >= kMaximumCommandsPerRobot) {
                     finishExecute(
                         call,
                         CommandError(
                             call->request.operation(),
                             operation::ERROR_CLASS_RESOURCE_EXHAUSTED,
                             "robot-queue-full",
                             "not executed: native command queue is full"));
                     return;
                   }
                   robot->commands.push_back({call, call->request.identity()});
                 },
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
        return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                            "status subscriber capacity exhausted");
      return owner_.subscribe(writer, scope);
    }
    grpc::Status Health(grpc::ServerContext *context,
                        const wire::HealthRequest *,
                        wire::HealthResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      response->set_serving(!owner_.shutdown_);
      response->set_instance_id(owner_.reference_.instance_id());
      {
        std::lock_guard<std::mutex> lock(owner_.table_mutex_);
        response->set_robots(owner_.robots_.size());
        response->set_members(owner_.uses_.size());
      }
      return grpc::Status::OK;
    }
    grpc::Status Shutdown(grpc::ServerContext *context,
                          const wire::ShutdownRequest *,
                          wire::ShutdownResponse *response) override {
      auto scope = owner_.admission_.begin(*context);
      if (!scope)
        return scope.status();
      response->set_serving(false);
      owner_.shutdown_ = true;
      owner_.notifyStatus();
      return grpc::Status::OK;
    }

  private:
    Server &owner_;
  };

public:
  Server(wire::RobotServerBootstrap bootstrap, RobotFactory factory,
         xgc2::xrpc::RuntimePolicy policy)
      : bootstrap_(std::move(bootstrap)), factory_(std::move(factory)),
        policy_(std::move(policy)),
        admission_(xgc2::xrpc::new_instance_id(), ProductGrpcLimits(policy_)),
        service_(*this) {
    reference_.set_target_id(bootstrap_.target_id());
    reference_.set_service("xgc2.robot-adapter");
    reference_.set_api_version("v1");
    reference_.set_instance_id(admission_.instance_id());
    reference_.set_profile("grpc.v1");
    reference_.mutable_endpoint()->set_kind("unix");
    reference_.mutable_endpoint()->set_address(bootstrap_.socket_path());
    command_jobs_.reserve(kMaximumCommandJobs);
    retained_robots_.reserve(kMaximumRobots + 1);
  }
  int run() {
    EnsurePrivateRobotSocketParent(bootstrap_.socket_path());
    std::unique_ptr<xgc2::xrpc::GrpcUnixServer> server;
    try {
      registration_ = std::thread([this] {
        try {
          registrationLoop();
        } catch (...) {
          workerFailed("registration", std::current_exception());
        }
        cleanupRobots();
      });
      scheduler_ = std::thread([this] {
        try {
          commandLoop();
        } catch (...) {
          workerFailed("commands", std::current_exception());
        }
        scheduler_alive_.store(false, std::memory_order_release);
        startup_changed_.notify_all();
      });
      for (std::size_t index = 0; index < callbacks_.size(); ++index)
        callbacks_[index] = std::thread([this, index] {
          try {
            pthread_setname_np(pthread_self(), "robot-callback");
            workerReady();
            while (!callbacks_stopping_)
              callback_queues_[index].callOne(ros::WallDuration(0.02));
          } catch (...) {
            workerFailed("callbacks", std::current_exception());
          }
        });
      {
        std::unique_lock<std::mutex> lock(startup_mutex_);
        startup_changed_.wait(lock,
                              [this] { return fatal_ || ready_workers_ == 4; });
      }
      // No method can enqueue work before all four fixed owners initialized.
      if (!fatal_) {
        xgc2::xrpc::UnixOptions endpoint;
        endpoint.path = bootstrap_.socket_path();
        server = std::make_unique<xgc2::xrpc::GrpcUnixServer>(
            endpoint, admission_, std::vector<grpc::Service *>{&service_});
        while (!signal_stop && !shutdown_ && (!ros::isStarted() || ros::ok()))
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    } catch (...) {
      workerFailed("startup", std::current_exception());
    }
    shutdown_ = true;
    admission_.request_stop();
    notifyStatus();
    if (server)
      server->request_stop();
    management_stopping_ = true;
    jobs_ready_.notify_all();
    if (registration_.joinable())
      registration_.join();
    stopping_ = true;
    if (scheduler_.joinable())
      scheduler_.join();
    callbacks_stopping_ = true;
    for (auto &queue : callback_queues_)
      queue.disable();
    for (auto &worker : callbacks_)
      if (worker.joinable())
        worker.join();
    // Failed construction may leave jobs queued, but none may run here.
    abortQueuedJobs();
    // NativeRobot objects outlive scheduler I/O and every callback worker,
    // including a throwing Stop implementation.
    retained_robots_.clear();
    while (server && !server->shutdown())
      std::cerr << "robot server: admitted work has not quiesced; retaining "
                   "endpoint ownership\n";
    {
      RosMasterDeadline deadline(std::chrono::seconds(3));
      ros::shutdown();
    }
    return fatal_ ? 1 : 0;
  }

private:
  void workerReady() {
    {
      std::lock_guard<std::mutex> lock(startup_mutex_);
      ++ready_workers_;
    }
    startup_changed_.notify_all();
  }
  void workerFailed(const char *owner, std::exception_ptr failure) noexcept {
    fatal_ = true;
    shutdown_ = true;
    try {
      std::rethrow_exception(failure);
    } catch (const std::exception &error) {
      std::cerr << "robot server " << owner << ": " << error.what() << '\n';
    } catch (...) {
      std::cerr << "robot server " << owner << ": unknown native failure\n";
    }
    admission_.request_stop();
    notifyStatus();
    jobs_ready_.notify_all();
    startup_changed_.notify_all();
  }
  static void abortJob(Job &job, bool started) noexcept {
    if (!job.abort)
      return;
    try {
      job.abort(started);
    } catch (...) {
      std::cerr << "robot server: failed to construct interrupted-work reply\n";
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
  bool postRegistration(Job job) {
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      if (management_stopping_ || shutdown_ ||
          registration_jobs_.size() >= kMaximumRegistrationJobs)
        return false;
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
  void registrationLoop() {
    pthread_setname_np(pthread_self(), "ros-library");
    {
      RosMasterDeadline deadline(std::chrono::seconds(3));
      node_.reset(new ros::NodeHandle());
      node_->setCallbackQueue(&callback_queues_[0]);
    }
    pthread_setname_np(pthread_self(), "robot-members");
    workerReady();
    for (;;) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(jobs_mutex_);
        jobs_ready_.wait(lock, [this] {
          return management_stopping_ || shutdown_ ||
                 !registration_jobs_.empty();
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
  void cleanupRobots() noexcept {
    try {
      {
        std::lock_guard<std::mutex> lock(table_mutex_);
        for (auto &entry : robots_) {
          entry.second->closing = true;
          retained_robots_.push_back(entry.second);
        }
        uses_.clear();
        robots_.clear();
        ++members_revision_;
        notifyStatus();
      }
      RosMasterDeadline cleanup(std::chrono::seconds(5));
      for (auto &robot : retained_robots_) {
        try {
          fenceCommands(robot, nullptr);
        } catch (...) {
          workerFailed("native fence", std::current_exception());
        }
        try {
          if (robot->native)
            robot->native->Stop();
        } catch (...) {
          workerFailed("native stop", std::current_exception());
        }
      }
    } catch (...) {
      workerFailed("native cleanup", std::current_exception());
    }
  }
  void commandLoop() {
    pthread_setname_np(pthread_self(), "robot-commands");
    std::exception_ptr failure;
    std::vector<Job> jobs;
    jobs.reserve(kMaximumCommandJobs);
    std::vector<std::shared_ptr<Context>> robots;
    {
      AsyncRosServices services;
      services_ = &services;
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
              robots.clear();
              robots.reserve(robots_.size());
              for (auto &entry : robots_)
                robots.push_back(entry.second);
              revision = members_revision_;
            }
          }
          const bool periodic = Steady::now() >= next_periodic;
          if (periodic)
            next_periodic = Steady::now() + std::chrono::milliseconds(100);
          for (auto &robot : robots) {
            if (robot->closing) {
              if (robot->native_call)
                services.Cancel(robot->native_call);
              continue;
            }
            if (robot->native_call && robot->active_call &&
                (robot->active_call->cancelled ||
                 robot->active_call->work.cancelled()))
              services.Cancel(robot->native_call);
            if (periodic && !shutdown_) {
              std::lock_guard<std::mutex> lock(robot->native_mutex);
              if (!robot->closing)
                robot->native->Periodic(ros::WallTime::now());
            }
            if (!shutdown_ && !robot->in_flight && !robot->commands.empty())
              dispatch(robot, services);
          }
          services.Pump(std::chrono::milliseconds(2));
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
        for (auto &entry : robots_)
          if (std::find(robots.begin(), robots.end(), entry.second) ==
              robots.end())
            robots.push_back(entry.second);
      }
      for (auto &robot : robots) {
        if (robot->native_call)
          services.Cancel(robot->native_call);
        for (auto &command : robot->commands)
          finishExecute(command.call,
                        CommandError(command.call->request.operation(),
                                     operation::ERROR_CLASS_CANCELLED,
                                     "server-stopped",
                                     "not executed: server shutdown"));
        robot->commands.clear();
      }
      try {
        services.Pump(std::chrono::milliseconds(0));
      } catch (...) {
        failure = std::current_exception();
        workerFailed("native completion", failure);
      }
    }
    services_ = nullptr;
    for (auto &robot : robots) {
      if (robot->active_call)
        finishExecute(
            robot->active_call,
            CommandError(
                robot->active_call->request.operation(),
                operation::ERROR_CLASS_UNCERTAIN, "native-result-unknown",
                "native owner exited without a result; reconcile outcome"));
      robot->active_call.reset();
      robot->native_call = 0;
      robot->in_flight = false;
    }
  }
  void dispatch(const std::shared_ptr<Context> &robot,
                AsyncRosServices &services) {
    auto command = std::move(robot->commands.front());
    robot->commands.pop_front();
    auto &request = command.call->request.operation();
    bool current;
    {
      std::lock_guard<std::mutex> lock(table_mutex_);
      const auto use = uses_.find(useKey(command.identity));
      current = use != uses_.end() && use->second->robot == robot &&
                use->second->identity.connection_epoch() ==
                    command.identity.connection_epoch();
    }
    if (!current || robot->uncertain) {
      finishExecute(
          command.call,
          CommandError(request, operation::ERROR_CLASS_REJECTED,
                       robot->uncertain ? "previous-result-unknown"
                                        : "member-removed",
                       robot->uncertain
                           ? "not executed: previous native result is unknown"
                           : "not executed: member cancelled or removed"));
      return;
    }
    if (command.call->cancelled || command.call->work.cancelled()) {
      finishExecute(command.call,
                    CommandError(request, operation::ERROR_CLASS_CANCELLED,
                                 "operation-cancelled",
                                 "not executed: call cancelled"));
      return;
    }
    if (request.context().deadline().deadline_unix_nanos() <=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count()) {
      finishExecute(command.call,
                    CommandError(request, operation::ERROR_CLASS_DEADLINE,
                                 "deadline-exceeded",
                                 "not executed: operation deadline elapsed"));
      return;
    }
    robot->in_flight = true;
    robot->active_call = command.call;
    robot->active_identity = command.identity;
    std::lock_guard<std::mutex> lock(robot->native_mutex);
    robot->native_call = robot->native->Execute(
        request, services, [robot, command](operation::OperationEvent result) {
          robot->in_flight = false;
          robot->native_call = 0;
          robot->active_call.reset();
          if (result.phase() == operation::OPERATION_PHASE_UNCERTAIN)
            robot->uncertain = true;
          finishExecute(command.call, std::move(result));
        });
  }
  template <class Request, class Response, class Submit>
  grpc::Status invoke(xgc2::xrpc::GrpcCallScope scope, const Request &request,
                      Response *response, Submit submit) {
    BoundedWaiter waiter(blocking_calls_, kMaximumBlockingCalls);
    if (!waiter.accepted)
      return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                          "native blocking call capacity exhausted");
    auto call = std::make_shared<Unary<Request, Response>>();
    call->request = request;
    call->work = scope.retain_work();
    if (!submit(call))
      return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                          "native dispatch queue is full");
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
          lock,
          std::min(scope.deadline(),
                   Steady::now() + std::chrono::milliseconds(20)),
          [&] { return call->finished; });
    }
    *response = std::move(call->response);
    return call->status;
  }
  void notifyStatus() {
    status_version_.fetch_add(1, std::memory_order_release);
    status_changed_.notify_all();
  }
  grpc::Status
  subscribe(grpc::ServerWriter<wire::SubscribeStatusResponse> *writer,
            const xgc2::xrpc::GrpcCallScope &scope) {
    const auto deadline = scope.deadline();
    Stream stream;
    const auto frame_limit =
        std::min<std::size_t>(131072, admission_.limits().response_bytes);
    std::uint64_t observed = UINT64_MAX;
    while (!shutdown_ && !scope.cancelled()) {
      const auto version = status_version_.load(std::memory_order_acquire);
      if (version == observed) {
        std::unique_lock<std::mutex> lock(status_mutex_);
        status_changed_.wait_until(
            lock,
            std::min(deadline, Steady::now() + std::chrono::milliseconds(20)),
            [&] {
              return shutdown_ || scope.cancelled() ||
                     status_version_.load(std::memory_order_acquire) !=
                         observed;
            });
        continue;
      }
      observed = version;
      std::map<std::string, std::shared_ptr<Use>> uses;
      {
        std::lock_guard<std::mutex> lock(table_mutex_);
        uses = uses_;
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
        const auto current = uses.find(i->first);
        if (current == uses.end() ||
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
      for (const auto &entry : uses) {
        const auto &use = entry.second;
        stream.identities[entry.first] = use->identity;
        std::vector<std::pair<std::string,
                              std::pair<std::uint64_t, wire::RobotMessage>>>
            updates;
        updates.reserve(kMaximumChannelsPerRobot);
        {
          std::lock_guard<std::mutex> lock(use->robot->state_mutex);
          for (const auto &channel : use->robot->latest)
            if (stream.seen[entry.first][channel.first] != channel.second.first)
              updates.push_back(channel);
        }
        // No native state mutex is held across a slow peer's blocking Write.
        for (const auto &channel : updates) {
          wire::MemberStatus update;
          *update.mutable_identity() = use->identity;
          *update.add_messages() = channel.second.second;
          if (update.ByteSizeLong() + 16 > frame_limit)
            return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                "native state exceeds response policy");
          if (stream.sending.ByteSizeLong() + update.ByteSizeLong() + 16 >
                  frame_limit &&
              !flush())
            return grpc::Status::CANCELLED;
          *stream.sending.add_members() = std::move(update);
          stream.seen[entry.first][channel.first] = channel.second.first;
        }
      }
      if (!flush())
        return grpc::Status::CANCELLED;
    }
    return scope.cancelled()
               ? grpc::Status(Steady::now() >= deadline
                                  ? grpc::StatusCode::DEADLINE_EXCEEDED
                                  : grpc::StatusCode::CANCELLED,
                              "status stream cancelled or expired")
               : grpc::Status::OK;
  }

  wire::ApplyMembersResponse apply(const wire::ApplyMembersRequest &request,
                                   const xgc2::xrpc::GrpcWorkPermit &work) {
    wire::ApplyMembersResponse result;
    for (const auto &member : request.members()) {
      auto *item = result.add_members();
      *item->mutable_identity() = member.identity();
      std::string error;
      auto error_class = operation::ERROR_CLASS_REJECTED;
      if (work.cancelled() || shutdown_ || signal_stop)
        error = "member registration cancelled";
      else if (!validIdentity(member.identity()) ||
               member.identity().target_id() != reference_.target_id() ||
               member.configuration().robot_id() !=
                   member.identity().robot_id())
        error = "invalid or cross-target member identity";
      if (member.configuration().channels_size() >
          static_cast<int>(kMaximumChannelsPerRobot))
        error = "native channel limit exceeded";
      const auto config = configFrom(member.configuration());
      const auto namespace_entry = config.parameters.find("namespace");
      if (namespace_entry == config.parameters.end() ||
          namespace_entry->second.empty())
        error = "native namespace required";
      for (const auto &name : {"ros_master_uri", "ros_ip"}) {
        const auto value = config.parameters.find(name);
        const auto expected = bootstrap_.ros_environment().find(name);
        if (value != config.parameters.end() && !value->second.empty() &&
            (expected == bootstrap_.ros_environment().end() ||
             expected->second != value->second))
          error = "member crosses server ROS environment";
      }
      std::shared_ptr<Context> robot, previous;
      bool update = false;
      if (error.empty()) {
        std::lock_guard<std::mutex> lock(table_mutex_);
        const auto use = uses_.find(useKey(member.identity()));
        if (use != uses_.end()) {
          previous = use->second->robot;
          if (use->second->identity.connection_epoch() !=
              member.identity().connection_epoch())
            error = "active member connection epoch conflicts";
        }
        if (use == uses_.end() && uses_.size() >= kMaximumMembers)
          error = "member table capacity exhausted";
        const auto found = robots_.find(namespace_entry->second);
        if (found == robots_.end() && robots_.size() >= kMaximumRobots)
          error = "native robot table capacity exhausted";
        if (found != robots_.end()) {
          robot = found->second;
          update = robot->closing && previous == robot;
          auto native_config = robot->configuration,
               requested = member.configuration();
          if (!google::protobuf::util::MessageDifferencer::Equals(native_config,
                                                                  requested)) {
            update = previous == robot;
            for (const auto &other : uses_)
              if (other.second->robot == robot &&
                  other.first != useKey(member.identity()))
                update = false;
            if (!update)
              error = "native ROS resource configuration conflicts";
          }
        }
      }
      if (error.empty() && (update || !robot)) {
        try {
          RosMasterDeadline deadline(std::chrono::seconds(3));
          if (update) {
            robot->closing = true;
            fenceCommands(robot, nullptr);
            robot->native->Stop();
            std::lock_guard<std::mutex> lock(robot->state_mutex);
            robot->latest.clear();
          } else {
            robot = std::make_shared<Context>();
            robot->closing = true;
            robot->callback_worker =
                next_callback_worker_++ % callback_queues_.size();
          }
          const std::weak_ptr<Context> weak = robot;
          ros::NodeHandle robot_node(*node_);
          robot_node.setCallbackQueue(
              &callback_queues_[robot->callback_worker]);
          auto native = factory_(
              robot_node, config,
              [this, weak](wire::RobotMessage message) {
                auto robot = weak.lock();
                if (!robot || robot->closing)
                  return;
                if (message.ByteSizeLong() > kMaximumNativeMessageBytes)
                  return;
                bool declared = false;
                for (const auto &channel : robot->configuration.channels())
                  if (channel.enabled() &&
                      channel.channel_id() == message.channel_id()) {
                    declared = true;
                    break;
                  }
                if (!declared)
                  return;
                std::lock_guard<std::mutex> lock(robot->state_mutex);
                auto &slot = robot->latest[message.channel_id()];
                slot.first = ++robot->state_version;
                slot.second = std::move(message);
                notifyStatus();
              },
              &error);
          if (deadline.failed() && error.empty())
            error = "ROS master registration did not complete";
          if (!native && error.empty())
            error = "native resource creation failed";
          if (!error.empty() && native) {
            try {
              native->Stop();
            } catch (...) {
              robot->native = native;
              retained_robots_.push_back(robot);
              workerFailed("failed factory cleanup", std::current_exception());
              error_class = operation::ERROR_CLASS_UNCERTAIN;
            }
          }
          if (error.empty()) {
            robot->native = std::move(native);
            robot->configuration = member.configuration();
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
      if (error.empty()) {
        robot->closing = false;
        std::shared_ptr<Context> released;
        {
          std::lock_guard<std::mutex> lock(table_mutex_);
          robots_[namespace_entry->second] = robot;
          uses_[useKey(member.identity())] =
              std::make_shared<Use>(Use{member.identity(), robot});
          ++members_revision_;
          notifyStatus();
          if (previous && previous != robot) {
            bool used = false;
            for (const auto &other : uses_)
              if (other.second->robot == previous)
                used = true;
            if (!used) {
              released = previous;
              previous->closing = true;
              robots_.erase(
                  previous->configuration.parameters().at("namespace"));
            }
          }
        }
        if (previous && previous != robot)
          fenceCommands(previous, &member.identity());
        if (released) {
          RosMasterDeadline deadline(std::chrono::seconds(3));
          released->native->Stop();
        }
      } else {
        item->mutable_error()->set_class_(error_class);
        item->mutable_error()->set_code("member-registration-failed");
        item->mutable_error()->set_message(error);
      }
    }
    return result;
  }
  wire::RemoveMembersResponse remove(const wire::RemoveMembersRequest &request,
                                     const xgc2::xrpc::GrpcWorkPermit &work) {
    RosMasterDeadline deadline(std::chrono::seconds(10));
    wire::RemoveMembersResponse result;
    for (const auto &identity : request.members()) {
      auto *item = result.add_members();
      *item->mutable_identity() = identity;
      if (work.cancelled()) {
        item->mutable_error()->set_class_(operation::ERROR_CLASS_CANCELLED);
        item->mutable_error()->set_code("member-removal-cancelled");
        continue;
      }
      std::shared_ptr<Context> released;
      std::shared_ptr<Context> affected;
      {
        std::lock_guard<std::mutex> lock(table_mutex_);
        const auto use = uses_.find(useKey(identity));
        if (use == uses_.end())
          continue;
        if (use->second->identity.connection_epoch() !=
            identity.connection_epoch()) {
          item->mutable_error()->set_class_(operation::ERROR_CLASS_REJECTED);
          item->mutable_error()->set_code("member-epoch-conflict");
          continue;
        }
        auto robot = use->second->robot;
        affected = robot;
        uses_.erase(use);
        ++members_revision_;
        notifyStatus();
        bool used = false;
        for (const auto &other : uses_)
          if (other.second->robot == robot)
            used = true;
        if (!used) {
          released = robot;
          robot->closing = true;
          robots_.erase(robot->configuration.parameters().at("namespace"));
        }
      }
      if (affected)
        fenceCommands(affected, &identity);
      if (released) {
        // Stop fences native callbacks before we acknowledge removal. Scheduler
        // sees closing and cannot publish or submit another side effect.
        released->native->Stop();
      }
    }
    return result;
  }
  void fenceCommands(const std::shared_ptr<Context> &robot,
                     const wire::MemberIdentity *identity) {
    const auto key = identity ? useKey(*identity) : std::string();
    auto done = std::make_shared<std::promise<void>>();
    auto ready = done->get_future();
    if (!scheduler_alive_.load(std::memory_order_acquire)) {
      // The scheduler flag changes only after its native services destruct.
      if (robot->active_call)
        finishExecute(
            robot->active_call,
            CommandError(robot->active_call->request.operation(),
                         operation::ERROR_CLASS_UNCERTAIN,
                         "native-result-unknown",
                         "native scheduler exited; reconcile outcome"));
      robot->active_call.reset();
      robot->native_call = 0;
      robot->in_flight = false;
      for (auto &command : robot->commands)
        finishExecute(command.call,
                      CommandError(command.call->request.operation(),
                                   operation::ERROR_CLASS_CANCELLED,
                                   "server-stopped",
                                   "not executed: native scheduler exited"));
      robot->commands.clear();
      return;
    }
    postFence(
        {[this, robot, key, done] {
           if (robot->native_call &&
               (key.empty() || useKey(robot->active_identity) == key))
             services_->Cancel(robot->native_call);
           for (auto i = robot->commands.begin(); i != robot->commands.end();) {
             if (key.empty() || useKey(i->identity) == key) {
               finishExecute(i->call,
                             CommandError(i->call->request.operation(),
                                          operation::ERROR_CLASS_CANCELLED,
                                          "member-removed",
                                          "not executed: member removed"));
               i = robot->commands.erase(i);
             } else
               ++i;
           }
           services_->Pump(std::chrono::milliseconds(0));
           done->set_value();
         },
         [done](bool) {
           done->set_exception(std::make_exception_ptr(
               std::runtime_error("native lifecycle barrier interrupted")));
         }});
    while (ready.wait_for(std::chrono::milliseconds(20)) !=
           std::future_status::ready) {
      if (!scheduler_alive_.load(std::memory_order_acquire)) {
        Job pending;
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          pending = std::move(fence_job_);
          fence_job_ = {};
        }
        abortJob(pending, false);
        fenceCommands(robot, identity);
        return;
      }
    }
    try {
      ready.get();
    } catch (...) {
      if (fatal_) {
        std::unique_lock<std::mutex> lock(startup_mutex_);
        startup_changed_.wait(lock, [this] {
          return !scheduler_alive_.load(std::memory_order_acquire);
        });
      }
      throw;
    }
  }
  std::array<ros::CallbackQueue, 2> callback_queues_;
  std::size_t next_callback_worker_ = 0;
  std::unique_ptr<ros::NodeHandle> node_;
  wire::RobotServerBootstrap bootstrap_;
  RobotFactory factory_;
  xgc2::xrpc::RuntimePolicy policy_;
  xgc2::xrpc::GrpcAdmission admission_;
  Service service_;
  operation::RuntimeServiceReference reference_;
  std::mutex status_mutex_;
  std::condition_variable status_changed_;
  std::atomic<std::uint64_t> status_version_{0};
  std::atomic<std::size_t> status_subscribers_{0}, blocking_calls_{0};
  std::mutex table_mutex_, jobs_mutex_, command_mutex_;
  std::map<std::string, std::shared_ptr<Use>> uses_;
  std::uint64_t members_revision_ = 0;
  std::map<std::string, std::shared_ptr<Context>> robots_;
  std::deque<Job> registration_jobs_;
  std::vector<Job> command_jobs_;
  Job fence_job_;
  std::condition_variable jobs_ready_;
  std::thread registration_, scheduler_;
  std::array<std::thread, 2> callbacks_;
  std::vector<std::shared_ptr<Context>> retained_robots_;
  std::mutex startup_mutex_;
  std::condition_variable startup_changed_;
  std::size_t ready_workers_ = 0;
  std::atomic<bool> fatal_{false}, scheduler_alive_{false};
  AsyncRosServices *services_ = nullptr;
  std::atomic<bool> shutdown_{false}, stopping_{false},
      management_stopping_{false}, callbacks_stopping_{false};
};
} // namespace

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
int RunRobotServer(int argc, char **argv, const std::string &node_name,
                   const std::string &provider, RobotFactory factory) {
  try {
    const auto arguments = ParseRobotServerArguments(argc, argv, provider);
    auto policy = StartupPolicy();
    if (arguments.check)
      return CheckRobotServer(arguments, provider, policy);
    ConfigureBoundedRosMaster();
    auto bootstrap = ReadRobotServerBootstrap(arguments, provider);
    for (const auto &name : {std::make_pair("ros_master_uri", "ROS_MASTER_URI"),
                             std::make_pair("ros_ip", "ROS_IP")}) {
      const auto found = bootstrap.ros_environment().find(name.first);
      if (found != bootstrap.ros_environment().end() && !found->second.empty())
        setenv(name.second, found->second.c_str(), 1);
    }
    if (bootstrap.ros_environment().count("ros_ip"))
      unsetenv("ROS_HOSTNAME");
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
    ros::master::setRetryTimeout(ros::WallDuration(3));
    Server server(std::move(bootstrap), std::move(factory), std::move(policy));
    return server.run();
  } catch (const std::exception &exception) {
    std::cerr << "robot server: " << exception.what() << '\n';
    return 1;
  }
}
} // namespace xgc2_ros1_robot_adapter
