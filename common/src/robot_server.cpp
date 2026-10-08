#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include "xgc2_ros1_robot_adapter/robot_server_entry.hpp"
#include <grpcpp/grpcpp.h>
#include <google/protobuf/util/message_differencer.h>
#include <ros/master.h>
#include <ros/callback_queue.h>
#include <array>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <mutex>
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
volatile std::sig_atomic_t signal_stop = 0;
void stopSignal(int) { signal_stop = 1; }
std::string useKey(const wire::MemberIdentity &id) {
  return id.target_id() + '\0' + id.run_id() + '\0' + id.robot_id();
}
bool validIdentity(const wire::MemberIdentity &id) {
  return ValidRobotIdentity(id.robot_id()) && !id.target_id().empty() && !id.run_id().empty() && !id.robot_id().empty() &&
         id.connection_epoch() && id.target_id().find('\0') == std::string::npos &&
         id.run_id().find('\0') == std::string::npos && id.robot_id().find('\0') == std::string::npos;
}
RobotConfig configFrom(const wire::RobotResource &wire) {
  RobotConfig result;
  result.robot_id = wire.robot_id(); result.profile_id = wire.profile_id(); result.profile_digest = wire.profile_digest();
  result.parameters.insert(wire.parameters().begin(), wire.parameters().end());
  for (const auto &channel : wire.channels()) result.channels.push_back({channel.channel_id(), channel.enabled()});
  return result;
}
struct Tag {
  std::function<void(bool)> callback;
  void proceed(bool ok) { auto function = std::move(callback); delete this; function(ok); }
};
template<class Request, class Response> struct Unary : std::enable_shared_from_this<Unary<Request, Response>> {
  grpc::ServerContext context;
  Request request;
  Response response;
  grpc::ServerAsyncResponseWriter<Response> writer{&context};
  std::atomic<bool> cancelled{false};
  Tag *done_tag = nullptr;
  void finish(Response result) {
    response = std::move(result);
    auto self = this->shared_from_this();
    writer.Finish(response, grpc::Status::OK, new Tag{[self](bool) {}});
  }
  template<class Accept, class Handle> static void listen(grpc::ServerCompletionQueue *cq, Accept accept, Handle handle) {
    auto call = std::make_shared<Unary>();
    call->done_tag = new Tag{[call](bool) { call->done_tag = nullptr; call->cancelled = call->context.IsCancelled(); }};
    call->context.AsyncNotifyWhenDone(call->done_tag);
    accept(&call->context, &call->request, &call->writer, cq, cq,
      new Tag{[call, cq, accept, handle](bool ok) {
        if (!ok) { delete call->done_tag; call->done_tag = nullptr; return; }
        listen(cq, accept, handle);
        handle(call);
      }});
  }
};
void finishExecute(const std::shared_ptr<Unary<wire::ExecuteRequest, wire::ExecuteResponse>> &call,
                   operation::OperationEvent result) {
  wire::ExecuteResponse response;
  *response.mutable_event() = std::move(result);
  call->finish(std::move(response));
}
class Server {
  struct Context;
  struct Use { wire::MemberIdentity identity; std::shared_ptr<Context> robot; };
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
    std::shared_ptr<Unary<wire::ExecuteRequest, wire::ExecuteResponse>> active_call;
    wire::MemberIdentity active_identity;
    std::mutex native_mutex; // only short non-network native access
  };
  struct Stream : std::enable_shared_from_this<Stream> {
    grpc::ServerContext context;
    wire::SubscribeStatusRequest request;
    grpc::ServerAsyncWriter<wire::SubscribeStatusResponse> writer{&context};
    wire::SubscribeStatusResponse sending;
    std::map<std::string, std::map<std::string, std::uint64_t>> seen;
    std::map<std::string, wire::MemberIdentity> identities;
    bool writing = false, finished = false;
    Tag *done_tag = nullptr;
    std::atomic<bool> cancelled{false};
  };
public:
  Server(wire::RobotServerBootstrap bootstrap, RobotFactory factory)
      : bootstrap_(std::move(bootstrap)), factory_(std::move(factory)) {}
  int run() {
    RobotSocketOwner socket_owner(bootstrap_.socket_path());
    grpc::ServerBuilder builder;
    builder.SetMaxReceiveMessageSize(16 * 1024 * 1024);
    builder.AddListeningPort("unix:" + bootstrap_.socket_path(), grpc::InsecureServerCredentials());
    builder.RegisterService(&service_);
    cq_ = builder.AddCompletionQueue();
    server_ = builder.BuildAndStart();
    if (!server_) throw std::runtime_error("robot server could not bind its UDS");
    socket_owner.RecordBoundSocket();
    registration_ = std::thread([this] { registrationLoop(); });
    scheduler_ = std::thread([this] { commandLoop(); });
    listen(); listenStream();
    std::array<std::thread, 2> callbacks;
    for (std::size_t index = 0; index < callbacks.size(); ++index) callbacks[index] = std::thread([this, index] {
      pthread_setname_np(pthread_self(), "robot-callback");
      // A member's callbacks stay on one of the two shared workers. A hot
      // member cannot occupy both workers while they contend for its mutex.
      while (!callbacks_stopping_) callback_queues_[index].callOne(ros::WallDuration(0.02));
    });
    pthread_setname_np(pthread_self(), "robot-grpc");
    while (!signal_stop && !shutdown_ && (!ros::isStarted() || ros::ok())) {
      void *tag; bool ok;
      if (cq_->AsyncNext(&tag, &ok, std::chrono::system_clock::now() + std::chrono::milliseconds(10)) == grpc::CompletionQueue::GOT_EVENT)
        static_cast<Tag *>(tag)->proceed(ok);
      flushStreams();
    }
    // Reject new RPCs, then drain both fixed workers before destroying CQ tags.
    server_->Shutdown(std::chrono::system_clock::now());
    management_stopping_ = true; jobs_ready_.notify_all();
    registration_.join();
    stopping_ = true; scheduler_.join();
    callbacks_stopping_ = true;
    for (auto &queue : callback_queues_) queue.disable();
    for (auto &worker : callbacks) worker.join();
    cq_->Shutdown();
    void *tag; bool ok;
    while (cq_->Next(&tag, &ok)) static_cast<Tag *>(tag)->proceed(ok);
    streams_.clear();
    { RosMasterDeadline deadline(std::chrono::seconds(3)); ros::shutdown(); }
    return 0;
  }
private:
  void postRegistration(std::function<void()> job) {
    { std::lock_guard<std::mutex> lock(jobs_mutex_); registration_jobs_.push_back(std::move(job)); }
    jobs_ready_.notify_one();
  }
  void postCommand(std::function<void()> job) {
    std::lock_guard<std::mutex> lock(command_mutex_); command_jobs_.push_back(std::move(job));
  }
  void registrationLoop() {
    pthread_setname_np(pthread_self(), "ros-library");
    {
      RosMasterDeadline deadline(std::chrono::seconds(3));
      node_.reset(new ros::NodeHandle());
      node_->setCallbackQueue(&callback_queues_[0]);
    }
    pthread_setname_np(pthread_self(), "robot-members");
    for (;;) {
      std::function<void()> job;
      {
        std::unique_lock<std::mutex> lock(jobs_mutex_);
        jobs_ready_.wait(lock, [this] { return management_stopping_ || !registration_jobs_.empty(); });
        if (registration_jobs_.empty()) break;
        job = std::move(registration_jobs_.front()); registration_jobs_.pop_front();
      }
      job();
    }
    std::vector<std::shared_ptr<Context>> robots;
    { std::lock_guard<std::mutex> lock(table_mutex_); for (auto &entry : robots_) { entry.second->closing = true; robots.push_back(entry.second); } uses_.clear(); robots_.clear(); ++members_revision_; }
    RosMasterDeadline cleanup(std::chrono::seconds(5));
    for (auto &robot : robots) { fenceCommands(robot, nullptr); robot->native->Stop(); }
  }
  void commandLoop() {
    pthread_setname_np(pthread_self(), "robot-commands");
    AsyncRosServices services;
    services_ = &services;
    auto next_periodic = Steady::now();
    std::vector<std::function<void()>> jobs;
    std::vector<std::shared_ptr<Context>> robots;
    std::uint64_t revision = UINT64_MAX;
    while (!stopping_) {
      jobs.clear();
      { std::lock_guard<std::mutex> lock(command_mutex_); jobs.swap(command_jobs_); }
      for (auto &job : jobs) job();
      { std::lock_guard<std::mutex> lock(table_mutex_);
        if (revision != members_revision_) {
          robots.clear(); robots.reserve(robots_.size());
          for (auto &entry : robots_) robots.push_back(entry.second);
          revision = members_revision_;
        }
      }
      const bool periodic = Steady::now() >= next_periodic;
      if (periodic) next_periodic = Steady::now() + std::chrono::milliseconds(100);
      for (auto &robot : robots) {
        if (robot->closing) { if (robot->native_call) services.Cancel(robot->native_call); continue; }
        if (robot->native_call && robot->active_call && robot->active_call->cancelled) services.Cancel(robot->native_call);
        if (periodic) { std::lock_guard<std::mutex> lock(robot->native_mutex); if (!robot->closing) robot->native->Periodic(ros::WallTime::now()); }
        if (!robot->in_flight && !robot->commands.empty()) dispatch(robot, services);
      }
      services.Pump(std::chrono::milliseconds(2));
    }
    // Never replay queued side effects on shutdown. Closing the real I/O ends
    // local waits and returns Unknown if any request bytes may have arrived.
    robots.clear();
    { std::lock_guard<std::mutex> lock(table_mutex_); for (auto &entry : robots_) robots.push_back(entry.second); }
    for (auto &robot : robots) {
      if (robot->native_call) services.Cancel(robot->native_call);
      for (auto &command : robot->commands) finishExecute(command.call, CommandError(command.call->request.operation(), operation::ERROR_CLASS_CANCELLED, "server-stopped", "not executed: server shutdown"));
      robot->commands.clear();
    }
    services.Pump(std::chrono::milliseconds(0));
  }
  void dispatch(const std::shared_ptr<Context> &robot, AsyncRosServices &services) {
    auto command = std::move(robot->commands.front()); robot->commands.pop_front();
    auto &request = command.call->request.operation();
    bool current;
    { std::lock_guard<std::mutex> lock(table_mutex_); const auto use = uses_.find(useKey(command.identity));
      current = use != uses_.end() && use->second->robot == robot && use->second->identity.connection_epoch() == command.identity.connection_epoch(); }
    if (!current || robot->uncertain) {
      finishExecute(command.call, CommandError(request, operation::ERROR_CLASS_REJECTED, robot->uncertain ? "previous-result-unknown" : "member-removed",
                                       robot->uncertain ? "not executed: previous native result is unknown" : "not executed: member cancelled or removed")); return;
    }
    if (command.call->cancelled) {
      finishExecute(command.call, CommandError(request, operation::ERROR_CLASS_CANCELLED, "operation-cancelled", "not executed: call cancelled")); return;
    }
    if (request.context().deadline().deadline_unix_nanos() <=
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()) {
      finishExecute(command.call, CommandError(request, operation::ERROR_CLASS_DEADLINE, "deadline-exceeded", "not executed: operation deadline elapsed")); return;
    }
    robot->in_flight = true;
    robot->active_call = command.call; robot->active_identity = command.identity;
    std::lock_guard<std::mutex> lock(robot->native_mutex);
    robot->native_call = robot->native->Execute(request, services, [robot, command](operation::OperationEvent result) {
      robot->in_flight = false; robot->native_call = 0;
      robot->active_call.reset();
      if (result.phase() == operation::OPERATION_PHASE_UNCERTAIN) robot->uncertain = true;
      finishExecute(command.call, std::move(result));
    });
  }
  void listen() {
    Unary<wire::ApplyMembersRequest, wire::ApplyMembersResponse>::listen(cq_.get(),
      [this](auto... args) { service_.RequestApplyMembers(args...); }, [this](auto call) {
        postRegistration([this, call] { call->finish(apply(call->request, call->cancelled)); });
      });
    Unary<wire::RemoveMembersRequest, wire::RemoveMembersResponse>::listen(cq_.get(),
      [this](auto... args) { service_.RequestRemoveMembers(args...); }, [this](auto call) {
        postRegistration([this, call] { call->finish(remove(call->request)); });
      });
    Unary<wire::ExecuteRequest, wire::ExecuteResponse>::listen(cq_.get(),
      [this](auto... args) { service_.RequestExecute(args...); }, [this](auto call) {
        postCommand([this, call] {
          std::shared_ptr<Context> robot;
          { std::lock_guard<std::mutex> lock(table_mutex_); const auto use = uses_.find(useKey(call->request.identity()));
            if (use != uses_.end() && use->second->identity.connection_epoch() == call->request.identity().connection_epoch()) robot = use->second->robot; }
          if (!robot || robot->closing) { finishExecute(call, CommandError(call->request.operation(), operation::ERROR_CLASS_REJECTED, "member-unavailable", "not executed: active member required")); return; }
          robot->commands.push_back({call, call->request.identity()});
        });
      });
    Unary<wire::HealthRequest, wire::HealthResponse>::listen(cq_.get(),
      [this](auto... args) { service_.RequestHealth(args...); }, [this](auto call) { wire::HealthResponse result; result.set_serving(!shutdown_); call->finish(std::move(result)); });
    Unary<wire::ShutdownRequest, wire::ShutdownResponse>::listen(cq_.get(),
      [this](auto... args) { service_.RequestShutdown(args...); }, [this](auto call) { wire::ShutdownResponse result; result.set_serving(false); call->finish(std::move(result)); shutdown_ = true; });
  }
  void listenStream() {
    auto stream = std::make_shared<Stream>();
    stream->done_tag = new Tag{[stream](bool) { stream->done_tag = nullptr; stream->cancelled = stream->context.IsCancelled(); }};
    stream->context.AsyncNotifyWhenDone(stream->done_tag);
    service_.RequestSubscribeStatus(&stream->context, &stream->request, &stream->writer, cq_.get(), cq_.get(), new Tag{[this, stream](bool ok) {
      if (!ok) { delete stream->done_tag; stream->done_tag = nullptr; return; }
      listenStream(); streams_.push_back(stream);
    }});
  }
  void flushStreams() {
    // Membership snapshots change only on management mutations. Hot status
    // turns reuse routes and protobuf storage instead of copying the full map.
    { std::lock_guard<std::mutex> lock(table_mutex_);
      if (status_revision_ != members_revision_) { status_uses_ = uses_; status_revision_ = members_revision_; }
    }
    const auto &uses = status_uses_;
    for (auto &stream : streams_) {
      if (stream->writing || stream->finished) continue;
      if (stream->cancelled) {
        stream->finished = true;
        stream->writer.Finish(grpc::Status::OK, new Tag{[stream](bool) {}}); continue;
      }
      stream->sending.Clear();
      for (auto i = stream->identities.begin(); i != stream->identities.end();) {
        const auto current = uses.find(i->first);
        if (current == uses.end() || current->second->identity.connection_epoch() != i->second.connection_epoch()) {
          auto *member = stream->sending.add_members(); *member->mutable_identity() = i->second; member->set_removed(true);
          stream->seen.erase(i->first); i = stream->identities.erase(i);
        } else ++i;
      }
      for (const auto &entry : uses) {
        const bool fresh = !stream->identities.count(entry.first);
        if (fresh) stream->identities.emplace(entry.first, entry.second->identity);
        auto &seen = stream->seen[entry.first];
        wire::MemberStatus *member = nullptr;
        std::lock_guard<std::mutex> lock(entry.second->robot->state_mutex);
        for (const auto &channel : entry.second->robot->latest) {
          if (fresh || seen[channel.first] != channel.second.first) {
            if (!member) { member = stream->sending.add_members(); *member->mutable_identity() = entry.second->identity; }
            auto *message = member->add_messages(); *message = channel.second.second;
            message->set_robot_id(entry.second->identity.robot_id()); seen[channel.first] = channel.second.first;
          }
        }
        if (fresh && !member) { member = stream->sending.add_members(); *member->mutable_identity() = entry.second->identity; }
      }
      if (!stream->sending.members_size()) continue;
      stream->writing = true;
      stream->writer.Write(stream->sending, new Tag{[stream](bool ok) { stream->writing = false; if (!ok) stream->cancelled = true; }});
    }
    streams_.erase(std::remove_if(streams_.begin(), streams_.end(), [](const auto &stream) { return stream->finished; }), streams_.end());
  }
  wire::ApplyMembersResponse apply(const wire::ApplyMembersRequest &request, const std::atomic<bool> &cancelled) {
    wire::ApplyMembersResponse result;
    for (const auto &member : request.members()) {
      auto *item = result.add_members(); *item->mutable_identity() = member.identity();
      std::string error;
      if (cancelled || shutdown_ || signal_stop) error = "member registration cancelled";
      else if (!validIdentity(member.identity()) || member.configuration().robot_id() != member.identity().robot_id()) error = "invalid member identity";
      const auto config = configFrom(member.configuration());
      const auto namespace_entry = config.parameters.find("namespace");
      if (namespace_entry == config.parameters.end() || namespace_entry->second.empty()) error = "native namespace required";
      for (const auto &name : {"ros_master_uri", "ros_ip"}) {
        const auto value = config.parameters.find(name); const auto expected = bootstrap_.ros_environment().find(name);
        if (value != config.parameters.end() && !value->second.empty() &&
            (expected == bootstrap_.ros_environment().end() || expected->second != value->second)) error = "member crosses server ROS environment";
      }
      std::shared_ptr<Context> robot, previous;
      bool update = false;
      if (error.empty()) {
        std::lock_guard<std::mutex> lock(table_mutex_);
        const auto use = uses_.find(useKey(member.identity()));
        if (use != uses_.end()) {
          previous = use->second->robot;
          if (use->second->identity.connection_epoch() != member.identity().connection_epoch()) error = "active member connection epoch conflicts";
        }
        const auto found = robots_.find(namespace_entry->second);
        if (found != robots_.end()) {
          robot = found->second;
          update = robot->closing && previous == robot;
          auto native_config = robot->configuration, requested = member.configuration();
          if (!google::protobuf::util::MessageDifferencer::Equals(native_config, requested)) {
            update = previous == robot;
            for (const auto &other : uses_) if (other.second->robot == robot && other.first != useKey(member.identity())) update = false;
            if (!update) error = "native ROS resource configuration conflicts";
          }
        }
      }
      if (error.empty() && (update || !robot)) {
        RosMasterDeadline deadline(std::chrono::seconds(3));
        if (update) {
          robot->closing = true;
          fenceCommands(robot, nullptr);
          robot->native->Stop();
          std::lock_guard<std::mutex> lock(robot->state_mutex); robot->latest.clear();
        } else {
          robot = std::make_shared<Context>();
          robot->callback_worker = next_callback_worker_++ % callback_queues_.size();
        }
        const std::weak_ptr<Context> weak = robot;
        ros::NodeHandle robot_node(*node_);
        robot_node.setCallbackQueue(&callback_queues_[robot->callback_worker]);
        auto native = factory_(robot_node, config, [weak](wire::RobotMessage message) {
          auto robot = weak.lock(); if (!robot || robot->closing) return;
          std::lock_guard<std::mutex> lock(robot->state_mutex);
          auto &slot = robot->latest[message.channel_id()]; slot.first = ++robot->state_version; slot.second = std::move(message);
        }, &error);
        if (deadline.failed() && error.empty()) error = "ROS master registration did not complete";
        if (!native && error.empty()) error = "native resource creation failed";
        if (!error.empty() && native) native->Stop();
        if (error.empty()) { robot->native = std::move(native); robot->configuration = member.configuration(); }
      }
      if (error.empty()) {
        robot->closing = false;
        std::shared_ptr<Context> released;
        {
        std::lock_guard<std::mutex> lock(table_mutex_);
        robots_[namespace_entry->second] = robot;
        uses_[useKey(member.identity())] = std::make_shared<Use>(Use{member.identity(), robot});
        ++members_revision_;
        if (previous && previous != robot) {
          bool used = false; for (const auto &other : uses_) if (other.second->robot == previous) used = true;
          if (!used) { released = previous; previous->closing = true; robots_.erase(previous->configuration.parameters().at("namespace")); }
        }
        }
        if (previous && previous != robot) fenceCommands(previous, &member.identity());
        if (released) { RosMasterDeadline deadline(std::chrono::seconds(3)); released->native->Stop(); }
      } else {
        item->mutable_error()->set_class_(operation::ERROR_CLASS_REJECTED);
        item->mutable_error()->set_code("member-registration-failed"); item->mutable_error()->set_message(error);
      }
    }
    return result;
  }
  wire::RemoveMembersResponse remove(const wire::RemoveMembersRequest &request) {
    RosMasterDeadline deadline(std::chrono::seconds(10));
    wire::RemoveMembersResponse result;
    for (const auto &identity : request.members()) {
      auto *item = result.add_members(); *item->mutable_identity() = identity;
      std::shared_ptr<Context> released;
      std::shared_ptr<Context> affected;
      { std::lock_guard<std::mutex> lock(table_mutex_);
        const auto use = uses_.find(useKey(identity));
        if (use == uses_.end()) continue;
        if (use->second->identity.connection_epoch() != identity.connection_epoch()) {
          item->mutable_error()->set_class_(operation::ERROR_CLASS_REJECTED); item->mutable_error()->set_code("member-epoch-conflict"); continue;
        }
        auto robot = use->second->robot; affected = robot; uses_.erase(use); ++members_revision_;
        bool used = false; for (const auto &other : uses_) if (other.second->robot == robot) used = true;
        if (!used) { released = robot; robot->closing = true; robots_.erase(robot->configuration.parameters().at("namespace")); }
      }
      if (affected) fenceCommands(affected, &identity);
      if (released) {
        // Stop fences native callbacks before we acknowledge removal. Scheduler
        // sees closing and cannot publish or submit another side effect.
        released->native->Stop();
      }
    }
    return result;
  }
  void fenceCommands(const std::shared_ptr<Context> &robot, const wire::MemberIdentity *identity) {
    const auto key = identity ? useKey(*identity) : std::string();
    auto done = std::make_shared<std::promise<void>>(); auto ready = done->get_future();
    postCommand([this, robot, key, done] {
      if (robot->native_call && (key.empty() || useKey(robot->active_identity) == key)) services_->Cancel(robot->native_call);
      for (auto i = robot->commands.begin(); i != robot->commands.end();) {
        if (key.empty() || useKey(i->identity) == key) {
          finishExecute(i->call, CommandError(i->call->request.operation(), operation::ERROR_CLASS_CANCELLED, "member-removed", "not executed: member removed"));
          i = robot->commands.erase(i);
        } else ++i;
      }
      services_->Pump(std::chrono::milliseconds(0));
      done->set_value();
    });
    ready.get();
  }
  std::array<ros::CallbackQueue, 2> callback_queues_;
  std::size_t next_callback_worker_ = 0;
  std::unique_ptr<ros::NodeHandle> node_;
  wire::RobotServerBootstrap bootstrap_;
  RobotFactory factory_;
  wire::RobotAdapterServerService::AsyncService service_;
  std::unique_ptr<grpc::Server> server_;
  std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  std::vector<std::shared_ptr<Stream>> streams_;
  std::mutex table_mutex_, jobs_mutex_, command_mutex_;
  std::map<std::string, std::shared_ptr<Use>> uses_, status_uses_;
  std::uint64_t members_revision_ = 0, status_revision_ = UINT64_MAX;
  std::map<std::string, std::shared_ptr<Context>> robots_;
  std::deque<std::function<void()>> registration_jobs_;
  std::vector<std::function<void()>> command_jobs_;
  std::condition_variable jobs_ready_;
  std::thread registration_, scheduler_;
  AsyncRosServices *services_ = nullptr;
  std::atomic<bool> shutdown_{false}, stopping_{false}, management_stopping_{false}, callbacks_stopping_{false};
};
} // namespace

operation::OperationEvent CommandError(const operation::OperationRequest &request, operation::ErrorClass error_class,
                                      const std::string &code, const std::string &detail) {
  operation::OperationEvent event;
  event.set_work_id(request.context().work_id());
  event.set_occurred_unix_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
  event.set_phase(error_class == operation::ERROR_CLASS_UNCERTAIN ? operation::OPERATION_PHASE_UNCERTAIN :
                  error_class == operation::ERROR_CLASS_REJECTED ? operation::OPERATION_PHASE_REJECTED :
                  error_class == operation::ERROR_CLASS_CANCELLED ? operation::OPERATION_PHASE_CANCELLED :
                  error_class == operation::ERROR_CLASS_DEADLINE ? operation::OPERATION_PHASE_EXPIRED : operation::OPERATION_PHASE_FAILED);
  auto *error = event.mutable_error(); error->set_class_(error_class); error->set_code(code); error->set_message(detail);
  return event;
}
int RunRobotServer(int argc, char **argv, const std::string &node_name,
                   const std::string &provider, RobotFactory factory) {
  try {
    const auto arguments = ParseRobotServerArguments(argc, argv, provider);
    if (arguments.check) {
      auto stub = wire::RobotAdapterServerService::NewStub(grpc::CreateChannel(
          "unix:" + arguments.socket_path, grpc::InsecureChannelCredentials()));
      grpc::ClientContext context;
      context.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(arguments.timeout_ms));
      context.set_wait_for_ready(true);
      wire::HealthRequest request;
      wire::HealthResponse response;
      const auto status = stub->Health(&context, request, &response);
      if (!status.ok() || !response.serving()) {
        std::cerr << "robot server health: " << (status.ok() ? "not serving" : status.error_message()) << '\n';
        return 1;
      }
      return 0;
    }
    ConfigureBoundedRosMaster();
    wire::RobotServerBootstrap bootstrap;
    if (!arguments.bootstrap_file.empty()) {
      struct stat info{};
      if (lstat(arguments.bootstrap_file.c_str(), &info) || !S_ISREG(info.st_mode) ||
          info.st_size <= 0 || info.st_size > 65536 || (info.st_mode & 077))
        throw std::runtime_error("bootstrap must be a private bounded regular file");
      std::ifstream file(arguments.bootstrap_file, std::ios::binary);
      if (!bootstrap.ParseFromIstream(&file) || bootstrap.provider_definition_id() != provider)
        throw std::runtime_error("invalid robot server bootstrap");
      ValidateRobotSocketPath(bootstrap.socket_path());
    } else {
      bootstrap.set_socket_path(arguments.socket_path);
      bootstrap.set_provider_definition_id(arguments.provider);
      if (!arguments.ros_master_uri.empty()) (*bootstrap.mutable_ros_environment())["ros_master_uri"] = arguments.ros_master_uri;
      if (!arguments.ros_ip.empty()) (*bootstrap.mutable_ros_environment())["ros_ip"] = arguments.ros_ip;
    }
    for (const auto &name : {std::make_pair("ros_master_uri", "ROS_MASTER_URI"), std::make_pair("ros_ip", "ROS_IP")}) {
      const auto found = bootstrap.ros_environment().find(name.first);
      if (found != bootstrap.ros_environment().end() && !found->second.empty()) setenv(name.second, found->second.c_str(), 1);
    }
    if (bootstrap.ros_environment().count("ros_ip")) unsetenv("ROS_HOSTNAME");
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
    std::signal(SIGINT, stopSignal); std::signal(SIGTERM, stopSignal);
    ros::master::setRetryTimeout(ros::WallDuration(3));
    Server server(std::move(bootstrap), std::move(factory));
    return server.run();
  } catch (const std::exception &exception) {
    std::cerr << "robot server: " << exception.what() << '\n'; return 1;
  }
}
} // namespace xgc2_ros1_robot_adapter
