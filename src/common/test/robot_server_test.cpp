// The type server core against a fake RosBinding: real gRPC over a Unix
// socket, no ROS. The fake binding scripts the master and records what the
// server does to its robots, so the lifecycle can be asserted exactly.
#include "xgc/robot/v1/server.grpc.pb.h"
#include "xgc2/xrpc/grpc.hpp"
#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <dirent.h>
#include <gtest/gtest.h>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace xgc2_ros1_robot_adapter {
namespace {
namespace wire = xgc::robot::v1;
namespace operation = xgc::adapter::v1;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;
using xgc2::xrpc::GrpcClientCall;
using xgc2::xrpc::GrpcClock;

// What the server did to its robots and to the ROS binding, in order.
struct World {
  std::mutex mutex;
  std::vector<std::string> log;
  struct Pending {
    operation::OperationRequest request;
    CommandCompletion complete;
    bool cancelled = false;
  };
  std::map<std::uint64_t, Pending> pending;
  std::uint64_t next_call = 0;
  std::set<std::string> stopped;

  void record(const std::string &line) {
    std::lock_guard<std::mutex> lock(mutex);
    log.push_back(line);
  }
  std::vector<std::string> snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    return log;
  }
  std::size_t position(const std::string &line) {
    const auto lines = snapshot();
    const auto found = std::find(lines.begin(), lines.end(), line);
    return found == lines.end() ? lines.size() : found - lines.begin();
  }
  bool contains(const std::string &line) { return position(line) < snapshot().size(); }
};

operation::OperationEvent event(const operation::OperationRequest &request,
                                operation::OperationPhase phase) {
  operation::OperationEvent result;
  result.set_work_id(request.context().work_id());
  result.set_phase(phase);
  return result;
}

class FakeIo final : public CommandIo {
public:
  explicit FakeIo(World &world) : world_(world) {}
  void Cancel(std::uint64_t id) override {
    std::lock_guard<std::mutex> lock(world_.mutex);
    const auto found = world_.pending.find(id);
    if (found != world_.pending.end())
      found->second.cancelled = true;
  }
  void Pump(std::chrono::milliseconds wait) override {
    std::vector<std::pair<CommandCompletion, operation::OperationEvent>> ready;
    {
      std::lock_guard<std::mutex> lock(world_.mutex);
      for (auto i = world_.pending.begin(); i != world_.pending.end();) {
        if (i->second.cancelled) {
          ready.emplace_back(std::move(i->second.complete),
                             CommandError(i->second.request,
                                          operation::ERROR_CLASS_CANCELLED,
                                          "cancelled", "fake call cancelled"));
          world_.log.push_back("cancelled:" + i->second.request.context().work_id());
          i = world_.pending.erase(i);
        } else {
          ++i;
        }
      }
    }
    for (auto &entry : ready)
      entry.first(std::move(entry.second));
    std::this_thread::sleep_for(std::min(wait, milliseconds(2)));
  }

private:
  World &world_;
};

class FakeBinding final : public RosBinding {
public:
  explicit FakeBinding(World &world) : world_(world) {}
  // ---- script
  std::atomic<long> master_pid{4242};
  std::atomic<bool> master_run_known{true};
  std::atomic<bool> master_run_changed{false};
  std::atomic<bool> master_answers{true};
  std::atomic<bool> master_calls_fail{false}; // BoundMasterCalls reports failure
  std::atomic<bool> running{true};
  std::atomic<bool> hold_connect{false};
  void releaseConnect() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hold_connect = false;
    }
    changed_.notify_all();
  }
  ServerOptions initialized;
  std::string initialized_name;

  // ---- RosBinding
  void Init(const ServerOptions &options, const std::string &name) override {
    initialized = options;
    initialized_name = name;
    world_.record("init");
  }
  void Connect() override {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [this] { return !hold_connect.load(); });
    world_.record("connect");
  }
  bool Running() const override { return running; }
  bool ProbeMaster(MasterIdentity *identity) override {
    if (!master_answers)
      return false;
    identity->uri = "http://devbox:11311/";
    identity->pid = master_pid;
    identity->run_id = !master_run_known ? "" : master_run_changed
                                                   ? "master-run-2"
                                                   : "master-run-1";
    return true;
  }
  void Shutdown() override { world_.record("ros-shutdown"); }
  std::size_t Lanes() const override { return 2; }
  void PumpLane(std::size_t, std::chrono::milliseconds wait) override {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, wait, [this] { return lanes_disabled_; });
  }
  void DisableLanes() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lanes_disabled_ = true;
    }
    changed_.notify_all();
  }
  std::unique_ptr<RosNode> CreateNode(std::size_t) override {
    return std::unique_ptr<RosNode>(new RosNode());
  }
  std::unique_ptr<CommandIo> CreateCommandIo() override {
    return std::unique_ptr<CommandIo>(new FakeIo(world_));
  }
  std::unique_ptr<MasterBudget>
  BoundMasterCalls(std::chrono::milliseconds) override {
    struct Budget final : MasterBudget {
      explicit Budget(bool failure) : failure_(failure) {}
      bool failed() const override { return failure_; }
      bool failure_;
    };
    return std::unique_ptr<MasterBudget>(new Budget(master_calls_fail));
  }
  void FlushPublications(std::chrono::milliseconds) override {
    world_.record("flush");
  }

private:
  World &world_;
  std::mutex mutex_;
  std::condition_variable changed_;
  bool lanes_disabled_ = false;
};

// A robot whose commands are selected by the endpoint id of the request:
//   "instant"   succeeds at once
//   "slow"      stays in flight until the test finishes it
//   "uncertain" ends with an uncertain outcome
class FakeRobot final : public NativeRobot {
public:
  FakeRobot(World &world, RobotConfig config, TelemetryEmitter emit)
      : world_(world), config_(std::move(config)), emit_(std::move(emit)) {
    publish();
  }
  ~FakeRobot() override { world_.record("destroy:" + config_.robot_id); }
  void Stop() override {
    world_.record("stop:" + config_.robot_id);
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.stopped.insert(config_.robot_id);
  }
  void Periodic() override { ++ticks_; if (ticks_ % 5 == 0) publish(); }
  std::uint64_t Execute(const operation::OperationRequest &request, CommandIo &,
                        CommandCompletion complete) override {
    const auto &kind = request.context().endpoint_id();
    world_.record("execute:" + config_.robot_id + ":" + kind);
    if (kind == "instant") {
      complete(event(request, operation::OPERATION_PHASE_SUCCEEDED));
      return 0;
    }
    if (kind == "uncertain") {
      complete(CommandError(request, operation::ERROR_CLASS_UNCERTAIN,
                            "native-result-unknown", "result unknown"));
      return 0;
    }
    std::lock_guard<std::mutex> lock(world_.mutex);
    const auto id = ++world_.next_call;
    world_.pending[id] = {request, std::move(complete), false};
    return id;
  }

private:
  void publish() {
    xgc::robot::v1::RobotMessage message;
    message.set_robot_id(config_.robot_id);
    message.set_channel_id("state.test");
    message.mutable_message()->set_sequence(++sequence_);
    emit_(std::move(message));
  }
  World &world_;
  RobotConfig config_;
  TelemetryEmitter emit_;
  std::uint64_t sequence_ = 0;
  unsigned ticks_ = 0;
};

RobotFactory factoryFor(World &world) {
  return [&world](RosNode &, const RobotConfig &config, TelemetryEmitter emit,
                  std::string *error) -> std::shared_ptr<NativeRobot> {
    if (config.profile_id == "rejected.profile") {
      *error = "profile does not match installed native mapping";
      return nullptr;
    }
    world.record("create:" + config.robot_id);
    return std::make_shared<FakeRobot>(world, config, std::move(emit));
  };
}

ServerTimings quickTimings() {
  ServerTimings timings;
  timings.master_probe_interval = milliseconds(20);
  timings.master_unreachable_after = milliseconds(300);
  timings.command_drain = milliseconds(400);
  timings.status_flush = milliseconds(150);
  timings.rpc_drain = milliseconds(2000);
  return timings;
}

std::string makePrivateDirectory() {
  char pattern[] = "/tmp/robot-server-test-XXXXXX";
  return mkdtemp(pattern);
}

wire::MemberIdentity identity(const std::string &robot, std::uint64_t epoch = 1,
                              const std::string &run = "run-test") {
  wire::MemberIdentity result;
  result.set_target_id("local");
  result.set_run_id(run);
  result.set_robot_id(robot);
  result.set_connection_epoch(epoch);
  return result;
}

wire::MemberRegistration registration(const std::string &robot,
                                      const std::string &robot_namespace,
                                      const std::string &profile = "fake.profile",
                                      std::uint64_t epoch = 1,
                                      const std::string &run = "run-test") {
  wire::MemberRegistration result;
  *result.mutable_identity() = identity(robot, epoch, run);
  auto *configuration = result.mutable_configuration();
  configuration->set_robot_id(robot);
  configuration->set_profile_id(profile);
  (*configuration->mutable_parameters())["namespace"] = robot_namespace;
  auto *channel = configuration->add_channels();
  channel->set_channel_id("state.test");
  channel->set_enabled(true);
  return result;
}

wire::ExecuteRequest command(const std::string &robot, const std::string &kind,
                             const std::string &work, std::uint64_t epoch = 1) {
  wire::ExecuteRequest request;
  *request.mutable_identity() = identity(robot, epoch);
  auto *context = request.mutable_operation()->mutable_context();
  context->set_work_id(work);
  context->set_endpoint_id(kind);
  context->mutable_deadline()->set_deadline_unix_nanos(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          (std::chrono::system_clock::now() + std::chrono::seconds(60))
              .time_since_epoch())
          .count());
  return request;
}

// One RPC outcome as a caller sees it.
template <class Response> struct Reply {
  grpc::Status status;
  xgc2::xrpc::Delivery delivery = xgc2::xrpc::Delivery::NotSent;
  Response response;
};

class Client {
public:
  explicit Client(const std::string &socket)
      : stub_(wire::RobotAdapterServerService::NewStub(
            xgc2::xrpc::make_grpc_unix_channel(socket))) {}

  Reply<wire::DescribeResponse> describe(std::uint32_t wait_ms = 0,
                                         milliseconds deadline = milliseconds(10000)) {
    Reply<wire::DescribeResponse> reply;
    grpc::ClientContext context;
    GrpcClientCall call(context, "", GrpcClock::now() + deadline, {}, "", true);
    wire::DescribeRequest request;
    request.set_wait_ready_ms(wait_ms);
    reply.status = call.invoke(
        [&] { return stub_->Describe(&context, request, &reply.response); });
    reply.delivery = call.delivery();
    if (reply.status.ok())
      instance_ = call.response_instance_id();
    return reply;
  }
  template <class Response, class Request, class Method>
  Reply<Response> call(const Request &request, Method method,
                       milliseconds deadline = milliseconds(10000)) {
    if (instance_.empty())
      describe();
    Reply<Response> reply;
    grpc::ClientContext context;
    GrpcClientCall call(context, instance_, GrpcClock::now() + deadline);
    reply.status = call.invoke([&] {
      return (stub_.get()->*method)(&context, request, &reply.response);
    });
    reply.delivery = call.delivery();
    return reply;
  }
  Reply<wire::ApplyMembersResponse> apply(const wire::ApplyMembersRequest &request) {
    return call<wire::ApplyMembersResponse>(
        request, &wire::RobotAdapterServerService::Stub::ApplyMembers);
  }
  Reply<wire::RemoveMembersResponse> remove(const wire::RemoveMembersRequest &request) {
    return call<wire::RemoveMembersResponse>(
        request, &wire::RobotAdapterServerService::Stub::RemoveMembers);
  }
  Reply<wire::ExecuteResponse> execute(const wire::ExecuteRequest &request,
                                       milliseconds deadline = milliseconds(10000)) {
    return call<wire::ExecuteResponse>(
        request, &wire::RobotAdapterServerService::Stub::Execute, deadline);
  }
  Reply<wire::HealthResponse> health() {
    return call<wire::HealthResponse>(
        wire::HealthRequest(), &wire::RobotAdapterServerService::Stub::Health);
  }
  Reply<wire::ShutdownResponse> shutdown(milliseconds deadline = milliseconds(10000)) {
    return call<wire::ShutdownResponse>(
        wire::ShutdownRequest(), &wire::RobotAdapterServerService::Stub::Shutdown,
        deadline);
  }

  // A status stream read on a thread; frames are kept for the test to inspect.
  class Watch {
  public:
    Watch(Client &client, milliseconds duration) {
      if (client.instance_.empty())
        client.describe();
      call_ = std::make_unique<GrpcClientCall>(
          context_, client.instance_,
          xgc2::xrpc::grpc_stream_deadline(xgc2::xrpc::GrpcLimits{},
                                           GrpcClock::now() + duration));
      if (!call_->mark_dispatched().ok())
        return;
      stream_ = client.stub_->SubscribeStatus(&context_, wire::SubscribeStatusRequest());
      if (!call_->receive_initial_metadata(*stream_).ok())
        return;
      reader_ = std::thread([this] {
        wire::SubscribeStatusResponse frame;
        while (stream_->Read(&frame)) {
          std::lock_guard<std::mutex> lock(mutex_);
          frames_.push_back(frame);
          changed_.notify_all();
        }
        const auto status = stream_->Finish();
        std::lock_guard<std::mutex> lock(mutex_);
        finished_ = true;
        status_ = status;
        changed_.notify_all();
      });
    }
    ~Watch() {
      if (reader_.joinable()) {
        context_.TryCancel();
        reader_.join();
      }
    }
    // Waits until a frame satisfies the predicate.
    template <class Predicate>
    bool wait(Predicate predicate, milliseconds budget = milliseconds(5000)) {
      std::unique_lock<std::mutex> lock(mutex_);
      return changed_.wait_for(lock, budget, [&] {
        for (const auto &frame : frames_)
          if (predicate(frame))
            return true;
        return false;
      });
    }
    bool waitFinished(milliseconds budget = milliseconds(5000)) {
      std::unique_lock<std::mutex> lock(mutex_);
      return changed_.wait_for(lock, budget, [&] { return finished_; });
    }
    grpc::Status status() {
      std::lock_guard<std::mutex> lock(mutex_);
      return status_;
    }

  private:
    grpc::ClientContext context_;
    std::unique_ptr<GrpcClientCall> call_;
    std::unique_ptr<grpc::ClientReader<wire::SubscribeStatusResponse>> stream_;
    std::thread reader_;
    std::mutex mutex_;
    std::condition_variable changed_;
    std::vector<wire::SubscribeStatusResponse> frames_;
    bool finished_ = false;
    grpc::Status status_;
  };

private:
  std::unique_ptr<wire::RobotAdapterServerService::Stub> stub_;
  std::string instance_;
};

// One server on its own thread.
class Fixture : public ::testing::Test {
protected:
  void SetUp() override {
    directory_ = makePrivateDirectory();
    binding_ = std::make_unique<FakeBinding>(world_);
  }
  void TearDown() override {
    binding_->releaseConnect();
    if (thread_.joinable()) {
      server_->RequestStop();
      thread_.join();
    }
    client_.reset();
    server_.reset();
    unlink(socket().c_str());
    rmdir(directory_.c_str());
  }
  std::string socket() const { return directory_ + "/server.sock"; }
  ServerOptions options() const {
    ServerOptions result;
    result.socket_path = socket();
    result.run_id = "run-test";
    result.node_name = "xgc_fake_run-test";
    result.provider = "fake-provider";
    result.target_id = "local";
    result.ros_master_uri = "http://127.0.0.1:11311";
    result.ros_ip = "127.0.0.1";
    return result;
  }
  void start(ServerTimings timings = quickTimings()) {
    server_ = std::make_unique<RobotServer>(options(), *binding_,
                                            factoryFor(world_), timings);
    finished_ = false;
    thread_ = std::thread([this] {
      exit_code_ = server_->Run();
      finished_ = true;
    });
    for (int attempt = 0;
         attempt < 500 && !finished_ && access(socket().c_str(), F_OK); ++attempt)
      std::this_thread::sleep_for(milliseconds(5));
    client_ = std::make_unique<Client>(socket());
  }
  void startReady() {
    start();
    const auto described = client_->describe(10000);
    ASSERT_TRUE(described.status.ok()) << described.status.error_message();
    ASSERT_TRUE(described.response.ready());
  }
  int join() {
    thread_.join();
    return exit_code_;
  }
  static nlohmann::json facts(const wire::DescribeResponse &response) {
    return nlohmann::json::parse(response.facts_json());
  }

  World world_;
  std::unique_ptr<FakeBinding> binding_;
  std::unique_ptr<RobotServer> server_;
  std::unique_ptr<Client> client_;
  std::thread thread_;
  std::string directory_;
  int exit_code_ = -1;
  std::atomic<bool> finished_{false};
};

using ServerLifecycle = Fixture;

TEST_F(ServerLifecycle, DescribeHoldsUntilTheRosNodeIsRegisteredThenReportsTheFacts) {
  binding_->hold_connect = true;
  start();
  const auto early = client_->describe(0);
  ASSERT_TRUE(early.status.ok());
  EXPECT_FALSE(early.response.ready());
  EXPECT_EQ("starting", facts(early.response)["state"]);
  EXPECT_EQ("xgc2.robot-adapter", early.response.service_ref().service());
  EXPECT_EQ("fake-provider", early.response.provider_definition_id());

  std::atomic<bool> answered{false};
  Reply<wire::DescribeResponse> held;
  const auto started = Clock::now();
  std::thread waiter([&] {
    Client second(socket());
    held = second.describe(8000);
    answered = true;
  });
  std::this_thread::sleep_for(milliseconds(250));
  EXPECT_FALSE(answered) << "a held Describe must not answer before readiness";
  binding_->releaseConnect();
  waiter.join();
  EXPECT_LT(Clock::now() - started, milliseconds(6000));
  ASSERT_TRUE(held.status.ok()) << held.status.error_message();
  EXPECT_TRUE(held.response.ready());
  const auto facts_json = facts(held.response);
  EXPECT_EQ("http://127.0.0.1:11311", facts_json["ros_master_uri"]);
  EXPECT_EQ("master-run-1", facts_json["ros_master_run_id"]);
  EXPECT_EQ(4242, facts_json["ros_master_pid"]);
  EXPECT_EQ("/xgc_fake_run_test", facts_json["ros_node"]);
  EXPECT_EQ("run-test", facts_json["run_id"]);
  EXPECT_EQ("ready", facts_json["state"]);
  EXPECT_EQ("xgc_fake_run_test", binding_->initialized_name);
  EXPECT_EQ("http://127.0.0.1:11311", binding_->initialized.ros_master_uri);
}

TEST_F(ServerLifecycle, ADescribeBudgetThatEndsBeforeReadinessAnswersNotReady) {
  binding_->hold_connect = true;
  start();
  const auto started = Clock::now();
  const auto answer = client_->describe(300);
  ASSERT_TRUE(answer.status.ok()) << answer.status.error_message();
  EXPECT_FALSE(answer.response.ready());
  EXPECT_GE(Clock::now() - started, milliseconds(250));
  EXPECT_LT(Clock::now() - started, milliseconds(3000));
  binding_->releaseConnect();
}

TEST_F(ServerLifecycle, DescribeRejectsAWaitBeyondThirtySeconds) {
  startReady();
  const auto answer = client_->describe(30001);
  EXPECT_EQ(grpc::StatusCode::INVALID_ARGUMENT, answer.status.error_code());
  EXPECT_EQ(xgc2::xrpc::Delivery::ResponseReceived, answer.delivery);
}

TEST_F(ServerLifecycle, WorkIsRefusedUntilTheServerIsReady) {
  binding_->hold_connect = true;
  start();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/r1");
  const auto refused = client_->apply(request);
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, refused.status.error_code());
  EXPECT_EQ(xgc2::xrpc::Delivery::ResponseReceived, refused.delivery);
  const auto health = client_->health();
  ASSERT_TRUE(health.status.ok());
  EXPECT_FALSE(health.response.serving());
  binding_->releaseConnect();
}

using ServerMembers = Fixture;

TEST_F(ServerMembers, ApplyRegistersEveryRobotOfTheRunInOneCallAndReportsFailuresPerMember) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  *request.add_members() = registration("r3", "/fleet/r3", "rejected.profile");
  *request.add_members() = registration("r4", "/fleet/r1");        // namespace in use
  *request.add_members() = registration("r5", "/fleet/r5", "fake.profile", 1,
                                        "another-run");           // foreign run
  *request.add_members() = registration("r6", "/fleet/r6", "fake.profile", 0);  // no epoch
  auto invalid = registration("r7", "/fleet/r7");
  invalid.mutable_configuration()->clear_parameters();              // no namespace
  *request.add_members() = invalid;
  auto foreign_environment = registration("r8", "/fleet/r8");
  (*foreign_environment.mutable_configuration()->mutable_parameters())
      ["ros_master_uri"] = "http://elsewhere:11311";
  *request.add_members() = foreign_environment;

  const auto applied = client_->apply(request);
  ASSERT_TRUE(applied.status.ok()) << applied.status.error_message();
  ASSERT_EQ(8, applied.response.members_size());
  std::map<std::string, operation::AdapterError> errors;
  for (const auto &member : applied.response.members())
    errors[member.identity().robot_id()] = member.error();
  EXPECT_TRUE(errors["r1"].code().empty());
  EXPECT_TRUE(errors["r2"].code().empty());
  EXPECT_EQ("profile does not match installed native mapping", errors["r3"].message());
  EXPECT_EQ("namespace-conflict", errors["r4"].code());
  EXPECT_NE(std::string::npos, errors["r5"].message().find("another Run"));
  EXPECT_FALSE(errors["r6"].code().empty());
  EXPECT_NE(std::string::npos, errors["r7"].message().find("namespace"));
  EXPECT_NE(std::string::npos, errors["r8"].message().find("ROS environment"));

  const auto health = client_->health();
  ASSERT_TRUE(health.status.ok());
  EXPECT_EQ(2u, health.response.members());
  EXPECT_EQ(2u, health.response.robots());
  EXPECT_TRUE(health.response.serving());
  EXPECT_FALSE(world_.contains("create:r3"));
}

TEST_F(ServerMembers, ApplyingTheSameMemberAgainChangesNothingAndAConflictingOneIsRefused) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  const auto again = client_->apply(request);
  ASSERT_TRUE(again.status.ok());
  EXPECT_TRUE(again.response.members(0).error().code().empty());
  const auto lines = world_.snapshot();
  EXPECT_EQ(1, std::count(lines.begin(), lines.end(), "create:r1"));

  wire::ApplyMembersRequest changed;
  *changed.add_members() = registration("r1", "/fleet/r1", "other.profile");
  EXPECT_EQ("member-configuration-conflict",
            client_->apply(changed).response.members(0).error().code());
  wire::ApplyMembersRequest other_epoch;
  *other_epoch.add_members() = registration("r1", "/fleet/r1", "fake.profile", 2);
  EXPECT_EQ("member-epoch-conflict",
            client_->apply(other_epoch).response.members(0).error().code());
}

TEST_F(ServerMembers, TheStatusStreamShowsMembersAndTheirRemoval) {
  startReady();
  Client::Watch watch(*client_, milliseconds(20000));
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());
  for (const char *robot : {"r1", "r2"})
    EXPECT_TRUE(watch.wait([robot](const wire::SubscribeStatusResponse &frame) {
      for (const auto &member : frame.members())
        if (member.identity().robot_id() == robot && member.messages_size() &&
            member.messages(0).channel_id() == "state.test")
          return true;
      return false;
    })) << robot;

  wire::RemoveMembersRequest removal;
  *removal.add_members() = identity("r1");
  const auto removed = client_->remove(removal);
  ASSERT_TRUE(removed.status.ok());
  EXPECT_TRUE(removed.response.members(0).error().code().empty());
  EXPECT_TRUE(watch.wait([](const wire::SubscribeStatusResponse &frame) {
    for (const auto &member : frame.members())
      if (member.identity().robot_id() == "r1" && member.removed())
        return true;
    return false;
  }));
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_FALSE(world_.contains("stop:r2"));
  // The final command of a stopped robot leaves the process before the robot
  // (and its publishers) is destroyed.
  EXPECT_LT(world_.position("stop:r1"), world_.position("flush"));
  EXPECT_LT(world_.position("flush"), world_.position("destroy:r1"));

  // Removing again, or removing what was never there, is not an error.
  EXPECT_TRUE(client_->remove(removal).response.members(0).error().code().empty());
  wire::RemoveMembersRequest unknown;
  *unknown.add_members() = identity("nobody");
  EXPECT_TRUE(client_->remove(unknown).response.members(0).error().code().empty());
  wire::RemoveMembersRequest wrong_epoch;
  *wrong_epoch.add_members() = identity("r2", 9);
  EXPECT_EQ("member-epoch-conflict",
            client_->remove(wrong_epoch).response.members(0).error().code());
}

using ServerCommands = Fixture;

TEST_F(ServerCommands, ExecuteRunsOnTheMembersQueueAndAnUncertainResultBlocksOnlyThatRobot) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());

  auto done = client_->execute(command("r1", "instant", "w1"));
  ASSERT_TRUE(done.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, done.response.event().phase());
  EXPECT_EQ("w1", done.response.event().work_id());

  auto uncertain = client_->execute(command("r1", "uncertain", "w2"));
  EXPECT_EQ(operation::OPERATION_PHASE_UNCERTAIN, uncertain.response.event().phase());
  auto blocked = client_->execute(command("r1", "instant", "w3"));
  EXPECT_EQ(operation::OPERATION_PHASE_REJECTED, blocked.response.event().phase());
  EXPECT_EQ("previous-result-unknown", blocked.response.event().error().code());
  auto other = client_->execute(command("r2", "instant", "w4"));
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, other.response.event().phase());
}

TEST_F(ServerCommands, ExecuteNeedsTheActiveMemberOfThisEpoch) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  EXPECT_EQ("member-unavailable",
            client_->execute(command("nobody", "instant", "w1")).response.event().error().code());
  EXPECT_EQ("member-unavailable",
            client_->execute(command("r1", "instant", "w2", 5)).response.event().error().code());
}

using ServerShutdown = Fixture;

TEST_F(ServerShutdown, DrainsInOrderThenAnswersAndExitsCleanly) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());
  Client::Watch watch(*client_, milliseconds(20000));

  // A command that is still in flight when the drain starts and never ends on
  // its own: the drain cancels it after its budget.
  Reply<wire::ExecuteResponse> in_flight;
  std::thread executor([&] {
    Client second(socket());
    in_flight = second.execute(command("r1", "slow", "slow-1"), milliseconds(10000));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  ASSERT_TRUE(world_.contains("execute:r1:slow"));

  const auto answer = client_->shutdown();
  ASSERT_TRUE(answer.status.ok()) << answer.status.error_message();
  EXPECT_FALSE(answer.response.serving());
  executor.join();
  ASSERT_TRUE(in_flight.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_CANCELLED, in_flight.response.event().phase());

  // By the time Shutdown answered, everything had been stopped, in order:
  // in-flight command cancelled before any robot stopped; every robot stopped.
  EXPECT_TRUE(world_.contains("cancelled:slow-1"));
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("stop:r2"));
  EXPECT_LT(world_.position("cancelled:slow-1"), world_.position("stop:r1"));
  EXPECT_EQ(0, join());
  EXPECT_TRUE(world_.contains("ros-shutdown"));
  EXPECT_LT(world_.position("stop:r2"), world_.position("ros-shutdown"));
  // One flush for all robots, after the last final command and before any
  // robot is destroyed.
  const auto lines = world_.snapshot();
  EXPECT_EQ(1, std::count(lines.begin(), lines.end(), "flush"));
  EXPECT_LT(world_.position("stop:r1"), world_.position("flush"));
  EXPECT_LT(world_.position("stop:r2"), world_.position("flush"));
  EXPECT_LT(world_.position("flush"), world_.position("destroy:r1"));
  EXPECT_LT(world_.position("flush"), world_.position("destroy:r2"));
  EXPECT_LT(world_.position("destroy:r2"), world_.position("ros-shutdown"));
  // The subscribers saw the members removed.
  EXPECT_TRUE(watch.waitFinished());
}

TEST_F(ServerShutdown, ACommandThatFinishesWithinTheBudgetIsNotCancelled) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  Reply<wire::ExecuteResponse> in_flight;
  std::thread executor([&] {
    Client second(socket());
    in_flight = second.execute(command("r1", "slow", "slow-1"));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  std::thread finisher([&] {
    std::this_thread::sleep_for(milliseconds(100));
    std::lock_guard<std::mutex> lock(world_.mutex);
    auto pending = world_.pending.begin();
    ASSERT_NE(world_.pending.end(), pending);
    pending->second.complete(event(pending->second.request,
                                   operation::OPERATION_PHASE_SUCCEEDED));
    world_.pending.erase(pending);
  });
  ASSERT_TRUE(client_->shutdown().status.ok());
  executor.join();
  finisher.join();
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, in_flight.response.event().phase());
  EXPECT_FALSE(world_.contains("cancelled:slow-1"));
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_EQ(0, join());
}

TEST_F(ServerShutdown, ASignalDrainsExactlyLikeTheRpc) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  server_->RequestStop();
  EXPECT_EQ(0, join());
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("ros-shutdown"));
}

TEST_F(ServerShutdown, AServerThatNeverGotReadyStillExitsOnRequest) {
  binding_->hold_connect = true;
  start();
  server_->RequestStop();
  binding_->releaseConnect();
  EXPECT_EQ(0, join());
}

TEST_F(ServerShutdown, NewWorkIsRefusedOnceTheDrainBegan) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  Reply<wire::ExecuteResponse> in_flight;
  std::thread executor([&] {
    Client second(socket());
    in_flight = second.execute(command("r1", "slow", "slow-1"));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  std::thread shutdown([&] { client_->shutdown(); });
  // While the in-flight command holds the drain, the server already refuses.
  Reply<wire::ExecuteResponse> late;
  Reply<wire::ApplyMembersResponse> late_apply;
  for (int attempt = 0; attempt < 100; ++attempt) {
    Client third(socket());
    late = third.execute(command("r1", "instant", "late"));
    wire::ApplyMembersRequest more;
    *more.add_members() = registration("r9", "/fleet/r9");
    late_apply = third.apply(more);
    if (late_apply.status.error_code() == grpc::StatusCode::FAILED_PRECONDITION)
      break;
    std::this_thread::sleep_for(milliseconds(5));
  }
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, late_apply.status.error_code());
  if (late.status.ok()) {
    EXPECT_NE(operation::OPERATION_PHASE_SUCCEEDED, late.response.event().phase());
  }
  shutdown.join();
  executor.join();
  EXPECT_EQ(0, join());
}

using ServerMasterWatch = Fixture;

TEST_F(ServerMasterWatch, ExitsWhenTheMasterIsReplacedAfterStoppingTheRobots) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  binding_->master_pid = 9999; // a restarted master has another pid
  EXPECT_EQ(kExitMasterChanged, join());
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("ros-shutdown"));
}

TEST_F(ServerMasterWatch, ExitsWhenTheMasterStaysUnreachable) {
  startReady();
  binding_->master_answers = false;
  const auto started = Clock::now();
  EXPECT_EQ(kExitMasterUnreachable, join());
  EXPECT_GE(Clock::now() - started, milliseconds(250));
}

TEST_F(ServerMasterWatch, ExitsWhenTheMasterRunIdChangesWithTheSamePid) {
  startReady();
  binding_->master_run_changed = true;
  EXPECT_EQ(kExitMasterChanged, join());
}

TEST_F(ServerMasterWatch, LearnsALateRunIdBeforeDetectingItsReplacement) {
  binding_->master_run_known = false;
  startReady();
  EXPECT_EQ("", facts(client_->describe().response)["ros_master_run_id"]);
  binding_->master_run_known = true;
  nlohmann::json described;
  for (int attempt = 0; attempt < 200; ++attempt) {
    described = facts(client_->describe().response);
    if (described["ros_master_run_id"] == "master-run-1")
      break;
    std::this_thread::sleep_for(milliseconds(5));
  }
  EXPECT_EQ("master-run-1", described["ros_master_run_id"]);
  EXPECT_TRUE(client_->health().response.serving());
  binding_->master_run_changed = true;
  EXPECT_EQ(kExitMasterChanged, join());
}

TEST_F(ServerMasterWatch, ABriefOutageIsNotAFailure) {
  startReady();
  binding_->master_answers = false;
  std::this_thread::sleep_for(milliseconds(120));
  binding_->master_answers = true;
  std::this_thread::sleep_for(milliseconds(400));
  EXPECT_TRUE(client_->health().response.serving());
  server_->RequestStop();
  EXPECT_EQ(0, join());
}

TEST_F(ServerMasterWatch, ExitsWhenRosShutsTheNodeDown) {
  startReady();
  binding_->running = false;
  EXPECT_EQ(kExitRosShutdown, join());
}

TEST_F(ServerMasterWatch, WaitsUntilTheMasterAcceptsTheNode) {
  binding_->master_calls_fail = true;
  start();
  const auto waiting = client_->describe(600);
  ASSERT_TRUE(waiting.status.ok());
  EXPECT_FALSE(waiting.response.ready());
  EXPECT_FALSE(finished_);
  binding_->master_calls_fail = false;
  EXPECT_TRUE(client_->describe(10000).response.ready());
  server_->RequestStop();
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerMasterWatch, WaitsForALateMasterWithoutAStartupDeadline) {
  binding_->master_answers = false;
  start();
  const auto started = Clock::now();
  const auto waiting = client_->describe(600);
  ASSERT_TRUE(waiting.status.ok());
  EXPECT_FALSE(waiting.response.ready());
  EXPECT_GE(Clock::now() - started, milliseconds(550));
  EXPECT_EQ("waiting for ROS master binding", facts(waiting.response)["reason"]);
  EXPECT_FALSE(finished_);
  EXPECT_FALSE(world_.contains("connect"));
  EXPECT_FALSE(world_.contains("create:r1"));
  binding_->master_answers = true;
  EXPECT_TRUE(client_->describe(10000).response.ready());
  server_->RequestStop();
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerMasterWatch, StopsCleanlyWhileWaitingForTheMaster) {
  binding_->master_answers = false;
  start();
  EXPECT_FALSE(client_->describe(0).response.ready());
  server_->RequestStop();
  EXPECT_EQ(kExitOk, join());
  EXPECT_TRUE(world_.contains("ros-shutdown"));
}

} // namespace
} // namespace xgc2_ros1_robot_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
