#include "xgc_scene/lidar/control.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <xgc2/xrpc/http.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace {
using namespace std::chrono_literals;
using namespace xgc2::xrpc;
using Json = nlohmann::json;
using Control = xgc_scene::lidar::Control;

class Directory {
 public:
  Directory() {
    char pattern[] = "/tmp/lidar-control-XXXXXX";
    const auto result = ::mkdtemp(pattern);
    if (!result) throw std::runtime_error("temporary directory failed");
    path = result;
  }
  ~Directory() { std::filesystem::remove_all(path); }
  std::string path;
};

// A real executor thread with explicit dispatch permits. Tests observe accepted
// operations over HTTP before allowing the fleet's control thread to apply them.
class Executor {
 public:
  Executor() : worker_([this] { Run(); }) {}
  ~Executor() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    worker_.join();
  }
  void Post(std::function<void()> fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(std::move(fn));
    ++posts_;
    condition_.notify_all();
  }
  void Resume() {
    std::lock_guard<std::mutex> lock(mutex_);
    automatic_ = true;
    condition_.notify_all();
  }
  void Dispatch() {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto expected = completed_ + 1;
    ++permits_;
    condition_.notify_all();
    if (!condition_.wait_until(lock, Clock::now() + 5s,
                               [&] { return completed_ >= expected; }))
      throw std::runtime_error("executor dispatch did not finish");
  }
  unsigned Posts() {
    std::lock_guard<std::mutex> lock(mutex_);
    return posts_;
  }
  std::thread::id ThreadId() const { return worker_.get_id(); }
 private:
  void Run() {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      condition_.wait(lock, [&] { return stopping_ || (!queue_.empty() && (automatic_ || permits_)); });
      if (stopping_) return;
      auto fn = std::move(queue_.front());
      queue_.pop_front();
      if (permits_) --permits_;
      lock.unlock();
      fn();
      lock.lock();
      ++completed_;
      condition_.notify_all();
    }
  }
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::function<void()>> queue_;
  bool stopping_ = false, automatic_ = false;
  unsigned permits_ = 0, posts_ = 0, completed_ = 0;
  std::thread worker_;
};

Json Body(std::uint64_t revision, bool enabled = false, unsigned timeout = 10000) {
  return {{"expected_revision", revision}, {"configuration", {{"enabled", enabled}}},
          {"operation_timeout_ms", timeout}};
}
Json Receipt(const std::string& id, std::uint64_t revision, bool enabled) {
  return {{"id", id}, {"kind", "source.configure"}, {"state", "succeeded"},
          {"result", {{"revision", revision}, {"configuration", {{"enabled", enabled}}},
                      {"persisted", nullptr}}}};
}
Json Failure(const std::string& id, const std::string& code) {
  return {{"id", id}, {"kind", "source.configure"}, {"state", "failed"},
          {"error", {{"code", code}}}, {"effects", {{"applied", false}}}};
}
void Error(const HttpResponse& response, int status, const std::string& code, const std::string& text) {
  EXPECT_EQ(response.status, status);
  EXPECT_EQ(Json::parse(response.body), (Json{{"error", {{"code", code}, {"message", text}}}}));
}

class ControlTest : public testing::Test {
 protected:
  void SetUp() override {
    socket_ = directory_.path + "/control.sock";
    control_ = std::make_unique<Control>(socket_, "fixture", true,
        [this](std::function<void()> fn) { executor_.Post(std::move(fn)); },
        [this](bool enabled) {
          std::unique_lock<std::mutex> lock(effects_mutex_);
          effects_.push_back(enabled);
          threads_.push_back(std::this_thread::get_id());
          effects_condition_.notify_all();
          effects_condition_.wait(lock, [&] { return !block_apply_; });
          if (fail_apply_) throw std::runtime_error("fixture apply failure");
        });
    control_->Start();
    const auto describe = Json::parse(Call("GET", "/v1/describe", "", "", false).body);
    instance_ = describe.at("service_ref").at("instance_id");
  }
  void TearDown() override {
    Unblock();
    if (control_) control_->Stop();
    EXPECT_FALSE(std::filesystem::exists(socket_));
  }
  HttpResponse Call(std::string method, std::string target, std::string body = "",
                    std::string id = "", bool bound = true, std::chrono::milliseconds budget = 5s,
                    StopToken token = {}) {
    HttpClient client(socket_, {}, bound ? instance_ : "");
    HttpRequest request;
    request.method = std::move(method);
    request.target = std::move(target);
    request.body = std::move(body);
    request.request_id = std::move(id);
    return client.call(std::move(request), Clock::now() + budget, token);
  }
  std::future<HttpResponse> Submit(std::uint64_t revision, bool enabled, std::string id,
                                 unsigned timeout = 10000, StopToken token = {}) {
    return Async("POST", "/v1/configure", Body(revision, enabled, timeout).dump(), std::move(id), token);
  }
  std::future<HttpResponse> Async(std::string method, std::string target, std::string body = "",
                                std::string id = "", StopToken token = {}, bool bound = true) {
    return std::async(std::launch::async, [this, method, target, body, id, token, bound] {
      return Call(method, target, body, id, bound, 5s, token);
    });
  }
  Json Status() { return Json::parse(Call("GET", "/v1/status").body); }
  void Admitted(std::uint64_t revision) {
    const auto until = Clock::now() + 3s;
    do {
      if (Status().at("desired").at("revision") == revision) return;
      std::this_thread::sleep_for(1ms);
    } while (Clock::now() < until);
    throw std::runtime_error("operation not admitted");
  }
  void Unblock() {
    std::lock_guard<std::mutex> lock(effects_mutex_);
    block_apply_ = false;
    effects_condition_.notify_all();
  }
  Directory directory_;
  Executor executor_;
  std::unique_ptr<Control> control_;
  std::string socket_, instance_;
  std::mutex effects_mutex_;
  std::condition_variable effects_condition_;
  std::vector<bool> effects_;
  std::vector<std::thread::id> threads_;
  bool block_apply_ = false, fail_apply_ = false;
};

TEST_F(ControlTest, DescribeStatusHealthAndRoutes) {
  const auto response = Call("GET", "/v1/describe?wait_ready_ms=30000", "", "describe", false);
  ASSERT_EQ(response.status, 200);
  const auto described = Json::parse(response.body);
  EXPECT_EQ(described.at("service_ref"), (Json{{"target_id", "fixture"},
      {"service", "xgc2.sensor.lidar"}, {"api_version", "v1"}, {"instance_id", instance_},
      {"profile", "http.v1"}, {"endpoint", {{"kind", "unix"}, {"address", socket_}}}}));
  EXPECT_EQ(described.at("capabilities"), Json::array({"source.enabled.configure"}));
  EXPECT_EQ(described.at("limits"), (Json{{"pending_operations", 16}, {"retained_operations", 128},
                                        {"operation_timeout_ms", 60000}}));
  EXPECT_EQ(described.at("persistence"), "none; launch configuration owns initial state");
  EXPECT_EQ(described.at("service"), "xgc2.sensor.lidar");
  EXPECT_EQ(described.at("api_version"), "v1");
  EXPECT_EQ(described.at("instance_id"), instance_);
  EXPECT_EQ(described.at("ready"), true);
  EXPECT_EQ(described.at("facts"), (Json{{"target_id", "fixture"}}));
  const Json expected = {{"lifecycle", "ready"}, {"persisted", nullptr},
      {"desired", {{"revision", 1}, {"configuration", {{"enabled", true}}}}},
      {"applied", {{"revision", 1}, {"configuration", {{"enabled", true}}}}}};
  EXPECT_EQ(Status(), expected);
  EXPECT_EQ(Json::parse(Call("GET", "/v1/health").body), expected);
  for (const auto& route : {"/absent", "/v1/describe-extra", "/v1/status?other=1", "/v1/configure"})
    Error(Call("GET", route), 404, "not_found", "unknown lidar source route");
  Error(Call("POST", "/v1/describe"), 404, "not_found", "unknown lidar source route");
  Error(Call("GET", "/v1/operations/missing"), 404, "not_found", "lidar operation unknown or expired");
  Error(Call("POST", "/v1/operations/missing/wait"), 404, "not_found", "lidar operation unknown or expired");
  for (const auto& route : {"/v1/operations/", "/v1/operations/missing", "/v1/operations/missing/wait/extra"})
    Error(Call("POST", route), 400, "invalid_argument", "unknown lidar operation route");
}

TEST_F(ControlTest, PeerFactsChangeWithoutChangingReadinessOrTargetIdentity) {
  std::atomic<bool> scene_ready{false};
  control_->SetFacts({{"profile", "gazebo"}, {"sensors", 2}, {"backend", "cpu"}},
      [&] { return Json{{"scene_ready", scene_ready.load()}, {"target_id", "other"}}; });
  auto describe = Json::parse(Call("GET", "/v1/describe", "", "", false).body);
  EXPECT_EQ(describe.at("ready"), true);
  EXPECT_EQ(describe.at("facts"), (Json{{"target_id", "fixture"}, {"profile", "gazebo"},
      {"sensors", 2}, {"backend", "cpu"}, {"scene_ready", false}}));
  scene_ready.store(true);
  describe = Json::parse(Call("GET", "/v1/describe", "", "", false).body);
  EXPECT_EQ(describe.at("ready"), true);
  EXPECT_EQ(describe.at("facts").at("scene_ready"), true);
  EXPECT_THROW(control_->SetFacts(Json::array()), std::invalid_argument);
}

TEST_F(ControlTest, ClosedConfigurationSchemaAndParser) {
  std::vector<std::pair<std::string, std::string>> cases = {
      {"", "invalid lidar JSON"}, {"{", "invalid lidar JSON"}, {"{} {}", "invalid lidar JSON"},
      {"/* comment */ {}", "invalid lidar JSON"}, {"{\"configuration\":{},\"configuration\":{}}", "invalid lidar JSON"},
      {"null", "configuration object required"}, {"[]", "configuration object required"},
      {"{}", "configuration object required"}};
  auto add = [&](Json value, const char* message) { cases.emplace_back(value.dump(), message); };
  auto body = Body(1);
  body["typo"] = 1;
  add(body, "unknown lidar field: typo");
  body = Body(1); body["configuration"]["typo"] = 1;
  add(body, "unknown lidar field: typo");
  for (const auto& configuration : {Json(nullptr), Json::array(), Json(false), Json(1), Json("enabled")}) {
    body = Body(1); body["configuration"] = configuration;
    add(body, "configuration object required");
  }
  body = Body(1); body["configuration"] = Json::object();
  add(body, "boolean enabled required");
  for (const auto& enabled : {Json(nullptr), Json(1), Json(0), Json("false"), Json::array(), Json::object()}) {
    body = Body(1); body["configuration"]["enabled"] = enabled;
    add(body, "boolean enabled required");
  }
  for (const auto& key : {"expected_revision", "operation_timeout_ms"}) {
    body = Body(1); body.erase(key); add(body, "positive integer required");
    for (const auto& value : {Json(nullptr), Json(false), Json(0), Json(-1), Json(1.0), Json("1"),
                             Json::array(), Json::object(), Json(9007199254740992ULL)}) {
      body = Body(1); body[key] = value; add(body, "positive integer required");
    }
  }
  body = Body(1); body["operation_timeout_ms"] = 60001;
  add(body, "positive integer required");
  for (const auto& item : cases) {
    SCOPED_TRACE(item.first);
    Error(Call("POST", "/v1/configure", item.first), 400, "invalid_argument", item.second);
  }
  EXPECT_EQ(Status().at("desired").at("revision"), 1);
  EXPECT_EQ(executor_.Posts(), 0u);
}

TEST_F(ControlTest, AcceptedReplayPayloadConflictRevisionConflictAndCompletion) {
  auto submitted = Submit(1, false, "disable-once");
  Admitted(2);
  EXPECT_EQ(submitted.wait_for(0ms), std::future_status::timeout);
  EXPECT_EQ(Status().at("applied").at("revision"), 1);
  const Json accepted = {{"id", "disable-once"}, {"kind", "source.configure"}, {"state", "accepted"}};
  const auto replay = Call("POST", "/v1/configure", Body(1).dump(), "disable-once");
  EXPECT_EQ(replay.status, 202);
  EXPECT_EQ(Json::parse(replay.body), accepted);
  const auto found = Call("GET", "/v1/operations/disable-once");
  EXPECT_EQ(found.status, 202);
  EXPECT_EQ(Json::parse(found.body), accepted);
  Error(Call("POST", "/v1/configure", Body(1, true).dump(), "disable-once"),
        409, "conflict", "lidar request ID payload conflict");
  Error(Call("POST", "/v1/configure", Body(1).dump() + " ", "disable-once"),
        409, "conflict", "lidar request ID payload conflict");
  Error(Call("POST", "/v1/configure", Body(1).dump(), "stale"),
        409, "conflict", "lidar desired revision conflict");
  executor_.Dispatch();
  const auto expected = Receipt("disable-once", 2, false);
  const auto completed = submitted.get();
  EXPECT_EQ(completed.status, 200);
  EXPECT_EQ(Json::parse(completed.body), expected);
  EXPECT_EQ(Json::parse(Call("POST", "/v1/configure", Body(1).dump(), "disable-once").body), expected);
  EXPECT_EQ(Json::parse(Call("GET", "/v1/operations/disable-once").body), expected);
  EXPECT_EQ(Json::parse(Call("POST", "/v1/operations/disable-once/wait", "ignored legacy wait body").body), expected);
  EXPECT_EQ(Status().at("applied"), Status().at("desired"));
  EXPECT_EQ(executor_.Posts(), 1u);
  std::lock_guard<std::mutex> lock(effects_mutex_);
  ASSERT_EQ(effects_.size(), 1u);
  EXPECT_FALSE(effects_[0]);
}

TEST_F(ControlTest, PendingLimitAndExecutorOrder) {
  std::vector<std::future<HttpResponse>> submissions;
  for (unsigned i = 0; i < 16; ++i) {
    submissions.push_back(Submit(i + 1, i % 2, "queued-" + std::to_string(i)));
    Admitted(i + 2);
  }
  Error(Call("POST", "/v1/configure", Body(17).dump(), "overflow"),
        429, "resource_exhausted", "lidar operation capacity");
  Error(Call("GET", "/v1/operations/overflow"), 404, "not_found", "lidar operation unknown or expired");
  EXPECT_EQ(Status().at("desired").at("revision"), 17);
  EXPECT_EQ(Status().at("applied").at("revision"), 1);
  EXPECT_EQ(executor_.Posts(), 1u);
  executor_.Dispatch();
  for (unsigned i = 0; i < 16; ++i)
    EXPECT_EQ(Json::parse(submissions[i].get().body), Receipt("queued-" + std::to_string(i), i + 2, i % 2));
  std::lock_guard<std::mutex> lock(effects_mutex_);
  ASSERT_EQ(effects_.size(), 16u);
  for (unsigned i = 0; i < 16; ++i) {
    EXPECT_EQ(effects_[i], static_cast<bool>(i % 2));
    EXPECT_EQ(threads_[i], threads_[0]);
    EXPECT_EQ(threads_[i], executor_.ThreadId());
    EXPECT_NE(threads_[i], std::this_thread::get_id());
  }
}

TEST_F(ControlTest, RetainedRingEvictsOldestCompletedAndAllowsExpiredIdentityReuse) {
  executor_.Resume();
  for (unsigned i = 0; i < 129; ++i) {
    const auto response = Call("POST", "/v1/configure", Body(i + 1, i % 2).dump(), "ring-" + std::to_string(i));
    ASSERT_EQ(response.status, 200);
    ASSERT_EQ(Json::parse(response.body), Receipt("ring-" + std::to_string(i), i + 2, i % 2));
  }
  Error(Call("GET", "/v1/operations/ring-0"), 404, "not_found", "lidar operation unknown or expired");
  Error(Call("POST", "/v1/operations/ring-0/wait"), 404, "not_found", "lidar operation unknown or expired");
  EXPECT_EQ(Json::parse(Call("GET", "/v1/operations/ring-1").body), Receipt("ring-1", 3, true));
  EXPECT_EQ(Json::parse(Call("GET", "/v1/operations/ring-128").body), Receipt("ring-128", 130, false));
  EXPECT_EQ(Json::parse(Call("POST", "/v1/configure", Body(130).dump(), "ring-0").body), Receipt("ring-0", 131, false));
  Error(Call("GET", "/v1/operations/ring-1"), 404, "not_found", "lidar operation unknown or expired");
  std::lock_guard<std::mutex> lock(effects_mutex_);
  ASSERT_EQ(effects_.size(), 130u);
}

TEST_F(ControlTest, ExpirationAndApplyFailurePreserveAppliedRevision) {
  auto expired = Submit(1, false, "expired", 1);
  Admitted(2);
  // Wait beyond the operation budget while the executor is deliberately paused.
  std::this_thread::sleep_for(10ms);
  executor_.Dispatch();
  EXPECT_EQ(Json::parse(expired.get().body), Failure("expired", "deadline_exceeded"));
  {
    std::lock_guard<std::mutex> lock(effects_mutex_);
    EXPECT_TRUE(effects_.empty());
    fail_apply_ = true;
  }
  EXPECT_EQ(Status().at("applied").at("revision"), 1);
  auto failed = Submit(2, false, "failed");
  Admitted(3);
  executor_.Dispatch();
  EXPECT_EQ(Json::parse(failed.get().body), Failure("failed", "internal"));
  EXPECT_EQ(Json::parse(Call("POST", "/v1/configure", Body(2).dump(), "failed").body), Failure("failed", "internal"));
  EXPECT_EQ(Json::parse(Call("POST", "/v1/operations/failed/wait").body), Failure("failed", "internal"));
  EXPECT_EQ(Status().at("applied").at("revision"), 1);
  EXPECT_EQ(Status().at("desired").at("revision"), 3);
}

TEST_F(ControlTest, WaiterLimitCancellationAndCompletion) {
  auto submitted = Submit(1, false, "waited");
  Admitted(2);
  std::vector<std::future<HttpResponse>> waiters;
  std::vector<StopSource> stops(5);
  for (auto& stop : stops)
    waiters.push_back(Async("POST", "/v1/operations/waited/wait", "", "", stop.get_token()));
  std::size_t rejected = waiters.size();
  const auto until = Clock::now() + 3s;
  while (rejected == waiters.size() && Clock::now() < until) {
    for (std::size_t i = 0; i < waiters.size(); ++i)
      if (waiters[i].wait_for(1ms) == std::future_status::ready) { rejected = i; break; }
  }
  ASSERT_LT(rejected, waiters.size());
  Error(waiters[rejected].get(), 429, "resource_exhausted", "lidar operation waiter limit");
  const auto cancelled = (rejected + 1) % waiters.size();
  stops[cancelled].request_stop();
  try { waiters[cancelled].get(); FAIL() << "cancelled waiter returned a response"; }
  catch (const HttpCallError& error) { EXPECT_EQ(error.code, "cancelled"); }
  auto replacement = Async("POST", "/v1/operations/waited/wait");
  // A cancelled waiter must be removed before admitting its replacement.
  EXPECT_EQ(replacement.wait_for(75ms), std::future_status::timeout);
  executor_.Dispatch();
  EXPECT_EQ(Json::parse(submitted.get().body), Receipt("waited", 2, false));
  EXPECT_EQ(Json::parse(replacement.get().body), Receipt("waited", 2, false));
  for (std::size_t i = 0; i < waiters.size(); ++i)
    if (i != rejected && i != cancelled) {
      EXPECT_EQ(Json::parse(waiters[i].get().body), Receipt("waited", 2, false));
    }
  std::lock_guard<std::mutex> lock(effects_mutex_);
  EXPECT_EQ(effects_.size(), 1u);
}

TEST_F(ControlTest, SubmissionCancellationDoesNotCancelBusinessEffect) {
  StopSource cancellation;
  auto submitted = Submit(1, false, "cancelled-submission", 10000, cancellation.get_token());
  Admitted(2);
  cancellation.request_stop();
  try { submitted.get(); FAIL() << "cancelled submission returned a response"; }
  catch (const HttpCallError& error) {
    EXPECT_EQ(error.code, "cancelled");
    EXPECT_EQ(error.delivery, Delivery::OutcomeUnknown);
  }
  executor_.Dispatch();
  EXPECT_EQ(Json::parse(Call("GET", "/v1/operations/cancelled-submission").body),
            Receipt("cancelled-submission", 2, false));
  std::lock_guard<std::mutex> lock(effects_mutex_);
  EXPECT_EQ(effects_.size(), 1u);
}

TEST_F(ControlTest, ReadinessWaitReleaseExpirationAndDeadline) {
  control_->SetReady(false);
  EXPECT_FALSE(Json::parse(Call("GET", "/v1/describe?wait_ready_ms=0", "", "", false).body).at("ready"));
  auto held = Async("GET", "/v1/describe?wait_ready_ms=30000", "", "", {}, false);
  EXPECT_EQ(held.wait_for(75ms), std::future_status::timeout);
  control_->SetReady(true);
  EXPECT_TRUE(Json::parse(held.get().body).at("ready"));
  control_->SetReady(false);
  auto start = Clock::now();
  const auto expired = Call("GET", "/v1/describe?wait_ready_ms=80", "", "", false);
  EXPECT_EQ(expired.status, 200);
  EXPECT_FALSE(Json::parse(expired.body).at("ready"));
  EXPECT_GE(Clock::now() - start, 75ms);
  start = Clock::now();
  const auto deadline = Call("GET", "/v1/describe?wait_ready_ms=30000", "", "", false, 500ms);
  EXPECT_EQ(deadline.status, 200);
  EXPECT_FALSE(Json::parse(deadline.body).at("ready"));
  EXPECT_LT(Clock::now() - start, 500ms);
}

TEST_F(ControlTest, ReadinessRejectsInvalidQueryAndReleasesCancelledCalls) {
  for (const auto& query : {"", "other=1", "wait_ready_ms=", "wait_ready_ms=-1", "wait_ready_ms=30001",
                            "wait_ready_ms=000000", "wait_ready_ms=1.0", "wait_ready_ms=1&other=2"})
    Error(Call("GET", std::string("/v1/describe?") + query, "", "", false), 400,
          "invalid_argument", "the only query is wait_ready_ms between 0 and 30000");
  control_->SetReady(false);
  for (unsigned i = 0; i < 40; ++i) {
    StopSource stop;
    auto cancelled = Async("GET", "/v1/describe?wait_ready_ms=30000", "", "", stop.get_token(), false);
    EXPECT_EQ(cancelled.wait_for(30ms), std::future_status::timeout);
    stop.request_stop();
    try { cancelled.get(); FAIL() << "cancelled discovery returned a response"; }
    catch (const HttpCallError& error) { EXPECT_EQ(error.code, "cancelled"); }
  }
  control_->SetReady(true);
  EXPECT_TRUE(Json::parse(Call("GET", "/v1/describe", "", "", false).body).at("ready"));
}

TEST_F(ControlTest, ShutdownReleasesWaitersAndQueuedCallbacksAreSafeAfterDestruction) {
  auto submitted = Submit(1, false, "shutdown");
  Admitted(2);
  auto waiter = Async("POST", "/v1/operations/shutdown/wait");
  control_->SetReady(false);
  auto discovery = Async("GET", "/v1/describe?wait_ready_ms=30000", "", "", {}, false);
  EXPECT_EQ(waiter.wait_for(75ms), std::future_status::timeout);
  EXPECT_EQ(discovery.wait_for(0ms), std::future_status::timeout);
  control_->Stop();
  EXPECT_EQ(Json::parse(submitted.get().body), Failure("shutdown", "unavailable"));
  EXPECT_EQ(Json::parse(waiter.get().body), Failure("shutdown", "unavailable"));
  EXPECT_FALSE(Json::parse(discovery.get().body).at("ready"));
  EXPECT_FALSE(std::filesystem::exists(socket_));
  control_.reset();
  executor_.Dispatch();
  {
    std::lock_guard<std::mutex> lock(effects_mutex_);
    EXPECT_TRUE(effects_.empty());
  }
  control_ = std::make_unique<Control>(socket_, "replacement", false,
      [this](std::function<void()> fn) { executor_.Post(std::move(fn)); }, [](bool) {});
  control_->Start();
  const auto replacement = Json::parse(Call("GET", "/v1/describe", "", "", false).body);
  EXPECT_NE(replacement.at("instance_id"), instance_);
  control_->Stop();
}

TEST_F(ControlTest, StoppingRejectsRequestsAndWaitsForActiveApply) {
  {
    std::lock_guard<std::mutex> lock(effects_mutex_);
    block_apply_ = true;
  }
  executor_.Resume();
  auto submitted = Submit(1, false, "active");
  {
    std::unique_lock<std::mutex> lock(effects_mutex_);
    ASSERT_TRUE(effects_condition_.wait_until(lock, Clock::now() + 3s, [&] { return !effects_.empty(); }));
  }
  auto stopped = std::async(std::launch::async, [this] { control_->Stop(); });
  EXPECT_EQ(Json::parse(submitted.get().body), Failure("active", "unavailable"));
  for (const auto& route : {"/v1/status", "/v1/health", "/v1/describe", "/v1/operations/active"})
    Error(Call("GET", route), 503, "unavailable", "lidar source stopping");
  Error(Call("POST", "/v1/configure", Body(2).dump()), 503, "unavailable", "lidar source stopping");
  Error(Call("POST", "/v1/operations/active/wait"), 503, "unavailable", "lidar source stopping");
  EXPECT_EQ(stopped.wait_for(0ms), std::future_status::timeout);
  Unblock();
  EXPECT_EQ(stopped.wait_for(3s), std::future_status::ready);
  stopped.get();
}

TEST_F(ControlTest, IdentityFenceAndStartLifecycle) {
  EXPECT_THROW(control_->Start(), std::logic_error);
  HttpClient unbound(socket_);
  HttpRequest request; request.method = "GET"; request.target = "/v1/status";
  EXPECT_EQ(unbound.call(request, Clock::now() + 3s).status, 409);
  HttpClient stale(socket_, {}, "stale-instance");
  EXPECT_THROW(stale.call(request, Clock::now() + 3s), HttpCallError);
  EXPECT_THROW((Control(socket_, "", true, [](std::function<void()>) {}, [](bool) {})), std::invalid_argument);
  EXPECT_THROW((Control(socket_, "fixture", true, {}, [](bool) {})), std::invalid_argument);
  EXPECT_THROW((Control(socket_, "fixture", true, [](std::function<void()>) {}, {})), std::invalid_argument);
  control_->Stop();
  control_->Stop();
  EXPECT_THROW(control_->Start(), std::logic_error);
}

TEST(ControlConstructionTest, ExecutorPostFailureReturnsTerminalReceipt) {
  Directory directory;
  const auto socket = directory.path + "/failed.sock";
  Control control(socket, "fixture", false, [](std::function<void()>) {
    throw std::runtime_error("executor unavailable");
  }, [](bool) { FAIL() << "apply reached after post failure"; });
  control.Start();
  HttpClient discovery(socket);
  HttpRequest request; request.method = "GET"; request.target = "/v1/describe";
  const auto described = Json::parse(discovery.call(request, Clock::now() + 3s).body);
  HttpClient client(socket, {}, described.at("instance_id").get<std::string>());
  request.method = "POST"; request.target = "/v1/configure";
  request.body = Body(1).dump(); request.request_id = "post-failed";
  EXPECT_EQ(Json::parse(client.call(request, Clock::now() + 3s).body), Failure("post-failed", "internal"));
  control.Stop();
  EXPECT_FALSE(std::filesystem::exists(socket));
}

TEST(ControlConstructionTest, DescribeMergesAdditionalFactsAndKeepsTargetIdentity) {
  Directory directory;
  const auto socket = directory.path + "/facts.sock";
  Control control(socket, "fixture", true, [](std::function<void()>) {}, [](bool) {},
                  Json{{"profile", "gazebo"}, {"sensors", 2}, {"backend", "cpu"}, {"target_id", "other"}});
  control.SetReady(false);
  control.Start();
  HttpClient client(socket);
  HttpRequest request; request.method = "GET"; request.target = "/v1/describe?wait_ready_ms=0";
  const auto describe = Json::parse(client.call(request, Clock::now() + 3s).body);
  EXPECT_FALSE(describe.at("ready"));
  EXPECT_EQ(describe.at("facts"), (Json{{"target_id", "fixture"}, {"profile", "gazebo"},
                                       {"sensors", 2}, {"backend", "cpu"}}));
  control.Stop();
  EXPECT_THROW((Control(socket, "fixture", true, [](std::function<void()>) {}, [](bool) {}, Json::array())),
               std::invalid_argument);
}
}  // namespace
