#include "xgc_scene/simulation_client.hpp"
#include "xgc_scene/json_io.hpp"

#include <gtest/gtest.h>
#include <xgc2/xrpc/unix.hpp>
#include <atomic>
#include <filesystem>
#include <future>
#include <mutex>
#include <thread>
#include <unistd.h>

namespace xgc_scene {
namespace {
using namespace std::chrono;
using xgc2::xrpc::HttpRequest;
using xgc2::xrpc::HttpReply;
void answer(const HttpReply& reply, const Json& value) {
  reply.complete({200, {{"Content-Type", "application/json"}}, value.dump()});
}
struct Directory {
  std::filesystem::path path;
  Directory() {
    char pattern[] = "/tmp/xgc-scene-peer-XXXXXX";
    const auto root = mkdtemp(pattern);
    if (!root) throw std::runtime_error("Cannot create peer directory");
    path = root;
  }
  ~Directory() { std::error_code error; std::filesystem::remove_all(path, error); }
};
class World {
 public:
  World(const std::filesystem::path& path, const std::string& instance, xgc2::xrpc::HttpServer::Handler handler,
        bool hold_discovery = false) {
    server_ = std::make_unique<xgc2::xrpc::HttpServer>(xgc2::xrpc::UnixOptions{path.string()},
      [this, instance, handler, hold_discovery](HttpRequest request, HttpReply reply) {
        if (request.target == "/v1/describe?wait_ready_ms=30000") {
          EXPECT_EQ(request.method, "GET");
          ++describes;
          if (hold_discovery) handler(std::move(request), std::move(reply));
          else answer(reply, {{"service", "xgc2.simulation"}, {"api_version", "v1"},
                              {"instance_id", instance}, {"ready", true},
                              {"service_ref", {{"target_id", "fixture"}}}});
        } else handler(std::move(request), std::move(reply));
      }, xgc2::xrpc::HttpLimits{}, xgc2::xrpc::HttpIdentity{instance, {"/v1/describe"}});
    thread_ = std::thread([this] { server_->run(); });
  }
  ~World() { server_->request_stop(); thread_.join(); server_.reset(); }
  std::atomic<int> describes{0};
 private:
  std::unique_ptr<xgc2::xrpc::HttpServer> server_;
  std::thread thread_;
};
std::optional<Json> report(SimulationClient& client, bool applied, milliseconds timeout = seconds(5)) {
  const auto end = steady_clock::now() + timeout;
  while (steady_clock::now() < end) {
    auto value = client.takeReport();
    if (value && value->at("applied") == applied) return value;
    std::this_thread::sleep_for(milliseconds(5));
  }
  return {};
}
void completed(HttpRequest request, HttpReply reply) {
  const auto payload = parseJson(request.body);
  answer(reply, {{"id", request.request_id}, {"state", "succeeded"},
                {"result", {{"epoch", payload.at("epoch")}, {"revision", payload.at("revision")}}}});
}
TEST(SimulationClient, LazyDiscoveryLateSocketRestartAndOneReplayPerRevision) {
  Directory root;
  const auto path = root.path / "0123456789abcdef0123456789abcdef.sock";
  SimulationClient client(path.string(), "fixture", seconds(2));
  std::this_thread::sleep_for(milliseconds(20));
  EXPECT_FALSE(client.takeReport());
  client.update(Json::object(), "epoch", 1, {{"playing", false}});
  ASSERT_TRUE(report(client, false));
  std::atomic<int> applies{0}, motions{0};
  auto handler = [&](HttpRequest request, HttpReply reply) {
    if (request.target == "/v1/extensions/scene/apply") ++applies; else ++motions;
    completed(std::move(request), std::move(reply));
  };
  {
    World world(path, "first-instance", handler);
    auto first = report(client, true);
    ASSERT_TRUE(first);
    EXPECT_EQ(first->at("instance_id"), "first-instance");
    EXPECT_EQ(world.describes.load(), 1);
    EXPECT_EQ(applies.load(), 1);
    std::this_thread::sleep_for(milliseconds(50));
    EXPECT_EQ(applies.load(), 1);
    EXPECT_EQ(world.describes.load(), 1);
    client.update(Json::object(), "epoch", 2, {{"playing", true}});
    ASSERT_TRUE(report(client, true));
    EXPECT_EQ(applies.load(), 2);
  }
  ASSERT_TRUE(report(client, false));
  {
    World world(path, "second-instance", handler);
    auto restarted = report(client, true);
    ASSERT_TRUE(restarted);
    EXPECT_EQ(restarted->at("instance_id"), "second-instance");
    EXPECT_EQ(restarted->at("epoch"), "epoch");
    EXPECT_EQ(restarted->at("revision"), 2);
    EXPECT_EQ(applies.load(), 3);
    EXPECT_EQ(motions.load(), 3);
    std::this_thread::sleep_for(milliseconds(50));
    EXPECT_EQ(applies.load(), 3);
  }
  client.close();
}
TEST(SimulationClient, AcceptedOperationUsesOneWaitAndDistinctTransportIdentity) {
  Directory root;
  std::mutex mutex;
  std::vector<HttpRequest> calls;
  std::string operation;
  World world(root.path / "world.sock", "world", [&](HttpRequest request, HttpReply reply) {
    std::lock_guard<std::mutex> lock(mutex);
    calls.push_back(request);
    if (request.target == "/v1/extensions/scene/apply") {
      operation = request.request_id;
      answer(reply, {{"id", operation}, {"state", "accepted"}});
    } else if (request.target == "/v1/operations/" + operation + "/wait")
      answer(reply, {{"id", operation}, {"state", "succeeded"}, {"result", {{"epoch", "epoch"}, {"revision", 6}}}});
    else completed(std::move(request), std::move(reply));
  });
  SimulationClient client((root.path / "world.sock").string(), "fixture", seconds(2));
  client.update(Json::object(), "epoch", 6, {{"playing", false}});
  ASSERT_TRUE(report(client, true));
  client.close();
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(calls.size(), 3u);
  EXPECT_NE(calls[0].request_id, calls[1].request_id);
  EXPECT_EQ(parseJson(calls[0].body).at("operation_timeout_ms"), 2000);
  EXPECT_EQ(parseJson(calls[1].body), Json::object());
  EXPECT_LT(calls[1].deadline, calls[0].deadline + milliseconds(2));
}
TEST(SimulationClient, LostCommittedReplyReadsTheOriginalFencedReceiptWithoutResubmitting) {
  for (const bool lose_wait_reply : {false, true}) {
    SCOPED_TRACE(lose_wait_reply ? "lost wait reply" : "lost mutation reply");
    Directory root;
    std::atomic<int> applies{0}, receipts{0}, waits{0};
    std::string operation;
    Json terminal;
    xgc2::xrpc::Clock::time_point submitted_deadline, observation_deadline;
    World world(root.path / "world.sock", "world", [&](HttpRequest request, HttpReply reply) {
      if (request.target == "/v1/extensions/scene/apply") {
        ++applies;
        operation = request.request_id;
        submitted_deadline = request.deadline;
        const auto payload = parseJson(request.body);
        EXPECT_EQ(payload.at("operation_timeout_ms"), 50);
        terminal = {{"id", operation}, {"state", "succeeded"},
                    {"result", {{"epoch", payload.at("epoch")}, {"revision", payload.at("revision")}}}};
        if (lose_wait_reply) answer(reply, {{"id", operation}, {"state", "accepted"}});
        // The mutation committed. Withhold its acknowledgement until the caller's budget expires.
      } else if (request.target == "/v1/operations/" + operation + "/wait") {
        ++waits;
        if (lose_wait_reply) {
          // The original wait acknowledgement is lost after completion.
          EXPECT_LT(request.deadline, submitted_deadline + milliseconds(2));
        } else {
          // Receipt lookup and recovery wait share the same observation budget.
          EXPECT_GT(request.deadline, observation_deadline - milliseconds(2));
          EXPECT_LT(request.deadline, observation_deadline + milliseconds(2));
          answer(reply, terminal);
        }
      } else if (request.target == "/v1/operations/" + operation) {
        ++receipts;
        EXPECT_EQ(request.method, "GET");
        EXPECT_TRUE(request.body.empty());
        EXPECT_NE(request.request_id, operation);
        EXPECT_GT(request.deadline, submitted_deadline);
        EXPECT_LE(request.deadline, xgc2::xrpc::Clock::now() + milliseconds(52));
        observation_deadline = request.deadline;
        bool fenced = false;
        for (const auto& header : request.headers)
          if (header.first == "X-Xrpc-Instance-ID" || header.first == "x-xrpc-instance-id") {
            EXPECT_EQ(header.second, "world");
            fenced = true;
          }
        EXPECT_TRUE(fenced);
        if (lose_wait_reply) answer(reply, terminal);
        else {
          // Consume part of the budget so a restarted wait deadline is observable.
          std::this_thread::sleep_for(milliseconds(10));
          answer(reply, {{"id", operation}, {"state", "running"}});
        }
      } else completed(std::move(request), std::move(reply));
    });
    SimulationClient client((root.path / "world.sock").string(), "fixture", milliseconds(50));
    client.update(Json::object(), "epoch", 6, {{"playing", false}});
    const auto applied = report(client, true, seconds(2));
    client.close();
    ASSERT_TRUE(applied);
    EXPECT_EQ(applied->at("instance_id"), "world");
    EXPECT_EQ(applied->at("revision"), 6);
    EXPECT_EQ(applies.load(), 1);
    EXPECT_EQ(receipts.load(), 1);
    EXPECT_EQ(waits.load(), 1);
    EXPECT_EQ(world.describes.load(), 1);
  }
}
TEST(SimulationClient, RefusedAndMismatchedApplicationsNeverAuthorizeOrLoop) {
  for (int fault = 0; fault < 3; ++fault) {
    Directory root;
    std::atomic<int> applies{0};
    World world(root.path / "world.sock", "world", [&](HttpRequest request, HttpReply reply) {
      ++applies;
      if (fault == 0) answer(reply, {{"id", request.request_id}, {"state", "failed"}, {"error", "native rejected"}});
      else if (fault == 1) answer(reply, {{"id", "other"}, {"state", "succeeded"}});
      else answer(reply, {{"id", request.request_id}, {"state", "succeeded"}, {"result", {{"epoch", "stale"}, {"revision", 8}}}});
    });
    SimulationClient client((root.path / "world.sock").string(), "fixture", seconds(1));
    client.update(Json::object(), "epoch", 7, {{"playing", false}});
    auto failed = report(client, false);
    ASSERT_TRUE(failed);
    EXPECT_FALSE(failed->at("message").get<std::string>().empty());
    std::this_thread::sleep_for(milliseconds(30));
    EXPECT_EQ(applies.load(), 1);
    client.close();
  }
}
TEST(SimulationClient, HeldDiscoveryIsSingleFlightAndCloseCancelsIt) {
  Directory root;
  std::promise<void> entered;
  auto waiting = entered.get_future();
  World world(root.path / "world.sock", "world", [&](HttpRequest, HttpReply) { entered.set_value(); }, true);
  SimulationClient client((root.path / "world.sock").string(), "fixture");
  client.update(Json::object(), "epoch", 1, {{"playing", false}});
  ASSERT_EQ(waiting.wait_for(seconds(1)), std::future_status::ready);
  for (int i = 2; i < 20; ++i) client.update(Json::object(), "epoch", i, {{"playing", false}});
  EXPECT_EQ(world.describes.load(), 1);
  const auto start = steady_clock::now();
  client.close();
  EXPECT_LT(steady_clock::now() - start, seconds(1));
}
TEST(SimulationClient, HeldDiscoveryReplaysOnlyTheLatestDesiredState) {
  Directory root;
  std::promise<HttpReply> held;
  auto discovery = held.get_future();
  std::atomic<int> applies{0};
  std::mutex mutex;
  Json applied;
  World world(root.path / "world.sock", "world", [&](HttpRequest request, HttpReply reply) {
    if (request.method == "GET") held.set_value(std::move(reply));
    else {
      if (request.target == "/v1/extensions/scene/apply") {
        std::lock_guard<std::mutex> lock(mutex);
        applied = parseJson(request.body);
        ++applies;
      }
      completed(std::move(request), std::move(reply));
    }
  }, true);
  SimulationClient client((root.path / "world.sock").string(), "fixture");
  client.update({{"name", "first"}}, "epoch", 1, {{"playing", false}});
  ASSERT_EQ(discovery.wait_for(seconds(1)), std::future_status::ready);
  for (int i = 2; i < 20; ++i) client.update({{"name", "latest"}}, "epoch", i, {{"playing", true}});
  EXPECT_EQ(world.describes.load(), 1);
  answer(discovery.get(), {{"service", "xgc2.simulation"}, {"api_version", "v1"}, {"instance_id", "world"}, {"ready", true}});
  ASSERT_TRUE(report(client, true));
  client.close();
  EXPECT_EQ(applies.load(), 1);
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(applied.at("revision"), 19);
  EXPECT_EQ(applied.at("document").at("name"), "latest");
}

TEST(SimulationClient, ValidatesEndpointAndLocalTargetBeforeDialling) {
  for (const auto& endpoint : {std::string(""), std::string("relative/world.sock"), std::string("/world\0.sock", 12)}) {
    EXPECT_THROW(SimulationClient(endpoint, "fixture"), SceneError);
  }
  for (const auto& target : {std::string(""), std::string("wrong/value"), std::string(129, 'x')}) {
    EXPECT_THROW(SimulationClient("/unused/world.sock", target), SceneError);
  }
  EXPECT_THROW(SimulationClient("/unused/world.sock", "fixture", milliseconds(0)), SceneError);
}

TEST(SimulationClient, RejectsMismatchedDiscoveredWorldIdentityBeforeApplying) {
  for (const auto* field : {"service", "api_version", "target_id", "instance_id"}) {
    Directory root;
    std::atomic<int> operations{0};
    const bool target_mismatch = std::string(field) == "target_id";
    const bool instance_mismatch = std::string(field) == "instance_id";
    const std::string message = target_mismatch ? "Discovered world target differs from the local target" :
                                instance_mismatch ? "Invalid discovered world instance" : "World host is not ready";
    World world(root.path / "world.sock", "world", [&](HttpRequest request, HttpReply reply) {
      if (request.method != "GET") {
        ++operations;
        completed(std::move(request), std::move(reply));
        return;
      }
      Json description{{"service", "xgc2.simulation"}, {"api_version", "v1"}, {"instance_id", "world"},
                       {"ready", true}, {"service_ref", {{"target_id", "fixture"}}}};
      if (target_mismatch) description["service_ref"]["target_id"] = "other-target";
      else description[field] = "wrong/value";
      answer(reply, description);
    }, true);
    SimulationClient client((root.path / "world.sock").string(), "fixture");
    client.update(Json::object(), "epoch", 1, {{"playing", false}});
    std::optional<Json> rejected;
    const auto deadline = steady_clock::now() + seconds(2);
    while (steady_clock::now() < deadline) {
      auto value = client.takeReport();
      if (value && value->at("message") == message) { rejected = std::move(value); break; }
      std::this_thread::sleep_for(milliseconds(5));
    }
    ASSERT_TRUE(rejected) << field;
    EXPECT_FALSE(rejected->at("applied"));
    EXPECT_EQ(operations.load(), 0);
    client.close();
  }
}
}  // namespace
}  // namespace xgc_scene
