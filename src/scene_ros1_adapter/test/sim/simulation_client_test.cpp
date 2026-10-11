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
Json reference(const std::filesystem::path& path) {
  return {{"target_id", "fixture"}, {"service", "xgc2.simulation"}, {"api_version", "v1"},
          {"instance_id", "original-instance"}, {"profile", "http.v1"},
          {"endpoint", {{"kind", "unix"}, {"address", path.string()}}}};
}
class World {
 public:
  World(const std::filesystem::path& path, const std::string& instance, xgc2::xrpc::HttpServer::Handler handler,
        bool hold_discovery = false) {
    server_ = std::make_unique<xgc2::xrpc::HttpServer>(xgc2::xrpc::UnixOptions{path.string()},
      [this, instance, handler, hold_discovery](HttpRequest request, HttpReply reply) {
        if (request.method == "GET") {
          EXPECT_EQ(request.target, "/v1/describe?wait_ready_ms=30000");
          ++describes;
          if (hold_discovery) handler(std::move(request), std::move(reply));
          else answer(reply, {{"service", "xgc2.simulation"}, {"api_version", "v1"},
                              {"instance_id", instance}, {"ready", true}});
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
  const auto path = root.path / "world.sock";
  SimulationClient client(reference(path), "fixture", seconds(2));
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
  SimulationClient client(reference(root.path / "world.sock"), "fixture", seconds(2));
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
    SimulationClient client(reference(root.path / "world.sock"), "fixture", seconds(1));
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
  SimulationClient client(reference(root.path / "world.sock"), "fixture");
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
  SimulationClient client(reference(root.path / "world.sock"), "fixture");
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

TEST(SimulationClient, ValidatesExplicitServiceAndLocalUnixTargetBeforeDialling) {
  const auto ref = reference("/unused/world.sock");
  for (const auto* field : {"service", "api_version", "instance_id", "target_id", "profile"}) {
    auto invalid = ref; invalid[field] = "wrong/value";
    EXPECT_THROW(SimulationClient(invalid, "fixture"), SceneError);
  }
  for (const auto& address : {Json("relative"), Json(1), Json("")}) {
    auto invalid = ref; invalid["endpoint"]["address"] = address;
    EXPECT_THROW(SimulationClient(invalid, "fixture"), SceneError);
  }
  EXPECT_THROW(SimulationClient(ref, "fixture", milliseconds(0)), SceneError);
}
}  // namespace
}  // namespace xgc_scene
