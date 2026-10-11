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

class World {
 public:
  explicit World(xgc2::xrpc::HttpServer::Handler handler) {
    char pattern[] = "/tmp/xgc-scene-simulation-XXXXXX";
    const auto directory = mkdtemp(pattern);
    if (!directory) throw std::runtime_error("Cannot create simulation test directory");
    root_ = directory;
    reference = Json{{"target_id", "fixture"}, {"service", "xgc2.simulation"}, {"api_version", "v1"},
                     {"instance_id", "world-instance"}, {"profile", "http.v1"},
                     {"endpoint", Json{{"kind", "unix"}, {"address", (root_ / "world.sock").string()}}}};
    xgc2::xrpc::HttpLimits limits;
    limits.request_bytes = limits.response_bytes = kMaxDocumentBytes;
    server_.reset(new xgc2::xrpc::HttpServer(
        xgc2::xrpc::UnixOptions{reference.at("endpoint").at("address").get<std::string>()},
        std::move(handler), limits, {"world-instance", {"/v1/describe"}}));
    thread_ = std::thread([this] { server_->run(); });
  }
  ~World() {
    server_->request_stop();
    if (thread_.joinable()) thread_.join();
    server_.reset();
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }
  Json reference;
 private:
  std::filesystem::path root_;
  std::unique_ptr<xgc2::xrpc::HttpServer> server_;
  std::thread thread_;
};

void answer(const HttpReply& reply, const Json& value, int status = 200) {
  reply.complete({status, {{"Content-Type", "application/json"}}, value.dump()});
}

TEST(SimulationClient, SubmitsOnceAndWaitsOnceWithDistinctTransportIdentity) {
  std::mutex mutex;
  std::vector<HttpRequest> calls;
  World world([&](HttpRequest request, HttpReply reply) {
    std::lock_guard<std::mutex> lock(mutex);
    calls.push_back(request);
    if (request.target == "/v1/entities") answer(reply, Json{{"id", request.request_id}, {"state", "accepted"}});
    else if (request.target == "/v1/operations/create-one/wait")
      answer(reply, Json{{"id", "create-one"}, {"state", "succeeded"}, {"result", Json{{"entity", Json{{"id", "one"}, {"generation", 1}}}}}});
    else { ADD_FAILURE() << request.target; answer(reply, Json::object(), 404); }
  });
  SimulationClient client(world.reference, "fixture", seconds(2));
  const auto result = client.operation("/v1/entities", Json{{"entity", Json::object()}}, "POST", "create-one");
  EXPECT_EQ(result.at("entity").at("generation"), 1);
  client.close();
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(calls.size(), 2u);
  EXPECT_EQ(calls[0].method, "POST");
  EXPECT_EQ(calls[1].method, "POST");
  EXPECT_EQ(calls[0].request_id, "create-one");
  EXPECT_NE(calls[0].request_id, calls[1].request_id);
  EXPECT_EQ(parseJson(calls[0].body).at("operation_timeout_ms"), 2000);
  EXPECT_EQ(parseJson(calls[1].body), Json::object());
  EXPECT_LT(calls[1].deadline, calls[0].deadline + milliseconds(2));
}

TEST(SimulationClient, ImmediateTerminalSuccessNeedsNoWaitAndApplyChecksRevision) {
  std::atomic<int> calls{0};
  World world([&](HttpRequest request, HttpReply reply) {
    ++calls;
    const auto payload = parseJson(request.body);
    Json result{{"epoch", payload.at("epoch")}, {"revision", payload.at("revision")}};
    if (payload.at("revision") == 7) result["revision"] = 8;
    answer(reply, Json{{"id", request.request_id}, {"state", "succeeded"}, {"result", result}});
  });
  SimulationClient client(world.reference, "fixture");
  EXPECT_EQ(client.apply(Json::object(), "epoch", 6), (Json{{"epoch", "epoch"}, {"revision", 6}}));
  try { client.apply(Json::object(), "epoch", 7); FAIL(); }
  catch (const SceneError& error) { EXPECT_STREQ(error.what(), "Simulation did not commit the requested scene revision"); }
  EXPECT_EQ(calls.load(), 2);
  client.close();
}

TEST(SimulationClient, MotionUsesOnlyItsExplicitExtension) {
  std::mutex mutex;
  std::string route;
  Json received;
  World world([&](HttpRequest request, HttpReply reply) {
    std::lock_guard<std::mutex> lock(mutex);
    route = request.target;
    received = parseJson(request.body);
    answer(reply, Json{{"id", request.request_id}, {"state", "succeeded"}, {"result", Json::object()}});
  });
  SimulationClient client(world.reference, "fixture");
  EXPECT_EQ(client.motion("pause", "epoch", 2), Json::object());
  client.close();
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(route, "/v1/extensions/scene/motion");
  EXPECT_EQ(received, (Json{{"operation", "pause"}, {"epoch", "epoch"}, {"revision", 2}, {"operation_timeout_ms", 30000}}));
}

TEST(SimulationClient, ValidatesExplicitServiceAndLocalUnixTargetBeforeDialling) {
  const Json reference{{"target_id", "fixture"}, {"service", "xgc2.simulation"}, {"api_version", "v1"},
                       {"instance_id", "world-instance"}, {"profile", "http.v1"},
                       {"endpoint", Json{{"kind", "unix"}, {"address", "/unused/world.sock"}}}};
  for (const auto* field : {"service", "api_version", "instance_id", "target_id", "profile"}) {
    auto bad = reference;
    bad[field] = "wrong/value";
    EXPECT_THROW(SimulationClient(bad, "fixture"), SceneError);
  }
  for (const auto& address : {Json("relative"), Json(1), Json("")}) {
    auto bad = reference;
    bad["endpoint"]["address"] = address;
    EXPECT_THROW(SimulationClient(bad, "fixture"), SceneError);
  }
  EXPECT_THROW(SimulationClient(Json::object(), "fixture"), SceneError);
  EXPECT_THROW(SimulationClient(reference, "fixture", milliseconds(0)), SceneError);
}

TEST(SimulationClient, WrongIdentityAndTerminalFailureAreNeverReplayed) {
  std::atomic<int> calls{0};
  World world([&](HttpRequest request, HttpReply reply) {
    ++calls;
    if (request.target == "/wrong") answer(reply, Json{{"id", "other"}, {"state", "succeeded"}});
    else answer(reply, Json{{"id", request.request_id}, {"state", "failed"}, {"error", "native rejected"}});
  });
  SimulationClient client(world.reference, "fixture");
  try { client.operation("/wrong", Json::object(), "POST", "wrong-id"); FAIL(); }
  catch (const SceneError& error) { EXPECT_STREQ(error.what(), "Simulation returned a different operation identity"); }
  try { client.operation("/failed", Json::object(), "POST", "failed-id"); FAIL(); }
  catch (const SceneError& error) { EXPECT_STREQ(error.what(), "Simulation operation failed-id did not complete: native rejected"); }
  EXPECT_EQ(calls.load(), 2);
  client.close();
}

TEST(SimulationClient, RunningWaitMustReturnTheOriginalSucceededIdentity) {
  std::atomic<int> calls{0};
  World world([&](HttpRequest request, HttpReply reply) {
    const auto count = ++calls;
    answer(reply, Json{{"id", count == 1 ? request.request_id : "other"}, {"state", count == 1 ? "running" : "succeeded"}});
  });
  SimulationClient client(world.reference, "fixture");
  try { client.operation("/operation", Json::object(), "POST", "running-id"); FAIL(); }
  catch (const SceneError& error) { EXPECT_STREQ(error.what(), "Simulation operation running-id did not complete: succeeded"); }
  EXPECT_EQ(calls.load(), 2);
  client.close();
}

TEST(SimulationClient, FaultAndLostResponsePreserveTheirOutcomeInformation) {
  std::atomic<int> calls{0};
  World world([&](HttpRequest request, HttpReply reply) {
    ++calls;
    if (request.target == "/fault") answer(reply, Json{{"error", Json{{"code", "unsupported"}, {"message", "No scene extension"}}}}, 501);
  });
  SimulationClient client(world.reference, "fixture", milliseconds(60));
  try { client.operation("/fault", Json::object(), "POST", "fault-id"); FAIL(); }
  catch (const SceneError& error) { EXPECT_STREQ(error.what(), "Simulation operation fault-id: No scene extension (unsupported)"); }
  try { client.operation("/lost", Json::object(), "POST", "lost-id"); FAIL(); }
  catch (const SceneError& error) {
    EXPECT_NE(std::string(error.what()).find("Simulation operation lost-id:"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("(outcome_unknown)"), std::string::npos);
  }
  EXPECT_EQ(calls.load(), 2);
  client.close();
}

TEST(SimulationClient, CloseCancelsTheOutstandingNotificationWait) {
  std::promise<void> waiting;
  auto entered = waiting.get_future();
  std::atomic<int> calls{0};
  World world([&](HttpRequest request, HttpReply reply) {
    ++calls;
    if (request.target == "/submit") answer(reply, Json{{"id", request.request_id}, {"state", "accepted"}});
    else waiting.set_value();
  });
  SimulationClient client(world.reference, "fixture", seconds(2));
  auto result = std::async(std::launch::async, [&] {
    try { client.operation("/submit", Json::object(), "POST", "close-id"); return std::string(); }
    catch (const SceneError& error) { return std::string(error.what()); }
  });
  EXPECT_EQ(entered.wait_for(seconds(1)), std::future_status::ready);
  client.close();
  ASSERT_EQ(result.wait_for(seconds(1)), std::future_status::ready);
  EXPECT_NE(result.get().find("(outcome_unknown)"), std::string::npos);
  EXPECT_EQ(calls.load(), 2);
}

}  // namespace
}  // namespace xgc_scene
