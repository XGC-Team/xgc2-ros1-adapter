#include "xgc_scene/service.hpp"
#include "xgc_scene/json_io.hpp"

#include <gtest/gtest.h>
#include <xgc2/xrpc/unix.hpp>

#include <atomic>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <future>
#include <map>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

namespace xgc_scene {
namespace {
using namespace std::chrono;
using xgc2::xrpc::HttpRequest;
using xgc2::xrpc::HttpResponse;

const Json empty{{"schema", "xgc2.scene.v1"}, {"id", "test"}, {"frame", "world"}, {"obstacles", Json::array()}};

class StoreNode final : public AuthoringNode {
 public:
  explicit StoreNode(const Json& initial = empty) : store(initial, {}, {}, {}, [] { return 100.0; }) {}
  bool online() const override { return is_online.load(); }
  Json envelope() override { return store.envelope(); }
  Json commandValue(const Json& value) override { ++commands; return store.command(value); }
  std::optional<Json> requestResult(const std::string& id) override { return store.requestResult(id); }
  SceneStore store;
  std::atomic<bool> is_online{true};
  std::atomic<int> commands{0};
};

class Directory {
 public:
  Directory() {
    char pattern[] = "/tmp/xgc-scene-wire-XXXXXX";
    const auto directory = mkdtemp(pattern);
    if (!directory) throw std::runtime_error("Cannot create wire test directory");
    path = directory;
  }
  ~Directory() { std::error_code error; std::filesystem::remove_all(path, error); }
  std::filesystem::path path;
};

class Host {
 public:
  explicit Host(SceneService& service) : service_(service), server_(
      xgc2::xrpc::UnixOptions{service.reference().at("endpoint").at("address").get<std::string>()},
      [&service](HttpRequest request, xgc2::xrpc::HttpReply reply) { service.handle(std::move(request), std::move(reply)); },
      SceneService::limits(), {service.reference().at("instance_id").get<std::string>(), {"/v1/describe"}}),
      thread_([this] { server_.run(); }) {}
  ~Host() { close(); }
  void close() {
    server_.request_stop();
    if (thread_.joinable()) thread_.join();
    service_.close();
  }
 private:
  SceneService& service_;
  xgc2::xrpc::HttpServer server_;
  std::thread thread_;
};

std::future<HttpResponse> call(SceneService& service, std::string method, std::string path,
                                const Json& body = nullptr, bool has_body = false) {
  auto promise = std::make_shared<std::promise<HttpResponse>>();
  auto result = promise->get_future();
  HttpRequest request;
  request.method = std::move(method); request.target = std::move(path); request.request_id = "fixture";
  request.deadline = xgc2::xrpc::Clock::now() + seconds(2);
  if (has_body) { request.body = dumpJson(body); request.headers = {{"Content-Type", "application/json"}}; }
  service.handle(std::move(request), [promise](HttpResponse response) { promise->set_value(std::move(response)); });
  return result;
}

// Raw Unix IO also observes the 409 body that a pinned SDK client correctly
// refuses to deliver. One connection per corpus request; every fd is closed.
HttpResponse raw(const std::string& path, const Json& request) {
  struct Descriptor { int value; ~Descriptor() { if (value >= 0) ::close(value); } } descriptor{::socket(AF_UNIX, SOCK_STREAM, 0)};
  if (descriptor.value < 0) throw std::runtime_error("Cannot create wire socket");
  timeval timeout{2, 0};
  setsockopt(descriptor.value, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(descriptor.value, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (path.size() >= sizeof(address.sun_path)) throw std::runtime_error("Wire socket path is too long");
  std::copy(path.begin(), path.end(), address.sun_path);
  if (::connect(descriptor.value, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
    throw std::runtime_error("Cannot connect wire socket");
  const auto body = request.at("body").is_null() ? std::string() : request.at("body").dump();
  std::string bytes = request.at("method").get<std::string>() + " " + request.at("path").get<std::string>() +
      " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\nX-Xrpc-Timeout-Ms: 1000\r\n";
  for (auto it = request.at("headers").begin(); it != request.at("headers").end(); ++it)
    bytes += it.key() + ": " + it.value().get<std::string>() + "\r\n";
  bytes += "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto sent = ::send(descriptor.value, bytes.data() + offset, bytes.size() - offset, MSG_NOSIGNAL);
    if (sent <= 0) throw std::runtime_error("Cannot send wire request");
    offset += static_cast<std::size_t>(sent);
  }
  bytes.clear();
  char buffer[4096];
  for (;;) {
    const auto received = ::recv(descriptor.value, buffer, sizeof(buffer), 0);
    if (received == 0) break;
    if (received < 0) throw std::runtime_error("Cannot receive wire reply");
    bytes.append(buffer, static_cast<std::size_t>(received));
  }
  const auto end = bytes.find("\r\n\r\n");
  if (end == std::string::npos) throw std::runtime_error("Incomplete wire reply");
  HttpResponse result;
  std::istringstream headers(bytes.substr(0, end));
  std::string line, protocol;
  std::getline(headers, line);
  std::istringstream status(line);
  status >> protocol >> result.status;
  while (std::getline(headers, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto colon = line.find(": ");
    if (colon != std::string::npos) result.headers.emplace_back(line.substr(0, colon), line.substr(colon + 2));
  }
  result.body = bytes.substr(end + 4);
  return result;
}

Json replace(const Json& value, const std::map<std::string, std::string>& bindings) {
  if (value.is_string()) {
    auto text = value.get<std::string>();
    for (const auto& binding : bindings) {
      std::size_t at = 0;
      while ((at = text.find(binding.first, at)) != std::string::npos) {
        text.replace(at, binding.first.size(), binding.second);
        at += binding.second.size();
      }
    }
    return text;
  }
  if (value.is_object()) {
    auto result = Json::object();
    for (auto it = value.begin(); it != value.end(); ++it) result[it.key()] = replace(it.value(), bindings);
    return result;
  }
  if (value.is_array()) {
    auto result = Json::array();
    for (const auto& item : value) result.push_back(replace(item, bindings));
    return result;
  }
  return value;
}

TEST(SceneWire, ReplaysEveryGoldenRouteAndEnvelope) {
  std::ifstream input(std::string(XGC_SCENE_GOLDEN_DIR) + "/corpus/wire/authoring-http.json");
  ASSERT_TRUE(input.good());
  const auto corpus = Json::parse(input);
  Directory directory;
  auto node = std::make_shared<StoreNode>(corpus.at("setup").at("stores").at("main").at("document"));
  const auto path = (directory.path / "scene.sock").string();
  SceneService service(path, "fixture", "gazebo");
  const auto instance = service.reference().at("instance_id").get<std::string>();
  const std::map<std::string, std::string> bindings{{"$work", directory.path.string()},
      {"$epoch", node->store.status().at("epoch").get<std::string>()}, {"$instance", instance}};
  service.ready(node);
  {
    Host host(service);
    std::size_t index = 0;
    for (const auto& step : corpus.at("steps")) {
      SCOPED_TRACE(index++);
      const auto request = replace(step.at("request"), bindings);
      auto expected = replace(step.at("response").at("value"), bindings);
      auto actual = raw(path, request);
      auto body = parseJson(actual.body);
      if (request.at("path").get<std::string>().find("/v1/describe") == 0) {
        // arch.md 4a and the profile agreement add these mandatory facts to
        // the old Python recording. Compare every recorded byte beneath them.
        EXPECT_EQ(body.at("service"), "xgc2.scene.authoring");
        EXPECT_EQ(body.at("api_version"), "v1");
        EXPECT_EQ(body.at("instance_id"), instance);
        EXPECT_EQ(body.at("ready"), true);
        EXPECT_EQ(body.at("facts").at("profile"), "gazebo");
        EXPECT_EQ(body.at("facts").at("lidar"), (Json{{"backend", "cpu"}, {"gpu_supported", false}}));
        // F10 commits local desired state while native application is asynchronous.
        const std::string completion = "The command commits local desired state. Native application completes asynchronously and is reported in the scene envelope; reconnect replays the current scene once per world instance.";
        EXPECT_EQ(body.at("completion"), completion);
        const auto old_completion = expected.at("body").at("completion").get<std::string>();
        expected["body"]["completion"] = completion;
        auto expected_text = expected.at("body_text").get<std::string>();
        const auto offset = expected_text.find(old_completion);
        ASSERT_NE(offset, std::string::npos);
        expected_text.replace(offset, old_completion.size(), completion);
        expected["body_text"] = expected_text;
        for (const auto* key : {"service", "api_version", "instance_id", "ready", "facts"}) body.erase(key);
        const auto extra = actual.body.rfind(",\"service\":");
        ASSERT_NE(extra, std::string::npos);
        actual.body = actual.body.substr(0, extra) + "}";
      }
      if (actual.status == 409) {
        // XRPC C++ 0.2.0 owns this fence and uses a different diagnostic than
        // Python 0.2.0; no adapter compatibility layer rewrites SDK errors.
        expected["body"]["error"]["message"] = "service instance identity mismatch";
        expected["body_text"] = "{\"error\":{\"code\":\"conflict\",\"message\":\"service instance identity mismatch\"}}";
      }
      EXPECT_EQ(actual.status, expected.at("status"));
      EXPECT_EQ(body, expected.at("body"));
      EXPECT_EQ(actual.body, expected.at("body_text"));
      for (auto it = expected.at("headers").begin(); it != expected.at("headers").end(); ++it) {
        const auto found = std::find_if(actual.headers.begin(), actual.headers.end(), [&](const auto& header) { return header.first == it.key(); });
        ASSERT_NE(found, actual.headers.end());
        EXPECT_EQ(found->second, it.value());
      }
      if (actual.status == 409) {
        xgc2::xrpc::HttpClient stale(path, SceneService::limits(), "stale");
        HttpRequest call;
        call.method = "GET"; call.target = "/v1/scene"; call.request_id = "instance-fence";
        try { stale.call(call, xgc2::xrpc::Clock::now() + seconds(1)); FAIL() << "Stale instance must fail"; }
        catch (const xgc2::xrpc::HttpCallError& error) {
          EXPECT_EQ(std::string(xgc2::xrpc::delivery_name(error.delivery)), expected.at("client").at("disposition"));
          EXPECT_EQ(error.code, "conflict");
        }
        stale.close();
      }
    }
  }
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(SceneService, ReadinessIsHeldThenReleasedByNotification) {
  SceneService service("/unused/scene.sock", "fixture", "physical");
  auto waiting = call(service, "GET", "/v1/describe?wait_ready_ms=30000");
  EXPECT_EQ(waiting.wait_for(milliseconds(20)), std::future_status::timeout);
  service.ready(std::make_shared<StoreNode>(), Json{{"store_ready", true}});
  ASSERT_EQ(waiting.wait_for(seconds(1)), std::future_status::ready);
  const auto result = parseJson(waiting.get().body);
  EXPECT_EQ(result.at("ready"), true);
  EXPECT_EQ(result.at("facts").at("store_ready"), true);
}

TEST(SceneService, UnboundUnixDescribeQueryIsHeldWhileStarting) {
  Directory directory;
  const auto path = (directory.path / "scene.sock").string();
  SceneService service(path, "fixture", "physical");
  Host host(service);
  xgc2::xrpc::HttpClient client(path, SceneService::limits());
  auto waiting = std::async(std::launch::async, [&] {
    HttpRequest request;
    request.method = "GET";
    request.target = "/v1/describe?wait_ready_ms=30000";
    request.request_id = "unbound-starting";
    return client.call(request, xgc2::xrpc::Clock::now() + seconds(2));
  });
  EXPECT_EQ(waiting.wait_for(milliseconds(20)), std::future_status::timeout);
  service.ready(std::make_shared<StoreNode>());
  ASSERT_EQ(waiting.wait_for(seconds(1)), std::future_status::ready);
  const auto response = waiting.get();
  EXPECT_EQ(response.status, 200);
  EXPECT_EQ(parseJson(response.body).at("ready"), true);
  client.close();
}

TEST(SceneService, ReadinessExpiresAndShutdownReleasesWaiters) {
  SceneService service("/unused/scene.sock", "fixture", "physical");
  auto timeout = call(service, "GET", "/v1/describe?wait_ready_ms=30");
  ASSERT_EQ(timeout.wait_for(seconds(1)), std::future_status::ready);
  EXPECT_EQ(parseJson(timeout.get().body).at("ready"), false);
  auto waiting = call(service, "GET", "/v1/describe?wait_ready_ms=30000");
  service.stopping();
  ASSERT_EQ(waiting.wait_for(seconds(1)), std::future_status::ready);
  EXPECT_EQ(parseJson(waiting.get().body).at("ready"), false);
  EXPECT_EQ(parseJson(call(service, "GET", "/v1/health").get().body), (Json{{"lifecycle", "stopping"}}));
}

TEST(SceneService, RejectsInvalidQueryCommandAndReceiptBodies) {
  SceneService service("/unused/scene.sock", "fixture", "physical");
  auto node = std::make_shared<StoreNode>();
  service.ready(node);
  for (const auto* query : {"wait_ready_ms=30001", "wait_ready_ms=-1", "wait_ready_ms=1&extra=2", "wait_ready_ms=", "x=1"})
    EXPECT_EQ(call(service, "GET", std::string("/v1/describe?") + query).get().status, 400);
  for (const auto& body : {Json(), Json::array(), Json("x"), Json{{"operation", "play"}, {"requestId", "body"}}}) {
    const auto result = call(service, "POST", "/v1/commands", body, true).get();
    EXPECT_EQ(result.status, 400);
    EXPECT_EQ(parseJson(result.body).at("error").at("message"), "Command identity must come from X-Request-ID");
  }
  for (const auto& body : {Json(), Json::array(), Json{{"request_id", 1}}, Json{{"request_id", "x"}, {"extra", 1}}})
    EXPECT_EQ(call(service, "POST", "/v1/commands/result", body, true).get().status, 400);
  EXPECT_EQ(node->commands.load(), 0);
  node->is_online = false;
  EXPECT_EQ(parseJson(call(service, "GET", "/v1/health").get().body).at("lifecycle"), "stopping");
}

TEST(SceneService, DeclaresProfilesAndBounds) {
  const auto limits = SceneService::limits();
  EXPECT_EQ(limits.inflight, 4u);
  EXPECT_EQ(limits.request_bytes, kMaxDocumentBytes);
  EXPECT_EQ(limits.response_bytes, kMaxDocumentBytes + 65536);
  EXPECT_EQ(limits.request_timeout, seconds(60));
  SceneService xsim("/unused/scene.sock", "fixture", "xsim");
  xsim.ready(std::make_shared<StoreNode>());
  const auto describe = parseJson(call(xsim, "GET", "/v1/describe").get().body);
  EXPECT_EQ(describe.at("capabilities"), Json::array({"scene.read", "scene.edit", "scene.save"}));
  EXPECT_EQ(describe.at("facts").at("scene_application"), "unsupported");
  for (const auto* operation : {"play", "pause", "reset", "resync"}) {
    const auto result = call(xsim, "POST", "/v1/commands", Json{{"operation", operation}}, true).get();
    EXPECT_EQ(result.status, 501);
    EXPECT_EQ(parseJson(result.body).at("error").at("code"), "unsupported");
  }
  EXPECT_EQ(call(xsim, "POST", "/v1/commands", Json{{"operation", "get"}}, true).get().status, 200);
  SceneService physical("/unused/scene.sock", "fixture", "gazebo");
  physical.ready(std::make_shared<StoreNode>(), Json{{"profile", "physical"}});
  const auto local = parseJson(call(physical, "GET", "/v1/describe").get().body);
  EXPECT_EQ(local.at("facts").at("profile"), "physical");
  EXPECT_EQ(local.at("facts").at("scene_application"), "visualization");
  EXPECT_FALSE(local.at("facts").contains("lidar"));
}

TEST(SceneService, ExpiredAndCancelledRequestsNeverReachTheStore) {
  SceneService service("/unused/scene.sock", "fixture", "physical");
  auto node = std::make_shared<StoreNode>();
  service.ready(node);
  HttpRequest request;
  request.method = "POST"; request.target = "/v1/commands"; request.request_id = "expired";
  request.headers = {{"Content-Type", "application/json"}};
  request.body = "{\"operation\":\"get\"}";
  request.deadline = xgc2::xrpc::Clock::now() - milliseconds(1);
  auto result = std::make_shared<std::promise<HttpResponse>>();
  auto future = result->get_future();
  service.handle(request, [result](HttpResponse response) { result->set_value(std::move(response)); });
  ASSERT_EQ(future.wait_for(seconds(1)), std::future_status::ready);
  EXPECT_EQ(future.get().status, 408);
  service.handle(request, [](HttpResponse) { ADD_FAILURE() << "Cancelled reply was completed"; }, [] { return true; });
  service.close();
  EXPECT_EQ(node->commands.load(), 0);
}

}  // namespace
}  // namespace xgc_scene
