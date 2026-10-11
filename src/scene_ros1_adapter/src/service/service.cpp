#include "xgc_scene/service.hpp"

#include "xgc_scene/json_io.hpp"

#include <xgc2/xrpc/json_http.hpp>
#include <xgc2/xrpc/unix.hpp>

#include <algorithm>
#include <cctype>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace xgc_scene {
namespace {
using namespace std::chrono;
using xgc2::xrpc::HttpRequest;
using xgc2::xrpc::HttpResponse;
using xgc2::xrpc::Clock;

HttpResponse response(const Json& value, int status = 200) {
  // The Python host uses compact separators, preserving dict insertion order
  // and its numeric spelling. The ROS envelope encoder uses spaced separators.
  const auto spaced = envelopeJson(value);
  std::string compact;
  compact.reserve(spaced.size());
  bool quoted = false, escaped = false;
  for (const char c : spaced) {
    if (quoted || c != ' ') compact += c;
    if (escaped) escaped = false;
    else if (quoted && c == '\\') escaped = true;
    else if (c == '"') quoted = !quoted;
  }
  if (compact.size() > kMaxDocumentBytes + 65536)
    return xgc2::xrpc::http_error(503, "resource_exhausted", "JSON response limit exceeded");
  return {status, {{"Content-Type", "application/json"}}, std::move(compact)};
}

HttpResponse fault(int status, const std::string& code, const std::string& message) {
  return response(Json{{"error", Json{{"code", code}, {"message", message}}}}, status);
}

bool waitBudget(const std::string& target, milliseconds& wait) {
  wait = milliseconds(0);
  if (target == "/v1/describe") return true;
  const std::string prefix = "/v1/describe?wait_ready_ms=";
  if (target.compare(0, prefix.size(), prefix) != 0) return false;
  const auto digits = target.substr(prefix.size());
  if (digits.empty() || digits.size() > 5 || digits.find_first_not_of("0123456789") != std::string::npos)
    return false;
  const auto count = std::stoul(digits);
  if (count > 30000) return false;
  wait = milliseconds(count);
  return true;
}

bool jsonContentType(const xgc2::xrpc::Headers& headers) {
  std::size_t count = 0;
  for (const auto& header : headers) {
    std::string name = header.first, type = header.second.substr(0, header.second.find(';'));
    const auto lower = [](unsigned char c) { return static_cast<char>(std::tolower(c)); };
    std::transform(name.begin(), name.end(), name.begin(), lower);
    if (name != "content-type") continue;
    ++count;
    std::transform(type.begin(), type.end(), type.begin(), lower);
    const auto first = type.find_first_not_of(" \t"), last = type.find_last_not_of(" \t");
    if (first == std::string::npos || type.substr(first, last - first + 1) != "application/json") return false;
  }
  return count == 1;
}
}  // namespace

class SceneService::Impl {
 public:
  Impl(std::string path, std::string target, std::string profile) : profile_(std::move(profile)) {
    if (profile_ != "gazebo" && profile_ != "xsim" && profile_ != "physical")
      throw SceneError("Scene profile must be gazebo, xsim or physical");
    reference_ = Json{{"target_id", std::move(target)}, {"service", "xgc2.scene.authoring"},
                      {"api_version", "v1"}, {"instance_id", xgc2::xrpc::new_instance_id()},
                      {"profile", "http.v1"}, {"endpoint", Json{{"kind", "unix"}, {"address", std::move(path)}}}};
    try {
      worker_ = std::thread([this] { work(); });
      readiness_ = std::thread([this] { waitReady(); });
    } catch (...) { close(); throw; }
  }
  ~Impl() { close(); }

  Json describeLocked() const {
    Json capabilities = Json::array({"scene.read", "scene.edit"});
    if (profile_ == "gazebo" || profile_ == "physical") capabilities.push_back("scene.motion");
    capabilities.push_back("scene.save");
    auto facts = facts_;
    facts["profile"] = profile_;
    facts["scene_application"] = profile_ == "gazebo" ? "native" : profile_ == "xsim" ? "unsupported" : "visualization";
    if (profile_ == "gazebo") {
      facts["lidar"] = Json{{"backend", "cpu"}, {"gpu_supported", false}};
    }
    return Json{{"service_ref", reference_}, {"capabilities", capabilities},
                {"limits", Json{{"document_bytes", kMaxDocumentBytes}, {"in_flight", 4}, {"request_results", 256}}},
                {"completion", "The command returns after the world operation completes. Transport loss can leave the result unknown; writes are not replayed."},
                {"service", "xgc2.scene.authoring"}, {"api_version", "v1"},
                {"instance_id", reference_.at("instance_id")}, {"ready", node_ && node_->online() && !stopping_},
                {"facts", facts}};
  }

  void ready(std::shared_ptr<AuthoringNode> node, Json facts) {
    if (!node || !facts.is_object()) throw SceneError("Scene readiness requires a node and object facts");
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    if (node_) throw SceneError("Scene service is already ready");
    if (facts.contains("profile")) {
      const auto profile = facts.at("profile");
      if (profile != "gazebo" && profile != "xsim" && profile != "physical")
        throw SceneError("Scene profile must be gazebo, xsim or physical");
      profile_ = profile.get<std::string>();
    }
    node_ = std::move(node);
    facts_ = std::move(facts);
    changed_.notify_all();
  }

  void stopping() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    changed_.notify_all();
  }

  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = closed_ = true;
    }
    changed_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (readiness_.joinable()) readiness_.join();
  }

  void handle(HttpRequest request, Complete complete, Cancelled cancelled) {
    if (!cancelled) cancelled = [] { return false; };
    const auto path = request.target.substr(0, request.target.find('?'));
    if (request.method == "GET" && path == "/v1/describe") {
      milliseconds wait;
      if (!waitBudget(request.target, wait)) {
        complete(fault(400, "invalid_argument", "the only query is wait_ready_ms between 0 and 30000"));
        return;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      if (node_ || stopping_ || wait.count() == 0) {
        const auto description = describeLocked();
        lock.unlock();
        complete(response(description));
      } else {
        // Like the Tools host, leave time for the IO owner to deliver the
        // current state before the caller's own deadline expires.
        const auto until = std::min(Clock::now() + wait, request.deadline - milliseconds(250));
        waiters_.push_back({until, std::move(complete), std::move(cancelled)});
        changed_.notify_all();
      }
      return;
    }
    if (request.method == "GET" && request.target == "/v1/health") {
      std::shared_ptr<AuthoringNode> node;
      bool stopping;
      { std::lock_guard<std::mutex> lock(mutex_); node = node_; stopping = stopping_; }
      complete(response(Json{{"lifecycle", !stopping && node && node->online() ? "ready" : "stopping"}}));
      return;
    }
    if (!((request.method == "GET" && request.target == "/v1/scene") ||
          (request.method == "POST" && (request.target == "/v1/commands" || request.target == "/v1/commands/result")))) {
      complete(fault(404, "not_found", "unknown scene authoring operation"));
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (!node_ || stopping_) {
      lock.unlock();
      complete(fault(503, "unavailable", "Scene authoring is not ready"));
    } else if (queue_.size() >= 4) {
      lock.unlock();
      complete(fault(503, "resource_exhausted", "Scene work capacity exhausted"));
    } else {
      queue_.push_back({std::move(request), std::move(complete), std::move(cancelled), node_});
      changed_.notify_all();
    }
  }

  Json reference_;

 private:
  struct Waiter { Clock::time_point until; Complete complete; Cancelled cancelled; };
  struct Work { HttpRequest request; Complete complete; Cancelled cancelled; std::shared_ptr<AuthoringNode> node; };
  std::string profile_;
  std::mutex mutex_;
  std::condition_variable changed_;
  bool stopping_ = false, closed_ = false;
  std::shared_ptr<AuthoringNode> node_;
  Json facts_ = Json::object();
  std::deque<Work> queue_;
  std::vector<Waiter> waiters_;
  std::thread worker_, readiness_;

  void waitReady() {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      if (waiters_.empty()) {
        if (closed_) return;
        changed_.wait(lock, [this] { return closed_ || !waiters_.empty(); });
        continue;
      }
      const auto next = std::min_element(waiters_.begin(), waiters_.end(),
          [](const Waiter& a, const Waiter& b) { return a.until < b.until; })->until;
      if (!node_ && !stopping_ && Clock::now() < next) {
        changed_.wait_until(lock, next);
        continue;
      }
      std::vector<Waiter> released;
      for (auto it = waiters_.begin(); it != waiters_.end();) {
        if (node_ || stopping_ || Clock::now() >= it->until) {
          released.push_back(std::move(*it));
          it = waiters_.erase(it);
        } else ++it;
      }
      const auto description = describeLocked();
      lock.unlock();
      for (auto& waiter : released) if (!waiter.cancelled()) waiter.complete(response(description));
      lock.lock();
    }
  }

  HttpResponse execute(Work& work) {
    if (work.cancelled() || Clock::now() >= work.request.deadline)
      return fault(408, "deadline_exceeded", "Scene request was cancelled or expired");
    if (work.request.method == "GET") return response(work.node->envelope());
    if (work.request.body.size() > kMaxDocumentBytes)
      return fault(413, "resource_exhausted", "JSON request limit exceeded");
    if (!work.request.body.empty() && !jsonContentType(work.request.headers))
      return fault(415, "invalid_argument", "application/json required");
    // Use XRPC's strict json_http parser for its depth/duplicate/UTF-8 limits.
    // The domain response keeps its own insertion order for golden bytes.
    xgc2::xrpc::Json parsed;
    Json value;
    if (!work.request.body.empty()) {
      if (!xgc2::xrpc::parse_json(work.request.body, parsed, {kMaxDocumentBytes, 32}))
        return fault(400, "invalid_argument", "invalid JSON request");
      value = parseJson(work.request.body);
    }
    if (work.request.target == "/v1/commands/result") {
      if (!value.is_object() || value.size() != 1 || !value.contains("request_id") || !value.at("request_id").is_string())
        return fault(400, "invalid_argument", "request_id is required");
      const auto result = work.node->requestResult(value.at("request_id").get<std::string>());
      if (!result) return fault(404, "not_found", "Request result is unknown or outside retained history");
      return response(*result);
    }
    if (!value.is_object() || value.contains("requestId"))
      return fault(400, "invalid_argument", "Command identity must come from X-Request-ID");
    if (profile_ == "xsim" && (value.value("operation", Json()) == "play" ||
        value.value("operation", Json()) == "pause" || value.value("operation", Json()) == "reset" ||
        value.value("operation", Json()) == "resync"))
      return fault(501, "unsupported", "Live scene application and motion are unsupported for the xsim profile");
    if (work.cancelled() || Clock::now() >= work.request.deadline)
      return fault(408, "deadline_exceeded", "Scene request was cancelled or expired");
    value["requestId"] = work.request.request_id;
    return response(work.node->commandValue(value));
  }

  void work() {
    for (;;) {
      Work work;
      bool stopping;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock, [this] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return;
        work = std::move(queue_.front());
        queue_.pop_front();
        stopping = stopping_;
      }
      if (work.cancelled()) continue;
      try {
        work.complete(stopping ? fault(503, "unavailable", "Scene authoring is stopping") : execute(work));
      } catch (const std::exception&) {
        work.complete(fault(500, "outcome_unknown", "Scene operation did not produce a known terminal result"));
      }
    }
  }
};

SceneService::SceneService(std::string path, std::string target_id, std::string profile)
    : impl_(new Impl(std::move(path), std::move(target_id), std::move(profile))) {}
SceneService::~SceneService() = default;
const Json& SceneService::reference() const { return impl_->reference_; }
xgc2::xrpc::HttpLimits SceneService::limits() {
  xgc2::xrpc::HttpLimits limits;
  limits.inflight = 4;
  limits.request_bytes = kMaxDocumentBytes;
  limits.response_bytes = kMaxDocumentBytes + 65536;
  limits.request_timeout = std::chrono::seconds(60);
  return limits;
}
void SceneService::ready(std::shared_ptr<AuthoringNode> node, Json facts) { impl_->ready(std::move(node), std::move(facts)); }
void SceneService::stopping() { impl_->stopping(); }
void SceneService::handle(xgc2::xrpc::HttpRequest request, Complete complete, Cancelled cancelled) {
  impl_->handle(std::move(request), std::move(complete), std::move(cancelled));
}
void SceneService::handle(xgc2::xrpc::HttpRequest request, xgc2::xrpc::HttpReply reply) {
  handle(std::move(request), [reply](xgc2::xrpc::HttpResponse result) { reply.complete(std::move(result)); },
         [reply] { return reply.cancelled(); });
}
void SceneService::close() { impl_->close(); }

}  // namespace xgc_scene
