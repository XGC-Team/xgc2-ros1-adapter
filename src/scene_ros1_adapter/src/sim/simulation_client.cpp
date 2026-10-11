#include "xgc_scene/simulation_client.hpp"

#include "xgc_scene/json_io.hpp"
#include "../core/scalars.hpp"

#include <xgc2/xrpc/json_http.hpp>
#include <xgc2/xrpc/unix.hpp>

#include "../peer_backoff.hpp"
#include <regex>
#include <filesystem>
#include <cmath>
#include <atomic>
#include <mutex>
#include <optional>
#include <thread>
#include <tuple>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/inotify.h>
#include <unistd.h>

namespace xgc_scene {
namespace {
class SimulationFault : public std::runtime_error {
 public:
  SimulationFault(const std::string& message, std::string code)
      : std::runtime_error(message), code(std::move(code)) {}
  std::string code;
};
}  // namespace


class SimulationClient::Impl {
 public:
  Impl(const std::string& endpoint, const std::string& target, std::chrono::milliseconds timeout)
      : path_(endpoint), target_(target), timeout_(timeout) {
    if (path_.empty() || path_.front() != '/' || path_.find('\0') != std::string::npos)
      throw SceneError("Scene parameter ~simulation_endpoint must be an absolute Unix socket path");
    if (!std::regex_match(target_, std::regex("[A-Za-z0-9._:-]{1,128}")))
      throw SceneError("Invalid simulation local target_id");
    if (timeout.count() <= 0) throw SceneError("Simulation timeout must be positive");
    wake_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (wake_ < 0) throw SceneError("Cannot create simulation wake descriptor");
  }
  ~Impl() { close(); if (wake_ >= 0) ::close(wake_); }
  void update(const Json& document, const std::string& epoch, std::uint64_t revision, const Json& motion) {
    auto desired = std::make_shared<Desired>(Desired{document, epoch, revision, motion, 0});
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return;
      desired->serial = ++serial_;
      desired_ = std::move(desired);
      if (!worker_.joinable()) worker_ = std::thread([this] { work(); });
    }
    wake();
  }
  std::optional<Json> takeReport() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(report_);
    report_.reset();
    return result;
  }
  void close() {
    std::shared_ptr<xgc2::xrpc::HttpClient> active;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
      active = active_;
    }
    if (active) active->close();
    wake();
    std::lock_guard<std::mutex> joining(join_mutex_);
    if (worker_.joinable()) worker_.join();
  }
 private:
  struct Desired { Json document; std::string epoch; std::uint64_t revision; Json motion; std::uint64_t serial; };
  std::string path_, target_;
  std::chrono::milliseconds timeout_;
  std::mutex mutex_, join_mutex_;
  bool closed_ = false;
  std::shared_ptr<const Desired> desired_;
  std::optional<Json> report_;
  std::shared_ptr<xgc2::xrpc::HttpClient> active_, client_;
  std::thread worker_;
  std::uint64_t serial_ = 0, attempts_ = 0;
  int wake_ = -1;
  void wake() { std::uint64_t one = 1; const auto ignored = ::write(wake_, &one, sizeof one); (void)ignored; }
  bool stopped() { std::lock_guard<std::mutex> lock(mutex_); return closed_; }
  void emit(const Desired& desired, bool applied, const std::string& instance, const std::string& message,
            const Json& native = Json::object()) {
    Json value{{"consumer", "gazebo"}, {"epoch", desired.epoch}, {"revision", desired.revision},
      {"applied", applied}, {"operational", applied}, {"capability", applied ? "ok" : ""},
      {"instance_id", instance}, {"message", message}, {"connect_attempts", attempts_},
      {"playing", desired.motion.value("playing", false)}};
    for (const auto* key : {"scene_time", "playing"}) if (native.contains(key)) value[key] = native.at(key);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!closed_) report_ = std::move(value);
  }
  std::shared_ptr<xgc2::xrpc::HttpClient> connection(const std::string& instance) {
    xgc2::xrpc::HttpLimits limits;
    limits.connections = limits.inflight = 1;
    limits.request_bytes = limits.response_bytes = kMaxDocumentBytes;
    limits.request_timeout = std::max(timeout_, std::chrono::milliseconds(31000));
    auto client = std::make_shared<xgc2::xrpc::HttpClient>(path_, limits, instance);
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = client;
    if (closed_) client->close();
    return client;
  }
  Json call(const std::string& route, const Json& value, const std::string& method,
            const std::string& request_id, xgc2::xrpc::Clock::time_point deadline) {
    xgc2::xrpc::HttpRequest request;
    request.method = method;
    request.target = route;
    request.request_id = request_id;
    request.body = dumpJson(value);
    request.headers = {{"Content-Type", "application/json"}};
    const auto response = client_->call(std::move(request), deadline);
    xgc2::xrpc::Json parsed;
    if (!xgc2::xrpc::parse_json(response.body, parsed, {kMaxDocumentBytes, 32}))
      throw SceneError("Simulation returned an invalid JSON response");
    const auto result = parseJson(response.body);
    if (!result.is_object()) throw SceneError("Simulation returned a non-object operation");
    if (response.status >= 400) {
      const auto error = result.value("error", Json::object());
      if (!error.is_object()) throw SceneError("Simulation returned an invalid fault");
      throw SimulationFault(error.value("message", std::string("Simulation request failed")),
                            error.value("code", std::string("internal")));
    }
    return result;
  }

  Json operation(const std::string& route, const Json& value, const std::string& method,
                 std::string request_id) {
    if (request_id.empty()) request_id = xgc2::xrpc::new_instance_id();
    const auto deadline = xgc2::xrpc::Clock::now() + timeout_;
    Json payload = value;
    payload["operation_timeout_ms"] = timeout_.count();
    try {
      auto result = call(route, payload, method, request_id, deadline);
      if (result.value("id", Json()) != request_id)
        throw SceneError("Simulation returned a different operation identity");
      const auto state = result.value("state", Json());
      if (state == "accepted" || state == "running") {
        // One notification-driven wait, with a distinct transport request ID.
        // There is no entity probe and neither submission nor wait is replayed.
        result = call("/v1/operations/" + request_id + "/wait", Json::object(), "POST",
                      xgc2::xrpc::new_instance_id(), deadline);
      }
      if (result.value("id", Json()) != request_id || result.value("state", Json()) != "succeeded") {
        const auto error = result.value("error", Json());
        const bool has_error = error.is_string() ? !error.get_ref<const std::string&>().empty() :
                               error.is_boolean() ? error.get<bool>() :
                               error.is_number() ? error.get<double>() != 0 : !error.is_null() && !error.empty();
        const auto diagnostic = has_error ? error : result.value("state", Json());
        throw SceneError("Simulation operation " + request_id + " did not complete: " +
                         (diagnostic.is_string() ? diagnostic.get<std::string>() : detail::pythonRepr(diagnostic)));
      }
      return result.value("result", Json::object());
    } catch (const SimulationFault& error) {
      throw SceneError("Simulation operation " + request_id + ": " + error.what() + " (" + error.code + ")");
    }
  }


  static bool socketMissing(const std::filesystem::path& path) {
    std::error_code error;
    return !std::filesystem::exists(path, error);
  }
  void work() {
    using namespace std::chrono;
    using Key = std::tuple<std::string, std::string, std::uint64_t>;
    PeerBackoff backoff;
    std::optional<Key> attempted;
    std::string instance, peer_message = "World host is waiting for discovery";
    bool connected = false, applied = false, missing = false;
    std::uint64_t delivered = 0, unavailable = 0;
    auto retry = xgc2::xrpc::Clock::now();
    const std::filesystem::path path(path_);
    const auto name = path.filename().string();
    // One directory watch also observes a clean peer disappearance/restart.
    int watch = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (watch >= 0 && inotify_add_watch(watch, path.parent_path().c_str(), IN_CREATE | IN_DELETE | IN_MOVED_FROM | IN_MOVED_TO) < 0) {
      ::close(watch); watch = -1;
    }
    while (!stopped()) {
      std::shared_ptr<const Desired> desired;
      { std::lock_guard<std::mutex> lock(mutex_); desired = desired_; }
      if (!desired) break;
      if (!connected && unavailable != desired->serial) {
        emit(*desired, false, instance, peer_message);
        unavailable = desired->serial;
      }
      if ((!connected && !missing && xgc2::xrpc::Clock::now() >= retry) ||
          (connected && delivered != desired->serial)) {
        try {
          if (!connected) {
            ++attempts_;
            client_ = connection("");
            xgc2::xrpc::HttpRequest request;
            request.method = "GET";
            request.target = "/v1/describe?wait_ready_ms=30000";
            request.request_id = xgc2::xrpc::new_instance_id();
            const auto response = client_->call(std::move(request), xgc2::xrpc::Clock::now() + seconds(31));
            const auto description = parseJson(response.body);
            if (response.status != 200 || description.value("service", Json()) != "xgc2.simulation" ||
                (description.value("api_version", Json()) != "v1" && description.value("api_version", Json()) != "1") ||
                !description.value("ready", false)) throw SceneError("World host is not ready");
            const auto discovered = description.at("instance_id").get<std::string>();
            if (!std::regex_match(discovered, std::regex("[A-Za-z0-9._:-]{1,128}")))
              throw SceneError("Invalid discovered world instance");
            if (description.contains("service_ref") && description.at("service_ref").value("target_id", Json()) != target_)
              throw SceneError("Discovered world target differs from the local target");
            instance = discovered;
            client_ = connection(instance);
            connected = true;
            backoff.success();
          }
          // Capture the latest state again after a held discovery returns.
          { std::lock_guard<std::mutex> lock(mutex_); desired = desired_; }
          const Key key{instance, desired->epoch, desired->revision};
          Json native = Json::object();
          if (!attempted || *attempted != key) {
            attempted = key;
            applied = false;
            native = operation("/v1/extensions/scene/apply",
                Json{{"document", desired->document}, {"epoch", desired->epoch}, {"revision", desired->revision}},
                "POST", xgc2::xrpc::new_instance_id());
            if (native.value("epoch", Json()) != desired->epoch || native.value("revision", Json()) != desired->revision)
              throw SceneError("Simulation did not commit the requested scene revision");
            applied = true;
          }
          if (applied) {
            native = operation("/v1/extensions/scene/motion",
                Json{{"operation", desired->motion.value("operation", std::string()) == "reset" ? "reset" : desired->motion.value("playing", false) ? "play" : "pause"},
                     {"epoch", desired->epoch}, {"revision", desired->revision}}, "POST", xgc2::xrpc::new_instance_id());
            if (!native.is_object() || (native.contains("epoch") && native.at("epoch") != desired->epoch) ||
                (native.contains("revision") && native.at("revision") != desired->revision) ||
                (native.contains("scene_time") && (!native.at("scene_time").is_number() || !std::isfinite(native.at("scene_time").get<double>()))) ||
                (native.contains("playing") && native.at("playing") != desired->motion.value("playing", false)))
              throw SceneError("Simulation returned mismatched motion state");
            emit(*desired, true, instance, "Native scene operation completed", native);
          }
          delivered = desired->serial;
        } catch (const xgc2::xrpc::HttpCallError& error) {
          if (error.delivery == xgc2::xrpc::Delivery::NotSent && !applied) attempted.reset();
          connected = false;
          missing = socketMissing(path);
          peer_message = std::string(error.what()) + " (" + xgc2::xrpc::delivery_name(error.delivery) + ")";
          emit(*desired, false, instance, peer_message);
          unavailable = desired->serial;
          retry = xgc2::xrpc::Clock::now() + backoff.failed("world_host", error.what());
        } catch (const std::exception& error) {
          emit(*desired, false, instance, error.what());
          if (connected) delivered = desired->serial;  // A native refusal is not a retry loop.
          else {
            peer_message = error.what();
            unavailable = desired->serial;
            retry = xgc2::xrpc::Clock::now() + backoff.failed("world_host", error.what());
          }
        }
        continue;
      }
      const auto remaining = duration_cast<milliseconds>(retry - xgc2::xrpc::Clock::now()).count();
      int wait = (!connected && !missing) ? static_cast<int>(std::max<std::int64_t>(1, remaining)) : -1;
      if (missing && watch < 0) { missing = false; wait = static_cast<int>(std::max<std::int64_t>(1, remaining)); }
      pollfd descriptors[] = {{wake_, POLLIN, 0}, {watch, POLLIN, 0}};
      if (::poll(descriptors, 2, wait) < 0) continue;
      if (descriptors[0].revents & POLLIN) { std::uint64_t value; while (::read(wake_, &value, sizeof value) > 0) {} }
      if (descriptors[1].revents & POLLIN) {
        alignas(inotify_event) char events[4096];
        ssize_t bytes;
        while ((bytes = ::read(watch, events, sizeof events)) > 0) {
          for (ssize_t offset = 0; offset < bytes;) {
            const auto* event = reinterpret_cast<const inotify_event*>(events + offset);
            if (event->len && name == event->name) {
              if (connected) retry = xgc2::xrpc::Clock::now() + backoff.failed("world_host", "Socket disappeared or changed");
              connected = false;
              missing = socketMissing(path);
              peer_message = "World host socket changed; waiting for discovery";
              emit(*desired, false, instance, peer_message);
              unavailable = desired->serial;
              if (!missing) retry = xgc2::xrpc::Clock::now();
            }
            offset += sizeof(inotify_event) + event->len;
          }
        }
      }
    }
    if (watch >= 0) ::close(watch);
  }
};
SimulationClient::SimulationClient(const std::string& endpoint, const std::string& target, std::chrono::milliseconds timeout)
    : impl_(new Impl(endpoint, target, timeout)) {}
SimulationClient::~SimulationClient() = default;
void SimulationClient::update(const Json& document, const std::string& epoch, std::uint64_t revision, const Json& motion) {
  impl_->update(document, epoch, revision, motion);
}
std::optional<Json> SimulationClient::takeReport() { return impl_->takeReport(); }
void SimulationClient::close() { impl_->close(); }
}  // namespace xgc_scene
