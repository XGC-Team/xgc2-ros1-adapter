#include "xgc_scene/node.hpp"

#include "xgc_scene/generation.hpp"
#include "xgc_scene/json_io.hpp"
#include <cmath>

namespace xgc_scene {
namespace {
std::optional<std::filesystem::path> pathOption(const std::filesystem::path& value) {
  return value.empty() ? std::optional<std::filesystem::path>() : value;
}
}  // namespace

SceneParameters SceneParameters::parse(const Json& values) {
  if (!values.is_object()) throw SceneError("Scene private parameters must be an object");
  SceneParameters result;
  const auto string = [&](const char* key, bool required = false, const std::string& fallback = "") {
    const auto found = values.find(key);
    if (found == values.end()) {
      if (required) throw SceneError(std::string("Scene parameter ~") + key + " is required");
      return fallback;
    }
    if (!found->is_string()) throw SceneError(std::string("Scene parameter ~") + key + " must be a string");
    if (required && found->get_ref<const std::string&>().empty())
      throw SceneError(std::string("Scene parameter ~") + key + " is required");
    return found->get<std::string>();
  };
  result.scene_file = string("scene_file", true);
  result.working_file = string("working_file");
  result.save_directory = string("save_directory");
  result.target_id = string("target_id", true);
  result.xrpc_socket = string("xrpc_socket", true);
  result.profile = string("profile", false, "gazebo");
  if (result.profile != "gazebo" && result.profile != "xsim" && result.profile != "physical")
    throw SceneError("Scene profile must be gazebo, xsim or physical");
  if (values.contains("frozen")) {
    if (!values.at("frozen").is_boolean()) throw SceneError("Scene parameter ~frozen must be a boolean");
    result.frozen = values.at("frozen").get<bool>();
  }
  result.generation_parameters = parseJson(string("generation_parameters_json", false, "{}"));
  if (!result.generation_parameters.is_object()) throw SceneError("Generation parameters must be an object");
  if (result.profile == "gazebo")
    result.simulation_service_ref = parseJson(string("simulation_service_ref_json", true));
  return result;
}

SceneNode::SceneNode(const SceneParameters& parameters, const std::string& name_space,
                     RosOutputs& outputs, Clock clock, std::shared_ptr<Simulation> simulation)
    : parameters_(parameters), outputs_(outputs), clock_(std::move(clock)), simulation_(std::move(simulation)),
      projection_(name_space) {
  const auto resolved = resolve(parameters_.scene_file, parameters_.working_file, parameters_.generation_parameters);
  parameters_.scene_file = resolved.second;
  initialize(resolved.first.first, resolved.first.second);
}

SceneNode::SceneNode(const Json& initial, const SceneParameters& parameters, const std::string& name_space,
                     RosOutputs& outputs, Clock clock, std::shared_ptr<Simulation> simulation,
                     std::optional<std::string> source_digest)
    : parameters_(parameters), outputs_(outputs), clock_(std::move(clock)), simulation_(std::move(simulation)),
      projection_(name_space) {
  initialize(initial, source_digest);
}

void SceneNode::initialize(const Json& initial, const std::optional<std::string>& source_digest) {
  if (!clock_) throw SceneError("Scene node requires a clock");
  if (parameters_.profile != "gazebo" && parameters_.profile != "xsim" && parameters_.profile != "physical")
    throw SceneError("Scene profile must be gazebo, xsim or physical");
  if (parameters_.profile == "gazebo" && !simulation_)
    throw SceneError("An explicit simulation-v1 ServiceRef is required");
  if (parameters_.profile != "gazebo" && simulation_)
    throw SceneError("Native scene application requires the gazebo profile");
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  store_.reset(new SceneStore(initial, pathOption(parameters_.scene_file), pathOption(parameters_.save_directory),
      [this](const Json& document, const std::string& epoch, std::uint64_t revision) { applyScene(document, epoch, revision); },
      clock_, source_digest, parameters_.frozen, pathOption(parameters_.working_file),
      [this](const std::string& operation, const std::string& epoch, std::uint64_t revision) { applyMotion(operation, epoch, revision); }));
  queueSimulation();
  receiveSimulation();
  publishDefinition();
}

SceneNode::~SceneNode() { shutdown(); }
bool SceneNode::online() const { return online_.load(); }

void SceneNode::applyScene(const Json& document, const std::string& epoch, std::uint64_t revision) {
  if (!simulation_) return;
  outputs_.application(epoch, revision, false);
  applied_snapshot_ = projection_.snapshot(document, epoch, revision, clock_());
  gazeboFailure(epoch, revision, "World host application pending");
}
void SceneNode::applyMotion(const std::string& operation, const std::string&, std::uint64_t) {
  if (simulation_) motion_operation_ = operation;
}
void SceneNode::queueSimulation() {
  if (!simulation_) return;
  const auto value = store_->envelope();
  gazeboFailure(value.at("epoch").get<std::string>(), value.at("revision").get<std::uint64_t>(), "World host application pending");
  Json motion{{"playing", value.at("playing")}, {"scene_time", store_->sceneTime()}, {"operation", motion_operation_}};
  simulation_->update(value.at("document"), value.at("epoch").get<std::string>(),
                      value.at("revision").get<std::uint64_t>(), motion);
  motion_operation_.clear();
}
void SceneNode::receiveSimulation() {
  if (!simulation_) return;
  auto report = simulation_->takeReport();
  if (!report) return;
  const auto status = store_->status();
  if (report->value("epoch", Json()) != status.at("epoch") || report->value("revision", Json()) != status.at("revision")) return;
  // The native wire can replay play/pause/reset, but cannot seek elapsed time.
  // A peer that reports a different phase must never authorize lidar truth.
  if (report->value("applied", false) && report->contains("playing") && report->at("playing") != status.at("playing")) {
    (*report)["applied"] = false;
    (*report)["message"] = "World host motion state is pending";
  }
  if (report->value("applied", false) && report->contains("scene_time") &&
      std::abs(report->at("scene_time").get<double>() - store_->sceneTime()) > 0.1) {
    (*report)["applied"] = false;
    (*report)["operational"] = false;
    (*report)["message"] = "World host scene time differs; native motion seek is unavailable";
  }
  if (!report->value("applied", false)) { (*report)["capability"] = ""; (*report)["operational"] = false; }
  (*report)["header_stamp"] = clock_();
  native_report_ = std::move(*report);
  outputs_.application(status.at("epoch").get<std::string>(), status.at("revision").get<std::uint64_t>(),
                       native_report_->value("applied", false));
  publishDocument();
}
Json SceneNode::peerFacts() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (parameters_.profile != "gazebo") return Json::object();
  const auto status = store_->status();
  auto result = applicationView(status.at("epoch").get<std::string>(), status.at("revision").get<std::uint64_t>());
  result["world_host"] = result.at("synchronized") == true ? "connected" : "waiting";
  return result;
}
void SceneNode::gazeboFailure(const std::string& epoch, std::uint64_t revision, const std::string& message) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (parameters_.profile != "gazebo") return;
  native_report_ = Json{{"consumer", "gazebo"}, {"epoch", epoch}, {"revision", revision},
                        {"applied", false}, {"operational", false}, {"capability", ""},
                        {"message", message}, {"header_stamp", clock_()}};
  outputs_.application(epoch, revision, false);
}

Json SceneNode::applicationView(const std::string& epoch, std::uint64_t revision) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const bool synchronized = !native_report_ || (native_report_->at("applied") == true &&
      native_report_->at("epoch") == epoch && native_report_->at("revision") == revision);
  return Json{{"consumers", native_report_ ? Json::array({*native_report_}) : Json::array()},
              {"synchronized", synchronized}, {"syncRetryable", !synchronized}};
}
void SceneNode::addPublicView(Json& result) {
  const auto public_view = applicationView(result.at("epoch").get<std::string>(), result.at("revision").get<std::uint64_t>());
  result["consumers"] = public_view.at("consumers");
  result["online"] = online();
  result["synchronized"] = public_view.at("synchronized");
  result["syncRetryable"] = public_view.at("syncRetryable");
}
Json SceneNode::envelope() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto result = store_->envelope();
  addPublicView(result);
  return result;
}

Json SceneNode::commandValue(const Json& command) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto previous = store_->status().at("revision");
  const auto rid = command.is_object() ? command.value("requestId", Json()) : Json();
  const auto id = rid.is_string() ? rid.get<std::string>() : std::string();
  const auto retained = store_->requestResult(id);
  auto result = store_->command(command);
  const auto cached = public_receipts_.find(id);
  if (retained && result == *retained && cached != public_receipts_.end()) {
    for (auto it = cached->second.begin(); it != cached->second.end(); ++it) result[it.key()] = it.value();
    return result;
  }
  if (store_->status().at("revision") != previous || !motion_operation_.empty()) {
    queueSimulation();
    receiveSimulation();
  }
  if (store_->status().at("revision") != previous) publishDefinition();
  else if (!command.is_object() || command.value("operation", Json()) != "get") {
    tick();
    publishDocument();
  }
  const auto receipt = store_->requestResult(id);
  const bool saved = receipt && *receipt == result;
  addPublicView(result);
  if (saved) {
    public_receipts_[id] = Json{{"consumers", result.at("consumers")}, {"online", result.at("online")},
                                {"synchronized", result.at("synchronized")}, {"syncRetryable", result.at("syncRetryable")}};
    if (public_receipts_.size() > 256) {
      for (auto it = public_receipts_.begin(); it != public_receipts_.end();) {
        if (!store_->requestResult(it->first)) it = public_receipts_.erase(it);
        else ++it;
      }
    }
  }
  return result;
}

std::optional<Json> SceneNode::requestResult(const std::string& request_id) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto receipt = store_->requestResult(request_id);
  if (!receipt) return {};
  const auto cached = public_receipts_.find(request_id);
  if (cached != public_receipts_.end()) {
    for (auto it = cached->second.begin(); it != cached->second.end(); ++it) (*receipt)[it.key()] = it.value();
  }
  return receipt;
}

bool SceneNode::publishDocument() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto status = store_->status();
  const auto public_view = applicationView(status.at("epoch").get<std::string>(), status.at("revision").get<std::uint64_t>());
  const auto view = Json::array({status, public_view, online()});
  if (published_view_ && *published_view_ == view) return false;
  status["sceneTime"] = store_->sceneTime();
  status["document"] = definition_;
  status["consumers"] = public_view.at("consumers");
  status["online"] = online();
  status["synchronized"] = public_view.at("synchronized");
  status["syncRetryable"] = public_view.at("syncRetryable");
  outputs_.document(encoder_.dumps(status));
  published_view_ = view;
  return true;
}

void SceneNode::publishDefinition() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto envelope = store_->envelope();
  const auto epoch = envelope.at("epoch").get<std::string>();
  const auto revision = envelope.at("revision").get<std::uint64_t>();
  if (parameters_.profile == "gazebo")
    outputs_.application(epoch, revision, applicationView(epoch, revision).at("synchronized").get<bool>());
  definition_ = envelope.at("document");
  xgc2_geometry_msgs::SceneSnapshot message;
  if (applied_snapshot_ && applied_snapshot_->epoch == epoch && applied_snapshot_->revision == revision)
    message = *applied_snapshot_;
  else message = projection_.snapshot(definition_, epoch, revision, clock_());
  projection_.definition(definition_, epoch, revision, message, outputs_);
  tick();
  publishDocument();
}

void SceneNode::tick() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!online()) return;
  receiveSimulation();
  const auto status = store_->status();
  projection_.tick(clock_(), store_->sceneTime(), status.at("playing").get<bool>(), outputs_);
}
void SceneNode::cancelSimulation() { if (simulation_) simulation_->close(); }
void SceneNode::shutdown() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!online_.exchange(false)) return;
  if (store_) publishDocument();
  if (simulation_) simulation_->close();
}
SceneProjection::Counters SceneNode::counters() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return projection_.counters();
}

}  // namespace xgc_scene
