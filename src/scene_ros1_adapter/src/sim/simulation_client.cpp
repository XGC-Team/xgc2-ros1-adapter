#include "xgc_scene/simulation_client.hpp"

#include "xgc_scene/json_io.hpp"
#include "../core/scalars.hpp"

#include <xgc2/xrpc/json_http.hpp>
#include <xgc2/xrpc/unix.hpp>

#include <regex>

namespace xgc_scene {
namespace {
class SimulationFault : public std::runtime_error {
 public:
  SimulationFault(const std::string& message, std::string code)
      : std::runtime_error(message), code(std::move(code)) {}
  std::string code;
};
std::string socketPath(const Json& reference, const std::string& local_target) {
  if (!reference.is_object() || reference.value("service", Json()) != "xgc2.simulation" ||
      reference.value("api_version", Json()) != "v1")
    throw SceneError("An explicit simulation-v1 ServiceRef is required");
  const std::regex identity("[A-Za-z0-9._:-]{1,128}");
  for (const auto& key : {"target_id", "instance_id"}) {
    if (!reference.contains(key) || !reference.at(key).is_string() ||
        !std::regex_match(reference.at(key).get_ref<const std::string&>(), identity))
      throw SceneError(std::string("Invalid simulation ServiceRef ") + key);
  }
  if (reference.at("target_id") != local_target)
    throw SceneError("Simulation Unix endpoint must belong to the local target");
  if (reference.value("profile", Json()) != "http.v1" || !reference.contains("endpoint") ||
      !reference.at("endpoint").is_object() || reference.at("endpoint").value("kind", Json()) != "unix" ||
      !reference.at("endpoint").contains("address") || !reference.at("endpoint").at("address").is_string())
    throw SceneError("Simulation requires an explicit http.v1 Unix endpoint");
  const auto path = reference.at("endpoint").at("address").get<std::string>();
  if (path.empty() || path.front() != '/' || path.find('\0') != std::string::npos)
    throw SceneError("Simulation Unix endpoint must be an absolute path");
  return path;
}
}  // namespace

SimulationClient::SimulationClient(const Json& reference, const std::string& local_target,
                                   std::chrono::milliseconds timeout) : timeout_(timeout) {
  const auto path = socketPath(reference, local_target);
  if (timeout.count() <= 0) throw SceneError("Simulation timeout must be positive");
  xgc2::xrpc::HttpLimits limits;
  limits.request_bytes = limits.response_bytes = kMaxDocumentBytes;
  limits.request_timeout = timeout;
  client_.reset(new xgc2::xrpc::HttpClient(path, limits, reference.at("instance_id").get<std::string>()));
}
SimulationClient::~SimulationClient() { close(); }

Json SimulationClient::call(const std::string& route, const Json& value, const std::string& method,
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

Json SimulationClient::operation(const std::string& route, const Json& value, const std::string& method,
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
  } catch (const xgc2::xrpc::HttpCallError& error) {
    throw SceneError("Simulation operation " + request_id + ": " + error.what() + " (" +
                     xgc2::xrpc::delivery_name(error.delivery) + ")");
  } catch (const SimulationFault& error) {
    throw SceneError("Simulation operation " + request_id + ": " + error.what() + " (" + error.code + ")");
  }
}

Json SimulationClient::apply(const Json& document, const std::string& epoch, std::uint64_t revision) {
  const auto result = operation("/v1/extensions/scene/apply",
                                Json{{"document", document}, {"epoch", epoch}, {"revision", revision}});
  if (!result.is_object() || result.value("epoch", Json()) != epoch || result.value("revision", Json()) != revision)
    throw SceneError("Simulation did not commit the requested scene revision");
  return result;
}
Json SimulationClient::motion(const std::string& operation_name, const std::string& epoch, std::uint64_t revision) {
  return operation("/v1/extensions/scene/motion",
                   Json{{"operation", operation_name}, {"epoch", epoch}, {"revision", revision}});
}
void SimulationClient::close() { if (client_) client_->close(); }

}  // namespace xgc_scene
