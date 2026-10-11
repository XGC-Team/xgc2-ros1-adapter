#include "xgc_scene/lidar_ros/parameters.hpp"

#include "xgc_scene/json_io.hpp"
#include <ros/node_handle.h>

#include <cmath>
#include <limits>

namespace xgc_scene::lidar_ros {
namespace {
[[noreturn]] void error(const std::string& key, const std::string& message) {
  throw SceneError("Lidar parameter ~" + key + ": " + message);
}
std::string text(const Json& values, const char* key, const char* fallback) {
  const auto found = values.find(key);
  if (found == values.end()) return fallback;
  if (!found->is_string()) error(key, "must be a string");
  return found->get<std::string>();
}
}  // namespace

Parameters Parameters::parse(const Json& values, const std::string& profile) {
  Parameters result;
  result.socket = text(values, "lidar_xrpc_socket", "");
  if (result.socket.empty() || result.socket.front() != '/' || result.socket.find('\0') != std::string::npos)
    error("lidar_xrpc_socket", "an absolute Unix socket path is required");
  if (values.contains("lidar_enabled")) {
    if (!values.at("lidar_enabled").is_boolean()) error("lidar_enabled", "must be a boolean");
    result.enabled = values.at("lidar_enabled").get<bool>();
  }
  xgc2_math::scene::SensorConfig defaults;
  try {
    const auto object = parseJson(text(values, "lidar_defaults_json", "{}"));
    if (!object.is_object()) throw SceneError("must be an object");
    for (auto it = object.begin(); it != object.end(); ++it) {
      const auto& key = it.key();
      const auto& value = it.value();
      const auto number = [&] {
        if (!value.is_number() || !std::isfinite(value.get<double>()))
          throw SceneError(key + " must be a finite number");
        return value.get<double>();
      };
      const auto integer = [&] {
        if (!value.is_number_integer() || value < std::numeric_limits<int>::min() ||
            value > std::numeric_limits<int>::max()) throw SceneError(key + " must be an integer");
        return value.get<int>();
      };
      const auto boolean = [&] {
        if (!value.is_boolean()) throw SceneError(key + " must be a boolean");
        return value.get<bool>();
      };
      if (key == "range") defaults.range = number();
      else if (key == "min_range") defaults.min_range = number();
      else if (key == "h_fov_deg") defaults.h_fov_deg = number();
      else if (key == "v_fov_deg") defaults.v_fov_deg = number();
      else if (key == "h_res") defaults.h_res = integer();
      else if (key == "v_res") defaults.v_res = integer();
      else if (key == "surface_spacing") defaults.surface_spacing = number();
      else if (key == "vertical_slab_tan") defaults.vertical_slab_tan = number();
      else if (key == "fx") defaults.fx = number();
      else if (key == "fy") defaults.fy = number();
      else if (key == "cx") defaults.cx = number();
      else if (key == "cy") defaults.cy = number();
      else if (key == "noise_std") defaults.noise_std = number();
      else if (key == "seed") {
        if (!value.is_number_integer() || value < 0 || value > std::numeric_limits<unsigned>::max())
          throw SceneError("seed must be an unsigned integer");
        defaults.seed = value.get<unsigned>();
      } else if (key == "yaw_only") result.options.yaw_only = boolean();
      else if (key == "pose_timeout") result.options.pose_timeout = number();
      else if (key == "worker_threads") result.options.worker_threads = integer();
      else if (key == "vehicle_bodies") result.options.vehicle_bodies = boolean();
      else if (key == "vehicle_body_radius") result.options.vehicle_body_radius = number();
      else throw SceneError("unknown key '" + key + "'");
    }
    if (!(defaults.surface_spacing > 0)) throw SceneError("surface_spacing must be > 0");
    xgc2_math::scene::resolveSensorConfig(&defaults);
    if (!(result.options.pose_timeout > 0)) throw SceneError("pose_timeout must be > 0");
    if (result.options.vehicle_bodies && !(result.options.vehicle_body_radius > 0))
      throw SceneError("vehicle_body_radius must be > 0");
  } catch (const std::exception& e) { error("lidar_defaults_json", e.what()); }
  const auto manifest = text(values, "lidar_fleet_json", "");
  if (!manifest.empty()) {
    try { result.manifest = lidar::parseManifest(manifest, defaults); }
    catch (const std::exception& e) { error("lidar_fleet_json", e.what()); }
  }
  result.have_fleet = profile == "gazebo" && !manifest.empty();
  if (!result.have_fleet) result.manifest = {};
  return result;
}

Parameters Parameters::read(ros::NodeHandle& private_node, const std::string& profile) {
  Json values = Json::object();
  for (const auto* key : {"lidar_xrpc_socket", "lidar_fleet_json", "lidar_defaults_json"}) {
    std::string value;
    if (private_node.getParam(key, value)) values[key] = value;
    else if (private_node.hasParam(key)) error(key, "must be a string");
  }
  bool enabled;
  if (private_node.getParam("lidar_enabled", enabled)) values["lidar_enabled"] = enabled;
  else if (private_node.hasParam("lidar_enabled")) error("lidar_enabled", "must be a boolean");
  return parse(values, profile);
}

Json Parameters::facts(const std::string& profile) const {
  return {{"profile", profile}, {"sensors", manifest.robots.size()},
          {"backend", profile == "gazebo" ? "cpu" : "none"}};
}
}  // namespace xgc_scene::lidar_ros
