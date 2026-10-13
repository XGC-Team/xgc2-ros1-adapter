#include "xgc_scene/lidar/roster.hpp"

#include <cmath>
#include <regex>
#include <utility>

namespace xgc_scene::lidar {

SessionInput parseSessionInput(std::string_view text) {
  if (text.size() > kMaxDocumentBytes)
    throw SceneError("frozen Experiment input exceeds 8 MiB");
  Json document;
  try {
    document = Json::parse(text.begin(), text.end());
  } catch (const Json::exception &) {
    throw SceneError(
        "invalid session input JSON (runMode, robots, visualizationTopics)");
  }
  if (!document.is_object())
    throw SceneError("session input must be an object with runMode, robots and "
                     "visualizationTopics");
  if (!document.contains("runMode") || !document["runMode"].is_string())
    throw SceneError("runMode must be simulation, hybrid or physical");
  const auto run_mode = document["runMode"].get<std::string>();
  if (run_mode != "simulation" && run_mode != "hybrid" &&
      run_mode != "physical")
    throw SceneError("runMode must be simulation, hybrid or physical");
  if (!document.contains("robots") || !document["robots"].is_array())
    throw SceneError("robots must be an array of objects");
  for (const auto &robot : document["robots"])
    if (!robot.is_object())
      throw SceneError("robots must be an array of objects");
  if (!document.contains("visualizationTopics") ||
      !document["visualizationTopics"].is_array())
    throw SceneError("visualizationTopics must be an array");
  return {run_mode, document["robots"], document["visualizationTopics"],
          std::move(document)};
}

Json buildFleetJson(const SessionInput &input, bool point_cloud) {
  if (input.robots.size() > kMaxFleetMembers)
    throw SceneError("world lidar supports at most 512 robots");
  Json fleet = {{"schemaVersion", 1}, {"robots", Json::array()}};
  if (input.run_mode == "physical")
    return fleet;

  const std::regex name_pattern(
      "/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*");
  for (const auto &robot : input.robots) {
    try {
      const auto binding = robot.value("binding", Json::object());
      if (input.run_mode == "hybrid" &&
          binding.value("hybridSource", std::string{}) != "simulation")
        continue;
      const auto sensors = binding.value("simulationSensors", Json::object());
      auto lidar = sensors.value("simpleLidar", Json(false));
      if (lidar.is_boolean())
        lidar = {{"enabled", lidar}};
      const auto mode = lidar.value("mode", std::string{});
      const auto preset = lidar.value("preset", std::string{});
      const bool publish_beams = lidar.value("publishBeams", false);
      if (!lidar.value("enabled", false) ||
          !(point_cloud || mode == "penetrating" || mode == "depth_frustum" || publish_beams ||
            (!preset.empty() && mode != "raycast")))
        continue;


      const auto authored_namespace = robot.at("namespace").get<std::string>();
      const auto start = authored_namespace.find_first_not_of('/');
      const auto name_space =
          "/" + (start == std::string::npos ? std::string{}
                                            : authored_namespace.substr(start));
      if (name_space.size() > 160 ||
          !std::regex_match(name_space, name_pattern))
        throw SceneError("invalid lidar namespace");
      Json sensor = {{"namespace", name_space},
                     {"backend", lidar.value("acceleration", std::string("auto")).empty() ? "auto" : lidar.value("acceleration", std::string("auto"))},
                     {"mode", "raycast"},
                     {"rateHz", 10},
                     {"rangeMeters", 20},
                     {"hFovDeg", 360},
                     {"vFovDeg", 30},
                     {"hRes", 360},
                     {"vRes", 32},
                     {"surfaceSpacing", 0.1},
                     {"keepBuried", false},
                     {"headingCrop", false},
                     {"headingCosMin", 0},
                     {"verticalSlabTan", 0},
                     {"publishBeams", publish_beams}};
      if (!preset.empty()) {
        sensor["mode"] = "penetrating";
        sensor["vFovDeg"] = 180;
        sensor["keepBuried"] = true;
        sensor["rangeMeters"] = 5;
        if (preset == "bridge_equivalent") {
          sensor["rangeMeters"] = 8;
        } else {
          sensor["headingCrop"] = true;
          sensor["verticalSlabTan"] = 1 / std::sqrt(3.0);
          if (preset == "zju_cpu_crop")
            sensor["headingCosMin"] = 0.5;
        }
      }
      if (!mode.empty())
        sensor["mode"] = mode;
      if (sensor["mode"] == "depth_frustum") {
        sensor["hFovDeg"] = 90;
        sensor["vFovDeg"] = 60;
      }
      for (const auto *key :
           {"rateHz", "rangeMeters", "hFovDeg", "vFovDeg", "hRes", "vRes"}) {
        const auto value = lidar.find(key);
        if (value != lidar.end() && value->get<double>() > 0)
          sensor[key] = *value;
      }
      if (sensor["mode"] == "penetrating" && publish_beams)
        throw SceneError("penetrating sampled clouds have no beams");
      fleet["robots"].push_back(std::move(sensor));
    } catch (const Json::exception &error) {
      throw SceneError(
          "robots namespace/binding.simulationSensors.simpleLidar: " +
          std::string(error.what()));
    }
  }
  try {
    for (const auto &topic : input.visualization_topics) {
      if (topic.at("topic") == "/xgc/scene/reference_cloud" &&
          topic.at("messageType") == "sensor_msgs/PointCloud2" &&
          topic.at("role") == "semantic" &&
          (topic.value("visible3d", false) || topic.value("visibleAr", false)))
        fleet["referenceCloud"] = {{"surfaceSpacing", 0.1}};
    }
  } catch (const Json::exception &error) {
    throw SceneError("visualizationTopics: " + std::string(error.what()));
  }
  if (fleet.dump().size() > kMaxManifestBytes)
    throw SceneError("fleet manifest exceeds 1 MiB");
  return fleet;
}

FleetManifest buildFleet(const SessionInput &input,
                         const xgc2_math::scene::SensorConfig &defaults, bool point_cloud) {
  const auto fleet = buildFleetJson(input, point_cloud);
  if (fleet["robots"].empty() && !fleet.contains("referenceCloud"))
    return {};
  return parseManifest(fleet.dump(), defaults);
}

} // namespace xgc_scene::lidar
