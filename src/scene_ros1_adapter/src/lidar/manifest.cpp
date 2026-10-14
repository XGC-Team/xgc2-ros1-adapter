#include "xgc_scene/lidar/manifest.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cmath>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace xgc_scene::lidar {

FleetManifest parseManifest(std::string_view text,
                            const xgc2_math::scene::SensorConfig &defaults) {
  using xgc2_math::scene::SensorConfig;
  if (text.empty() || text.size() > kMaxManifestBytes)
    throw std::invalid_argument("fleet manifest must contain 1..1048576 bytes");
  boost::property_tree::ptree fleet;
  std::istringstream input{std::string(text)};
  boost::property_tree::read_json(input, fleet);
  if (fleet.get<int>("schemaVersion") != 1)
    throw std::invalid_argument("unsupported fleet sensor schema");
  const auto backend = fleet.get<std::string>("backend", "cpu");
  const auto &robots = fleet.get_child("robots");
  const std::regex name_pattern(
      "/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*");
  FleetManifest result;
  const auto bodies = fleet.get_child_optional("bodies");
  if (bodies) {
    result.body_roster = true;
    if (bodies->size() > kMaxFleetMembers)
      throw std::invalid_argument("body roster exceeds 512 simulated members");
    std::set<int> ids;
    std::set<std::string> namespaces;
    for (const auto &entry : *bodies) {
      const auto &spec = entry.second;
      BodySpec body;
      body.id = spec.get<int>("id");
      body.name_space = spec.get<std::string>("namespace");
      body.pose_topic = spec.get<std::string>("poseTopic");
      body.radius = spec.get<double>("geometry.radiusMeters");
      body.body_returns = spec.get<bool>("bodyReturns", false);
      if (body.id < 0 || !ids.insert(body.id).second ||
          !std::regex_match(body.name_space, name_pattern) ||
          !namespaces.insert(body.name_space).second ||
          !std::regex_match(body.pose_topic, name_pattern) ||
          spec.get<std::string>("geometry.type") != "sphere" ||
          !std::isfinite(body.radius) || body.radius <= 0.0)
        throw std::invalid_argument(
            "invalid body id/namespace/truth topic/sphere geometry");
      result.bodies.push_back(std::move(body));
    }
  }
  const auto reference = fleet.get_child_optional("referenceCloud");
  if (reference)
    result.reference = ReferenceSpec{reference->get<double>("surfaceSpacing")};
  if ((robots.empty() && !reference) || robots.size() > kMaxFleetMembers)
    throw std::invalid_argument(
        "fleet needs 1..512 robots or a scene reference cloud");
  if (result.reference) {
    const double spacing = result.reference->surface_spacing;
    if (!std::isfinite(spacing) || spacing <= 0.0 || spacing > 10.0)
      throw std::invalid_argument("invalid reference map spacing");
  }
  std::set<std::string> namespaces;
  result.robots.reserve(robots.size());
  for (const auto &entry : robots) {
    const auto &spec = entry.second;
    SensorSpec sensor;
    sensor.backend = spec.get<std::string>("backend", backend);
    if (sensor.backend != "cpu" && sensor.backend != "gpu" && sensor.backend != "auto")
      throw std::invalid_argument("sensor backend must be auto, cpu or gpu");
    sensor.name_space = spec.get<std::string>("namespace");
    if (!std::regex_match(sensor.name_space, name_pattern) ||
        !namespaces.insert(sensor.name_space).second)
      throw std::invalid_argument("invalid or duplicated fleet namespace");
    sensor.pose_topic = sensor.name_space + "/pose";
    sensor.body_id = static_cast<int>(result.robots.size() + 1);
    if (result.body_roster) {
      const auto body = std::find_if(
          result.bodies.begin(), result.bodies.end(),
          [&](const BodySpec &b) { return b.name_space == sensor.name_space; });
      sensor.body_id = spec.get<int>("bodyId", body == result.bodies.end() ? sensor.body_id : body->id);
      if (body != result.bodies.end() && sensor.body_id != body->id)
        throw std::invalid_argument(
            "sensor bodyId disagrees with its world member");
      if (body != result.bodies.end() && spec.get<std::string>("mode", "raycast") != "penetrating")
        sensor.pose_topic = body->pose_topic;
    }
    sensor.pose_topic = spec.get<std::string>("poseTopic", sensor.pose_topic);
    const std::string mode = spec.get<std::string>("mode", "raycast");
    sensor.config = defaults;
    auto &c = sensor.config;
    if (mode == "raycast")
      c.mode = SensorConfig::kRaycast;
    else if (mode == "penetrating")
      c.mode = SensorConfig::kPenetrating;
    else if (mode == "depth_frustum")
      c.mode = SensorConfig::kDepthFrustum;
    else
      throw std::invalid_argument("invalid fleet sensor mode");
    c.range = spec.get<double>("rangeMeters", c.range);
    c.min_range = spec.get<double>("minRangeMeters", c.min_range);
    c.h_fov_deg = spec.get<double>("hFovDeg", c.h_fov_deg);
    c.v_fov_deg = spec.get<double>("vFovDeg", c.v_fov_deg);
    c.h_res = spec.get<int>("hRes", c.h_res);
    c.v_res = spec.get<int>("vRes", c.v_res);
    c.surface_spacing = spec.get<double>("surfaceSpacing", c.surface_spacing);
    c.penetrating_keep_buried = spec.get<bool>("keepBuried", false);
    c.penetrating_heading_crop = spec.get<bool>("headingCrop", false);
    c.heading_cos_min = spec.get<double>("headingCosMin", 0.0);
    c.vertical_slab_tan =
        spec.get<double>("verticalSlabTan", c.vertical_slab_tan);
    if (c.mode == SensorConfig::kDepthFrustum) {
      c.width = c.h_res;
      c.height = c.v_res;
    }
    sensor.rate_hz = spec.get<double>("rateHz", 10.0);
    if (!std::isfinite(sensor.rate_hz) || sensor.rate_hz <= 0.0 ||
        sensor.rate_hz > 100.0)
      throw std::invalid_argument("invalid fleet sensor rate");
    sensor.publish_beams = spec.get<bool>("publishBeams", false);
    // WorldLidar checks spacing in all modes before resolving its
    // configuration.
    if (!std::isfinite(c.surface_spacing) || c.surface_spacing <= 0.0)
      throw std::invalid_argument(
          "scene sample spacing must be finite and > 0");
    xgc2_math::scene::resolveSensorConfig(&c);
    sensor.publish_beams =
        sensor.publish_beams && c.mode != SensorConfig::kPenetrating;
    result.robots.push_back(std::move(sensor));
  }
  return result;
}

} // namespace xgc_scene::lidar
