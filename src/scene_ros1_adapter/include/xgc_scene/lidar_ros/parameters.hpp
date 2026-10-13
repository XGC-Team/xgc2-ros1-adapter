#pragma once

#include "xgc_scene/common.hpp"
#include "xgc_scene/lidar/engine.hpp"
#include "xgc_scene/lidar/roster.hpp"
#include <iosfwd>
#include <optional>

namespace xgc_scene::lidar_ros {

// Called only for --session-stdin, before ROS or either service is started.
// An empty stream means no fleet; all present documents are validated.
std::optional<lidar::SessionInput> readSessionInput(std::istream& input);

struct Parameters {
  std::string socket;
  bool enabled = true;
  lidar::FleetManifest manifest;
  lidar::EngineOptions options;
  bool have_fleet = false;
  static Parameters parse(const Json& values, const std::string& profile,
                          const std::optional<lidar::SessionInput>& session = std::nullopt, bool point_cloud = false);
  Json facts(const std::string& profile) const;
};

}  // namespace xgc_scene::lidar_ros
