#pragma once

#include "xgc_scene/common.hpp"
#include "xgc_scene/lidar/engine.hpp"

namespace ros { class NodeHandle; }

namespace xgc_scene::lidar_ros {

struct Parameters {
  std::string socket;
  bool enabled = true;
  lidar::FleetManifest manifest;
  lidar::EngineOptions options;
  bool have_fleet = false;
  static Parameters parse(const Json& values, const std::string& profile);
  static Parameters read(ros::NodeHandle& private_node, const std::string& profile);
  Json facts(const std::string& profile) const;
};

}  // namespace xgc_scene::lidar_ros
