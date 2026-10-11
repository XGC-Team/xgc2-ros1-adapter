#pragma once

#include "xgc_scene/lidar/engine.hpp"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

namespace xgc_scene::lidar_ros {

xgc2_math::Pose3 poseOf(const geometry_msgs::Pose& pose);
sensor_msgs::PointCloud2 cloudLayout(bool beams = false);
void fillCloud(sensor_msgs::PointCloud2& message, lidar::CloudView cloud);

// Updated only by connect/disconnect callbacks on the engine's owner lane.
struct Demand {
  std::size_t points = 0, beams = 0;
  bool change(bool beam_topic, bool connected);
};

}  // namespace xgc_scene::lidar_ros
