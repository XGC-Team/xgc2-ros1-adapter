#pragma once

#include <cstdint>
#include <string>

namespace xgc2_ros1_robot_adapter {

// One selected VRPN source/tracker/sensor binding. The Zenoh key is derived
// from these identities, never from a ROS slot name.
struct ZenohLocalizationBinding {
  std::string endpoint;
  std::string source;
  std::string tracker;
  std::int32_t sensor = 0;
};

struct ZenohPoseSample {
  std::int64_t report_time_nanos = 0;
  std::int32_t clock_domain = 0;
  double position[3] = {0.0, 0.0, 0.0};
  double orientation[4] = {0.0, 0.0, 0.0, 1.0};
};

struct ZenohLocalizationConfig {
  bool enabled = false;
  std::string endpoint;
  std::string source;
  std::string tracker;
  std::int32_t sensor = 0;
  std::string frame_id = "world";
};

} // namespace xgc2_ros1_robot_adapter
