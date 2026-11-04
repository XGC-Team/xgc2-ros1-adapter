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

struct ZenohVelocitySample {
  std::int64_t report_time_nanos = 0;
  std::int32_t clock_domain = 0;
  double linear[3] = {0.0, 0.0, 0.0};
  double rotation[4] = {0.0, 0.0, 0.0, 1.0};
  double interval_seconds = 0.0;
};

struct ZenohAccelerationSample {
  std::int64_t report_time_nanos = 0;
  std::int32_t clock_domain = 0;
  double linear[3] = {0.0, 0.0, 0.0};
  double rotation[4] = {0.0, 0.0, 0.0, 1.0};
  double interval_seconds = 0.0;
};

// Converts a VRPN rotation-over-interval quaternion and its dt into an
// angular-rate vector. This is a numeric conversion, not a quality gate.
bool zenohAngularRate(const double rotation_xyzw[4], double interval_seconds,
                      double output[3]);

struct ZenohLocalizationConfig {
  bool enabled = false;
  std::string endpoint;
  std::string source;
  std::string tracker;
  std::int32_t sensor = 0;
  std::string frame_id = "world";
};

} // namespace xgc2_ros1_robot_adapter
