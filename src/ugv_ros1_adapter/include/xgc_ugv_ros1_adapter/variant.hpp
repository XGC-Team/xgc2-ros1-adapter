#pragma once

#include <string>

#include "xgc2_ros1_robot_adapter/contract.hpp"

namespace xgc_ugv_ros1_adapter {
namespace contract = xgc2_ros1_robot_adapter::contract;

// Velocity limits of a chassis, and whether it can drive sideways.
struct MotionLimits {
  double maximum_linear_mps;
  double maximum_angular_rps;
  // Holonomic chassis: the lateral intent drives linear.y. Otherwise the
  // lateral component of a shared controller's intent is ignored.
  bool lateral;
};

// Where a vehicle's "online" fact comes from.
enum class HealthSource {
  // The chassis state word (control mode, base state, fault code).
  kChassisState,
  // The on-board IMU, the only stream the vehicle publishes without being
  // commanded.
  kImu,
};

// One robot type served by the UGV runtime. The runtime is the same code for
// every type; this is all that differs between them, and the generated profile
// contract carries the topics and rates.
struct UgvVariant {
  const char *provider;         // provider definition id
  const char *name;             // for logs and rejection messages
  const char *short_name;       // for fault codes and health summaries
  const char *processor_prefix; // processors are "<prefix>.vrpn-pose", ...
  MotionLimits motion;
  HealthSource health;
  // Smooth the battery voltage before it is mapped to a percentage.
  bool filter_battery;
  // Observation window of the stream-rate estimates, in seconds. Zero
  // estimates over every health emit; a longer window keeps a slow topic from
  // being quantized by the emit period.
  double stream_rate_window_seconds;
  const contract::Contract &contract;
};

const UgvVariant &scoutMini();
const UgvVariant &mecanumUgv();
// nullptr for a provider this runtime does not serve.
const UgvVariant *variantFor(const std::string &provider);

} // namespace xgc_ugv_ros1_adapter
