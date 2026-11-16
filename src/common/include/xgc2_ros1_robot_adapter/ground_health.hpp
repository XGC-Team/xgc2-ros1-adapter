#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace xgc2_ros1_robot_adapter {

struct BatteryCurvePoint {
  double voltage_v = 0.0;
  double percentage = 0.0;
};

bool parseBatteryCurve(const char *const *entries, std::size_t count,
                       std::vector<BatteryCurvePoint> *curve,
                       std::string *error);
bool batteryPercentage(const std::vector<BatteryCurvePoint> &curve,
                       double voltage_v, double *percentage);

// Voltage-only display smoothing. Does not estimate current, IR drop or BMS SOC.
class BatteryVoltageFilter {
public:
  bool update(double voltage_v, double monotonic_seconds, double *filtered_v);
private:
  std::deque<double> samples_;
  double last_seconds_ = 0.0;
  double filtered_v_ = 0.0;
};

enum class PositioningHealthState {
  kUnspecified,
  kWarmingUp,
  kStable,
  kJittering,
  kMoving,
  kTimedOut,
  kActive,
  kFrozen,
};

enum class PositioningHealthReason {
  kUnspecified,
  kInsufficientSamples,
  kStationaryWindowStable,
  kStationaryJitterExceeded,
  kRobotMoving,
  kVrpnTimeout,
  kWindowVariationObserved,
  kRepeatFrameWindowFrozen,
};

struct PositioningHealthConfig {
  std::size_t frame_number = 5u;
  double comparison_threshold_m = 1.0e-10;
  double timeout_seconds = 0.25;
};

struct PositioningHealthResult {
  PositioningHealthState state = PositioningHealthState::kUnspecified;
  PositioningHealthReason reason = PositioningHealthReason::kUnspecified;
  std::uint64_t observed_age_ms = 0u;
  double comparison_metric_m = 0.0;
  std::size_t sample_count = 0u;
};

bool validPositioningHealthConfig(const PositioningHealthConfig &config,
                                  std::string *error);
bool parsePositioningHealthConfig(
    const std::map<std::string, std::string> &parameters,
    PositioningHealthConfig *config, std::string *error);

class PositioningHealthWindow {
public:
  explicit PositioningHealthWindow(PositioningHealthConfig config);

  void recordPose(double observed_seconds, double x, double y, double z);
  PositioningHealthResult evaluate(double now_seconds) const;
  const PositioningHealthConfig &config() const { return config_; }

private:
  PositioningHealthConfig config_;
  std::vector<std::array<double, 3>> registers_;
  std::size_t first_ = 0u, retained_ = 0u;
  double last_observed_seconds_ = 0.0;
  double comparison_metric_m_ = 0.0;
  bool has_observation_ = false;
  bool active_ = false;
};

enum class MocapInputState {
  kUnobserved,
  kActive,
  kFrozen,
  kTimedOut,
  kInvalid,
};

struct MocapInputHealth {
  MocapInputState state = MocapInputState::kUnobserved;
  PositioningHealthResult positioning;
  bool latest_sample_invalid = false;
  std::uint64_t invalid_sample_count = 0u;
  double attitude_divergence_rad = 0.0;
  bool attitude_diverged = false;
};

enum class VisionOutputState {
  kDisabled,
  kUnobserved,
  kPublishingValid,
  kPublishingInvalid,
};

enum class EstimatorHealthState {
  kUnobserved,
  kStale,
  kHealthy,
  kDegraded,
};

// Generic EKF/estimator status flags observed from MAVLink #230
// (mavros_msgs/EstimatorStatus). These do not prove external-vision fusion.
struct EstimatorStatusFlags {
  bool observed = false;
  bool stale = false;
  bool attitude_ok = false;
  bool velocity_horiz_ok = false;
  bool velocity_vert_ok = false;
  bool pos_horiz_rel_ok = false;
  bool pos_horiz_abs_ok = false;
  bool pos_vert_abs_ok = false;
  bool pos_vert_agl_ok = false;
  bool const_pos_mode = false;
  bool pred_pos_horiz_rel_ok = false;
  bool pred_pos_horiz_abs_ok = false;
  bool gps_glitch = false;
  bool accel_error = false;
};

struct EstimatorHealth {
  EstimatorHealthState state = EstimatorHealthState::kUnobserved;
  EstimatorStatusFlags flags;
  std::vector<std::string> active_degraded_flags;
};

struct Px4HealthTruthInput {
  bool mocap_observed = false;
  bool mocap_input_timed_out = false;
  bool latest_mocap_invalid = false;
  std::uint64_t invalid_mocap_count = 0u;
  PositioningHealthResult positioning;
  bool attitude_diverged = false;
  double attitude_divergence_rad = 0.0;
  bool vision_output_enabled = false;
  VisionOutputState vision_output_state = VisionOutputState::kDisabled;
  EstimatorStatusFlags estimator_flags;
};

struct Px4HealthTruth {
  MocapInputHealth mocap_input;
  VisionOutputState vision_output = VisionOutputState::kDisabled;
  EstimatorHealth estimator_health;
  // EV fusion inside PX4 EKF2 cannot be inferred from MAVLink #230 or
  // position proximity; always reported explicitly as "UNKNOWN".
  std::string ev_fusion_state = "UNKNOWN";
};

const char *mocapInputStateName(MocapInputState state);
const char *visionOutputStateName(VisionOutputState state);
const char *estimatorHealthStateName(EstimatorHealthState state);
Px4HealthTruth evaluatePx4HealthTruth(const Px4HealthTruthInput &input);

} // namespace xgc2_ros1_robot_adapter
