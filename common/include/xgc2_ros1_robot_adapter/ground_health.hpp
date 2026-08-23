#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
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

enum class PositioningHealthState {
  kUnspecified,
  kWarmingUp,
  kStable,
  kJittering,
  kMoving,
  kTimedOut,
};

enum class PositioningHealthReason {
  kUnspecified,
  kInsufficientSamples,
  kStationaryWindowStable,
  kStationaryJitterExceeded,
  kRobotMoving,
  kVrpnTimeout,
};

struct PositioningHealthConfig {
  double window_seconds = 0.0;
  std::size_t minimum_samples = 0u;
  double stationary_speed_threshold_mps = 0.0;
  double maximum_position_spread_m = 0.0;
  double vrpn_timeout_seconds = 0.0;
};

struct PositioningHealthResult {
  PositioningHealthState state = PositioningHealthState::kUnspecified;
  PositioningHealthReason reason = PositioningHealthReason::kUnspecified;
  std::uint64_t observed_age_ms = 0u;
  double window_spread_m = 0.0;
  std::size_t sample_count = 0u;
};

bool validPositioningHealthConfig(const PositioningHealthConfig &config,
                                  std::string *error);

class PositioningHealthWindow {
public:
  explicit PositioningHealthWindow(PositioningHealthConfig config);

  void recordPose(double observed_seconds, double x, double y, double z);
  void recordVelocity(double observed_seconds, double x, double y, double z);
  PositioningHealthResult evaluate(double now_seconds);

private:
  struct PoseSample {
    double observed_seconds = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  void prune(double now_seconds);
  double windowSpreadMeters() const;

  PositioningHealthConfig config_;
  std::deque<PoseSample> poses_;
  double last_velocity_seconds_ = 0.0;
  double last_speed_mps_ = 0.0;
  bool has_velocity_ = false;
  bool last_velocity_was_moving_ = false;
};

} // namespace xgc2_ros1_robot_adapter
