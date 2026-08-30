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
  double timeout_seconds = 0.1;
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

private:
  PositioningHealthConfig config_;
  std::array<std::deque<double>, 3> registers_;
  double last_observed_seconds_ = 0.0;
  double comparison_metric_m_ = 0.0;
  bool has_observation_ = false;
  bool active_ = false;
};

} // namespace xgc2_ros1_robot_adapter
