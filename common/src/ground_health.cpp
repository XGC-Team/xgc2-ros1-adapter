#include "xgc2_ros1_robot_adapter/ground_health.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iterator>
#include <limits>

namespace xgc2_ros1_robot_adapter {
namespace {

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

bool parseFiniteDouble(const std::string &value, double *output) {
  if (output == nullptr || value.empty())
    return false;
  errno = 0;
  char *end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (errno != 0 || end == value.c_str() || end == nullptr || *end != '\0' ||
      !std::isfinite(parsed)) {
    return false;
  }
  *output = parsed;
  return true;
}

std::uint64_t ageMillis(double now_seconds, double observed_seconds) {
  if (!std::isfinite(now_seconds) || !std::isfinite(observed_seconds) ||
      now_seconds < observed_seconds) {
    return 0u;
  }
  const double milliseconds = (now_seconds - observed_seconds) * 1000.0;
  if (milliseconds >=
      static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  const double rounded = std::floor(std::max(0.0, milliseconds) + 0.5);
  if (rounded >=
      static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return static_cast<std::uint64_t>(rounded);
}

} // namespace

bool parseBatteryCurve(const char *const *entries, std::size_t count,
                       std::vector<BatteryCurvePoint> *curve,
                       std::string *error) {
  if (curve == nullptr)
    return fail(error, "battery curve output is required");
  curve->clear();
  if (entries == nullptr || count == 0u)
    return true;
  if (count < 2u)
    return fail(error, "battery curve requires at least two points");

  for (std::size_t index = 0u; index < count; ++index) {
    const std::string entry(entries[index] == nullptr ? "" : entries[index]);
    const std::size_t separator = entry.find('=');
    if (separator == std::string::npos || separator == 0u ||
        separator + 1u >= entry.size() ||
        entry.find('=', separator + 1u) != std::string::npos) {
      return fail(error, "battery curve points must use voltage=percentage");
    }
    BatteryCurvePoint point;
    if (!parseFiniteDouble(entry.substr(0u, separator), &point.voltage_v) ||
        !parseFiniteDouble(entry.substr(separator + 1u), &point.percentage)) {
      return fail(error, "battery curve contains a non-finite point");
    }
    if (point.percentage < 0.0 || point.percentage > 1.0)
      return fail(error, "battery curve percentage must be within [0,1]");
    if (!curve->empty()) {
      const auto &previous = curve->back();
      if (point.voltage_v <= previous.voltage_v)
        return fail(error, "battery curve voltages must increase strictly");
      if (point.percentage < previous.percentage)
        return fail(error, "battery curve percentages must not decrease");
    }
    curve->push_back(point);
  }
  return true;
}

bool batteryPercentage(const std::vector<BatteryCurvePoint> &curve,
                       double voltage_v, double *percentage) {
  if (percentage == nullptr || curve.size() < 2u || !std::isfinite(voltage_v))
    return false;
  if (voltage_v <= curve.front().voltage_v) {
    *percentage = curve.front().percentage;
    return true;
  }
  if (voltage_v >= curve.back().voltage_v) {
    *percentage = curve.back().percentage;
    return true;
  }
  for (std::size_t index = 1u; index < curve.size(); ++index) {
    const auto &upper = curve[index];
    if (voltage_v > upper.voltage_v)
      continue;
    const auto &lower = curve[index - 1u];
    const double ratio =
        (voltage_v - lower.voltage_v) / (upper.voltage_v - lower.voltage_v);
    *percentage =
        lower.percentage + ratio * (upper.percentage - lower.percentage);
    return true;
  }
  return false;
}

bool validPositioningHealthConfig(const PositioningHealthConfig &config,
                                  std::string *error) {
  if (config.frame_number < 1u || config.frame_number > 999u)
    return fail(error, "positioning frame number must be between 1 and 999");
  if (!std::isfinite(config.comparison_threshold_m) ||
      config.comparison_threshold_m < 1.0e-10 ||
      config.comparison_threshold_m > 10.0)
    return fail(error,
                "positioning comparison threshold must be between 1e-10 and 10 metres");
  if (!std::isfinite(config.timeout_seconds) || config.timeout_seconds <= 0.0)
    return fail(error, "positioning timeout must be positive");
  return true;
}

bool parsePositioningHealthConfig(
    const std::map<std::string, std::string> &parameters,
    PositioningHealthConfig *config, std::string *error) {
  if (config == nullptr)
    return fail(error, "positioning health configuration output is required");
  const auto frames = parameters.find("positioning_frame_number");
  const auto threshold =
      parameters.find("positioning_comparison_threshold_m");
  if (frames == parameters.end() || threshold == parameters.end()) {
    return fail(
        error,
        "positioning_frame_number and positioning_comparison_threshold_m are required");
  }
  PositioningHealthConfig candidate;
  try {
    std::size_t parsed = 0u;
    const unsigned long long value = std::stoull(frames->second, &parsed, 10);
    if (parsed != frames->second.size())
      return fail(error, "positioning frame number is not canonical");
    candidate.frame_number = static_cast<std::size_t>(value);
    parsed = 0u;
    candidate.comparison_threshold_m = std::stod(threshold->second, &parsed);
    if (parsed != threshold->second.size())
      return fail(error, "positioning comparison threshold is not canonical");
  } catch (const std::exception &) {
    return fail(error, "positioning health parameters are not canonical numbers");
  }
  // XGC1 exposes frame_number and comparison_threshold but keeps the proven
  // 100 ms freshness boundary fixed in the detector.
  candidate.timeout_seconds = 0.1;
  if (!validPositioningHealthConfig(candidate, error))
    return false;
  *config = candidate;
  return true;
}

PositioningHealthWindow::PositioningHealthWindow(PositioningHealthConfig config)
    : config_(config) {}

void PositioningHealthWindow::recordPose(double observed_seconds, double x,
                                         double y, double z) {
  if (!std::isfinite(observed_seconds) || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(z)) {
    return;
  }
  if (has_observation_ && observed_seconds < last_observed_seconds_) {
    for (auto &values : registers_)
      values.clear();
    active_ = false;
    comparison_metric_m_ = 0.0;
  }
  last_observed_seconds_ = observed_seconds;
  has_observation_ = true;
  const std::array<double, 3> newest{{x, y, z}};
  for (std::size_t axis = 0u; axis < registers_.size(); ++axis)
    registers_[axis].push_back(newest[axis]);

  // XGC1 evaluates after frame_number retained samples plus the newest frame,
  // then removes the oldest frame. Keep this order exactly.
  if (registers_[0].size() <= config_.frame_number) {
    active_ = false;
    comparison_metric_m_ = 0.0;
    return;
  }
  active_ = false;
  comparison_metric_m_ = 0.0;
  const double threshold_squared =
      config_.comparison_threshold_m * config_.comparison_threshold_m;
  for (std::size_t axis = 0u; axis < registers_.size(); ++axis) {
    double sum_squared_difference = 0.0;
    for (const double value : registers_[axis]) {
      const double difference = value - newest[axis];
      sum_squared_difference += difference * difference;
    }
    comparison_metric_m_ =
        std::max(comparison_metric_m_, std::sqrt(sum_squared_difference));
    // XGC1 physical PX4 and UGV modules enable the single-axis check: natural
    // variation on any axis proves that the source is not replaying one pose.
    active_ = active_ || sum_squared_difference > threshold_squared;
  }
  for (auto &values : registers_)
    values.pop_front();
}

PositioningHealthResult
PositioningHealthWindow::evaluate(double now_seconds) const {
  PositioningHealthResult result;
  result.comparison_metric_m = comparison_metric_m_;
  result.sample_count = registers_[0].size();
  result.state = PositioningHealthState::kTimedOut;
  result.reason = PositioningHealthReason::kVrpnTimeout;
  if (!has_observation_ || !std::isfinite(now_seconds) ||
      now_seconds < last_observed_seconds_)
    return result;
  result.observed_age_ms = ageMillis(now_seconds, last_observed_seconds_);
  if (now_seconds - last_observed_seconds_ >= config_.timeout_seconds)
    return result;
  result.state = active_ ? PositioningHealthState::kActive
                         : PositioningHealthState::kFrozen;
  result.reason = active_
                      ? PositioningHealthReason::kWindowVariationObserved
                      : PositioningHealthReason::kRepeatFrameWindowFrozen;
  return result;
}

} // namespace xgc2_ros1_robot_adapter
