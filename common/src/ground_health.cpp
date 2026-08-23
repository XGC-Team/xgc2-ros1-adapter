#include "xgc2_ros1_robot_adapter/ground_health.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
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
  if (!std::isfinite(config.window_seconds) || config.window_seconds <= 0.0)
    return fail(error, "positioning window must be positive");
  if (config.minimum_samples < 2u)
    return fail(error, "positioning window requires at least two samples");
  if (!std::isfinite(config.stationary_speed_threshold_mps) ||
      config.stationary_speed_threshold_mps < 0.0) {
    return fail(error, "stationary speed threshold must be non-negative");
  }
  if (!std::isfinite(config.maximum_position_spread_m) ||
      config.maximum_position_spread_m < 0.0) {
    return fail(error, "positioning spread threshold must be non-negative");
  }
  if (!std::isfinite(config.vrpn_timeout_seconds) ||
      config.vrpn_timeout_seconds <= 0.0) {
    return fail(error, "VRPN timeout must be positive");
  }
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
  if (!poses_.empty() && observed_seconds < poses_.back().observed_seconds)
    poses_.clear();
  poses_.push_back({observed_seconds, x, y, z});
  prune(observed_seconds);
}

void PositioningHealthWindow::recordVelocity(double observed_seconds, double x,
                                             double y, double z) {
  if (!std::isfinite(observed_seconds) || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(z)) {
    return;
  }
  if (has_velocity_ && observed_seconds < last_velocity_seconds_) {
    poses_.clear();
    last_velocity_was_moving_ = false;
  }
  const double speed_mps = std::hypot(std::hypot(x, y), z);
  const bool moving = speed_mps > config_.stationary_speed_threshold_mps;
  if ((moving || last_velocity_was_moving_ != moving) && !poses_.empty()) {
    const PoseSample latest_pose = poses_.back();
    poses_.clear();
    poses_.push_back(latest_pose);
  }
  last_velocity_seconds_ = observed_seconds;
  last_speed_mps_ = speed_mps;
  has_velocity_ = true;
  last_velocity_was_moving_ = moving;
}

PositioningHealthResult PositioningHealthWindow::evaluate(double now_seconds) {
  PositioningHealthResult result;
  if (poses_.empty() || !std::isfinite(now_seconds) ||
      now_seconds < poses_.back().observed_seconds) {
    result.state = PositioningHealthState::kTimedOut;
    result.reason = PositioningHealthReason::kVrpnTimeout;
    return result;
  }

  result.observed_age_ms =
      ageMillis(now_seconds, poses_.back().observed_seconds);
  const double pose_age_seconds = now_seconds - poses_.back().observed_seconds;
  if (pose_age_seconds > config_.vrpn_timeout_seconds) {
    result.state = PositioningHealthState::kTimedOut;
    result.reason = PositioningHealthReason::kVrpnTimeout;
    result.sample_count = poses_.size();
    return result;
  }
  prune(now_seconds);
  result.sample_count = poses_.size();

  const bool velocity_fresh =
      has_velocity_ && now_seconds >= last_velocity_seconds_ &&
      now_seconds - last_velocity_seconds_ <= config_.vrpn_timeout_seconds;
  if (!velocity_fresh) {
    result.state = PositioningHealthState::kWarmingUp;
    result.reason = PositioningHealthReason::kInsufficientSamples;
    return result;
  }
  if (last_speed_mps_ > config_.stationary_speed_threshold_mps) {
    result.state = PositioningHealthState::kMoving;
    result.reason = PositioningHealthReason::kRobotMoving;
    return result;
  }
  if (poses_.size() < config_.minimum_samples) {
    result.state = PositioningHealthState::kWarmingUp;
    result.reason = PositioningHealthReason::kInsufficientSamples;
    return result;
  }

  result.window_spread_m = windowSpreadMeters();
  if (result.window_spread_m > config_.maximum_position_spread_m) {
    result.state = PositioningHealthState::kJittering;
    result.reason = PositioningHealthReason::kStationaryJitterExceeded;
  } else {
    result.state = PositioningHealthState::kStable;
    result.reason = PositioningHealthReason::kStationaryWindowStable;
  }
  return result;
}

void PositioningHealthWindow::prune(double now_seconds) {
  if (!std::isfinite(now_seconds))
    return;
  const double oldest_allowed = now_seconds - config_.window_seconds;
  while (!poses_.empty() && poses_.front().observed_seconds < oldest_allowed)
    poses_.pop_front();
}

double PositioningHealthWindow::windowSpreadMeters() const {
  double maximum = 0.0;
  for (auto left = poses_.begin(); left != poses_.end(); ++left) {
    for (auto right = std::next(left); right != poses_.end(); ++right) {
      maximum = std::max(maximum, std::hypot(std::hypot(left->x - right->x,
                                                        left->y - right->y),
                                             left->z - right->z));
    }
  }
  return maximum;
}

} // namespace xgc2_ros1_robot_adapter
