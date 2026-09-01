#include "xgc2_ros1_robot_adapter/localization_projection.hpp"

#include <cmath>
#include <regex>
#include <sstream>

namespace xgc2_ros1_robot_adapter {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool parseFinite(const std::map<std::string, std::string> &parameters,
                 const char *name, double *output, std::string *error) {
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    if (error != nullptr)
      *error = std::string("missing localization parameter ") + name;
    return false;
  }
  std::size_t parsed = 0u;
  try {
    const double value = std::stod(found->second, &parsed);
    if (parsed != found->second.size() || !finite(value))
      throw std::invalid_argument("not finite");
    *output = value;
    return true;
  } catch (const std::exception &) {
    if (error != nullptr)
      *error = std::string("localization parameter ") + name +
               " must be finite";
    return false;
  }
}

bool finiteVector(const geometry_msgs::Vector3 &value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

} // namespace

bool parseLocalizationProjectionConfig(
    const std::map<std::string, std::string> &parameters,
    LocalizationProjectionConfig *output, std::string *error) {
  if (output == nullptr) {
    if (error != nullptr)
      *error = "localization projection output is required";
    return false;
  }
  const auto root = parameters.find("mocap_source_root");
  static const std::regex root_pattern(
      "^/[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*$");
  if (root == parameters.end() ||
      !std::regex_match(root->second, root_pattern)) {
    if (error != nullptr)
      *error = "mocap_source_root must be an absolute ROS namespace";
    return false;
  }
  LocalizationProjectionConfig candidate;
  candidate.source_root = root->second;
  if (!parseFinite(parameters, "localization_offset_x", &candidate.offset_x,
                   error) ||
      !parseFinite(parameters, "localization_offset_y", &candidate.offset_y,
                   error) ||
      !parseFinite(parameters, "localization_offset_z", &candidate.offset_z,
                   error)) {
    return false;
  }
  *output = candidate;
  return true;
}

bool projectLocalizationPose(const geometry_msgs::PoseStamped &source,
                             const LocalizationProjectionConfig &config,
                             geometry_msgs::PoseStamped *output) {
  if (output == nullptr || !finite(source.pose.position.x) ||
      !finite(source.pose.position.y) || !finite(source.pose.position.z) ||
      !finite(source.pose.orientation.x) ||
      !finite(source.pose.orientation.y) ||
      !finite(source.pose.orientation.z) ||
      !finite(source.pose.orientation.w)) {
    return false;
  }
  const double norm =
      std::hypot(std::hypot(source.pose.orientation.x,
                            source.pose.orientation.y),
                 std::hypot(source.pose.orientation.z,
                            source.pose.orientation.w));
  if (!finite(norm) || norm < 1.0e-6)
    return false;
  *output = source;
  output->pose.position.x += config.offset_x;
  output->pose.position.y += config.offset_y;
  output->pose.position.z += config.offset_z;
  return finite(output->pose.position.x) && finite(output->pose.position.y) &&
         finite(output->pose.position.z);
}

bool validLocalizationTwist(const geometry_msgs::TwistStamped &source) {
  return finiteVector(source.twist.linear) && finiteVector(source.twist.angular);
}

bool validLocalizationAcceleration(const geometry_msgs::AccelStamped &source) {
  return finiteVector(source.accel.linear) && finiteVector(source.accel.angular);
}

VisionPublishCadence::VisionPublishCadence(double target_rate_hz,
                                           std::size_t window)
    : target_rate_hz_(target_rate_hz), window_(window) {
  if (!finite(target_rate_hz_) || target_rate_hz_ <= 0.0 || window_ == 0u)
    throw std::invalid_argument("vision publish cadence is invalid");
}

bool VisionPublishCadence::take(double now_seconds) {
  if (!finite(now_seconds))
    return false;
  bool emit = published_times_.empty();
  if (!emit) {
    const double since_last = now_seconds - published_times_.back();
    if (since_last < 0.0 || since_last >= 1.0 / target_rate_hz_) {
      emit = true;
    } else {
      const double span = now_seconds - published_times_.front();
      emit = span > 0.0 &&
             static_cast<double>(published_times_.size()) / span <
                 target_rate_hz_;
    }
  }
  if (!emit)
    return false;
  published_times_.push_back(now_seconds);
  while (published_times_.size() > window_)
    published_times_.pop_front();
  return true;
}

} // namespace xgc2_ros1_robot_adapter
