#include "xgc2_ros1_robot_adapter/localization_projection.hpp"

#include <cmath>
#include <regex>
#include <sstream>
#include <nlohmann/json.hpp>

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

bool parseLocalizationExtrinsic(
    const std::map<std::string, std::string> &parameters,
    LocalizationProjectionConfig *output, std::string *error) {
  const auto found = parameters.find("mocap_marker_from_body");
  if (found == parameters.end())
    return true;
  try {
    const auto value = nlohmann::json::parse(found->second);
    if (!value.is_object() || value.size() != 2u ||
        !value.at("translation").is_array() || value.at("translation").size() != 3u ||
        !value.at("rotation").is_array() || value.at("rotation").size() != 4u)
      throw std::invalid_argument("invalid SE3");
    auto candidate = *output;
    candidate.marker_from_body_translation = value.at("translation").get<std::array<double, 3>>();
    candidate.marker_from_body_rotation = value.at("rotation").get<std::array<double, 4>>();
    for (const auto v : candidate.marker_from_body_translation)
      if (!finite(v)) throw std::invalid_argument("invalid translation");
    double norm = 0.0;
    for (const auto v : candidate.marker_from_body_rotation) norm += v * v;
    if (!finite(norm) || std::abs(norm - 1.0) > 1e-6)
      throw std::invalid_argument("invalid quaternion");
    candidate.has_extrinsic = true;
    *output = candidate;
    return true;
  } catch (const std::exception &) {
    if (error != nullptr)
      *error = "mocap_marker_from_body requires finite translation[3] and unit XYZW rotation[4]";
    return false;
  }
}

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
  if (!parseLocalizationOffsets(parameters, &candidate, error))
    return false;
  *output = candidate;
  return true;
}

bool parseLocalizationOffsets(
    const std::map<std::string, std::string> &parameters,
    LocalizationProjectionConfig *output, std::string *error) {
  if (output == nullptr) {
    if (error != nullptr)
      *error = "localization projection output is required";
    return false;
  }
  if (!parseFinite(parameters, "localization_offset_x", &output->offset_x,
                   error) ||
      !parseFinite(parameters, "localization_offset_y", &output->offset_y,
                   error) ||
      !parseFinite(parameters, "localization_offset_z", &output->offset_z,
                   error)) {
    return false;
  }
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
  if (config.has_extrinsic) {
    const double x = source.pose.orientation.x / norm;
    const double y = source.pose.orientation.y / norm;
    const double z = source.pose.orientation.z / norm;
    const double w = source.pose.orientation.w / norm;
    const auto &t = config.marker_from_body_translation;
    // q * t * q^-1. Rotate the lever in the measured marker orientation.
    const double ux = 2.0 * (y * t[2] - z * t[1]);
    const double uy = 2.0 * (z * t[0] - x * t[2]);
    const double uz = 2.0 * (x * t[1] - y * t[0]);
    output->pose.position.x += t[0] + w * ux + y * uz - z * uy;
    output->pose.position.y += t[1] + w * uy + z * ux - x * uz;
    output->pose.position.z += t[2] + w * uz + x * uy - y * ux;
    const auto &r = config.marker_from_body_rotation;
    output->pose.orientation.x = w * r[0] + x * r[3] + y * r[2] - z * r[1];
    output->pose.orientation.y = w * r[1] - x * r[2] + y * r[3] + z * r[0];
    output->pose.orientation.z = w * r[2] + x * r[1] - y * r[0] + z * r[3];
    output->pose.orientation.w = w * r[3] - x * r[0] - y * r[1] - z * r[2];
  }
  output->pose.position.x += config.offset_x;
  output->pose.position.y += config.offset_y;
  output->pose.position.z += config.offset_z;
  return finite(output->pose.position.x) && finite(output->pose.position.y) &&
         finite(output->pose.position.z);
}

bool validLocalizationTwist(const geometry_msgs::TwistStamped &source) {
  return validLocalizationLinear(source) &&
         finiteVector(source.twist.angular);
}

bool validLocalizationLinear(const geometry_msgs::TwistStamped &source) {
  return finiteVector(source.twist.linear);
}

geometry_msgs::AccelStamped
projectVrpnAcceleration(const geometry_msgs::TwistStamped &source) {
  geometry_msgs::AccelStamped output;
  output.header = source.header;
  output.accel.linear = source.twist.linear;
  output.accel.angular = source.twist.angular;
  return output;
}

bool validLocalizationAcceleration(const geometry_msgs::AccelStamped &source) {
  return finiteVector(source.accel.linear) && finiteVector(source.accel.angular);
}

} // namespace xgc2_ros1_robot_adapter
