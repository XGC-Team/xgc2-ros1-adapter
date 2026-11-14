#pragma once

#include <map>
#include <array>
#include <string>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace xgc2_ros1_robot_adapter {

struct LocalizationProjectionConfig {
  std::string source_root;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;
  // T_marker_body, ROS body FLU. Applied before the world translation.
  bool has_extrinsic = false;
  std::array<double, 3> marker_from_body_translation{{0.0, 0.0, 0.0}};
  std::array<double, 4> marker_from_body_rotation{{0.0, 0.0, 0.0, 1.0}};
};

bool parseLocalizationExtrinsic(
    const std::map<std::string, std::string> &parameters,
    LocalizationProjectionConfig *output, std::string *error);

bool parseLocalizationProjectionConfig(
    const std::map<std::string, std::string> &parameters,
    LocalizationProjectionConfig *output, std::string *error);

// World-offset fields only. Zenoh localization does not carry a ROS source
// root, so its binding uses this instead of requiring mocap_source_root.
bool parseLocalizationOffsets(
    const std::map<std::string, std::string> &parameters,
    LocalizationProjectionConfig *output, std::string *error);

bool projectLocalizationPose(const geometry_msgs::PoseStamped &source,
                             const LocalizationProjectionConfig &config,
                             geometry_msgs::PoseStamped *output);
bool validLocalizationTwist(const geometry_msgs::TwistStamped &source);
bool validLocalizationLinear(const geometry_msgs::TwistStamped &source);
geometry_msgs::AccelStamped
projectVrpnAcceleration(const geometry_msgs::TwistStamped &source);
bool validLocalizationAcceleration(const geometry_msgs::AccelStamped &source);

} // namespace xgc2_ros1_robot_adapter
