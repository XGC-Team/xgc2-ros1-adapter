#pragma once

#include <cstddef>
#include <deque>
#include <map>
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
};

bool parseLocalizationProjectionConfig(
    const std::map<std::string, std::string> &parameters,
    LocalizationProjectionConfig *output, std::string *error);

bool projectLocalizationPose(const geometry_msgs::PoseStamped &source,
                             const LocalizationProjectionConfig &config,
                             geometry_msgs::PoseStamped *output);
bool validLocalizationTwist(const geometry_msgs::TwistStamped &source);
bool validLocalizationAcceleration(const geometry_msgs::AccelStamped &source);

class VisionPublishCadence {
public:
  explicit VisionPublishCadence(double target_rate_hz = 30.0,
                                std::size_t window = 5u);
  bool take(double now_seconds);

private:
  double target_rate_hz_;
  std::size_t window_;
  std::deque<double> published_times_;
};

} // namespace xgc2_ros1_robot_adapter
