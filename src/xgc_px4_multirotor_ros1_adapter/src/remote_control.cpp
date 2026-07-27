#include "xgc_px4_multirotor_ros1_adapter/remote_control.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

double clamp(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

} // namespace

std::shared_ptr<RemoteControlPublisher> RemoteControlPublisher::Create(
    ros::NodeHandle node_handle, const std::string &topic,
    double altitude_meters, double maximum_linear_velocity_mps,
    double maximum_yaw_rate_rps, std::string *error) {
  if (topic.empty() || !std::isfinite(altitude_meters) ||
      altitude_meters <= 0.0 ||
      !std::isfinite(maximum_linear_velocity_mps) ||
      maximum_linear_velocity_mps <= 0.0 ||
      !std::isfinite(maximum_yaw_rate_rps) ||
      maximum_yaw_rate_rps <= 0.0) {
    fail(error, "PX4 remote-control publisher configuration is invalid");
    return nullptr;
  }
  try {
    auto publisher = std::make_shared<ros::Publisher>(
        node_handle.advertise<mavros_msgs::PositionTarget>(topic, 5, false));
    if (!*publisher) {
      fail(error, "ROS master did not accept PX4 remote-control publisher: " +
                      topic);
      return nullptr;
    }
    return std::make_shared<RemoteControlPublisher>(
        [publisher](const mavros_msgs::PositionTarget &target) {
          publisher->publish(target);
        },
        altitude_meters, maximum_linear_velocity_mps,
        maximum_yaw_rate_rps);
  } catch (const std::exception &exception) {
    fail(error, std::string("cannot create PX4 remote-control publisher: ") +
                    exception.what());
    return nullptr;
  }
}

RemoteControlPublisher::RemoteControlPublisher(
    PublishFunction publish, double altitude_meters,
    double maximum_linear_velocity_mps, double maximum_yaw_rate_rps)
    : publish_(std::move(publish)), altitude_meters_(altitude_meters),
      maximum_linear_velocity_mps_(maximum_linear_velocity_mps),
      maximum_yaw_rate_rps_(maximum_yaw_rate_rps) {
  target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target_.type_mask =
      mavros_msgs::PositionTarget::IGNORE_PX |
      mavros_msgs::PositionTarget::IGNORE_PY |
      mavros_msgs::PositionTarget::IGNORE_VZ |
      mavros_msgs::PositionTarget::IGNORE_AFX |
      mavros_msgs::PositionTarget::IGNORE_AFY |
      mavros_msgs::PositionTarget::IGNORE_AFZ |
      mavros_msgs::PositionTarget::FORCE |
      mavros_msgs::PositionTarget::IGNORE_YAW;
  // MAVROS exposes local position to ROS in ENU and performs the native NED
  // transform in the setpoint plugin. The operator contract therefore uses
  // positive-up one metre here.
  target_.position.z = altitude_meters_;
}

RemoteControlPublisher::~RemoteControlPublisher() { Stop(); }

bool RemoteControlPublisher::SetIntent(
    std::uint32_t gear, std::int32_t longitudinal, std::int32_t lateral,
    std::int32_t yaw, std::string *error) {
  if (gear < 1u || gear > 3u || longitudinal < -1 || longitudinal > 1 ||
      lateral < -1 || lateral > 1 || yaw < -1 || yaw > 1) {
    return fail(error, "PX4 remote intent is outside its discrete bounds");
  }
  const double scale = static_cast<double>(gear) / 3.0;
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return fail(error, "PX4 remote-control publisher is stopped");
  target_.velocity.x =
      clamp(static_cast<double>(longitudinal) *
                maximum_linear_velocity_mps_ * scale,
            maximum_linear_velocity_mps_);
  target_.velocity.y =
      clamp(static_cast<double>(lateral) * maximum_linear_velocity_mps_ * scale,
            maximum_linear_velocity_mps_);
  target_.yaw_rate =
      clamp(static_cast<double>(yaw) * maximum_yaw_rate_rps_ * scale,
            maximum_yaw_rate_rps_);
  active_ = true;
  return publishLocked(error);
}

bool RemoteControlPublisher::publishLocked(std::string *error) {
  if (!publish_)
    return fail(error, "PX4 remote-control publisher is unavailable");
  try {
    target_.header.stamp = ros::Time::now();
    publish_(target_);
    if (error != nullptr)
      error->clear();
    return true;
  } catch (...) {
    return fail(error, "PX4 remote-control publication failed");
  }
}

void RemoteControlPublisher::PublishPeriodic() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stopped_ && active_)
    publishLocked(nullptr);
}

void RemoteControlPublisher::Stop() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return;
  target_.velocity.x = 0.0;
  target_.velocity.y = 0.0;
  target_.yaw_rate = 0.0;
  if (active_)
    publishLocked(nullptr);
  active_ = false;
  stopped_ = true;
  publish_ = {};
}

} // namespace xgc_px4_multirotor_ros1_adapter
