#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

namespace xgc_px4_multirotor_ros1_adapter {

class RemoteControlPublisher {
public:
  using PublishFunction =
      std::function<void(const mavros_msgs::PositionTarget &)>;

  static std::shared_ptr<RemoteControlPublisher>
  Create(ros::NodeHandle node_handle, const std::string &topic,
         double altitude_meters, double maximum_linear_velocity_mps,
         double maximum_yaw_rate_rps, std::string *error);

  RemoteControlPublisher(PublishFunction publish, double altitude_meters,
                         double maximum_linear_velocity_mps,
                         double maximum_yaw_rate_rps);
  ~RemoteControlPublisher();

  bool SetIntent(std::uint32_t gear, std::int32_t longitudinal,
                 std::int32_t lateral, std::int32_t yaw, std::string *error);
  void PublishPeriodic();
  void Stop() noexcept;

private:
  bool publishLocked(std::string *error);

  std::mutex mutex_;
  PublishFunction publish_;
  mavros_msgs::PositionTarget target_;
  double altitude_meters_;
  double maximum_linear_velocity_mps_;
  double maximum_yaw_rate_rps_;
  bool active_ = false;
  bool stopped_ = false;
};

} // namespace xgc_px4_multirotor_ros1_adapter
