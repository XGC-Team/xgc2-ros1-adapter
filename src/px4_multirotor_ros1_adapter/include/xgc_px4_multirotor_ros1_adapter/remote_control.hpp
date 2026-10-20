#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

namespace xgc_px4_multirotor_ros1_adapter {

// Publishes the setpoint stream of a motion intent. Stop() publishes a final
// zero velocity while the stream is active; the ROS publisher lives until the
// destructor so the owner can flush publications before destroying it.
// CoreSilent() is the dead-man: an intent that nobody refreshes or ends because
// Core is gone gets one zero setpoint and its stream ends once the silence
// passes the limit (zero disables it).
class RemoteControlPublisher {
public:
  using PublishFunction =
      std::function<void(const mavros_msgs::PositionTarget &)>;

  static std::shared_ptr<RemoteControlPublisher>
  Create(ros::NodeHandle node_handle, const std::string &topic,
         double altitude_meters, double maximum_linear_velocity_mps,
         double maximum_yaw_rate_rps,
         std::chrono::milliseconds core_silence_limit, std::string *error);

  RemoteControlPublisher(PublishFunction publish, double altitude_meters,
                         double maximum_linear_velocity_mps,
                         double maximum_yaw_rate_rps,
                         std::chrono::milliseconds core_silence_limit =
                             std::chrono::milliseconds::zero());
  ~RemoteControlPublisher();

  bool SetIntent(std::uint32_t gear, std::int32_t longitudinal,
                 std::int32_t lateral, std::int32_t yaw, std::string *error);
  bool Release(std::string *error);
  // Core has had no status subscription for `silent_for`. Ends an active
  // intent once that reaches the limit; returns true when it did.
  bool CoreSilent(std::chrono::milliseconds silent_for);
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
  const std::chrono::milliseconds core_silence_limit_;
  bool active_ = false;
  bool stopped_ = false;
};

} // namespace xgc_px4_multirotor_ros1_adapter
