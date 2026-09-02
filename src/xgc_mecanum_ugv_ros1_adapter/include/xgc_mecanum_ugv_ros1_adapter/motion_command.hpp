#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace xgc_mecanum_ugv_ros1_adapter {

// Wheeltec Mecanum chassis linear capability. These are not Scout SDK limits
// and not the old swarm-sync-sim 1.5 m/s remote scale.
constexpr double kMecanumMaximumLinearVelocityMetersPerSecond = 1.0;
constexpr double kMecanumMaximumAngularVelocityRadiansPerSecond =
    1.5707963267948966;

bool motionIntentCommand(std::uint32_t gear, std::int32_t longitudinal,
                         std::int32_t lateral, std::int32_t yaw,
                         geometry_msgs::Twist *command,
                         std::string *error);

// Holds the last discrete operator intent and republishes a non-zero Twist
// locally at 10 Hz. A zero intent publishes once and releases the stream so
// close/Stop does not keep /cmd_vel alive. Stop() while the stream is still
// active serializes a final zero; after an idle zero it does not publish
// again.
class MotionCommandPublisher {
public:
  using PublishFunction =
      std::function<void(const geometry_msgs::Twist &command)>;

  static std::shared_ptr<MotionCommandPublisher>
  Create(ros::NodeHandle node_handle, const std::string &topic,
         std::string *error);

  explicit MotionCommandPublisher(PublishFunction publish);
  ~MotionCommandPublisher();

  MotionCommandPublisher(const MotionCommandPublisher &) = delete;
  MotionCommandPublisher &operator=(const MotionCommandPublisher &) = delete;

  bool SetIntent(std::uint32_t gear, std::int32_t longitudinal,
                 std::int32_t lateral, std::int32_t yaw, std::string *error);
  void PublishPeriodic();
  void Stop() noexcept;

private:
  bool publishLocked(const geometry_msgs::Twist &command,
                     std::string *error) noexcept;

  std::mutex mutex_;
  PublishFunction publish_;
  geometry_msgs::Twist command_;
  bool active_ = false;
  bool stopped_ = false;
};

} // namespace xgc_mecanum_ugv_ros1_adapter
