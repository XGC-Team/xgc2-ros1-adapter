#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace xgc_mecanum_ugv_ros1_adapter {

// Preserve the deployed swarm-sync-sim Mecanum contract. These are not Scout
// chassis limits.
constexpr double kMecanumMaximumLinearVelocityMetersPerSecond = 1.5;
constexpr double kMecanumMaximumAngularVelocityRadiansPerSecond =
    1.5707963267948966;

bool motionIntentCommand(std::uint32_t gear, std::int32_t longitudinal,
                         std::int32_t yaw, geometry_msgs::Twist *command,
                         std::string *error);

// Holds the last discrete operator intent and republishes its Twist locally.
// All publication is serialized with Stop(), so no stale non-zero command can
// be published after the final zero command.
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
                 std::int32_t yaw, std::string *error);
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
