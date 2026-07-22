#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace xgc_mecanum_ugv_ros1_adapter {

constexpr double kMecanumMaximumLinearVelocityMetersPerSecond = 1.5;
constexpr double kMecanumMaximumAngularVelocityRadiansPerSecond =
    1.5707963267948966;

bool motionIntentCommand(std::uint32_t gear, std::int32_t longitudinal,
                         std::int32_t yaw, geometry_msgs::Twist *command,
                         std::string *error);

// Holds one workflow-authorized discrete operator intent and republishes its
// Twist locally until its lease expires. The public operation is the only way
// to create or change an intent. Internal lease pulses may only extend the
// exact owner/command already installed here.
//
// Every mutation and publication is serialized. A generation fence makes a
// late cancellation/release harmless after a newer pulse or intent wins.
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

  bool SetIntent(const std::string &owner, std::uint32_t gear,
                 std::int32_t longitudinal, std::int32_t yaw,
                 const ros::WallTime &expires_at, std::uint64_t *generation,
                 std::string *error);
  bool RenewIntent(const std::string &owner, std::uint32_t gear,
                   std::int32_t longitudinal, std::int32_t yaw,
                   const ros::WallTime &expires_at,
                   std::uint64_t *generation, std::string *error);
  void Release(const std::string &owner, std::uint64_t generation) noexcept;
  void PublishPeriodic(const ros::WallTime &now = ros::WallTime::now());
  void Stop() noexcept;

private:
  bool publishLocked(const geometry_msgs::Twist &command,
                     std::string *error) noexcept;
  void publishZeroAndClearLocked() noexcept;
  bool nextGenerationLocked(std::uint64_t *generation,
                            std::string *error);

  std::mutex mutex_;
  PublishFunction publish_;
  geometry_msgs::Twist command_;
  std::string owner_;
  ros::WallTime expires_at_;
  std::uint64_t generation_ = 0;
  bool active_ = false;
  bool stopped_ = false;
};

} // namespace xgc_mecanum_ugv_ros1_adapter
