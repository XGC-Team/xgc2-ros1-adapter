#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "xgc_ugv_ros1_adapter/variant.hpp"

namespace xgc_ugv_ros1_adapter {

// Maps one discrete operator intent (gear 1-3, each axis -1..1) to a velocity
// command of this chassis, scaled by gear and clamped to its limits.
bool motionIntentCommand(const MotionLimits &limits, std::uint32_t gear,
                         std::int32_t longitudinal, std::int32_t lateral,
                         std::int32_t yaw, geometry_msgs::Twist *command,
                         std::string *error);

// Holds the last discrete operator intent and republishes it locally at 10 Hz,
// including a zero Twist while the remote window is open and latched on Stop.
// Release() publishes one last zero and ends the stream so close / leaving the
// Experiment does not keep /cmd_vel alive. Stop() while the stream is still
// active serializes a final zero; after Release() it does not publish again.
// The ROS publisher lives until the destructor: roscpp sends a message from
// its transport thread, so the owner flushes publications before it destroys
// the publisher that carried the final zero.
class MotionCommandPublisher {
public:
  using PublishFunction =
      std::function<void(const geometry_msgs::Twist &command)>;

  static std::shared_ptr<MotionCommandPublisher>
  Create(ros::NodeHandle node_handle, const std::string &topic,
         const MotionLimits &limits, std::string *error);

  MotionCommandPublisher(PublishFunction publish, MotionLimits limits);
  ~MotionCommandPublisher();

  MotionCommandPublisher(const MotionCommandPublisher &) = delete;
  MotionCommandPublisher &operator=(const MotionCommandPublisher &) = delete;

  bool SetIntent(std::uint32_t gear, std::int32_t longitudinal,
                 std::int32_t lateral, std::int32_t yaw, std::string *error);
  bool Release(std::string *error);
  void PublishPeriodic();
  void Stop() noexcept;

private:
  bool publishLocked(const geometry_msgs::Twist &command,
                     std::string *error) noexcept;

  std::mutex mutex_;
  PublishFunction publish_;
  const MotionLimits limits_;
  geometry_msgs::Twist command_;
  bool active_ = false;
  bool stopped_ = false;
};

} // namespace xgc_ugv_ros1_adapter
