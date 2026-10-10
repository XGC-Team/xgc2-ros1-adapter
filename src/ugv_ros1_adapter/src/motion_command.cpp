#include "xgc_ugv_ros1_adapter/motion_command.hpp"

#include <algorithm>
#include <exception>
#include <utility>

namespace xgc_ugv_ros1_adapter {
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

bool motionIntentCommand(const MotionLimits &limits, std::uint32_t gear,
                         std::int32_t longitudinal, std::int32_t lateral,
                         std::int32_t yaw, geometry_msgs::Twist *command,
                         std::string *error) {
  if (command == nullptr)
    return fail(error, "motion command output is required");
  if (gear < 1u || gear > 3u)
    return fail(error, "gear must be between 1 and 3");
  if (longitudinal < -1 || longitudinal > 1)
    return fail(error, "longitudinal intent must be between -1 and 1");
  if (yaw < -1 || yaw > 1)
    return fail(error, "yaw intent must be between -1 and 1");
  if (lateral < -1 || lateral > 1)
    return fail(error, "lateral intent must be between -1 and 1");

  geometry_msgs::Twist candidate;
  const double gear_scale = static_cast<double>(gear) / 3.0;
  candidate.linear.x =
      clamp(static_cast<double>(longitudinal) * limits.maximum_linear_mps *
                gear_scale,
            limits.maximum_linear_mps);
  // A shared controller may carry lateral intent; a non-holonomic chassis
  // ignores that one component.
  if (limits.lateral)
    candidate.linear.y =
        clamp(static_cast<double>(lateral) * limits.maximum_linear_mps *
                  gear_scale,
              limits.maximum_linear_mps);
  candidate.angular.z =
      clamp(static_cast<double>(yaw) * limits.maximum_angular_rps * gear_scale,
            limits.maximum_angular_rps);
  *command = candidate;
  if (error != nullptr)
    error->clear();
  return true;
}

std::shared_ptr<MotionCommandPublisher>
MotionCommandPublisher::Create(ros::NodeHandle node_handle,
                               const std::string &topic,
                               const MotionLimits &limits, std::string *error) {
  if (topic.empty()) {
    fail(error, "motion command topic must not be empty");
    return nullptr;
  }
  try {
    auto publisher = std::make_shared<ros::Publisher>(
        node_handle.advertise<geometry_msgs::Twist>(topic, 5, false));
    if (!*publisher) {
      fail(error, "ROS master did not accept motion command publisher: " +
                      topic);
      return nullptr;
    }
    if (error != nullptr)
      error->clear();
    return std::make_shared<MotionCommandPublisher>(
        [publisher](const geometry_msgs::Twist &command) {
          publisher->publish(command);
        },
        limits);
  } catch (const std::exception &exception) {
    fail(error, "cannot create motion command publisher " + topic + ": " +
                    exception.what());
    return nullptr;
  }
}

MotionCommandPublisher::MotionCommandPublisher(PublishFunction publish,
                                               MotionLimits limits)
    : publish_(std::move(publish)), limits_(limits) {}

MotionCommandPublisher::~MotionCommandPublisher() { Stop(); }

bool MotionCommandPublisher::publishLocked(const geometry_msgs::Twist &command,
                                           std::string *error) noexcept {
  if (!publish_) {
    if (error != nullptr) {
      try {
        *error = "motion command publisher is unavailable";
      } catch (...) {
      }
    }
    return false;
  }
  try {
    publish_(command);
  } catch (const std::exception &exception) {
    if (error != nullptr) {
      try {
        *error = std::string("motion command publication failed: ") +
                 exception.what();
      } catch (...) {
      }
    }
    return false;
  } catch (...) {
    if (error != nullptr) {
      try {
        *error = "motion command publication failed";
      } catch (...) {
      }
    }
    return false;
  }
  if (error != nullptr)
    error->clear();
  return true;
}

bool MotionCommandPublisher::SetIntent(std::uint32_t gear,
                                       std::int32_t longitudinal,
                                       std::int32_t lateral,
                                       std::int32_t yaw,
                                       std::string *error) {
  geometry_msgs::Twist command;
  if (!motionIntentCommand(limits_, gear, longitudinal, lateral, yaw, &command,
                           error))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return fail(error, "motion command publisher is stopped");
  // Publish every state change immediately, including a latched Stop zero.
  // The 10 Hz stream stays up while the remote window is open.
  if (!publishLocked(command, error))
    return false;
  command_ = command;
  active_ = true;
  return true;
}

bool MotionCommandPublisher::Release(std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return fail(error, "motion command publisher is stopped");
  geometry_msgs::Twist stop;
  if (!publishLocked(stop, error))
    return false;
  command_ = stop;
  active_ = false;
  return true;
}

void MotionCommandPublisher::PublishPeriodic() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_ || !active_)
    return;
  publishLocked(command_, nullptr);
}

void MotionCommandPublisher::Stop() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return;
  geometry_msgs::Twist stop;
  command_ = stop;
  if (active_)
    publishLocked(command_, nullptr);
  active_ = false;
  stopped_ = true;
  publish_ = {};
}

} // namespace xgc_ugv_ros1_adapter
