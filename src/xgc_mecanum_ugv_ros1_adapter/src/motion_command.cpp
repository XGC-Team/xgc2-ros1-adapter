#include "xgc_mecanum_ugv_ros1_adapter/motion_command.hpp"

#include <algorithm>
#include <exception>
#include <limits>
#include <utility>

namespace xgc_mecanum_ugv_ros1_adapter {
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

bool motionIntentCommand(std::uint32_t gear, std::int32_t longitudinal,
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

  geometry_msgs::Twist candidate;
  const double gear_scale = static_cast<double>(gear) / 3.0;
  candidate.linear.x = clamp(
      static_cast<double>(longitudinal) *
          kMecanumMaximumLinearVelocityMetersPerSecond * gear_scale,
      kMecanumMaximumLinearVelocityMetersPerSecond);
  candidate.angular.z = clamp(
      static_cast<double>(yaw) *
          kMecanumMaximumAngularVelocityRadiansPerSecond * gear_scale,
      kMecanumMaximumAngularVelocityRadiansPerSecond);
  *command = candidate;
  if (error != nullptr)
    error->clear();
  return true;
}

std::shared_ptr<MotionCommandPublisher>
MotionCommandPublisher::Create(ros::NodeHandle node_handle,
                               const std::string &topic, std::string *error) {
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
        });
  } catch (const std::exception &exception) {
    fail(error, "cannot create motion command publisher " + topic + ": " +
                    exception.what());
    return nullptr;
  }
}

MotionCommandPublisher::MotionCommandPublisher(PublishFunction publish)
    : publish_(std::move(publish)) {}

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

bool MotionCommandPublisher::nextGenerationLocked(std::uint64_t *generation,
                                                  std::string *error) {
  if (generation_ == std::numeric_limits<std::uint64_t>::max())
    return fail(error, "motion command lease generation is exhausted");
  ++generation_;
  if (generation != nullptr)
    *generation = generation_;
  return true;
}

void MotionCommandPublisher::publishZeroAndClearLocked() noexcept {
  geometry_msgs::Twist stop;
  command_ = stop;
  publishLocked(stop, nullptr);
  owner_.clear();
  expires_at_ = ros::WallTime();
  active_ = false;
}

bool MotionCommandPublisher::SetIntent(const std::string &owner,
                                       std::uint32_t gear,
                                       std::int32_t longitudinal,
                                       std::int32_t yaw,
                                       const ros::WallTime &expires_at,
                                       std::uint64_t *generation,
                                       std::string *error) {
  geometry_msgs::Twist command;
  if (!motionIntentCommand(gear, longitudinal, yaw, &command, error))
    return false;
  if (owner.empty())
    return fail(error, "motion command lease owner is required");
  if (expires_at.isZero() || expires_at <= ros::WallTime::now())
    return fail(error, "motion command lease deadline has elapsed");

  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return fail(error, "motion command publisher is stopped");
  if (generation_ == std::numeric_limits<std::uint64_t>::max())
    return fail(error, "motion command lease generation is exhausted");
  const std::uint64_t candidate_generation = generation_ + 1u;
  // Do not advance the lease fence until native publication succeeds. A
  // failed replacement must leave the previous command releasable.
  if (!publishLocked(command, error))
    return false;
  generation_ = candidate_generation;
  if (generation != nullptr)
    *generation = generation_;
  command_ = command;
  const bool inactive = command.linear.x == 0.0 && command.angular.z == 0.0;
  if (inactive) {
    owner_.clear();
    expires_at_ = ros::WallTime();
    active_ = false;
  } else {
    owner_ = owner;
    expires_at_ = expires_at;
    active_ = true;
  }
  return true;
}

bool MotionCommandPublisher::RenewIntent(const std::string &owner,
                                         std::uint32_t gear,
                                         std::int32_t longitudinal,
                                         std::int32_t yaw,
                                         const ros::WallTime &expires_at,
                                         std::uint64_t *generation,
                                         std::string *error) {
  geometry_msgs::Twist command;
  if (!motionIntentCommand(gear, longitudinal, yaw, &command, error))
    return false;
  if (owner.empty())
    return fail(error, "motion command lease owner is required");
  if (expires_at.isZero() || expires_at <= ros::WallTime::now())
    return fail(error, "motion command lease deadline has elapsed");

  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return fail(error, "motion command publisher is stopped");
  if (!active_ || owner_ != owner)
    return fail(error, "motion command lease owner is not active");
  if (command.linear.x != command_.linear.x ||
      command.linear.y != command_.linear.y ||
      command.linear.z != command_.linear.z ||
      command.angular.x != command_.angular.x ||
      command.angular.y != command_.angular.y ||
      command.angular.z != command_.angular.z) {
    return fail(error, "motion command lease pulse cannot change intent");
  }
  if (expires_at <= expires_at_)
    return fail(error, "motion command lease pulse is stale");
  if (!nextGenerationLocked(generation, error))
    return false;
  expires_at_ = expires_at;
  if (error != nullptr)
    error->clear();
  return true;
}

void MotionCommandPublisher::Release(const std::string &owner,
                                     std::uint64_t generation) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_ || !active_ || owner_ != owner || generation_ != generation)
    return;
  publishZeroAndClearLocked();
}

void MotionCommandPublisher::PublishPeriodic(const ros::WallTime &now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_ || !active_)
    return;
  if (expires_at_.isZero() || expires_at_ <= now) {
    publishZeroAndClearLocked();
    return;
  }
  publishLocked(command_, nullptr);
}

void MotionCommandPublisher::Stop() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_)
    return;
  // Before the first accepted intent cmd_vel remains genuinely absent. An
  // active lease, however, is always fenced with one final zero.
  if (active_)
    publishZeroAndClearLocked();
  stopped_ = true;
  publish_ = {};
}

} // namespace xgc_mecanum_ugv_ros1_adapter
