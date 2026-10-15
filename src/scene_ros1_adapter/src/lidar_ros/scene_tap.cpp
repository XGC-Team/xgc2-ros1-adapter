#include "xgc_scene/lidar_ros/scene_tap.hpp"
#include "xgc_scene/lidar_ros/cloud.hpp"

#include <algorithm>
#include <xgc2_math/scene/scene_conversion.hpp>
#include <ros/console.h>

namespace xgc_scene::lidar_ros {
void SceneTap::invalidate() {
  // Engine revisions are monotonic installations, including dynamic state
  // changes and new epochs whose document revision may restart at one.
  engine_->invalidateScene(++installation_);
  // Clear the reference immediately on this lane. The scene is unready,
  // so this tick cannot schedule a scan.
  engine_->tick(now_());
}
bool SceneTap::synchronized() const {
  return applied_ && application_epoch_ == epoch_ && application_revision_ == revision_;
}
void SceneTap::install() {
  if (!have_snapshot_ || !synchronized() ||
      (!have_state_ && std::find(dynamic_.begin(), dynamic_.end(), true) != dynamic_.end())) return;
  try {
    std::vector<xgc2_math::scene::Obstacle> obstacles;
    for (const auto& description : descriptions_) {
      const auto parent = xgc2_math::normalizedQuaternion(description.pose.orientation);
      for (const auto& part : description.parts) {
        auto shape = part.geometry;
        shape.position = description.pose.position + parent * part.pose.position;
        shape.orientation = parent * part.pose.orientation;
        obstacles.push_back(std::move(shape));
      }
    }
    engine_->setScene(obstacles, installation_);
    error_.clear();
  } catch (const std::exception& error) {
    engine_->invalidateScene(installation_);
    error_ = error.what();
    ROS_ERROR_STREAM("Lidar scene refused: " << error_);
  }
}

void SceneTap::application(const std::string& epoch, std::uint64_t revision, bool synchronized_value) {
  outputs_.application(epoch, revision, synchronized_value);
  if (!engine_) return;
  const bool changed = epoch != application_epoch_ || revision != application_revision_ || applied_ != synchronized_value;
  application_epoch_ = epoch;
  application_revision_ = revision;
  applied_ = synchronized_value;
  if (changed || !applied_) invalidate();
  if (applied_ && !engine_->sceneReady()) install();
}

void SceneTap::snapshot(const xgc2_geometry_msgs::SceneSnapshot& message) {
  outputs_.snapshot(message);
  if (!engine_) return;
  if (have_snapshot_ && message.epoch == epoch_ && message.revision <= revision_) return;
  invalidate();
  have_snapshot_ = false;
  have_state_ = false;
  descriptions_.clear();
  dynamic_.clear();
  epoch_ = message.epoch;
  revision_ = message.revision;
  if (message.header.frame_id != "world") {
    error_ = "snapshot frame must be world";
    return;
  }
  descriptions_.reserve(message.obstacles.size());
  dynamic_.reserve(message.obstacles.size());
  try {
  for (const auto& obstacle : message.obstacles) {
    Description description;
    description.id = obstacle.id;
    description.pose = poseOf(obstacle.pose);
    description.parts.reserve(obstacle.parts.size());
    for (const auto& part : obstacle.parts) {
      Part converted;
      converted.geometry = xgc2_math::scene::obstacleFromGeometry({part.geometry.type, part.geometry.parameters});
      converted.pose = poseOf(part.pose);
      description.parts.push_back(std::move(converted));
    }
    descriptions_.push_back(std::move(description));
    dynamic_.push_back(obstacle.dynamic);
  }
  } catch (const std::exception& error) {
    descriptions_.clear();
    dynamic_.clear();
    error_ = std::string("snapshot geometry: ") + error.what();
    return;
  }
  have_snapshot_ = true;
  install();
}

void SceneTap::state(const xgc2_geometry_msgs::SceneState& message) {
  outputs_.state(message);
  if (!engine_ || !have_snapshot_ || message.epoch != epoch_ || message.revision != revision_) return;
  if (message.header.frame_id != "world") {
    have_state_ = false;
    invalidate();
    error_ = "state frame must be world";
    return;
  }
  bool changed = false;
  for (std::size_t i = 0; i < descriptions_.size(); ++i) {
    if (!dynamic_[i]) continue;
    auto& description = descriptions_[i];
    const auto found = std::find_if(message.obstacles.begin(), message.obstacles.end(),
        [&](const auto& obstacle) { return obstacle.id == description.id; });
    if (found == message.obstacles.end()) {
      have_state_ = false;
      invalidate();
      error_ = "state lacks dynamic obstacle '" + description.id + "'";
      return;
    }
    const auto pose = poseOf(found->pose);
    if (pose.position != description.pose.position || pose.orientation.coeffs() != description.pose.orientation.coeffs()) {
      description.pose = pose;
      changed = true;
    }
  }
  have_state_ = true;
  if (changed) invalidate();
  if (!engine_->sceneReady()) install();
}
}  // namespace xgc_scene::lidar_ros
