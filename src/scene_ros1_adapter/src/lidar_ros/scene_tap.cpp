#include "xgc_scene/lidar_ros/scene_tap.hpp"
#include "xgc_scene/lidar_ros/cloud.hpp"

#include <algorithm>
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
    engine_->setScene(xgc2_math::scene::toObstacles(descriptions_), installation_);
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
  for (const auto& obstacle : message.obstacles) {
    xgc2_math::scene::SceneObstacleDescription description;
    description.id = obstacle.id;
    description.pose = poseOf(obstacle.pose);
    description.parts.reserve(obstacle.parts.size());
    for (const auto& part : obstacle.parts) {
      xgc2_math::scene::ScenePartDescription converted;
      converted.type = part.geometry.type;
      converted.pose = poseOf(part.pose);
      converted.size = {part.geometry.size.x, part.geometry.size.y, part.geometry.size.z};
      converted.radius = part.geometry.radius;
      converted.height = part.geometry.height;
      if (converted.type == "ring") {
        converted.radii = {part.geometry.radii[0], part.geometry.radii[1]};
        converted.tubeRadius = part.geometry.tube_radius;
      }
      converted.vertices.reserve(part.geometry.vertices.size());
      for (const auto& vertex : part.geometry.vertices)
        converted.vertices.emplace_back(vertex.x, vertex.y, vertex.z);
      description.parts.push_back(std::move(converted));
    }
    descriptions_.push_back(std::move(description));
    dynamic_.push_back(obstacle.dynamic);
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
    auto& description = descriptions_[i];
    const auto found = std::find_if(message.obstacles.begin(), message.obstacles.end(),
        [&](const auto& obstacle) { return obstacle.id == description.id; });
    if (found == message.obstacles.end()) {
      if (!dynamic_[i]) continue;
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
