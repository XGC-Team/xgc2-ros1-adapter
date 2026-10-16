#include "xgc_scene/lidar_ros/scene_tap.hpp"
#include "xgc_scene/lidar_ros/cloud.hpp"

#include <algorithm>
#include <cmath>
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
      (!have_state_ && !dynamic_.empty())) return;
  try {
    std::vector<xgc2_math::scene::Obstacle> obstacles = ground_shapes_;
    for (const auto& description : descriptions_) {
      const auto parent = xgc2_math::normalizedQuaternion(description.pose.orientation);
      for (const auto& part : description.parts) {
        auto shape = *part.geometry;
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
  ground_shapes_.clear();
  dynamic_.clear();
  dynamic_slots_.clear();
  state_positions_.clear();
  epoch_ = message.epoch;
  revision_ = message.revision;
  if (message.header.frame_id != "world") {
    error_ = "snapshot frame must be world";
    return;
  }
  descriptions_.reserve(message.obstacles.size());
  const auto moving = std::count_if(message.obstacles.begin(), message.obstacles.end(),
      [](const auto& obstacle) { return obstacle.dynamic; });
  dynamic_.reserve(moving);
  dynamic_slots_.reserve(moving);
  state_positions_.reserve(moving);
  try {
  std::vector<std::shared_ptr<const xgc2_math::scene::Obstacle>> geometries;
  geometries.reserve(message.geometries.size());
  for (const auto& geometry : message.geometries)
    geometries.push_back(std::make_shared<const xgc2_math::scene::Obstacle>(
        xgc2_math::scene::obstacleFromGeometry({geometry.type, geometry.parameters})));
  const auto& ground = message.ground;
  if (ground.mode != xgc2_geometry_msgs::SceneGround::MODE_MODEL && !ground.parts.empty())
    throw std::invalid_argument("non-model ground must not have parts");
  if (ground.mode == xgc2_geometry_msgs::SceneGround::MODE_MODEL) {
    if (ground.parts.empty()) throw std::invalid_argument("ground model has no parts");
    const auto placement = poseOf(ground.pose);
    const auto parent = xgc2_math::normalizedQuaternion(placement.orientation);
    ground_shapes_.reserve(ground.parts.size());
    for (const auto& part : ground.parts) {
      if (part.geometry_index >= geometries.size())
        throw std::invalid_argument("ground geometry index out of range");
      const auto local = poseOf(part.pose);
      auto shape = *geometries[part.geometry_index];
      shape.position = placement.position + parent * local.position;
      shape.orientation = parent * local.orientation;
      ground_shapes_.push_back(std::move(shape));
    }
  } else if (ground.mode == xgc2_geometry_msgs::SceneGround::MODE_DEFAULT_PLANE) {
    if (!std::isfinite(ground.z)) throw std::invalid_argument("ground z must be finite");
    // The existing numeric query/sampling path does not synthesize an infinite plane.
  } else if (ground.mode != xgc2_geometry_msgs::SceneGround::MODE_NONE) {
    throw std::invalid_argument("unsupported ground mode: " + ground.mode);
  }
  for (const auto& obstacle : message.obstacles) {
    Description description;
    description.id = obstacle.id;
    description.pose = poseOf(obstacle.pose);
    description.parts.reserve(obstacle.parts.size());
    for (const auto& part : obstacle.parts) {
      Part converted;
      if (part.geometry_index >= geometries.size())
        throw std::invalid_argument("snapshot geometry index out of range");
      converted.geometry = geometries[part.geometry_index];
      converted.pose = poseOf(part.pose);
      description.parts.push_back(std::move(converted));
    }
    if (obstacle.dynamic) {
      const auto slot = dynamic_slots_.emplace(obstacle.id, state_positions_.size());
      if (slot.second) state_positions_.push_back(0);
      dynamic_.push_back({descriptions_.size(), slot.first->second});
    }
    descriptions_.push_back(std::move(description));
  }
  } catch (const std::exception& error) {
    descriptions_.clear();
    ground_shapes_.clear();
    dynamic_.clear();
    dynamic_slots_.clear();
    state_positions_.clear();
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
  if (dynamic_.empty()) {
    have_state_ = true;
    if (!engine_->sceneReady()) install();
    return;
  }
  bool changed = false;
  const auto missing = message.obstacles.size();
  std::fill(state_positions_.begin(), state_positions_.end(), missing);
  // Keep the first state for each ID; inspect required slots in definition
  // order so missing-state failures retain their original ordering.
  for (std::size_t i = 0; i < message.obstacles.size(); ++i) {
    const auto slot = dynamic_slots_.find(message.obstacles[i].id);
    if (slot != dynamic_slots_.end() && state_positions_[slot->second] == missing)
      state_positions_[slot->second] = i;
  }
  for (const auto& moving : dynamic_) {
    auto& description = descriptions_[moving.description];
    const auto position = state_positions_[moving.state];
    if (position == missing) {
      have_state_ = false;
      invalidate();
      error_ = "state lacks dynamic obstacle '" + description.id + "'";
      return;
    }
    const auto pose = poseOf(message.obstacles[position].pose);
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
