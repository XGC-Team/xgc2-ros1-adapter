#pragma once

#include "xgc_scene/lidar/engine.hpp"
#include "xgc_scene/ros_outputs.hpp"
#include <functional>
#include <unordered_map>

namespace xgc_scene::lidar_ros {

// One definition is retained solely to pair geometry with its state. There is
// no compiled-scene cache or scan buffer here; those belong to Engine.
class SceneTap final : public RosOutputs {
 public:
  SceneTap(RosOutputs& outputs, lidar::Engine* engine, std::function<std::int64_t()> now)
      : outputs_(outputs), engine_(engine), now_(std::move(now)) {}
  void application(const std::string& epoch, std::uint64_t revision, bool synchronized) override;
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot& message) override;
  void state(const xgc2_geometry_msgs::SceneState& message) override;
  void document(const std::string& text) override { outputs_.document(text); }
  void markers(const visualization_msgs::MarkerArray& message) override { outputs_.markers(message); }
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) override { outputs_.staticTransforms(messages); }
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) override { outputs_.dynamicTransforms(messages); }
  const std::string& error() const { return error_; }

 private:
  void invalidate();
  void install();
  bool synchronized() const;
  RosOutputs& outputs_;
  lidar::Engine* engine_;
  std::function<std::int64_t()> now_;
  std::string epoch_, application_epoch_, error_;
  std::uint64_t revision_ = 0, application_revision_ = 0, installation_ = 0;
  bool have_snapshot_ = false, have_state_ = false, applied_ = false;
  struct Part {
    std::shared_ptr<const xgc2_math::scene::Obstacle> geometry; // shared definition table entry
    xgc2_math::Pose3 pose;
  };
  struct Description {
    std::string id;
    xgc2_math::Pose3 pose;
    std::vector<Part> parts;
  };
  std::vector<Description> descriptions_;
  // Static numeric ground, separate from business identities/dynamic slots.
  std::vector<xgc2_math::scene::Obstacle> ground_shapes_;
  struct Dynamic {
    std::size_t description, state;
  };
  std::vector<Dynamic> dynamic_;
  std::unordered_map<std::string, std::size_t> dynamic_slots_;
  std::vector<std::size_t> state_positions_;
};

}  // namespace xgc_scene::lidar_ros
