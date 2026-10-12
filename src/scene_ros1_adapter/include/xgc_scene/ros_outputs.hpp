#pragma once

#include "xgc_scene/common.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <memory>
#include <string>
#include <vector>

namespace ros { class NodeHandle; }

namespace xgc_scene {

class RosOutputs {
 public:
  virtual ~RosOutputs() = default;
  // Native application boundary, before any new definition/state is visible.
  // Pending, failed or mismatched application must invalidate fleet truth.
  virtual void application(const std::string&, std::uint64_t, bool) {}
  virtual void snapshot(const xgc2_geometry_msgs::SceneSnapshot& message) = 0;
  virtual void state(const xgc2_geometry_msgs::SceneState& message) = 0;
  virtual void document(const std::string& text) = 0;
  virtual void markers(const visualization_msgs::MarkerArray& message) = 0;
  virtual void staticTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) = 0;
  virtual void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) = 0;
};

// Only this implementation calls roscpp publishers and TF broadcasters.
class RosPublishers final : public RosOutputs {
 public:
  explicit RosPublishers(ros::NodeHandle& node);
  ~RosPublishers() override;
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot& message) override;
  void state(const xgc2_geometry_msgs::SceneState& message) override;
  void document(const std::string& text) override;
  void markers(const visualization_msgs::MarkerArray& message) override;
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) override;
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) override;
  void fixedCloud(const sensor_msgs::PointCloud2& message);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

xgc2_geometry_msgs::SceneObstacle obstacleMessage(const Json& item);
xgc2_geometry_msgs::SceneObstacleState stateMessage(const Json& value);
std::vector<visualization_msgs::Marker> partMarkers(
    const std::string& obstacle_id, const xgc2_geometry_msgs::ScenePart& part, const std::string& frame);

// Value caches replace Python's immutable-object identity caches: the store
// returns JSON copies. Revisions gate scans; unchanged obstacle messages and
// markers survive a revision. Hold states never run through motion per tick.
class SceneProjection {
 public:
  struct Counters {
    std::size_t obstacle_messages = 0, part_markers = 0, hold_states = 0, moving_states = 0;
    std::size_t frame_scans = 0, snapshot_scans = 0, definition_scans = 0, state_scans = 0;
  };
  explicit SceneProjection(const std::string& name_space);
  ~SceneProjection();
  xgc2_geometry_msgs::SceneSnapshot snapshot(const Json& document, const std::string& epoch,
                                           std::uint64_t revision, double stamp);
  void definition(const Json& document, const std::string& epoch, std::uint64_t revision,
                  const xgc2_geometry_msgs::SceneSnapshot& snapshot, RosOutputs& outputs);
  void tick(double stamp, double elapsed, bool playing, RosOutputs& outputs);
  const Counters& counters() const;
  const std::string& frame(const std::string& obstacle_id) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc_scene
