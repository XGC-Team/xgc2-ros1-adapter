#include "xgc_scene/ros_outputs.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace xgc_scene {

class RosPublishers::Impl {
 public:
  explicit Impl(ros::NodeHandle& node)
      : snapshot(node.advertise<xgc2_geometry_msgs::SceneSnapshot>("snapshot", 1, true)),
        state(node.advertise<xgc2_geometry_msgs::SceneState>("state", 1)),
        document(node.advertise<std_msgs::String>("document", 1, true)),
        markers(node.advertise<visualization_msgs::MarkerArray>("markers", 1, true)) {}
  ros::Publisher snapshot, state, document, markers;
  tf2_ros::StaticTransformBroadcaster fixed;
  tf2_ros::TransformBroadcaster moving;
};

RosPublishers::RosPublishers(ros::NodeHandle& node) : impl_(new Impl(node)) {}
RosPublishers::~RosPublishers() = default;
void RosPublishers::snapshot(const xgc2_geometry_msgs::SceneSnapshot& message) { impl_->snapshot.publish(message); }
void RosPublishers::state(const xgc2_geometry_msgs::SceneState& message) { impl_->state.publish(message); }
void RosPublishers::document(const std::string& text) {
  std_msgs::String message;
  message.data = text;
  impl_->document.publish(message);
}
void RosPublishers::markers(const visualization_msgs::MarkerArray& message) { impl_->markers.publish(message); }
void RosPublishers::staticTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) {
  impl_->fixed.sendTransform(messages);
}
void RosPublishers::dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>& messages) {
  impl_->moving.sendTransform(messages);
}

}  // namespace xgc_scene
