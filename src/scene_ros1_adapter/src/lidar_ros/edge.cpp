#include "xgc_scene/lidar_ros/edge.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <ros/advertise_options.h>
#include <algorithm>

namespace xgc_scene::lidar_ros {
Edge::Edge(ros::NodeHandle& handle, const Parameters& parameters) {
  if (!parameters.have_fleet) return;
  engine_ = std::make_unique<lidar::Engine>(parameters.manifest, *this, parameters.options);
  engine_->setEnabled(parameters.enabled);
  const auto& manifest = engine_->manifest();
  sensors_.resize(manifest.robots.size());
  double rate = 1.0;  // A reference-only fleet still delivers reference actions.
  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    const auto& spec = manifest.robots[i];
    auto& sensor = sensors_[i];
    const auto advertise = [&](bool beams) {
      auto options = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
          spec.name_space + (beams ? "/world_lidar/beams" : "/world_lidar/points"), 1,
          [this, i, beams](const ros::SingleSubscriberPublisher&) {
            engine_->setDemand(i, sensors_[i].demand.change(beams, true));
          },
          [this, i, beams](const ros::SingleSubscriberPublisher&) {
            engine_->setDemand(i, sensors_[i].demand.change(beams, false));
          }, ros::VoidConstPtr(), handle.getCallbackQueue());
      return handle.advertise(options);
    };
    sensor.points = advertise(false);
    if (spec.publish_beams) sensor.beams = advertise(true);
    sensor.pose = handle.subscribe<geometry_msgs::PoseStamped>(spec.pose_topic, 1,
        [this, i](const geometry_msgs::PoseStamped::ConstPtr& message) {
          engine_->setPose(i, poseOf(message->pose), message->header.stamp.toNSec(), message->header.frame_id);
        });
    rate = i == 0 ? spec.rate_hz : std::max(rate, spec.rate_hz);
  }
  bodies_.reserve(manifest.bodies.size());
  for (std::size_t i = 0; i < manifest.bodies.size(); ++i)
    bodies_.push_back(handle.subscribe<geometry_msgs::PoseStamped>(manifest.bodies[i].pose_topic, 1,
        [this, i](const geometry_msgs::PoseStamped::ConstPtr& message) {
          engine_->setBodyPose(i, poseOf(message->pose), message->header.stamp.toNSec(), message->header.frame_id);
        }));
  if (manifest.reference) {
    auto options = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>("/xgc/scene/reference_cloud", 1,
        [this](const ros::SingleSubscriberPublisher&) { engine_->setReferenceDemand(++reference_demand_ > 0); },
        [this](const ros::SingleSubscriberPublisher&) {
          if (reference_demand_) --reference_demand_;
          engine_->setReferenceDemand(reference_demand_ > 0);
        }, ros::VoidConstPtr(), handle.getCallbackQueue());
    options.latch = true;
    reference_ = handle.advertise(options);
  }
  timer_ = handle.createTimer(ros::Duration(1.0 / std::min(rate, 1000.0)),
      [this](const ros::TimerEvent&) { engine_->tick(ros::Time::now().toNSec()); });
}

Edge::~Edge() { shutdown(); }
void Edge::setEnabled(bool enabled) { if (engine_) engine_->setEnabled(enabled); }
void Edge::stopTimer() { timer_.stop(); }
void Edge::shutdown() {
  stopTimer();
  engine_.reset();
  for (auto& sensor : sensors_) {
    sensor.pose.shutdown();
    sensor.points.shutdown();
    sensor.beams.shutdown();
  }
  for (auto& body : bodies_) body.shutdown();
  reference_.shutdown();
}

void Edge::onPoints(std::size_t sensor, lidar::CloudView cloud) {
  auto& output = sensors_[sensor];
  fillCloud(output.points_message, cloud);
  // const-message publish serializes synchronously, including intraprocess
  // subscriptions. Never pass a shared pointer to these reused messages.
  output.points.publish(output.points_message);
}
void Edge::onBeams(std::size_t sensor, lidar::CloudView cloud) {
  auto& output = sensors_[sensor];
  fillCloud(output.beams_message, cloud);
  output.beams.publish(output.beams_message);
}
void Edge::onReference(lidar::CloudView cloud, lidar::ReferenceAction) {
  fillCloud(reference_message_, cloud);
  reference_.publish(reference_message_);
}
}  // namespace xgc_scene::lidar_ros
