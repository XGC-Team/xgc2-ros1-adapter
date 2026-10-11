#pragma once

#include "xgc_scene/lidar_ros/cloud.hpp"
#include "xgc_scene/lidar_ros/parameters.hpp"
#include <ros/node_handle.h>
#include <atomic>

namespace xgc_scene::lidar_ros {

class Edge final : public lidar::Sink {
 public:
  Edge(ros::NodeHandle& handle, const Parameters& parameters);
  ~Edge() override;
  lidar::Engine* engine() { return engine_.get(); }
  void setEnabled(bool enabled);
  void stopTimer();
  void shutdown();
  Json peerFacts() const { return {{"scene_ready", scene_ready_.load()}}; }
  void onPoints(std::size_t sensor, lidar::CloudView cloud) override;
  void onBeams(std::size_t sensor, lidar::CloudView cloud) override;
  void onReference(lidar::CloudView cloud, lidar::ReferenceAction action) override;

 private:
  struct Sensor {
    ros::Publisher points, beams;
    ros::Subscriber pose;
    sensor_msgs::PointCloud2 points_message = cloudLayout(), beams_message = cloudLayout(true);
    Demand demand;
  };
  std::vector<Sensor> sensors_;
  std::vector<ros::Subscriber> bodies_;
  ros::Publisher reference_;
  sensor_msgs::PointCloud2 reference_message_ = cloudLayout();
  std::size_t reference_demand_ = 0;
  ros::Timer timer_;
  std::atomic<bool> scene_ready_{false};
  // Explicit shutdown joins the pool before any ROS handles or messages die.
  std::unique_ptr<lidar::Engine> engine_;
};

}  // namespace xgc_scene::lidar_ros
