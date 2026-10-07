// Private benchmark source: one roscpp node publishes all robots. A Python
// source's per-TCP connection workers/backlog would distort this load test.
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 6 && argc != 8 && argc != 9) return 2;
  const std::string kind = argv[1], report = argv[3], ready = argv[4], start = argv[5];
  const int count = std::stoi(argv[2]);
  const bool direct = argc >= 8 && std::string(argv[6]) == "direct";
  const double rate = argc >= 8 ? std::stod(argv[7]) : 30.0;
  const int burst_per_tick = argc == 9 ? std::stoi(argv[8]) : 0;
  if (rate <= 0 || count <= 0 || burst_per_tick < 0) return 2;
  ros::init(argc, argv, "private_forwarding_source", ros::init_options::AnonymousName);
  ros::NodeHandle node;
  struct Sources { ros::Publisher pose, twist, accel, imu; };
  std::vector<Sources> sources; sources.reserve(count);
  for (int i = 0; i < count; ++i) {
    const auto root = (direct ? "/" : "/vrpn_client_node/") + kind + std::to_string(i);
    // Buffer the hot member's whole batch at the source. Otherwise a tiny
    // publisher queue would discard the burst before it reaches the Adapter.
    const int queue = i == 0 && burst_per_tick ? burst_per_tick + 20 : 20;
    const auto imu_topic = "/" + kind + std::to_string(i) +
      (kind == "px4" ? "/mavros/imu/data" : kind == "scout" ? "/imu/data_raw" : "/imu");
    sources.push_back({node.advertise<geometry_msgs::PoseStamped>(root + "/pose", queue),
      node.advertise<geometry_msgs::TwistStamped>(root + "/twist", queue),
      direct ? ros::Publisher() : node.advertise<geometry_msgs::AccelStamped>(root + "/accel", queue),
      direct ? node.advertise<sensor_msgs::Imu>(imu_topic, 20) : ros::Publisher()});
  }
  const auto until = ros::WallTime::now() + ros::WallDuration(45);
  for (;;) {
    bool connected = true;
    for (const auto &source : sources) connected = connected && source.pose.getNumSubscribers() && source.twist.getNumSubscribers() &&
      (direct ? source.imu.getNumSubscribers() : source.accel.getNumSubscribers());
    if (connected) { std::ofstream(ready) << "ready\n"; break; }
    if (!ros::ok() || ros::WallTime::now() > until) return 3;
    ros::WallDuration(.01).sleep();
  }
  while (!std::ifstream(start).good()) {
    if (!ros::ok() || ros::WallTime::now() > until) return 4;
    ros::WallDuration(.01).sleep();
  }
  geometry_msgs::PoseStamped pose; pose.header.frame_id = "source-world";
  pose.pose.position.x = 3; pose.pose.position.y = 4; pose.pose.position.z = 5; pose.pose.orientation.w = 1;
  geometry_msgs::TwistStamped twist; twist.header.frame_id = "source-world"; twist.twist.linear.x = 1; twist.twist.angular.z = .5;
  geometry_msgs::AccelStamped accel; accel.header.frame_id = "source-world"; accel.accel.linear.x = 2; accel.accel.angular.z = .25;
  sensor_msgs::Imu imu; imu.header.frame_id = "source-body"; imu.orientation.w = 1;
  imu.angular_velocity.z = .5; imu.linear_acceleration.z = 9.80665;
  for (int i = 0; i < 9; ++i) {
    imu.orientation_covariance[i] = (i % 4 == 0) ? .001 : 0;
    imu.angular_velocity_covariance[i] = (i % 4 == 0) ? .002 : 0;
    imu.linear_acceleration_covariance[i] = (i % 4 == 0) ? .003 : 0;
  }
  const auto began = ros::WallTime::now();
  const int ticks = int(3 * rate);
  int burst_samples = 0;
  int imu_samples = 0;
  bool burst_started = false, recovered = false;
  auto publish = [&](const Sources &source) {
    pose.header.stamp = ros::Time::now(); source.pose.publish(pose);
    twist.header.stamp = ros::Time::now(); source.twist.publish(twist);
    if (!direct) { accel.header.stamp = ros::Time::now(); source.accel.publish(accel); }
  };
  for (int tick = 0; tick < ticks; ++tick) {
    for (const auto &source : sources) publish(source);
    if (direct && int((tick + 1) * 30 / rate) > int(tick * 30 / rate)) {
      for (const auto &source : sources) {
        imu.header.stamp = ros::Time::now(); source.imu.publish(imu); ++imu_samples;
      }
    }
    if (burst_per_tick && tick >= int(rate * .5) && tick < int(rate * 2.5)) {
      if (!burst_started) { std::ofstream(report + ".burst") << "burst\n"; burst_started = true; }
      for (int n = 0; n < burst_per_tick; ++n) publish(sources.front());
      burst_samples += burst_per_tick * (direct ? 2 : 3);
    } else if (burst_started && !recovered) {
      std::ofstream(report + ".recovered") << "recovered\n";
      recovered = true;
    }
    const auto delay = began + ros::WallDuration((tick + 1) / rate) - ros::WallTime::now();
    if (delay.toSec() > 0) delay.sleep();
  }
  std::ofstream(report) << "{\"published_samples\":" << ticks * count * (direct ? 2 : 3) + burst_samples + imu_samples
      << ",\"burst_samples\":" << burst_samples
      << ",\"imu_samples\":" << imu_samples
      << ",\"duration_s\":" << (ros::WallTime::now() - began).toSec() << "}\n";
  ros::WallDuration(.1).sleep(); ros::shutdown(); return 0;
}
