#include "xgc_scene/lidar_ros/cloud.hpp"

#include <cstring>

namespace xgc_scene::lidar_ros {
xgc2_math::Pose3 poseOf(const geometry_msgs::Pose& pose) {
  xgc2_math::Pose3 result;
  result.position = {pose.position.x, pose.position.y, pose.position.z};
  result.orientation = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                        pose.orientation.y, pose.orientation.z);
  return result;
}

sensor_msgs::PointCloud2 cloudLayout(bool beams) {
  sensor_msgs::PointCloud2 message;
  message.header.frame_id = "world";
  message.height = 1;
  message.is_bigendian = false;
  message.is_dense = true;
  const char* names[] = {"x", "y", "z", "dx", "dy", "dz", "range", "hit"};
  message.fields.resize(beams ? 8 : 3);
  for (std::size_t i = 0; i < message.fields.size(); ++i) {
    auto& field = message.fields[i];
    field.name = names[i];
    field.offset = 4 * i;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
  }
  message.point_step = beams ? 32 : 12;
  return message;
}

void fillCloud(sensor_msgs::PointCloud2& message, lidar::CloudView cloud) {
  message.header.stamp.fromNSec(cloud.stamp_ns);
  message.width = cloud.count;
  message.row_step = message.point_step * message.width;
  message.data.resize(cloud.byteSize());
  if (cloud.byteSize()) std::memcpy(message.data.data(), cloud.data, cloud.byteSize());
}

bool Demand::change(bool beam_topic, bool connected) {
  auto& count = beam_topic ? beams : points;
  if (connected) ++count;
  else if (count) --count;
  return points || beams;
}
}  // namespace xgc_scene::lidar_ros
