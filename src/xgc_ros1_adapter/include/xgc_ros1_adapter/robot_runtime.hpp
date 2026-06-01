#pragma once

#include <map>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "adapter/v1/adapter.pb.h"

namespace xgc_ros1_adapter {

std::string trimSlash(const std::string& value);
std::string topic(const std::string& ns, const std::string& name);

void setTopicIfPresent(const google::protobuf::Map<std::string, std::string>& topics,
                       const std::string& key,
                       std::string& target);

void setServiceIfPresent(
    const google::protobuf::Map<std::string, xgc::adapter::v1::ServiceBinding>& services,
    const std::string& key,
    std::string& target);

double fieldDouble(const google::protobuf::Map<std::string, std::string>& fields,
                   const std::string& key,
                   double fallback);

bool fieldBool(const google::protobuf::Map<std::string, std::string>& fields,
               const std::string& key,
               bool fallback);

std::string fieldString(const google::protobuf::Map<std::string, std::string>& fields,
                        const std::string& key,
                        const std::string& fallback);

struct RobotRuntime {
  std::string robot_id = "ugv1";
  std::string robot_type = "ugv";
  std::string robot_namespace = "ugv1";
  std::string pose_topic;
  std::string twist_topic;
  std::string imu_topic;
  std::string cmd_vel_topic;
  std::string takeoff_topic;
  std::string land_topic;
  std::string state_topic;
  std::string extended_state_topic;
  std::string battery_topic;
  std::string global_position_topic;
  std::string stage_topic;
  std::string angle_topic;
  std::string ias_topic;
  std::string pos_topic;
  std::string baro_topic;
  std::string arm_service;
  std::string mode_service;
  std::string command_service;
  std::string takeoff_service;
  std::string land_service;

  ros::Publisher cmd_vel_pub;
  ros::Publisher position_target_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::ServiceClient arm_client;
  ros::ServiceClient mode_client;
  ros::ServiceClient command_client;
  ros::ServiceClient takeoff_client;
  ros::ServiceClient land_client;
  ros::Subscriber pose_sub;
  ros::Subscriber twist_sub;
  ros::Subscriber twist_plain_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber position_target_sub;
  ros::Subscriber state_sub;
  ros::Subscriber extended_state_sub;
  ros::Subscriber battery_sub;
  ros::Subscriber global_position_sub;
  ros::Subscriber stage_sub;
  ros::Subscriber angle_sub;
  ros::Subscriber ias_sub;
  ros::Subscriber pos_sub;
  ros::Subscriber baro_sub;

  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped twist;
  sensor_msgs::Imu imu;
  geometry_msgs::Twist cmd_vel;
  mavros_msgs::State mavros_state;
  mavros_msgs::ExtendedState extended_state;
  sensor_msgs::BatteryState battery;
  sensor_msgs::NavSatFix global_position;
  std_msgs::Int32 stage;
  geometry_msgs::Vector3 angle;
  std_msgs::Float32 ias;
  geometry_msgs::Vector3 pos;
  std_msgs::Float32 baro;
  bool has_pose = false;
  bool has_twist = false;
  bool has_imu = false;
  bool has_cmd_vel = false;
  bool has_mavros_state = false;
  bool has_extended_state = false;
  bool has_battery = false;
  bool has_global_position = false;
  bool has_stage = false;
  bool has_angle = false;
  bool has_ias = false;
  bool has_pos = false;
  bool has_baro = false;
  int pose_count = 0;
  int twist_count = 0;
  int imu_count = 0;
  int cmd_vel_count = 0;
  int state_count = 0;
  int extended_state_count = 0;
  int battery_count = 0;
  int global_position_count = 0;
  int stage_count = 0;
  int angle_count = 0;
  int ias_count = 0;
  int pos_count = 0;
  int baro_count = 0;
  double pose_freq_hz = 0.0;
  double twist_freq_hz = 0.0;
  double imu_freq_hz = 0.0;
  double cmd_vel_freq_hz = 0.0;
  double state_freq_hz = 0.0;
  double extended_state_freq_hz = 0.0;
  double battery_freq_hz = 0.0;
  double global_position_freq_hz = 0.0;
  double stage_freq_hz = 0.0;
  double angle_freq_hz = 0.0;
  double ias_freq_hz = 0.0;
  double pos_freq_hz = 0.0;
  double baro_freq_hz = 0.0;
  ros::WallTime last_pose_wall;
  ros::WallTime last_twist_wall;
  ros::WallTime last_imu_wall;
  ros::WallTime last_cmd_vel_wall;
  ros::WallTime last_state_wall;
  ros::WallTime last_extended_state_wall;
  ros::WallTime last_battery_wall;
  ros::WallTime last_global_position_wall;
  ros::WallTime last_stage_wall;
  ros::WallTime last_angle_wall;
  ros::WallTime last_ias_wall;
  ros::WallTime last_pos_wall;
  ros::WallTime last_baro_wall;

  geometry_msgs::Twist current_remote_cmd;
  mavros_msgs::PositionTarget current_position_target_cmd;
  bool remote_enabled = false;
  bool pending_stop_publish = false;
  double remote_publish_rate_hz = 20.0;
};

}  // namespace xgc_ros1_adapter
