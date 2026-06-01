#include "xgc_ros1_adapter/robot_behavior.hpp"

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace xgc_ros1_adapter {
namespace {

void setEulerFromQuaternion(double qx, double qy, double qz, double qw,
                            xgc::adapter::v1::InstrumentState* instruments) {
  tf2::Quaternion quat(qx, qy, qz, qw);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  instruments->set_roll_deg(roll * 180.0 / M_PI);
  instruments->set_pitch_deg(pitch * 180.0 / M_PI);
  instruments->set_yaw_deg(yaw * 180.0 / M_PI);
}

void addTopicMetric(xgc::adapter::v1::RobotState* state,
                    const std::string& key,
                    const std::string& topic_name,
                    double hz,
                    const ros::WallTime& last) {
  auto* metric = state->add_topic_metrics();
  metric->set_key(key);
  metric->set_topic(topic_name);
  metric->set_frequency_hz(hz);
  metric->set_last_message_unix_nanos(last.isZero() ? 0 : last.toNSec());
}

}  // namespace

bool hasFeature(const RobotBehaviorSpec& behavior, RobotFeature feature) {
  for (const auto item : behavior.features) {
    if (item == feature) {
      return true;
    }
  }
  return false;
}

void commonSetDefaultEndpoints(RobotRuntime& robot) {
  robot.pose_topic = topic(robot.robot_namespace, "pose");
  robot.twist_topic = topic(robot.robot_namespace, "twist");
  robot.imu_topic = topic(robot.robot_namespace, "imu");
  robot.cmd_vel_topic = topic(robot.robot_namespace, "cmd_vel");
  robot.takeoff_topic = topic(robot.robot_namespace, "takeoff");
  robot.land_topic = topic(robot.robot_namespace, "land");
  robot.state_topic = topic(robot.robot_namespace, "mavros/state");
  robot.extended_state_topic = topic(robot.robot_namespace, "mavros/extended_state");
  robot.battery_topic = topic(robot.robot_namespace, "mavros/battery");
  robot.global_position_topic = topic(robot.robot_namespace, "mavros/global_position/global");
  robot.stage_topic = topic(robot.robot_namespace, "stage");
  robot.angle_topic = topic(robot.robot_namespace, "angle");
  robot.ias_topic = topic(robot.robot_namespace, "ias");
  robot.pos_topic = topic(robot.robot_namespace, "pos");
  robot.baro_topic = topic(robot.robot_namespace, "baro");
  robot.arm_service = topic(robot.robot_namespace, "mavros/cmd/arming");
  robot.mode_service = topic(robot.robot_namespace, "mavros/set_mode");
  robot.command_service = topic(robot.robot_namespace, "mavros/cmd/command");
  robot.takeoff_service = topic(robot.robot_namespace, "mavros/cmd/takeoff");
  robot.land_service = topic(robot.robot_namespace, "mavros/cmd/land");
}

void commonApplyMapping(const xgc::adapter::v1::RobotMapping& mapping, RobotRuntime& robot) {
  setTopicIfPresent(mapping.topics(), "pose", robot.pose_topic);
  setTopicIfPresent(mapping.topics(), "twist", robot.twist_topic);
  setTopicIfPresent(mapping.topics(), "imu", robot.imu_topic);
  setTopicIfPresent(mapping.topics(), "cmd_vel", robot.cmd_vel_topic);
  setTopicIfPresent(mapping.topics(), "command_velocity", robot.cmd_vel_topic);
  setTopicIfPresent(mapping.topics(), "takeoff", robot.takeoff_topic);
  setTopicIfPresent(mapping.topics(), "land", robot.land_topic);
  setTopicIfPresent(mapping.topics(), "state", robot.state_topic);
  setTopicIfPresent(mapping.topics(), "extended_state", robot.extended_state_topic);
  setTopicIfPresent(mapping.topics(), "battery", robot.battery_topic);
  setTopicIfPresent(mapping.topics(), "global_position", robot.global_position_topic);
  setTopicIfPresent(mapping.topics(), "stage", robot.stage_topic);
  setTopicIfPresent(mapping.topics(), "angle", robot.angle_topic);
  setTopicIfPresent(mapping.topics(), "ias", robot.ias_topic);
  setTopicIfPresent(mapping.topics(), "pos", robot.pos_topic);
  setTopicIfPresent(mapping.topics(), "baro", robot.baro_topic);
  setServiceIfPresent(mapping.services(), "arm", robot.arm_service);
  setServiceIfPresent(mapping.services(), "mode", robot.mode_service);
  setServiceIfPresent(mapping.services(), "command", robot.command_service);
  setServiceIfPresent(mapping.services(), "takeoff", robot.takeoff_service);
  setServiceIfPresent(mapping.services(), "land", robot.land_service);
}

void commonAppendRegistration(const RobotRuntime& robot, xgc::adapter::v1::RobotMapping* out) {
  out->set_robot_id(robot.robot_id);
  out->set_robot_type(robot.robot_type);
  out->set_namespace_("/" + trimSlash(robot.robot_namespace));
  (*out->mutable_topics())["pose"] = robot.pose_topic;
  (*out->mutable_topics())["twist"] = robot.twist_topic;
  (*out->mutable_topics())["imu"] = robot.imu_topic;
  (*out->mutable_topics())["cmd_vel"] = robot.cmd_vel_topic;
  (*out->mutable_topics())["state"] = robot.state_topic;
  (*out->mutable_topics())["extended_state"] = robot.extended_state_topic;
  (*out->mutable_topics())["battery"] = robot.battery_topic;
  auto* remote = out->mutable_remote_control();
  remote->set_command_topic(robot.cmd_vel_topic);
  remote->set_publish_rate_hz(robot.remote_publish_rate_hz);
  remote->set_linear_velocity_xy(0.3);
  remote->set_angular_velocity_z(0.6);
}

bool commonShouldPublishState(const RobotRuntime& robot) {
  return robot.has_pose || robot.has_twist || robot.has_imu || robot.has_mavros_state || robot.has_stage;
}

void commonAddState(xgc::adapter::v1::PushRobotStateRequest& request, const RobotRuntime& robot) {
  auto* state = request.add_states();
  state->set_robot_id(robot.robot_id);
  state->set_status("online");
  if (robot.has_stage) {
    state->set_mode("stage " + std::to_string(robot.stage.data));
  } else {
    state->set_mode(robot.has_mavros_state ? robot.mavros_state.mode : "simulation");
  }
  state->set_battery(robot.has_battery ? robot.battery.percentage : 1.0);
  if (robot.has_pose) {
    auto* out_pose = state->mutable_pose();
    out_pose->set_frame_id(robot.pose.header.frame_id);
    out_pose->set_x(robot.pose.pose.position.x);
    out_pose->set_y(robot.pose.pose.position.y);
    out_pose->set_z(robot.pose.pose.position.z);
    out_pose->set_qx(robot.pose.pose.orientation.x);
    out_pose->set_qy(robot.pose.pose.orientation.y);
    out_pose->set_qz(robot.pose.pose.orientation.z);
    out_pose->set_qw(robot.pose.pose.orientation.w);
  }
  if (robot.has_twist) {
    auto* out_twist = state->mutable_velocity();
    out_twist->set_frame_id(robot.twist.header.frame_id);
    out_twist->set_linear_x(robot.twist.twist.linear.x);
    out_twist->set_linear_y(robot.twist.twist.linear.y);
    out_twist->set_linear_z(robot.twist.twist.linear.z);
    out_twist->set_angular_x(robot.twist.twist.angular.x);
    out_twist->set_angular_y(robot.twist.twist.angular.y);
    out_twist->set_angular_z(robot.twist.twist.angular.z);
  }
  auto* instruments = state->mutable_instruments();
  if (robot.has_angle) {
    instruments->set_roll_deg(robot.angle.x);
    instruments->set_pitch_deg(robot.angle.y);
    instruments->set_yaw_deg(robot.angle.z);
  } else if (robot.has_imu) {
    setEulerFromQuaternion(robot.imu.orientation.x, robot.imu.orientation.y, robot.imu.orientation.z,
                           robot.imu.orientation.w, instruments);
  } else if (robot.has_pose) {
    setEulerFromQuaternion(robot.pose.pose.orientation.x, robot.pose.pose.orientation.y,
                           robot.pose.pose.orientation.z, robot.pose.pose.orientation.w, instruments);
  }
  if (robot.has_pose) {
    instruments->set_height_m(robot.pose.pose.position.z);
  }
  if (robot.has_baro) {
    instruments->set_height_m(robot.baro.data);
  }
  if (robot.has_twist) {
    instruments->set_speed_m_s(std::sqrt(
        robot.twist.twist.linear.x * robot.twist.twist.linear.x +
        robot.twist.twist.linear.y * robot.twist.twist.linear.y +
        robot.twist.twist.linear.z * robot.twist.twist.linear.z));
  }
  if (robot.has_ias) {
    instruments->set_speed_m_s(robot.ias.data);
  }
  if (robot.has_cmd_vel) {
    instruments->set_expected_speed_m_s(std::sqrt(
        robot.cmd_vel.linear.x * robot.cmd_vel.linear.x +
        robot.cmd_vel.linear.y * robot.cmd_vel.linear.y +
        robot.cmd_vel.linear.z * robot.cmd_vel.linear.z));
    auto* expected = instruments->mutable_expected_velocity();
    expected->set_frame_id("base_link");
    expected->set_linear_x(robot.cmd_vel.linear.x);
    expected->set_linear_y(robot.cmd_vel.linear.y);
    expected->set_linear_z(robot.cmd_vel.linear.z);
    expected->set_angular_x(robot.cmd_vel.angular.x);
    expected->set_angular_y(robot.cmd_vel.angular.y);
    expected->set_angular_z(robot.cmd_vel.angular.z);
  }
  addTopicMetric(state, "pose", robot.pose_topic, robot.pose_freq_hz, robot.last_pose_wall);
  addTopicMetric(state, "twist", robot.twist_topic, robot.twist_freq_hz, robot.last_twist_wall);
  addTopicMetric(state, "imu", robot.imu_topic, robot.imu_freq_hz, robot.last_imu_wall);
  addTopicMetric(state, "cmd_vel", robot.cmd_vel_topic, robot.cmd_vel_freq_hz, robot.last_cmd_vel_wall);
  addTopicMetric(state, "state", robot.state_topic, robot.state_freq_hz, robot.last_state_wall);
  addTopicMetric(state, "extended_state", robot.extended_state_topic, robot.extended_state_freq_hz,
                 robot.last_extended_state_wall);
  addTopicMetric(state, "battery", robot.battery_topic, robot.battery_freq_hz, robot.last_battery_wall);
  if (hasFeature(behaviorForType(robot.robot_type), RobotFeature::FixedWingTelemetry)) {
    addTopicMetric(state, "global_position", robot.global_position_topic, robot.global_position_freq_hz,
                   robot.last_global_position_wall);
    addTopicMetric(state, "stage", robot.stage_topic, robot.stage_freq_hz, robot.last_stage_wall);
    addTopicMetric(state, "angle", robot.angle_topic, robot.angle_freq_hz, robot.last_angle_wall);
    addTopicMetric(state, "ias", robot.ias_topic, robot.ias_freq_hz, robot.last_ias_wall);
    addTopicMetric(state, "pos", robot.pos_topic, robot.pos_freq_hz, robot.last_pos_wall);
    addTopicMetric(state, "baro", robot.baro_topic, robot.baro_freq_hz, robot.last_baro_wall);
  }
  auto* health = state->mutable_health();
  health->set_online(robot.has_imu || robot.has_pose || robot.has_stage ||
                     (robot.has_mavros_state && robot.mavros_state.connected));
  health->set_source("ros1");
  if (robot.has_stage) {
    health->set_message("fixed-wing stage telemetry");
  } else if (robot.has_mavros_state) {
    health->set_message(robot.mavros_state.connected ? "mavros state connected" : "mavros state disconnected");
  } else {
    health->set_message(robot.has_imu ? "imu heartbeat" : "waiting for imu heartbeat");
  }
}

const RobotBehaviorSpec& behaviorForType(const std::string& robot_type) {
  if (robot_type == "tello") {
    return telloBehavior();
  }
  if (robot_type == "px4" || robot_type == "minidrone") {
    return px4RotorBehavior();
  }
  if (robot_type == "fw_plane") {
    return fixedWingBehavior();
  }
  return ugvBehavior();
}

}  // namespace xgc_ros1_adapter
