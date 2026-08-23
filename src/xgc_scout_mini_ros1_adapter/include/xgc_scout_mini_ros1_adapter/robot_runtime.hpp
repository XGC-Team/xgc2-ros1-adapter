#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc/semantic/ground/v1/chassis.pb.h"
#include "xgc2_ros1_robot_adapter/ground_health.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"

namespace xgc_scout_mini_ros1_adapter {

std::string topicName(const std::string &robot_namespace,
                      const std::string &relative_name);
bool validRobotNamespace(const std::string &value, std::string *error);
bool validMocapRigidBodyName(const std::string &value, std::string *error);
bool resolveMotionCommandTopic(
    const xgc2_ros1_robot_adapter::RobotConfig &config, std::string *topic,
    std::string *error);
bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds);
bool scoutIsOnline(bool status_fresh);
double vrpnForwardSpeedMetersPerSecond(
    double velocity_x, double velocity_y, double velocity_z,
    double orientation_x, double orientation_y, double orientation_z,
    double orientation_w);
xgc::semantic::ground::v1::ChassisStatus::ControlMode
scoutControlMode(std::uint8_t native_mode);

struct ScoutChassisState {
  unsigned control_mode = 0;
  unsigned base_state = 0;
  unsigned fault_code = 0;
};

std::uint32_t packScoutChassisState(unsigned control_mode, unsigned base_state,
                                    unsigned fault_code);
bool unpackScoutChassisState(std::uint32_t word, ScoutChassisState *out);
bool validateNativeProfileContract(std::string *error);
class RobotRuntime : public std::enable_shared_from_this<RobotRuntime> {
public:
  using EnvelopeEmitter = std::function<void(std::string item)>;

  static std::shared_ptr<RobotRuntime>
  Create(ros::NodeHandle node_handle,
         const xgc2_ros1_robot_adapter::RobotConfig &config,
         std::uint64_t spec_revision, EnvelopeEmitter emitter,
         std::string *error);

  ~RobotRuntime();

  // Synchronously fences every ROS callback. After Stop returns no telemetry
  // item can be emitted by this runtime.
  void Stop();

  const std::string &robotId() const { return robot_id_; }
  const std::string &profileId() const { return profile_id_; }
  std::uint64_t specRevision() const { return spec_revision_; }
  bool channelEnabled(const std::string &channel_id) const;

  void emitPeriodic(const ros::WallTime &now);

private:
  class CallbackGuard {
  public:
    explicit CallbackGuard(RobotRuntime *runtime);
    ~CallbackGuard();
    explicit operator bool() const { return active_; }

  private:
    RobotRuntime *runtime_;
    bool active_;
  };

  struct SourceTracker {
    ros::WallTime last_seen;
    ros::WallTime window_started;
    std::uint64_t source_samples = 0;
    std::uint64_t output_samples = 0;
    std::uint64_t dropped_samples = 0;
    double source_rate_hz = 0.0;
    double output_rate_hz = 0.0;
    double stale_after_seconds = 1.0;
  };

  RobotRuntime(ros::NodeHandle node_handle, std::string robot_id,
               std::string profile_id, std::string robot_namespace,
               std::uint64_t spec_revision,
               std::set<std::string> enabled_channels,
               std::set<std::string> required_channels,
               std::string mocap_rigid_body, std::string pose_endpoint,
               std::string vrpn_velocity_endpoint,
               std::string command_velocity_endpoint, std::string imu_endpoint,
               std::string voltage_endpoint, std::string chassis_state_endpoint,
               xgc2_ros1_robot_adapter::PositioningHealthConfig positioning_config,
               std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> battery_curve,
               EnvelopeEmitter emitter);

  bool install(std::string *error);
  bool beginCallback();
  void endCallback();
  bool shouldEmitLocked(const std::string &channel_id,
                        const ros::WallTime &now);
  xgc::robot::v1::RobotMessage
  makeEnvelopeLocked(const std::string &channel_id,
                     const ros::Time &source_stamp,
                     const google::protobuf::Message &payload);
  void emit(std::vector<xgc::robot::v1::RobotMessage> messages);
  void ensureSourceLocked(const std::string &endpoint,
                          double stale_after_seconds);
  void recordSourceLocked(const std::string &endpoint,
                          const ros::WallTime &now);
  void recordOutputLocked(const std::string &endpoint);
  bool sourceFreshLocked(const std::string &endpoint,
                         const ros::WallTime &now) const;
  std::uint64_t sourceAgeMillisLocked(const std::string &endpoint,
                                      const ros::WallTime &now) const;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &message);
  void vrpnVelocityCallback(
      const geometry_msgs::TwistStamped::ConstPtr &message);
  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &message);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &message);
  void voltageCallback(const std_msgs::Float32::ConstPtr &message);
  void chassisStateCallback(const std_msgs::UInt32::ConstPtr &message);
  void emitHealthLocked(const ros::WallTime &now,
                        std::vector<xgc::robot::v1::RobotMessage> *messages);
  void emitStreamHealthLocked(const ros::WallTime &now,
                              std::vector<xgc::robot::v1::RobotMessage> *messages);

  ros::NodeHandle node_handle_;
  const std::string robot_id_;
  const std::string profile_id_;
  const std::string robot_namespace_;
  const std::string mocap_rigid_body_;
  const std::uint64_t spec_revision_;
  const std::set<std::string> enabled_channels_;
  const std::set<std::string> required_channels_;
  const EnvelopeEmitter emitter_;

  const std::string pose_endpoint_;
  const std::string vrpn_velocity_endpoint_;
  const std::string command_velocity_endpoint_;
  const std::string imu_endpoint_;
  const std::string voltage_endpoint_;
  const std::string chassis_state_endpoint_;
  xgc2_ros1_robot_adapter::PositioningHealthWindow positioning_health_;
  const std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> battery_curve_;

  mutable std::mutex mutex_;
  std::condition_variable callbacks_idle_;
  bool stopping_ = false;
  bool stop_complete_ = false;
  std::size_t active_callbacks_ = 0;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> sequences_;

  ScoutChassisState scout_status_;
  ros::Time status_stamp_;
  bool has_chassis_ = false;
  geometry_msgs::Quaternion vrpn_orientation_;
  bool has_vrpn_orientation_ = false;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber vrpn_velocity_subscriber_;
  ros::Subscriber command_velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber voltage_subscriber_;
  ros::Subscriber chassis_state_subscriber_;
};

} // namespace xgc_scout_mini_ros1_adapter
