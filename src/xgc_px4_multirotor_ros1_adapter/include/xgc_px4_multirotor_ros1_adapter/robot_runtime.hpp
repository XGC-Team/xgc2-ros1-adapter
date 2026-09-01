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

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/TimesyncStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc/semantic/common/v1/acceleration.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc2_ros1_robot_adapter/ground_health.hpp"
#include "xgc2_ros1_robot_adapter/localization_projection.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"

namespace xgc_px4_multirotor_ros1_adapter {

struct NativeProfileConfig {
  std::string pose_endpoint;
  std::string velocity_endpoint;
  std::string imu_endpoint;
  std::string power_endpoint;
  std::string state_endpoint;
  std::string extended_state_endpoint;
  std::string mocap_endpoint;
  std::string vision_pose_endpoint;
  std::string mocap_velocity_endpoint;
  std::string mocap_acceleration_endpoint;
  std::string canonical_pose_endpoint;
  std::string canonical_velocity_endpoint;
  std::string canonical_acceleration_endpoint;
  std::string local_setpoint_endpoint;
  std::string attitude_setpoint_endpoint;
  std::string timesync_endpoint;
  std::string arm_service_endpoint;
  std::string mode_service_endpoint;
  std::string reboot_service_endpoint;
  std::string remote_control_endpoint;
  double offboard_source_timeout_seconds = 0.0;
  double offboard_minimum_rate_hz = 0.0;
  double reboot_state_timeout_seconds = 0.0;
  double maximum_operation_timeout_seconds = 0.0;
  double remote_control_altitude_meters = 0.0;
  double remote_control_maximum_linear_velocity_mps = 0.0;
  double remote_control_maximum_yaw_rate_rps = 0.0;
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization;
  std::vector<std::string> allowed_modes;
};

bool BuildNativeProfileConfig(
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    NativeProfileConfig *output, std::string *error);

std::string topicName(const std::string &robot_namespace,
                      const std::string &relative_name);
bool resolveRemoteControlTopic(
    const xgc2_ros1_robot_adapter::RobotConfig &config, std::string *topic,
    std::string *error);
bool validRobotNamespace(const std::string &value, std::string *error);
bool validMocapRigidBodyName(const std::string &value, std::string *error);
bool loadPositioningLivenessConfig(
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    xgc2_ros1_robot_adapter::PositioningHealthConfig *output,
    std::string *error);
bool validVisionPose(const geometry_msgs::PoseStamped &message);
bool normalizeVisionPose(geometry_msgs::PoseStamped *message);
double positionDistanceMeters(const geometry_msgs::Point &left,
                              const geometry_msgs::Point &right);
std::uint32_t localSetpointValidFields(std::uint16_t type_mask);
std::uint32_t attitudeSetpointValidFields(std::uint8_t type_mask);
bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds);
bool px4IsOnline(bool state_known, bool state_fresh, bool connected);
class RobotRuntime : public std::enable_shared_from_this<RobotRuntime> {
public:
  using EnvelopeEmitter = std::function<void(std::string item)>;

  static std::shared_ptr<RobotRuntime>
  Create(ros::NodeHandle node_handle,
         const xgc2_ros1_robot_adapter::RobotConfig &config,
         std::uint64_t spec_revision, EnvelopeEmitter emitter,
         std::string *error);

  ~RobotRuntime();

  // Native side effects remain gated while source-open is preparing. The
  // owner activates only after every native resource is committed.
  void Activate() noexcept;
  void Deactivate() noexcept;

  // Synchronously fences every ROS callback and native publisher. After Stop
  // returns no telemetry item or vision pose can be emitted by this runtime.
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
               NativeProfileConfig native_profile,
               xgc2_ros1_robot_adapter::PositioningHealthConfig
                   positioning_health_config,
               EnvelopeEmitter emitter);

  bool install(std::string *error);
  bool installPx4(std::string *error);
  bool channelRequired(const std::string &channel_id) const;
  bool beginCallback();
  void endCallback();

  bool shouldEmitLocked(const std::string &channel_id,
                        const ros::WallTime &now);
  xgc::robot::v1::RobotMessage
  makeEnvelopeLocked(const std::string &channel_id,
                     const ros::Time &source_stamp,
                     const google::protobuf::Message &payload);
  void emit(std::vector<xgc::robot::v1::RobotMessage> messages);
  void ensureSourceLocked(const std::string &channel_id,
                          double stale_after_seconds);
  void recordSourceLocked(const std::string &channel_id,
                          const ros::WallTime &now);
  void recordStateSourceLocked(const std::string &channel_id,
                               bool count_sample);
  void recordOutputLocked(const std::string &channel_id);
  void emitPositionErrorLocked(
      const ros::Time &source_stamp, const ros::WallTime &now,
      std::vector<xgc::robot::v1::RobotMessage> *messages);
  void setPositioningHealthLocked(
      const ros::WallTime &now,
      xgc::semantic::common::v1::VehicleHealth *payload) const;
  std::uint64_t sourceAgeMillisLocked(const std::string &channel_id,
                                      const ros::WallTime &now) const;

  void px4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &message);
  void mocapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &message);
  void mocapVelocityCallback(
      const geometry_msgs::TwistStamped::ConstPtr &message);
  void mocapAccelerationCallback(
      const geometry_msgs::AccelStamped::ConstPtr &message);
  void
  px4VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &message);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &message);
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr &message);
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr &message);
  void mavrosExtendedStateCallback(
      const mavros_msgs::ExtendedState::ConstPtr &message);
  void
  localSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr &message);
  void attitudeSetpointCallback(
      const mavros_msgs::AttitudeTarget::ConstPtr &message);
  void
  timesyncStatusCallback(const mavros_msgs::TimesyncStatus::ConstPtr &message);

  void
  emitPx4PeriodicLocked(const ros::WallTime &now,
                        std::vector<xgc::robot::v1::RobotMessage> *messages);
  void
  emitStreamHealthLocked(const ros::WallTime &now,
                         std::vector<xgc::robot::v1::RobotMessage> *messages);

  ros::NodeHandle node_handle_;
  const std::string robot_id_;
  const std::string profile_id_;
  const std::string robot_namespace_;
  const std::uint64_t spec_revision_;
  const std::set<std::string> enabled_channels_;
  const std::set<std::string> required_channels_;
  const EnvelopeEmitter emitter_;

  const double offboard_source_timeout_seconds_;
  const double offboard_minimum_rate_hz_;
  xgc2_ros1_robot_adapter::PositioningHealthWindow positioning_health_;

  const std::string pose_endpoint_;
  const std::string velocity_endpoint_;
  const std::string imu_endpoint_;
  const std::string power_endpoint_;
  const std::string state_endpoint_;
  const std::string extended_state_endpoint_;
  const std::string mocap_endpoint_;
  const std::string vision_pose_endpoint_;
  const std::string mocap_velocity_endpoint_;
  const std::string mocap_acceleration_endpoint_;
  const std::string canonical_pose_endpoint_;
  const std::string canonical_velocity_endpoint_;
  const std::string canonical_acceleration_endpoint_;
  const std::string local_setpoint_endpoint_;
  const std::string attitude_setpoint_endpoint_;
  const std::string timesync_endpoint_;

  mutable std::mutex mutex_;
  std::condition_variable callbacks_idle_;
  bool stopping_ = false;
  bool stop_complete_ = false;
  std::size_t active_callbacks_ = 0;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> sequences_;

  mavros_msgs::State mavros_state_;
  mavros_msgs::ExtendedState mavros_extended_state_;
  mavros_msgs::PositionTarget local_setpoint_;
  mavros_msgs::AttitudeTarget attitude_setpoint_;
  bool has_mavros_state_ = false;
  bool has_mavros_extended_state_ = false;
  bool has_local_setpoint_ = false;
  bool has_attitude_setpoint_ = false;
  bool valid_local_setpoint_ = false;
  bool valid_attitude_setpoint_ = false;
  ros::WallTime mavros_state_last_seen_;
  ros::WallTime mavros_extended_state_last_seen_;
  geometry_msgs::Point local_position_;
  geometry_msgs::Point mocap_position_;
  std::string local_position_frame_id_;
  bool has_local_position_ = false;
  bool has_mocap_position_ = false;
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization_;
  xgc2_ros1_robot_adapter::VisionPublishCadence vision_publish_cadence_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber mocap_subscriber_;
  ros::Subscriber mocap_velocity_subscriber_;
  ros::Subscriber mocap_acceleration_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber power_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber extended_state_subscriber_;
  ros::Subscriber local_setpoint_subscriber_;
  ros::Subscriber attitude_setpoint_subscriber_;
  ros::Subscriber timesync_subscriber_;
  ros::Publisher canonical_pose_publisher_;
  ros::Publisher canonical_velocity_publisher_;
  ros::Publisher canonical_acceleration_publisher_;
  ros::Publisher vision_pose_publisher_;
};

} // namespace xgc_px4_multirotor_ros1_adapter
