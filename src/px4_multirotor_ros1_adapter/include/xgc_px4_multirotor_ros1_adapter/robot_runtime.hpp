#pragma once

#include <condition_variable>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
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
#include <std_msgs/String.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc/semantic/common/v1/acceleration.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc2_ros1_robot_adapter/contract.hpp"
#include "xgc2_ros1_robot_adapter/ground_health.hpp"
#include "xgc2_ros1_robot_adapter/localization_projection.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace contract = xgc2_ros1_robot_adapter::contract;

struct NativeProfileConfig {
  std::string pose_endpoint;
  std::string velocity_endpoint;
  std::string imu_endpoint;
  std::string power_endpoint;
  std::string controller_status_endpoint;
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
  // Core silence after which an active motion intent is released; 0 = never.
  std::int64_t remote_control_core_silence_ms = 0;
  bool publish_vision = true;
  double vision_publish_rate_hz = 30.0;
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
  using EnvelopeEmitter = std::function<void(xgc::robot::v1::RobotMessage)>;

  static std::shared_ptr<RobotRuntime>
  Create(ros::NodeHandle node_handle,
         const xgc2_ros1_robot_adapter::RobotConfig &config,
         EnvelopeEmitter emitter, std::string *error);

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
  bool channelEnabled(const std::string &channel_id) const;

  void emitPeriodic(const ros::WallTime &now);
  void SetStatusDemand(const std::set<std::string> &channels);
  std::pair<bool, std::string> DescribeHealth();

private:
  friend class StatusDemandHarness;
  class CallbackGuard {
  public:
    explicit CallbackGuard(RobotRuntime *runtime);
    ~CallbackGuard();
    explicit operator bool() const { return active_; }

  private:
    RobotRuntime *runtime_;
    std::size_t epoch_ = 0;
    bool active_;
  };
  class StatusUpdateGuard {
  public:
    explicit StatusUpdateGuard(RobotRuntime *runtime);
    ~StatusUpdateGuard();
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
               std::set<std::string> enabled_channels,
               std::set<std::string> required_channels,
               NativeProfileConfig native_profile,
               xgc2_ros1_robot_adapter::PositioningHealthConfig
                   positioning_health_config,
               EnvelopeEmitter emitter);

  bool install(std::string *error);
  bool installPx4(std::string *error);
  bool channelRequired(const std::string &channel_id) const;
  bool beginCallback(std::size_t *epoch);
  void endCallback(std::size_t epoch);
  bool beginStatusUpdate();
  void endStatusUpdate();

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
  void controllerStatusCallback(const std_msgs::String::ConstPtr &message);
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
  void emitSnapshotLocked(const std::string &channel, const ros::WallTime &now,
                          std::vector<xgc::robot::v1::RobotMessage> *messages);
  xgc::semantic::common::v1::VehicleHealth healthLocked(const ros::WallTime &now) const;

  ros::NodeHandle node_handle_;
  const std::string robot_id_;
  const std::string profile_id_;
  const std::string robot_namespace_;
  const std::set<std::string> enabled_channels_;
  const std::set<std::string> required_channels_;
  std::map<std::string, xgc2_ros1_robot_adapter::MessageSchema> message_schemas_;
  const EnvelopeEmitter emitter_;

  const double offboard_source_timeout_seconds_;
  const double offboard_minimum_rate_hz_;
  xgc2_ros1_robot_adapter::PositioningHealthWindow positioning_health_;

  const std::string pose_endpoint_;
  const std::string velocity_endpoint_;
  const std::string imu_endpoint_;
  const std::string power_endpoint_;
  const std::string controller_status_endpoint_;
  const std::string state_endpoint_;
  const std::string extended_state_endpoint_;
  const std::string mocap_endpoint_;
  const std::string vision_pose_endpoint_;
  const bool publish_vision_;
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
  std::size_t active_status_updates_ = 0;
  std::array<std::size_t, 2> active_callback_epochs_{};
  std::size_t callback_epoch_ = 0;
  std::set<std::string> status_demand_;
  std::string forced_channel_;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> sequences_;

  mavros_msgs::State mavros_state_;
  mavros_msgs::ExtendedState mavros_extended_state_;
  mavros_msgs::PositionTarget local_setpoint_;
  mavros_msgs::AttitudeTarget attitude_setpoint_;
  bool has_mavros_state_ = false;
  bool flight_mode_valid_ = false;
  bool has_mavros_extended_state_ = false;
  bool has_local_setpoint_ = false;
  bool has_attitude_setpoint_ = false;
  ros::WallTime mavros_state_last_seen_;
  ros::WallTime mavros_extended_state_last_seen_;
  geometry_msgs::Point local_position_;
  geometry_msgs::Point mocap_position_;
  std::string local_position_frame_id_;
  ros::Time localization_error_stamp_;
  bool has_local_position_ = false;
  bool has_mocap_position_ = false;
  bool local_position_frame_valid_ = false;
  geometry_msgs::PoseStamped::ConstPtr latest_pose_, latest_mocap_pose_;
  geometry_msgs::PoseStamped latest_vision_pose_;
  bool has_vision_pose_ = false;
  geometry_msgs::TwistStamped::ConstPtr latest_mocap_velocity_, latest_velocity_;
  geometry_msgs::AccelStamped::ConstPtr latest_mocap_acceleration_;
  sensor_msgs::Imu::ConstPtr latest_imu_;
  std_msgs::String::ConstPtr latest_controller_;
  ros::Time latest_controller_stamp_;
  struct PowerSample {
    bool present = false;
    float percentage = 0, voltage = 0, current = 0, temperature = 0;
    std::uint8_t status = 0;
    ros::Time stamp;
  } latest_power_;
  struct TimesyncSample {
    bool present = false;
    std::uint64_t remote_timestamp_ns = 0;
    std::int64_t observed_offset_ns = 0, estimated_offset_ns = 0;
    float round_trip_time_ms = 0;
    ros::Time stamp;
  } latest_timesync_;
  bool local_setpoint_frame_valid_ = false, attitude_setpoint_frame_valid_ = false;
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization_;
  xgc2_ros1_robot_adapter::VisionPublishCadence vision_publish_cadence_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber mocap_subscriber_;
  ros::Subscriber mocap_velocity_subscriber_;
  ros::Subscriber mocap_acceleration_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber power_subscriber_;
  ros::Subscriber controller_status_subscriber_;
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
