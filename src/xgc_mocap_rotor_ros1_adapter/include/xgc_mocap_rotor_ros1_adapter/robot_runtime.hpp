#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc_mocap_rotor_ros1_adapter/wire_contract.hpp"

namespace xgc_mocap_rotor_ros1_adapter {

bool ValidateNativeProfileContract(std::string *error);
bool ValidateRobotNamespace(const std::string &value, std::string *error);
bool ShouldResetWireEpoch(std::uint64_t previous_sequence,
                          std::uint64_t incoming_sequence,
                          std::int64_t previous_source_time_millis,
                          std::int64_t incoming_source_time_millis,
                          std::int64_t previous_uptime_millis,
                          std::int64_t incoming_uptime_millis,
                          double link_age_seconds, double stale_after_seconds);

class RobotRuntime : public std::enable_shared_from_this<RobotRuntime> {
public:
  using EnvelopeEmitter = std::function<void(std::string)>;

  static std::shared_ptr<RobotRuntime>
  Create(ros::NodeHandle node_handle,
         const xgc2_ros1_robot_adapter::RobotConfig &config,
         std::uint64_t spec_revision, EnvelopeEmitter emitter,
         std::string *error);

  ~RobotRuntime();
  RobotRuntime(const RobotRuntime &) = delete;
  RobotRuntime &operator=(const RobotRuntime &) = delete;

  bool HandleWireFrame(WireChannel channel, const std::string &payload,
                       std::string *error);
  void EmitPeriodic(const ros::WallTime &now);
  void Stop();
  const std::string &robot_id() const noexcept { return robot_id_; }

private:
  struct SourceTracker {
    ros::WallTime last_seen;
    ros::WallTime window_started;
    std::uint64_t source_samples = 0;
    std::uint64_t output_samples = 0;
    std::uint64_t dropped_samples = 0;
    double source_rate_hz = 0.0;
    double output_rate_hz = 0.0;
    double stale_after_seconds = 0.0;
  };

  RobotRuntime(ros::NodeHandle node_handle, std::string robot_id,
               std::string profile_id, std::string robot_namespace,
               std::uint64_t spec_revision,
               std::set<std::string> enabled_channels, EnvelopeEmitter emitter);

  bool install(std::string *error);
  bool handlePose(const ros::WallTime &received, const std::string &payload,
                  std::string *error);
  bool handleVelocity(const ros::WallTime &received, const std::string &payload,
                      std::string *error);
  bool handleImu(const ros::WallTime &received, const std::string &payload,
                 std::string *error);
  bool handlePower(const ros::WallTime &received, const std::string &payload,
                   std::string *error);
  bool handleFlightState(const ros::WallTime &received,
                         const std::string &payload, std::string *error);
  bool handleHeartbeat(const ros::WallTime &received,
                       const std::string &payload, std::string *error);

  bool acceptWireSequenceLocked(WireChannel channel, std::uint64_t sequence,
                                std::string *error);
  bool acceptHeartbeatSequenceLocked(const ros::WallTime &received,
                                     std::uint64_t sequence,
                                     std::int64_t source_time_millis,
                                     std::int64_t uptime_millis,
                                     std::string *error);
  void resetWireEpochLocked();
  bool channelEnabled(const std::string &channel_id) const;
  bool shouldEmitLocked(const std::string &channel_id,
                        const ros::WallTime &now);
  void recordSourceLocked(const std::string &channel_id,
                          const ros::WallTime &now);
  void recordOutputLocked(const std::string &channel_id);
  std::uint64_t sourceAgeMillisLocked(const std::string &channel_id,
                                      const ros::WallTime &now) const;
  bool sourceFreshLocked(const std::string &channel_id,
                         const ros::WallTime &now) const;
  xgc::robot::v1::RobotMessage
  makeEnvelopeLocked(const std::string &channel_id,
                     std::int64_t source_time_millis,
                     const google::protobuf::Message &payload);
  void emit(std::vector<xgc::robot::v1::RobotMessage> messages);
  void emitHealthLocked(const ros::WallTime &now,
                        std::vector<xgc::robot::v1::RobotMessage> *messages);
  void
  emitStreamHealthLocked(const ros::WallTime &now,
                         std::vector<xgc::robot::v1::RobotMessage> *messages);

  ros::NodeHandle node_handle_;
  const std::string robot_id_;
  const std::string profile_id_;
  const std::string robot_namespace_;
  const std::string frame_prefix_;
  const std::uint64_t spec_revision_;
  const std::set<std::string> enabled_channels_;
  const EnvelopeEmitter emitter_;

  mutable std::mutex mutex_;
  bool stopping_ = false;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> semantic_sequences_;
  std::map<WireChannel, std::uint64_t> wire_sequences_;
  std::int64_t last_forwarder_source_time_millis_ = 0;
  std::int64_t last_forwarder_uptime_millis_ = -1;
  std::uint64_t forwarder_restarts_ = 0;
  std::uint64_t decode_errors_ = 0;
  std::uint64_t rejected_frames_ = 0;
  bool flight_connected_ = false;
  bool flight_armed_ = false;
  std::string flight_mode_;
  std::uint32_t flight_system_status_ = 0;
  std::uint32_t flight_landed_state_ = 0;
  std::vector<std::string> flight_faults_;
  nav_msgs::Odometry odometry_;
  nav_msgs::Path path_;

  ros::Publisher pose_publisher_;
  ros::Publisher velocity_publisher_;
  ros::Publisher odometry_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher imu_publisher_;
  ros::Publisher power_publisher_;
  ros::Publisher flight_state_publisher_;
  ros::Publisher forwarder_status_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

} // namespace xgc_mocap_rotor_ros1_adapter
