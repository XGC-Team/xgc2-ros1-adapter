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

#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc_unitree_b2_ros1_adapter/wire_tcp_server.hpp"

namespace xgc_unitree_b2_ros1_adapter {

bool validWireHost(const std::string &value, std::string *error);
bool validRobotNamespace(const std::string &value, std::string *error);
bool parseWirePort(const std::string &value, std::uint16_t *port,
                   std::string *error);
bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds);
bool validateNativeProfileContract(std::string *error);

class RobotRuntime : public std::enable_shared_from_this<RobotRuntime> {
 public:
  using EnvelopeEmitter = std::function<void(std::string)>;
  static std::shared_ptr<RobotRuntime> Create(
      ros::NodeHandle node_handle,
      const xgc2_ros1_robot_adapter::RobotConfig &config,
      std::uint64_t spec_revision, EnvelopeEmitter emitter,
      std::string *error);
  ~RobotRuntime();
  void Stop();
  const std::string &robotId() const { return robot_id_; }
  std::uint64_t specRevision() const { return spec_revision_; }
  bool channelEnabled(const std::string &channel_id) const;
  void emitPeriodic(const ros::WallTime &now);

 private:
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
               std::string wire_host, std::uint16_t wire_port,
               EnvelopeEmitter emitter);
  bool install(std::string *error);
  void onWireFrame(const std::string &key, const std::string &payload);
  void handleOdom(const ros::WallTime &received, const std::string &payload);
  void handleJoints(const ros::WallTime &received, const std::string &payload,
                    bool arm);
  void handlePower(const ros::WallTime &received, const std::string &payload);
  void handleDriver(const ros::WallTime &received, const std::string &payload);
  void handleHeartbeat(const ros::WallTime &received,
                       const std::string &payload, bool arm);
  void emitHealthLocked(const ros::WallTime &now,
                        std::vector<xgc::robot::v1::RobotMessage> *messages);
  void emitStreamHealthLocked(
      const ros::WallTime &now,
      std::vector<xgc::robot::v1::RobotMessage> *messages);
  bool baseOnlineLocked(const ros::WallTime &now) const;
  bool shouldEmitLocked(const std::string &channel_id,
                        const ros::WallTime &now);
  xgc::robot::v1::RobotMessage makeEnvelopeLocked(
      const std::string &channel_id, std::int64_t source_time_millis,
      const google::protobuf::Message &payload);
  void emit(std::vector<xgc::robot::v1::RobotMessage> messages);
  void recordSourceLocked(const std::string &source, const ros::WallTime &now);
  void recordOutputLocked(const std::string &source);
  std::uint64_t sourceAgeMillisLocked(const std::string &source,
                                      const ros::WallTime &now) const;
  void publishCombinedJoints(std::int64_t source_time_millis);

  ros::NodeHandle node_handle_;
  const std::string robot_id_;
  const std::string profile_id_;
  const std::string robot_namespace_;
  const std::string frame_prefix_;
  const std::string base_frame_;
  const std::uint64_t spec_revision_;
  const std::set<std::string> enabled_channels_;
  const std::string wire_host_;
  const std::uint16_t wire_port_;
  const EnvelopeEmitter emitter_;
  std::unique_ptr<WireTcpServer> wire_;

  mutable std::mutex mutex_;
  bool stopping_ = false;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> sequences_;
  std::uint64_t decode_errors_ = 0;
  std::uint64_t dropped_frames_ = 0;
  int driver_level_ = 3;
  std::string driver_summary_ = "no driver status";
  std::vector<std::string> driver_faults_;
  bool has_online_state_ = false;
  bool last_online_state_ = false;

  nav_msgs::Path path_;
  std::map<std::string, double> leg_positions_;
  std::map<std::string, double> arm_positions_;
  ros::Publisher odom_publisher_;
  ros::Publisher leg_publisher_;
  ros::Publisher arm_publisher_;
  ros::Publisher combined_joint_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher power_publisher_;
  ros::Publisher driver_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace xgc_unitree_b2_ros1_adapter
