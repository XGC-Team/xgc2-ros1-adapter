#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <scout_msgs/ScoutStatus.h>
#include <sensor_msgs/Imu.h>

#include "xgc/adapter/v1/adapter.pb.h"
#include "xgc/v1/message.pb.h"

namespace xgc_scout_mini_ros1_adapter {

constexpr double kScoutStatusStaleAfterSeconds = 1.0;

std::string topicName(const std::string &robot_namespace,
                      const std::string &relative_name);
bool validRobotNamespace(const std::string &value, std::string *error);
bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds);
bool scoutIsOnline(bool odometry_fresh, bool status_fresh);
bool validateNonEmptyAdapterPlan(const xgc::adapter::v1::AdapterPlan &plan,
                                 std::string *error);

class RobotRuntime : public std::enable_shared_from_this<RobotRuntime> {
public:
  using EnvelopeEmitter = std::function<void(std::uint64_t plan_revision,
                                             xgc::v1::Message message)>;

  static std::shared_ptr<RobotRuntime>
  Create(ros::NodeHandle node_handle, const xgc::adapter::v1::RobotPlan &plan,
         std::uint64_t plan_revision, EnvelopeEmitter emitter,
         std::string *error);

  ~RobotRuntime();

  const std::string &robotId() const { return robot_id_; }
  const std::string &profileId() const { return profile_id_; }
  std::uint64_t planRevision() const { return plan_revision_; }
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
               std::uint64_t plan_revision,
               std::set<std::string> enabled_channels, EnvelopeEmitter emitter);

  bool install(std::string *error);
  bool shouldEmitLocked(const std::string &channel_id,
                        const ros::WallTime &now);
  xgc::v1::Message makeEnvelopeLocked(const std::string &channel_id,
                                      std::uint32_t message_id,
                                      const ros::Time &source_stamp,
                                      const google::protobuf::Message &payload);
  void emit(std::vector<xgc::v1::Message> messages);
  void ensureSourceLocked(const std::string &endpoint,
                          double stale_after_seconds);
  void recordSourceLocked(const std::string &endpoint,
                          const ros::WallTime &now);
  void recordOutputLocked(const std::string &endpoint);
  bool sourceFreshLocked(const std::string &endpoint,
                         const ros::WallTime &now) const;
  std::uint64_t sourceAgeMillisLocked(const std::string &endpoint,
                                      const ros::WallTime &now) const;

  void odometryCallback(const nav_msgs::Odometry::ConstPtr &message);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &message);
  void statusCallback(const scout_msgs::ScoutStatus::ConstPtr &message);
  void emitHealthLocked(const ros::WallTime &now,
                        std::vector<xgc::v1::Message> *messages);
  void emitChannelHealthLocked(const ros::WallTime &now,
                               std::vector<xgc::v1::Message> *messages);

  ros::NodeHandle node_handle_;
  const std::string robot_id_;
  const std::string profile_id_;
  const std::string robot_namespace_;
  const std::uint64_t plan_revision_;
  const std::set<std::string> enabled_channels_;
  const EnvelopeEmitter emitter_;

  const std::string odometry_endpoint_;
  const std::string imu_endpoint_;
  const std::string status_endpoint_;

  mutable std::mutex mutex_;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> sequences_;

  scout_msgs::ScoutStatus scout_status_;
  bool has_odometry_ = false;
  bool has_status_ = false;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber status_subscriber_;
};

} // namespace xgc_scout_mini_ros1_adapter
