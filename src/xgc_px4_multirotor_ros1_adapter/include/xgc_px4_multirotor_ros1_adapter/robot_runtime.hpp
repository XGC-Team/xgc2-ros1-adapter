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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include "xgc/adapter/v1/adapter.pb.h"
#include "xgc/v1/message.pb.h"
#include "xgc2/adapter_link/client.hpp"

namespace xgc_px4_multirotor_ros1_adapter {

std::string topicName(const std::string &robot_namespace,
                      const std::string &relative_name);
bool validRobotNamespace(const std::string &value, std::string *error);
bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds);
constexpr double kPx4PoseStaleAfterSeconds = 1.0;
bool px4IsOnline(bool state_known, bool state_fresh, bool connected);
bool isAllowedPx4Mode(const std::string &mode);
bool validateNonEmptyAdapterPlan(const xgc::adapter::v1::AdapterPlan &plan,
                                 std::string *error);

enum class Px4RebootReadiness {
  kReady,
  kStateUnknown,
  kStateStale,
  kDisconnected,
  kArmed,
};

Px4RebootReadiness evaluatePx4RebootReadiness(bool state_known,
                                              bool state_fresh, bool connected,
                                              bool armed);
const char *px4RebootReadinessDetail(Px4RebootReadiness readiness);

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
  xgc2::adapter_link::OperationExecutionResult
  executeOperation(const xgc::adapter::v1::OperationRequest &request,
                   double timeout_seconds);

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
  void installPx4();

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

  void px4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &message);
  void
  px4VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &message);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &message);
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr &message);
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr &message);
  void mavrosExtendedStateCallback(
      const mavros_msgs::ExtendedState::ConstPtr &message);

  void emitPx4PeriodicLocked(const ros::WallTime &now,
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

  const std::string pose_endpoint_;
  const std::string velocity_endpoint_;
  const std::string imu_endpoint_;
  const std::string power_endpoint_;
  const std::string state_endpoint_;
  const std::string extended_state_endpoint_;
  const std::string arm_endpoint_;
  const std::string mode_endpoint_;
  const std::string command_endpoint_;

  mutable std::mutex mutex_;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> sequences_;

  mavros_msgs::State mavros_state_;
  mavros_msgs::ExtendedState mavros_extended_state_;
  bool has_mavros_state_ = false;
  bool has_mavros_extended_state_ = false;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber power_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber extended_state_subscriber_;
  ros::ServiceClient arm_client_;
  ros::ServiceClient mode_client_;
  ros::ServiceClient command_client_;
};

} // namespace xgc_px4_multirotor_ros1_adapter
