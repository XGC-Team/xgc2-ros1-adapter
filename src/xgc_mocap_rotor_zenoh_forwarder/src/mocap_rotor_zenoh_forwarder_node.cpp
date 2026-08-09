#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include "xgc_mocap_rotor_zenoh_forwarder/forwarder_contract.hpp"
#include "xgc_mocap_rotor_zenoh_forwarder/wire_encoder.hpp"
#include "xgc_mocap_rotor_zenoh_forwarder/zenoh_publisher.hpp"

namespace xgc_mocap_rotor_zenoh_forwarder {
namespace {

constexpr std::size_t kSourceCount = 6u;
constexpr double kFlightSourceFreshSeconds = 5.0;

enum class SourceIndex : std::size_t {
  kLocalPose = 0u,
  kLocalVelocity = 1u,
  kImu = 2u,
  kPower = 3u,
  kFlightState = 4u,
  kExtendedState = 5u,
};

struct SourceTracker {
  const char *id = "";
  std::uint64_t samples = 0u;
  ros::WallTime last_seen;
};

std::int64_t systemMillis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::int64_t sourceMillis(const ros::Time &stamp) {
  if (stamp.isZero())
    return systemMillis();
  return static_cast<std::int64_t>(stamp.sec) * 1000 +
         static_cast<std::int64_t>(stamp.nsec / 1000000u);
}

bool requiredString(ros::NodeHandle *private_node, const char *name,
                    std::string *value, std::string *error) {
  if (private_node == nullptr || value == nullptr ||
      !private_node->getParam(name, *value) || value->empty()) {
    if (error != nullptr)
      *error = std::string("required private parameter is missing: ") + name;
    return false;
  }
  return true;
}

class ForwarderNode {
public:
  ForwarderNode()
      : private_node_("~"),
        sources_{{{"local_pose", 0u, ros::WallTime()},
                  {"local_velocity", 0u, ros::WallTime()},
                  {"imu", 0u, ros::WallTime()},
                  {"power", 0u, ros::WallTime()},
                  {"flight_state", 0u, ros::WallTime()},
                  {"extended_state", 0u, ros::WallTime()}}},
        started_at_(ros::WallTime::now()) {}

  ~ForwarderNode() { publisher_.Stop(); }

  bool Start(std::string *error) {
    if (!loadConfig(error) || !ros::master::check()) {
      if (error != nullptr && error->empty())
        *error = "the Mocap Rotor onboard ROS master is unavailable";
      return false;
    }
    if (!publisher_.Start(config_.robot_id, config_.zenoh_connect, error))
      return false;

    pose_subscriber_ = node_.subscribe(config_.local_pose_topic, 50,
                                       &ForwarderNode::onPose, this);
    velocity_subscriber_ = node_.subscribe(
        config_.local_velocity_topic, 50, &ForwarderNode::onVelocity, this);
    imu_subscriber_ =
        node_.subscribe(config_.imu_topic, 100, &ForwarderNode::onImu, this);
    power_subscriber_ =
        node_.subscribe(config_.power_topic, 20, &ForwarderNode::onPower, this);
    state_subscriber_ = node_.subscribe(config_.flight_state_topic, 20,
                                        &ForwarderNode::onState, this);
    extended_state_subscriber_ = node_.subscribe(
        config_.extended_state_topic, 20, &ForwarderNode::onExtendedState,
        this);
    heartbeat_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0), &ForwarderNode::onHeartbeat, this);

    ROS_INFO_STREAM("XGC Mocap Rotor onboard Forwarder robot="
                    << config_.robot_id << " zenoh=" << config_.zenoh_connect
                    << " mode=read-only source_master="
                    << ros::master::getURI());
    return true;
  }

private:
  bool loadConfig(std::string *error) {
    return requiredString(&private_node_, "robot_id", &config_.robot_id,
                          error) &&
           requiredString(&private_node_, "zenoh_connect",
                          &config_.zenoh_connect, error) &&
           requiredString(&private_node_, "local_pose_topic",
                          &config_.local_pose_topic, error) &&
           requiredString(&private_node_, "local_velocity_topic",
                          &config_.local_velocity_topic, error) &&
           requiredString(&private_node_, "imu_topic", &config_.imu_topic,
                          error) &&
           requiredString(&private_node_, "power_topic", &config_.power_topic,
                          error) &&
           requiredString(&private_node_, "flight_state_topic",
                          &config_.flight_state_topic, error) &&
           requiredString(&private_node_, "extended_state_topic",
                          &config_.extended_state_topic, error) &&
           requiredString(&private_node_, "pose_child_frame_id",
                          &config_.pose_child_frame_id, error) &&
           ValidateForwarderConfig(config_, error);
  }

  void record(SourceIndex source) {
    auto &tracker = sources_.at(static_cast<std::size_t>(source));
    ++tracker.samples;
    tracker.last_seen = ros::WallTime::now();
  }

  bool allow(UplinkChannel channel) {
    const std::size_t index = static_cast<std::size_t>(channel);
    const ros::WallTime now = ros::WallTime::now();
    const ros::WallTime last = last_publish_.at(index);
    if (!last.isZero() && now >= last &&
        (now - last).toSec() < 1.0 / MaximumRateHz(channel)) {
      ++stats_.throttled;
      return false;
    }
    last_publish_.at(index) = now;
    return true;
  }

  std::uint64_t sequence(UplinkChannel channel) {
    return ++sequences_.at(static_cast<std::size_t>(channel));
  }

  void rejected(const std::string &error) {
    ++stats_.rejected_source;
    ROS_WARN_STREAM_THROTTLE(5.0,
                             "Mocap Rotor source sample rejected: " << error);
  }

  void put(UplinkChannel channel, const std::string &payload) {
    std::string error;
    if (!publisher_.Put(channel, payload, &error)) {
      ++stats_.publish_failure;
      ROS_ERROR_STREAM_THROTTLE(5.0, error);
      return;
    }
    ++stats_.publish_success;
  }

  void onPose(const geometry_msgs::PoseStamped::ConstPtr &message) {
    record(SourceIndex::kLocalPose);
    std::string payload;
    std::string error;
    const auto next = sequence(UplinkChannel::kLocalPose);
    if (!EncodePose(*message, config_.pose_child_frame_id, next,
                    sourceMillis(message->header.stamp), &payload, &error)) {
      rejected(error);
      return;
    }
    if (allow(UplinkChannel::kLocalPose))
      put(UplinkChannel::kLocalPose, payload);
  }

  void onVelocity(const geometry_msgs::TwistStamped::ConstPtr &message) {
    record(SourceIndex::kLocalVelocity);
    std::string payload;
    std::string error;
    const auto next = sequence(UplinkChannel::kLocalVelocity);
    if (!EncodeVelocity(*message, next, sourceMillis(message->header.stamp),
                        &payload, &error)) {
      rejected(error);
      return;
    }
    if (allow(UplinkChannel::kLocalVelocity))
      put(UplinkChannel::kLocalVelocity, payload);
  }

  void onImu(const sensor_msgs::Imu::ConstPtr &message) {
    record(SourceIndex::kImu);
    std::string payload;
    std::string error;
    const auto next = sequence(UplinkChannel::kImu);
    if (!EncodeImu(*message, next, sourceMillis(message->header.stamp),
                   &payload, &error)) {
      rejected(error);
      return;
    }
    if (allow(UplinkChannel::kImu))
      put(UplinkChannel::kImu, payload);
  }

  void onPower(const sensor_msgs::BatteryState::ConstPtr &message) {
    record(SourceIndex::kPower);
    std::string payload;
    std::string error;
    const auto next = sequence(UplinkChannel::kPower);
    if (!EncodePower(*message, next, sourceMillis(message->header.stamp),
                     &payload, &error)) {
      rejected(error);
      return;
    }
    if (allow(UplinkChannel::kPower))
      put(UplinkChannel::kPower, payload);
  }

  void onState(const mavros_msgs::State::ConstPtr &message) {
    record(SourceIndex::kFlightState);
    state_ = *message;
    have_state_ = true;
    publishFlightState();
  }

  void onExtendedState(const mavros_msgs::ExtendedState::ConstPtr &message) {
    record(SourceIndex::kExtendedState);
    extended_state_ = *message;
    have_extended_state_ = true;
    publishFlightState();
  }

  void publishFlightState() {
    if (!have_state_ || !have_extended_state_)
      return;
    const ros::WallTime now = ros::WallTime::now();
    const auto &state_seen =
        sources_.at(static_cast<std::size_t>(SourceIndex::kFlightState))
            .last_seen;
    const auto &extended_seen =
        sources_.at(static_cast<std::size_t>(SourceIndex::kExtendedState))
            .last_seen;
    if ((now - state_seen).toSec() > kFlightSourceFreshSeconds ||
        (now - extended_seen).toSec() > kFlightSourceFreshSeconds ||
        !allow(UplinkChannel::kFlightState)) {
      return;
    }
    const std::int64_t timestamp =
        std::max(sourceMillis(state_.header.stamp),
                 sourceMillis(extended_state_.header.stamp));
    std::string payload;
    std::string error;
    if (!EncodeFlightState(state_, extended_state_,
                           sequence(UplinkChannel::kFlightState), timestamp,
                           &payload, &error)) {
      rejected(error);
      return;
    }
    put(UplinkChannel::kFlightState, payload);
  }

  void onHeartbeat(const ros::WallTimerEvent &) {
    const ros::WallTime now = ros::WallTime::now();
    std::vector<HeartbeatChannel> channels;
    channels.reserve(sources_.size());
    for (const auto &source : sources_) {
      const bool ready = !source.last_seen.isZero();
      channels.push_back(HeartbeatChannel{
          source.id, source.samples,
          ready ? static_cast<std::int64_t>((now - source.last_seen).toNSec() /
                                            1000000u)
                : -1,
          ready});
    }
    std::string payload;
    std::string error;
    if (!EncodeHeartbeat(
            config_.robot_id, sequence(UplinkChannel::kForwarderHeartbeat),
            systemMillis(),
            static_cast<std::int64_t>((now - started_at_).toNSec() / 1000000u),
            channels, stats_, &payload, &error)) {
      rejected(error);
      return;
    }
    put(UplinkChannel::kForwarderHeartbeat, payload);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ForwarderConfig config_;
  ZenohPublisher publisher_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber power_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber extended_state_subscriber_;
  ros::WallTimer heartbeat_timer_;
  std::array<SourceTracker, kSourceCount> sources_;
  std::array<ros::WallTime, 6u> last_publish_{};
  std::array<std::uint64_t, 6u> sequences_{};
  HeartbeatStats stats_;
  mavros_msgs::State state_;
  mavros_msgs::ExtendedState extended_state_;
  bool have_state_ = false;
  bool have_extended_state_ = false;
  ros::WallTime started_at_;
};

} // namespace
} // namespace xgc_mocap_rotor_zenoh_forwarder

int main(int argc, char **argv) {
  ros::init(argc, argv, "xgc_mocap_rotor_zenoh_forwarder");
  xgc_mocap_rotor_zenoh_forwarder::ForwarderNode node;
  std::string error;
  if (!node.Start(&error)) {
    ROS_FATAL_STREAM("Mocap Rotor Forwarder startup refused: " << error);
    return 2;
  }
  ros::spin();
  return 0;
}
