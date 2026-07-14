#include "xgc_scout_mini_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <ros/names.h>

#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"

namespace xgc_scout_mini_ros1_adapter {
namespace {

constexpr const char *kProfile = "scout-mini.ros1.v1";

std::string cleanTopicPart(const std::string &value) {
  std::size_t begin = 0;
  std::size_t end = value.size();
  while (begin < end && value[begin] == '/') {
    ++begin;
  }
  while (end > begin && value[end - 1] == '/') {
    --end;
  }
  return value.substr(begin, end - begin);
}

template <typename RosVector>
void copyVector(const RosVector &source,
                xgc::semantic::common::v1::Vector3 *target) {
  target->set_x(source.x);
  target->set_y(source.y);
  target->set_z(source.z);
}

template <typename RosQuaternion>
void copyQuaternion(const RosQuaternion &source,
                    xgc::semantic::common::v1::Quaternion *target) {
  target->set_x(source.x);
  target->set_y(source.y);
  target->set_z(source.z);
  target->set_w(source.w);
}

template <typename Covariance>
void copyCovariance(const Covariance &source,
                    google::protobuf::RepeatedField<double> *target) {
  target->Reserve(static_cast<int>(source.size()));
  for (const double value : source) {
    target->Add(value);
  }
}

void addFault(xgc::semantic::common::v1::VehicleHealth *health,
              const std::string &code, const std::string &summary,
              std::uint32_t severity) {
  auto *fault = health->add_faults();
  fault->set_code(code);
  fault->set_summary(summary);
  fault->set_severity(severity);
}

} // namespace

std::string topicName(const std::string &robot_namespace,
                      const std::string &relative_name) {
  const std::string clean_namespace = cleanTopicPart(robot_namespace);
  const std::string clean_name = cleanTopicPart(relative_name);
  if (clean_namespace.empty()) {
    return "/" + clean_name;
  }
  return "/" + clean_namespace + "/" + clean_name;
}

bool validRobotNamespace(const std::string &value, std::string *error) {
  if (value.empty() || value == "/") {
    if (error != nullptr) {
      *error = "namespace must not be empty or root";
    }
    return false;
  }
  if (value.front() != '/') {
    if (error != nullptr) {
      *error = "namespace must be absolute";
    }
    return false;
  }
  if (value.back() == '/') {
    if (error != nullptr) {
      *error = "namespace must not have a trailing slash";
    }
    return false;
  }
  if (value.find("//") != std::string::npos) {
    if (error != nullptr) {
      *error = "namespace must not contain repeated slashes";
    }
    return false;
  }
  std::string ros_error;
  if (!ros::names::validate(value, ros_error)) {
    if (error != nullptr) {
      *error = ros_error;
    }
    return false;
  }
  return true;
}

bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds) {
  if (last_seen.isZero() || stale_after_seconds <= 0.0 || now < last_seen) {
    return false;
  }
  return (now - last_seen).toSec() <= stale_after_seconds;
}

bool scoutIsOnline(bool odometry_fresh, bool status_fresh) {
  return odometry_fresh && status_fresh;
}

bool validateNonEmptyAdapterPlan(const xgc::adapter::v1::AdapterPlan &plan,
                                 std::string *error) {
  if (plan.robots_size() != 0) {
    return true;
  }
  if (error != nullptr) {
    *error = "adapter plan must contain at least one robot";
  }
  return false;
}

std::shared_ptr<RobotRuntime> RobotRuntime::Create(
    ros::NodeHandle node_handle, const xgc::adapter::v1::RobotPlan &plan,
    std::uint64_t plan_revision, EnvelopeEmitter emitter, std::string *error) {
  if (plan.profile_id() != kProfile) {
    if (error != nullptr) {
      *error = "unsupported profile: " + plan.profile_id();
    }
    return nullptr;
  }
  const auto namespace_it = plan.parameters().find("namespace");
  if (namespace_it == plan.parameters().end()) {
    if (error != nullptr) {
      *error = "robot plan is missing required namespace parameter";
    }
    return nullptr;
  }
  std::string namespace_error;
  if (!validRobotNamespace(namespace_it->second, &namespace_error)) {
    if (error != nullptr) {
      *error = "invalid ROS namespace: " + namespace_error;
    }
    return nullptr;
  }

  std::set<std::string> enabled_channels;
  for (const auto &channel : plan.channels()) {
    if (channel.enabled()) {
      enabled_channels.insert(channel.channel_id());
    }
  }

  auto runtime = std::shared_ptr<RobotRuntime>(
      new RobotRuntime(std::move(node_handle), plan.robot_id(),
                       plan.profile_id(), namespace_it->second, plan_revision,
                       std::move(enabled_channels), std::move(emitter)));
  if (!runtime->install(error)) {
    return nullptr;
  }
  return runtime;
}

RobotRuntime::RobotRuntime(ros::NodeHandle node_handle, std::string robot_id,
                           std::string profile_id, std::string robot_namespace,
                           std::uint64_t plan_revision,
                           std::set<std::string> enabled_channels,
                           EnvelopeEmitter emitter)
    : node_handle_(std::move(node_handle)), robot_id_(std::move(robot_id)),
      profile_id_(std::move(profile_id)),
      robot_namespace_(cleanTopicPart(robot_namespace)),
      plan_revision_(plan_revision),
      enabled_channels_(std::move(enabled_channels)),
      emitter_(std::move(emitter)),
      odometry_endpoint_(topicName(robot_namespace_, "odom")),
      imu_endpoint_(topicName(robot_namespace_, "imu/data_raw")),
      status_endpoint_(topicName(robot_namespace_, "scout_status")) {}

RobotRuntime::~RobotRuntime() = default;

bool RobotRuntime::channelEnabled(const std::string &channel_id) const {
  return enabled_channels_.count(channel_id) != 0;
}

bool RobotRuntime::install(std::string *error) {
  if (profile_id_ != kProfile) {
    if (error != nullptr) {
      *error = "unsupported profile: " + profile_id_;
    }
    return false;
  }

  const std::weak_ptr<RobotRuntime> weak_self = shared_from_this();
  if (channelEnabled("state.pose") || channelEnabled("state.velocity") ||
      channelEnabled("state.health")) {
    ensureSourceLocked(odometry_endpoint_, 1.0);
    odometry_subscriber_ = node_handle_.subscribe<nav_msgs::Odometry>(
        odometry_endpoint_, 20,
        [weak_self](const nav_msgs::Odometry::ConstPtr &message) {
          if (const auto self = weak_self.lock()) {
            self->odometryCallback(message);
          }
        });
  }
  if (channelEnabled("state.imu")) {
    ensureSourceLocked(imu_endpoint_, 1.0);
    imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
        imu_endpoint_, 20,
        [weak_self](const sensor_msgs::Imu::ConstPtr &message) {
          if (const auto self = weak_self.lock()) {
            self->imuCallback(message);
          }
        });
  }
  if (channelEnabled("state.power") || channelEnabled("state.health")) {
    ensureSourceLocked(status_endpoint_, kScoutStatusStaleAfterSeconds);
    status_subscriber_ = node_handle_.subscribe<scout_msgs::ScoutStatus>(
        status_endpoint_, 10,
        [weak_self](const scout_msgs::ScoutStatus::ConstPtr &message) {
          if (const auto self = weak_self.lock()) {
            self->statusCallback(message);
          }
        });
  }

  ROS_INFO_STREAM("XGC Scout Mini AdapterLink robot="
                  << robot_id_ << " namespace=/" << robot_namespace_
                  << " revision=" << plan_revision_);
  return true;
}

bool RobotRuntime::shouldEmitLocked(const std::string &channel_id,
                                    const ros::WallTime &now) {
  contract::ChannelMetadata metadata;
  if (!contract::channelMetadata(profile_id_, channel_id, &metadata) ||
      metadata.output_rate_hz <= 0.0) {
    return false;
  }
  auto &last = last_output_[channel_id];
  if (!last.isZero() && now >= last &&
      (now - last).toSec() < (1.0 / metadata.output_rate_hz)) {
    return false;
  }
  last = now;
  return true;
}

xgc::v1::Message RobotRuntime::makeEnvelopeLocked(
    const std::string &channel_id, std::uint32_t message_id,
    const ros::Time &source_stamp, const google::protobuf::Message &payload) {
  contract::MessageMetadata metadata;
  if (!contract::messageMetadata(message_id, &metadata)) {
    throw std::logic_error(
        "message ID is absent from generated XGC2 contract metadata");
  }
  xgc::v1::Message envelope;
  envelope.set_robot_id(robot_id_);
  envelope.set_channel_id(channel_id);
  envelope.set_sequence(++sequences_[channel_id]);
  if (!source_stamp.isZero()) {
    auto *time = envelope.mutable_source_time();
    time->set_nanoseconds(static_cast<std::int64_t>(source_stamp.toNSec()));
    time->set_clock_domain(ros::Time::isSimTime()
                               ? xgc::v1::CLOCK_DOMAIN_SIMULATION
                               : xgc::v1::CLOCK_DOMAIN_ROS);
  }
  envelope.set_observed_unix_nanos(
      static_cast<std::int64_t>(ros::WallTime::now().toNSec()));
  envelope.set_message_id(message_id);
  if (!payload.SerializeToString(envelope.mutable_payload())) {
    throw std::runtime_error("failed to serialize semantic telemetry payload");
  }
  return envelope;
}

void RobotRuntime::emit(std::vector<xgc::v1::Message> messages) {
  for (auto &message : messages) {
    emitter_(plan_revision_, std::move(message));
  }
}

void RobotRuntime::ensureSourceLocked(const std::string &endpoint,
                                      double stale_after_seconds) {
  sources_[endpoint].stale_after_seconds = stale_after_seconds;
}

void RobotRuntime::recordSourceLocked(const std::string &endpoint,
                                      const ros::WallTime &now) {
  auto &source = sources_[endpoint];
  if (source.window_started.isZero()) {
    source.window_started = now;
  }
  source.last_seen = now;
  ++source.source_samples;
}

void RobotRuntime::recordOutputLocked(const std::string &endpoint) {
  ++sources_[endpoint].output_samples;
}

bool RobotRuntime::sourceFreshLocked(const std::string &endpoint,
                                     const ros::WallTime &now) const {
  const auto it = sources_.find(endpoint);
  return it != sources_.end() && sourceIsFresh(it->second.last_seen, now,
                                               it->second.stale_after_seconds);
}

std::uint64_t
RobotRuntime::sourceAgeMillisLocked(const std::string &endpoint,
                                    const ros::WallTime &now) const {
  const auto it = sources_.find(endpoint);
  if (it == sources_.end() || it->second.last_seen.isZero() ||
      now < it->second.last_seen) {
    return 0;
  }
  return static_cast<std::uint64_t>(
      std::max(0.0, (now - it->second.last_seen).toSec() * 1000.0));
}

void RobotRuntime::odometryCallback(
    const nav_msgs::Odometry::ConstPtr &message) {
  std::vector<xgc::v1::Message> output;
  const ros::WallTime now = ros::WallTime::now();
  bool emitted = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_odometry_ = true;
    recordSourceLocked(odometry_endpoint_, now);
    if (channelEnabled("state.pose") && shouldEmitLocked("state.pose", now)) {
      xgc::semantic::common::v1::PoseEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      payload.set_child_frame_id(message->child_frame_id);
      copyVector(message->pose.pose.position, payload.mutable_position());
      copyQuaternion(message->pose.pose.orientation,
                     payload.mutable_orientation());
      copyCovariance(message->pose.covariance, payload.mutable_covariance());
      output.push_back(makeEnvelopeLocked("state.pose", 2001,
                                          message->header.stamp, payload));
      emitted = true;
    }
    if (channelEnabled("state.velocity") &&
        shouldEmitLocked("state.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      payload.set_frame_id(message->child_frame_id);
      copyVector(message->twist.twist.linear, payload.mutable_linear());
      copyVector(message->twist.twist.angular, payload.mutable_angular());
      copyCovariance(message->twist.covariance, payload.mutable_covariance());
      output.push_back(makeEnvelopeLocked("state.velocity", 2002,
                                          message->header.stamp, payload));
      emitted = true;
    }
    if (emitted) {
      recordOutputLocked(odometry_endpoint_);
    } else {
      ++sources_[odometry_endpoint_].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::imuCallback(const sensor_msgs::Imu::ConstPtr &message) {
  std::vector<xgc::v1::Message> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked(imu_endpoint_, now);
    if (shouldEmitLocked("state.imu", now)) {
      xgc::semantic::common::v1::ImuEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyQuaternion(message->orientation, payload.mutable_orientation());
      copyVector(message->angular_velocity, payload.mutable_angular_velocity());
      copyVector(message->linear_acceleration,
                 payload.mutable_linear_acceleration());
      copyCovariance(message->orientation_covariance,
                     payload.mutable_orientation_covariance());
      copyCovariance(message->angular_velocity_covariance,
                     payload.mutable_angular_velocity_covariance());
      copyCovariance(message->linear_acceleration_covariance,
                     payload.mutable_linear_acceleration_covariance());
      output.push_back(makeEnvelopeLocked("state.imu", 2003,
                                          message->header.stamp, payload));
      recordOutputLocked(imu_endpoint_);
    } else {
      ++sources_[imu_endpoint_].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::statusCallback(
    const scout_msgs::ScoutStatus::ConstPtr &message) {
  std::vector<xgc::v1::Message> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scout_status_ = *message;
    has_status_ = true;
    recordSourceLocked(status_endpoint_, now);
    if (channelEnabled("state.power") && shouldEmitLocked("state.power", now)) {
      xgc::semantic::common::v1::PowerStatus payload;
      if (std::isfinite(message->battery_voltage)) {
        payload.set_voltage_v(message->battery_voltage);
      }
      output.push_back(makeEnvelopeLocked("state.power", 2004,
                                          message->header.stamp, payload));
      recordOutputLocked(status_endpoint_);
    } else if (channelEnabled("state.power")) {
      ++sources_[status_endpoint_].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::emitPeriodic(const ros::WallTime &now) {
  std::vector<xgc::v1::Message> messages;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    emitHealthLocked(now, &messages);
    emitChannelHealthLocked(now, &messages);
  }
  emit(std::move(messages));
}

void RobotRuntime::emitHealthLocked(const ros::WallTime &now,
                                    std::vector<xgc::v1::Message> *messages) {
  if (!channelEnabled("state.health") ||
      !shouldEmitLocked("state.health", now)) {
    return;
  }

  const bool odometry_fresh =
      has_odometry_ && sourceFreshLocked(odometry_endpoint_, now);
  const bool status_fresh =
      has_status_ && sourceFreshLocked(status_endpoint_, now);
  xgc::semantic::common::v1::VehicleHealth payload;
  payload.set_online(scoutIsOnline(odometry_fresh, status_fresh));
  if (!has_odometry_) {
    addFault(&payload, "scout.odometry.missing",
             "Scout odometry has not been observed", 2);
  } else if (!odometry_fresh) {
    addFault(&payload, "scout.odometry.stale",
             "Scout odometry exceeded its freshness limit", 2);
  }
  if (!has_status_) {
    addFault(&payload, "scout.status.missing",
             "Scout chassis status has not been observed", 2);
  } else if (!status_fresh) {
    addFault(&payload, "scout.status.stale",
             "Scout chassis status exceeded its freshness limit", 2);
  } else if (scout_status_.fault_code != 0) {
    addFault(&payload, "scout.chassis.fault",
             "Scout chassis reports fault code " +
                 std::to_string(scout_status_.fault_code),
             2);
  }
  if (payload.online() && payload.faults_size() == 0) {
    payload.set_summary("Scout telemetry is healthy");
  } else if (has_status_ && scout_status_.fault_code != 0) {
    payload.set_summary("Scout chassis reports a fault");
  } else {
    payload.set_summary("Scout telemetry is incomplete or stale");
  }
  const ros::Time stamp =
      has_status_ ? scout_status_.header.stamp : ros::Time::now();
  messages->push_back(makeEnvelopeLocked("state.health", 2005, stamp, payload));
  recordOutputLocked(odometry_endpoint_);
  recordOutputLocked(status_endpoint_);
}

void RobotRuntime::emitChannelHealthLocked(
    const ros::WallTime &now, std::vector<xgc::v1::Message> *messages) {
  if (!channelEnabled("diagnostic.channel-health") ||
      !shouldEmitLocked("diagnostic.channel-health", now)) {
    return;
  }
  for (auto &entry : sources_) {
    auto &source = entry.second;
    if (source.window_started.isZero()) {
      source.window_started = now;
    }
    const double elapsed = now >= source.window_started
                               ? (now - source.window_started).toSec()
                               : 0.0;
    if (elapsed > 0.0) {
      source.source_rate_hz =
          static_cast<double>(source.source_samples) / elapsed;
      source.output_rate_hz =
          static_cast<double>(source.output_samples) / elapsed;
    }
    source.source_samples = 0;
    source.output_samples = 0;
    source.window_started = now;

    xgc::semantic::common::v1::ChannelHealth payload;
    payload.set_channel_id(entry.first);
    payload.set_source_rate_hz(source.source_rate_hz);
    payload.set_output_rate_hz(source.output_rate_hz);
    payload.set_dropped_samples(source.dropped_samples);
    payload.set_source_age_ms(sourceAgeMillisLocked(entry.first, now));
    payload.set_stale(
        !sourceIsFresh(source.last_seen, now, source.stale_after_seconds));
    messages->push_back(makeEnvelopeLocked("diagnostic.channel-health", 2010,
                                           ros::Time::now(), payload));
  }
}

} // namespace xgc_scout_mini_ros1_adapter
