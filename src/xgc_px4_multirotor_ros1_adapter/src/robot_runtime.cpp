#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <ros/names.h>

#include "xgc/semantic/aerial/v1/control.pb.h"
#include "xgc/semantic/aerial/v1/flight.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

constexpr const char *kPx4Profile = "px4.multirotor.ros1.v1";

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

xgc2::adapter_link::OperationExecutionResult
rejected(xgc::adapter::v1::ResultCode code, const std::string &detail) {
  xgc2::adapter_link::OperationExecutionResult result;
  result.phase = xgc::adapter::v1::OPERATION_PHASE_REJECTED;
  result.code = code;
  result.detail = detail;
  return result;
}

xgc2::adapter_link::OperationExecutionResult
failed(xgc::adapter::v1::ResultCode code, const std::string &detail,
       std::int32_t native_code = 0) {
  xgc2::adapter_link::OperationExecutionResult result;
  result.phase = xgc::adapter::v1::OPERATION_PHASE_FAILED;
  result.code = code;
  result.detail = detail;
  result.native_code = native_code;
  return result;
}

xgc2::adapter_link::OperationExecutionResult
succeeded(const std::string &detail, std::int32_t native_code = 0) {
  xgc2::adapter_link::OperationExecutionResult result;
  result.phase = xgc::adapter::v1::OPERATION_PHASE_SUCCEEDED;
  result.code = xgc::adapter::v1::RESULT_CODE_OK;
  result.detail = detail;
  result.native_code = native_code;
  return result;
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

bool px4IsOnline(bool state_known, bool state_fresh, bool connected) {
  return state_known && state_fresh && connected;
}

bool isAllowedPx4Mode(const std::string &mode) {
  static const std::set<std::string> kAllowedModes = {"OFFBOARD", "POSCTL",
                                                      "ALTCTL", "STABILIZED"};
  return kAllowedModes.count(mode) != 0;
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

Px4RebootReadiness evaluatePx4RebootReadiness(bool state_known,
                                              bool state_fresh, bool connected,
                                              bool armed) {
  if (!state_known)
    return Px4RebootReadiness::kStateUnknown;
  if (!state_fresh)
    return Px4RebootReadiness::kStateStale;
  if (!connected)
    return Px4RebootReadiness::kDisconnected;
  if (armed)
    return Px4RebootReadiness::kArmed;
  return Px4RebootReadiness::kReady;
}

const char *px4RebootReadinessDetail(Px4RebootReadiness readiness) {
  switch (readiness) {
  case Px4RebootReadiness::kReady:
    return "autopilot is connected and disarmed";
  case Px4RebootReadiness::kStateUnknown:
    return "autopilot state is unknown";
  case Px4RebootReadiness::kStateStale:
    return "autopilot state is stale";
  case Px4RebootReadiness::kDisconnected:
    return "autopilot is disconnected";
  case Px4RebootReadiness::kArmed:
    return "armed autopilot cannot be rebooted";
  }
  return "autopilot reboot readiness is unknown";
}

std::shared_ptr<RobotRuntime> RobotRuntime::Create(
    ros::NodeHandle node_handle, const xgc::adapter::v1::RobotPlan &plan,
    std::uint64_t plan_revision, EnvelopeEmitter emitter, std::string *error) {
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
      pose_endpoint_(topicName(robot_namespace_, "mavros/local_position/pose")),
      velocity_endpoint_(
          topicName(robot_namespace_, "mavros/local_position/velocity_local")),
      imu_endpoint_(topicName(robot_namespace_, "mavros/imu/data")),
      power_endpoint_(topicName(robot_namespace_, "mavros/battery")),
      state_endpoint_(topicName(robot_namespace_, "mavros/state")),
      extended_state_endpoint_(
          topicName(robot_namespace_, "mavros/extended_state")),
      arm_endpoint_(topicName(robot_namespace_, "mavros/cmd/arming")),
      mode_endpoint_(topicName(robot_namespace_, "mavros/set_mode")),
      command_endpoint_(topicName(robot_namespace_, "mavros/cmd/command")) {}

RobotRuntime::~RobotRuntime() = default;

bool RobotRuntime::channelEnabled(const std::string &channel_id) const {
  return enabled_channels_.count(channel_id) != 0;
}

bool RobotRuntime::install(std::string *error) {
  if (profile_id_ != kPx4Profile) {
    if (error != nullptr) {
      *error = "unsupported profile: " + profile_id_;
    }
    return false;
  }
  installPx4();

  ROS_INFO_STREAM("XGC AdapterLink robot="
                  << robot_id_ << " profile=" << profile_id_ << " namespace=/"
                  << robot_namespace_ << " revision=" << plan_revision_);
  return true;
}

void RobotRuntime::installPx4() {
  const std::weak_ptr<RobotRuntime> weak_self = shared_from_this();
  if (channelEnabled("state.pose")) {
    ensureSourceLocked(pose_endpoint_, kPx4PoseStaleAfterSeconds);
    pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>(
        pose_endpoint_, 20,
        [weak_self](const geometry_msgs::PoseStamped::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->px4PoseCallback(message);
        });
  }
  if (channelEnabled("state.velocity")) {
    ensureSourceLocked(velocity_endpoint_, 1.0);
    velocity_subscriber_ = node_handle_.subscribe<geometry_msgs::TwistStamped>(
        velocity_endpoint_, 20,
        [weak_self](const geometry_msgs::TwistStamped::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->px4VelocityCallback(message);
        });
  }
  if (channelEnabled("state.imu")) {
    ensureSourceLocked(imu_endpoint_, 1.0);
    imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
        imu_endpoint_, 20,
        [weak_self](const sensor_msgs::Imu::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->imuCallback(message);
        });
  }
  if (channelEnabled("state.power")) {
    ensureSourceLocked(power_endpoint_, 3.0);
    power_subscriber_ = node_handle_.subscribe<sensor_msgs::BatteryState>(
        power_endpoint_, 10,
        [weak_self](const sensor_msgs::BatteryState::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->batteryCallback(message);
        });
  }
  if (channelEnabled("state.health") || channelEnabled("state.flight")) {
    ensureSourceLocked(state_endpoint_, 2.0);
    ensureSourceLocked(extended_state_endpoint_, 2.0);
    state_subscriber_ = node_handle_.subscribe<mavros_msgs::State>(
        state_endpoint_, 10,
        [weak_self](const mavros_msgs::State::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->mavrosStateCallback(message);
        });
    extended_state_subscriber_ =
        node_handle_.subscribe<mavros_msgs::ExtendedState>(
            extended_state_endpoint_, 10,
            [weak_self](const mavros_msgs::ExtendedState::ConstPtr &message) {
              if (const auto self = weak_self.lock())
                self->mavrosExtendedStateCallback(message);
            });
  }
  if (channelEnabled("operation.arm")) {
    arm_client_ =
        node_handle_.serviceClient<mavros_msgs::CommandBool>(arm_endpoint_);
  }
  if (channelEnabled("operation.mode")) {
    mode_client_ =
        node_handle_.serviceClient<mavros_msgs::SetMode>(mode_endpoint_);
  }
  if (channelEnabled("operation.autopilot-reboot")) {
    command_client_ =
        node_handle_.serviceClient<mavros_msgs::CommandLong>(command_endpoint_);
  }
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
  auto *time = envelope.mutable_source_time();
  const ros::Time effective_stamp =
      source_stamp.isZero() ? ros::Time::now() : source_stamp;
  time->set_nanoseconds(static_cast<std::int64_t>(effective_stamp.toNSec()));
  time->set_clock_domain(ros::Time::isSimTime()
                             ? xgc::v1::CLOCK_DOMAIN_SIMULATION
                             : xgc::v1::CLOCK_DOMAIN_ROS);
  envelope.set_observed_unix_nanos(
      static_cast<std::int64_t>(ros::WallTime::now().toNSec()));
  envelope.set_message_id(message_id);
  envelope.set_schema_version(metadata.version);
  envelope.set_schema_fingerprint(metadata.fingerprint);
  envelope.set_encoding(xgc::v1::PAYLOAD_ENCODING_PROTOBUF);
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
  auto &source = sources_[endpoint];
  source.stale_after_seconds = stale_after_seconds;
}

void RobotRuntime::recordSourceLocked(const std::string &endpoint,
                                      const ros::WallTime &now) {
  auto &source = sources_[endpoint];
  if (source.window_started.isZero())
    source.window_started = now;
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
  const double milliseconds = (now - it->second.last_seen).toSec() * 1000.0;
  return static_cast<std::uint64_t>(std::max(0.0, milliseconds));
}

void RobotRuntime::px4PoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &message) {
  std::vector<xgc::v1::Message> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked(pose_endpoint_, now);
    if (shouldEmitLocked("state.pose", now)) {
      xgc::semantic::common::v1::PoseEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyVector(message->pose.position, payload.mutable_position());
      copyQuaternion(message->pose.orientation, payload.mutable_orientation());
      output.push_back(makeEnvelopeLocked("state.pose", 2001,
                                          message->header.stamp, payload));
      recordOutputLocked(pose_endpoint_);
    } else {
      ++sources_[pose_endpoint_].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::px4VelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &message) {
  std::vector<xgc::v1::Message> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked(velocity_endpoint_, now);
    if (shouldEmitLocked("state.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyVector(message->twist.linear, payload.mutable_linear());
      copyVector(message->twist.angular, payload.mutable_angular());
      output.push_back(makeEnvelopeLocked("state.velocity", 2002,
                                          message->header.stamp, payload));
      recordOutputLocked(velocity_endpoint_);
    } else {
      ++sources_[velocity_endpoint_].dropped_samples;
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

void RobotRuntime::batteryCallback(
    const sensor_msgs::BatteryState::ConstPtr &message) {
  std::vector<xgc::v1::Message> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked(power_endpoint_, now);
    if (shouldEmitLocked("state.power", now)) {
      xgc::semantic::common::v1::PowerStatus payload;
      if (std::isfinite(message->percentage))
        payload.set_percentage(message->percentage);
      if (std::isfinite(message->voltage))
        payload.set_voltage_v(message->voltage);
      if (std::isfinite(message->current))
        payload.set_current_a(message->current);
      if (std::isfinite(message->temperature))
        payload.set_temperature_c(message->temperature);
      payload.set_charging(
          message->power_supply_status ==
          sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
      output.push_back(makeEnvelopeLocked("state.power", 2004,
                                          message->header.stamp, payload));
      recordOutputLocked(power_endpoint_);
    } else {
      ++sources_[power_endpoint_].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::mavrosStateCallback(
    const mavros_msgs::State::ConstPtr &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  mavros_state_ = *message;
  has_mavros_state_ = true;
  recordSourceLocked(state_endpoint_, ros::WallTime::now());
}

void RobotRuntime::mavrosExtendedStateCallback(
    const mavros_msgs::ExtendedState::ConstPtr &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  mavros_extended_state_ = *message;
  has_mavros_extended_state_ = true;
  recordSourceLocked(extended_state_endpoint_, ros::WallTime::now());
}

void RobotRuntime::emitPeriodic(const ros::WallTime &now) {
  std::vector<xgc::v1::Message> messages;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    emitPx4PeriodicLocked(now, &messages);
    emitChannelHealthLocked(now, &messages);
  }
  emit(std::move(messages));
}

void RobotRuntime::emitPx4PeriodicLocked(
    const ros::WallTime &now, std::vector<xgc::v1::Message> *messages) {
  const bool state_fresh =
      has_mavros_state_ && sourceFreshLocked(state_endpoint_, now);
  const bool extended_fresh = has_mavros_extended_state_ &&
                              sourceFreshLocked(extended_state_endpoint_, now);

  if (channelEnabled("state.health") && shouldEmitLocked("state.health", now)) {
    xgc::semantic::common::v1::VehicleHealth payload;
    const bool online =
        px4IsOnline(has_mavros_state_, state_fresh, mavros_state_.connected);
    payload.set_online(online);
    if (!has_mavros_state_) {
      payload.set_summary("MAVROS state has not been observed");
      addFault(&payload, "mavros.state.missing",
               "MAVROS state has not been observed", 2);
    } else if (!state_fresh) {
      payload.set_summary("MAVROS state is stale");
      addFault(&payload, "mavros.state.stale",
               "MAVROS state exceeded its freshness limit", 2);
    } else if (!mavros_state_.connected) {
      payload.set_summary("MAVROS is disconnected from the autopilot");
      addFault(&payload, "mavros.autopilot.disconnected",
               "MAVROS is not connected to the autopilot", 2);
    } else {
      payload.set_summary("MAVROS telemetry is healthy");
    }
    if (!has_mavros_extended_state_) {
      addFault(&payload, "mavros.extended-state.missing",
               "MAVROS extended state has not been observed", 1);
    } else if (!extended_fresh) {
      addFault(&payload, "mavros.extended-state.stale",
               "MAVROS extended state exceeded its freshness limit", 1);
    }
    const ros::Time stamp =
        has_mavros_state_ ? mavros_state_.header.stamp : ros::Time::now();
    messages->push_back(
        makeEnvelopeLocked("state.health", 2005, stamp, payload));
    recordOutputLocked(state_endpoint_);
    recordOutputLocked(extended_state_endpoint_);
  }

  if (channelEnabled("state.flight") && shouldEmitLocked("state.flight", now)) {
    xgc::semantic::aerial::v1::FlightStatus payload;
    payload.set_connected(state_fresh && mavros_state_.connected);
    payload.set_armed(state_fresh && mavros_state_.armed);
    payload.set_mode(state_fresh ? mavros_state_.mode : std::string());
    payload.set_system_status(state_fresh ? mavros_state_.system_status : 0u);
    payload.set_landed_state(
        extended_fresh ? mavros_extended_state_.landed_state : 0u);
    const ros::Time stamp =
        has_mavros_state_ ? mavros_state_.header.stamp : ros::Time::now();
    messages->push_back(
        makeEnvelopeLocked("state.flight", 3001, stamp, payload));
    recordOutputLocked(state_endpoint_);
    recordOutputLocked(extended_state_endpoint_);
  }
}

void RobotRuntime::emitChannelHealthLocked(
    const ros::WallTime &now, std::vector<xgc::v1::Message> *messages) {
  if (!channelEnabled("diagnostic.channel-health") ||
      !shouldEmitLocked("diagnostic.channel-health", now)) {
    return;
  }
  for (auto &entry : sources_) {
    auto &source = entry.second;
    if (source.window_started.isZero())
      source.window_started = now;
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

xgc2::adapter_link::OperationExecutionResult RobotRuntime::executeOperation(
    const xgc::adapter::v1::OperationRequest &request, double timeout_seconds) {
  if (profile_id_ != kPx4Profile) {
    return rejected(xgc::adapter::v1::RESULT_CODE_UNSUPPORTED,
                    "profile does not support operations");
  }
  const auto &envelope = request.message();
  const double timeout = std::max(0.001, std::min(5.0, timeout_seconds));

  if (envelope.channel_id() == "operation.arm") {
    xgc::semantic::aerial::v1::ArmRequest payload;
    if (!payload.ParseFromString(envelope.payload())) {
      return rejected(xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT,
                      "ArmRequest payload is invalid");
    }
    if (!arm_client_.waitForExistence(ros::Duration(timeout))) {
      return failed(xgc::adapter::v1::RESULT_CODE_NOT_READY,
                    "MAVROS arming service is unavailable");
    }
    mavros_msgs::CommandBool service;
    service.request.value = payload.armed();
    if (!arm_client_.call(service)) {
      return failed(xgc::adapter::v1::RESULT_CODE_TRANSPORT_ERROR,
                    "MAVROS arming service call failed");
    }
    if (!service.response.success) {
      return failed(xgc::adapter::v1::RESULT_CODE_REJECTED,
                    "autopilot rejected arming state", service.response.result);
    }
    return succeeded(payload.armed() ? "autopilot armed" : "autopilot disarmed",
                     service.response.result);
  }

  if (envelope.channel_id() == "operation.mode") {
    xgc::semantic::aerial::v1::ModeRequest payload;
    if (!payload.ParseFromString(envelope.payload()) ||
        payload.mode().empty()) {
      return rejected(xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT,
                      "ModeRequest must contain a non-empty mode");
    }
    if (!isAllowedPx4Mode(payload.mode())) {
      return rejected(
          xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT,
          "PX4 mode must be OFFBOARD, POSCTL, ALTCTL, or STABILIZED");
    }
    if (!mode_client_.waitForExistence(ros::Duration(timeout))) {
      return failed(xgc::adapter::v1::RESULT_CODE_NOT_READY,
                    "MAVROS set-mode service is unavailable");
    }
    mavros_msgs::SetMode service;
    service.request.base_mode = 0;
    service.request.custom_mode = payload.mode();
    if (!mode_client_.call(service)) {
      return failed(xgc::adapter::v1::RESULT_CODE_TRANSPORT_ERROR,
                    "MAVROS set-mode service call failed");
    }
    if (!service.response.mode_sent) {
      return failed(xgc::adapter::v1::RESULT_CODE_REJECTED,
                    "autopilot rejected requested mode");
    }
    return succeeded("autopilot mode request accepted");
  }

  if (envelope.channel_id() == "operation.autopilot-reboot") {
    xgc::semantic::aerial::v1::AutopilotRebootRequest payload;
    if (!envelope.payload().empty() ||
        !payload.ParseFromString(envelope.payload())) {
      return rejected(xgc::adapter::v1::RESULT_CODE_INVALID_ARGUMENT,
                      "AutopilotRebootRequest must have an empty payload");
    }
    Px4RebootReadiness reboot_readiness;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const ros::WallTime now = ros::WallTime::now();
      reboot_readiness = evaluatePx4RebootReadiness(
          has_mavros_state_,
          has_mavros_state_ && sourceFreshLocked(state_endpoint_, now),
          has_mavros_state_ && mavros_state_.connected,
          has_mavros_state_ && mavros_state_.armed);
    }
    if (reboot_readiness != Px4RebootReadiness::kReady) {
      const auto code = reboot_readiness == Px4RebootReadiness::kArmed
                            ? xgc::adapter::v1::RESULT_CODE_REJECTED
                            : xgc::adapter::v1::RESULT_CODE_NOT_READY;
      return rejected(code, px4RebootReadinessDetail(reboot_readiness));
    }
    if (!command_client_.waitForExistence(ros::Duration(timeout))) {
      return failed(xgc::adapter::v1::RESULT_CODE_NOT_READY,
                    "MAVROS command service is unavailable");
    }
    mavros_msgs::CommandLong service;
    service.request.broadcast = false;
    service.request.command = 246;
    service.request.confirmation = 0;
    service.request.param1 = 1.0;
    if (!command_client_.call(service)) {
      return failed(xgc::adapter::v1::RESULT_CODE_TRANSPORT_ERROR,
                    "MAVROS reboot command service call failed");
    }
    if (!service.response.success) {
      return failed(xgc::adapter::v1::RESULT_CODE_REJECTED,
                    "autopilot rejected reboot command",
                    service.response.result);
    }
    return succeeded("autopilot rebooting", service.response.result);
  }

  return rejected(xgc::adapter::v1::RESULT_CODE_UNSUPPORTED,
                  "operation channel is not implemented by the PX4 profile");
}

} // namespace xgc_px4_multirotor_ros1_adapter
