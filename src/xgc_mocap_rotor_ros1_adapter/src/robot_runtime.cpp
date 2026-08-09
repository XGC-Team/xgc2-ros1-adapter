#include "xgc_mocap_rotor_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <utility>

#include <boost/array.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <nlohmann/json.hpp>

#include "xgc/semantic/aerial/v1/flight.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc_mocap_rotor_ros1_adapter/generated_contract.hpp"

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

using Json = nlohmann::json;
constexpr std::size_t kMaximumPathPoses = 2000u;
constexpr std::size_t kMaximumFaults = 32u;
constexpr std::size_t kMaximumFaultTextBytes = 256u;

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

template <typename T> T required(const Json &value, const char *name) {
  return value.at(name).get<T>();
}

double finiteNumber(const Json &value, const char *name) {
  const double result = required<double>(value, name);
  if (!std::isfinite(result))
    throw std::runtime_error(std::string(name) + " must be finite");
  return result;
}

double nullableFiniteNumber(const Json &value, const char *name) {
  const auto &item = value.at(name);
  if (item.is_null())
    return std::numeric_limits<double>::quiet_NaN();
  const double result = item.get<double>();
  if (!std::isfinite(result))
    throw std::runtime_error(std::string(name) + " must be finite or null");
  return result;
}

void requireObjectSize(const Json &value, std::size_t maximum,
                       const char *label) {
  if (!value.is_object() || value.size() > maximum)
    throw std::runtime_error(std::string(label) + " is not a bounded object");
}

void copyVector(const Json &source,
                xgc::semantic::common::v1::Vector3 *semantic,
                geometry_msgs::Vector3 *ros_vector = nullptr) {
  requireObjectSize(source, 3u, "vector");
  const double x = finiteNumber(source, "x");
  const double y = finiteNumber(source, "y");
  const double z = finiteNumber(source, "z");
  semantic->set_x(x);
  semantic->set_y(y);
  semantic->set_z(z);
  if (ros_vector != nullptr) {
    ros_vector->x = x;
    ros_vector->y = y;
    ros_vector->z = z;
  }
}

void copyPoint(const Json &source, xgc::semantic::common::v1::Vector3 *semantic,
               geometry_msgs::Point *point) {
  geometry_msgs::Vector3 vector;
  copyVector(source, semantic, &vector);
  point->x = vector.x;
  point->y = vector.y;
  point->z = vector.z;
}

void copyQuaternion(const Json &source,
                    xgc::semantic::common::v1::Quaternion *semantic,
                    geometry_msgs::Quaternion *ros_quaternion) {
  requireObjectSize(source, 4u, "quaternion");
  double x = finiteNumber(source, "x");
  double y = finiteNumber(source, "y");
  double z = finiteNumber(source, "z");
  double w = finiteNumber(source, "w");
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm < 0.5 || norm > 1.5)
    throw std::runtime_error("quaternion norm is outside the safe range");
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  semantic->set_x(x);
  semantic->set_y(y);
  semantic->set_z(z);
  semantic->set_w(w);
  ros_quaternion->x = x;
  ros_quaternion->y = y;
  ros_quaternion->z = z;
  ros_quaternion->w = w;
}

template <typename RepeatedField>
void copyCovariance(const Json &source, RepeatedField *semantic,
                    boost::array<double, 9u> *ros_covariance) {
  if (!source.is_array() || source.size() != 9u)
    throw std::runtime_error("IMU covariance must contain exactly 9 values");
  for (std::size_t index = 0; index < source.size(); ++index) {
    const double value = source.at(index).get<double>();
    if (!std::isfinite(value))
      throw std::runtime_error("IMU covariance values must be finite");
    semantic->Add(value);
    (*ros_covariance)[index] = value;
  }
}

std::int64_t sourceTimeMillis(const Json &value) {
  const auto millis = required<std::int64_t>(value, "t_ms");
  if (millis <= 0 ||
      millis > std::numeric_limits<std::int64_t>::max() / 1000000)
    throw std::runtime_error("source t_ms is out of range");
  return millis;
}

std::uint64_t wireSequence(const Json &value) {
  const auto sequence = required<std::uint64_t>(value, "sequence");
  if (sequence == 0u)
    throw std::runtime_error("wire sequence must be positive");
  return sequence;
}

void requireVersion(const Json &value) {
  requireObjectSize(value, 32u, "wire payload");
  if (required<int>(value, "v") != 1)
    throw std::runtime_error("unsupported Mocap Rotor wire version");
}

ros::Time rosStamp(std::int64_t millis) {
  ros::Time result;
  result.fromNSec(static_cast<std::uint64_t>(millis) * 1000000u);
  return result;
}

std::string canonicalFrame(const std::string &value) {
  static const std::regex pattern(
      "^[A-Za-z][A-Za-z0-9_.\\/-]{0,126}[A-Za-z0-9_]$");
  if (value.size() < 2u || !std::regex_match(value, pattern) ||
      value.find("//") != std::string::npos ||
      value.find("..") != std::string::npos) {
    throw std::runtime_error("wire frame_id is not canonical");
  }
  return value;
}

std::string qualifyFrame(const std::string &frame_prefix,
                         const std::string &source, bool child) {
  const std::string frame = canonicalFrame(source);
  if (!child && (frame == "world" || frame == "map"))
    return frame;
  if (frame.compare(0, frame_prefix.size(), frame_prefix) == 0)
    return frame;
  return frame_prefix + frame;
}

double staleSeconds(const std::string &profile_id,
                    const std::string &channel_id) {
  contract::ChannelMetadata metadata{};
  if (!contract::channelMetadata(profile_id, channel_id, &metadata) ||
      metadata.stale_after_millis == 0u) {
    throw std::logic_error("missing Mocap Rotor freshness metadata for " +
                           channel_id);
  }
  return static_cast<double>(metadata.stale_after_millis) / 1000.0;
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

bool ValidateRobotNamespace(const std::string &value, std::string *error) {
  static const std::regex pattern("^/[A-Za-z_][A-Za-z0-9_]*$");
  if (!std::regex_match(value, pattern))
    return fail(error, "namespace must be one absolute Experiment slot name");
  return true;
}

bool ShouldResetWireEpoch(std::uint64_t previous_sequence,
                          std::uint64_t incoming_sequence,
                          std::int64_t previous_source_time_millis,
                          std::int64_t incoming_source_time_millis,
                          std::int64_t previous_uptime_millis,
                          std::int64_t incoming_uptime_millis,
                          double link_age_seconds, double stale_after_seconds) {
  if (previous_sequence == 0u || incoming_sequence > previous_sequence ||
      previous_source_time_millis <= 0 ||
      incoming_source_time_millis <= previous_source_time_millis ||
      incoming_uptime_millis < 0 || link_age_seconds < 0.0 ||
      stale_after_seconds <= 0.0) {
    return false;
  }
  const bool uptime_restarted = previous_uptime_millis >= 0 &&
                                incoming_uptime_millis < previous_uptime_millis;
  return uptime_restarted || link_age_seconds > stale_after_seconds;
}

bool ValidateNativeProfileContract(std::string *error) {
  std::size_t parameter_count = 0u;
  const auto *parameters =
      contract::profileParameters(contract::kProfileId, &parameter_count);
  if (parameters == nullptr || parameter_count != 4u)
    return fail(error,
                "Mocap Rotor profile must expose four runtime parameters");
  std::size_t channel_count = 0u;
  const auto *channels =
      contract::profileChannels(contract::kProfileId, &channel_count);
  if (channels == nullptr || channel_count != 9u)
    return fail(error,
                "Mocap Rotor profile must expose nine read-only channels");
  for (std::size_t index = 0u; index < channel_count; ++index) {
    if (channels[index].kind != contract::ChannelKind::kStreamOut ||
        channels[index].output_message_id == 0u ||
        channels[index].input_message_id != 0u ||
        channels[index].operation_timeout_millis != 0u) {
      return fail(
          error, "Mocap Rotor profile contains a command or incomplete stream");
    }
    contract::MessageMetadata message{};
    if (!contract::messageMetadata(channels[index].output_message_id,
                                   &message) ||
        message.type_name == nullptr || message.version == 0u ||
        message.fingerprint == 0u) {
      return fail(error,
                  "Mocap Rotor output schema is absent from the registry");
    }
  }
  return true;
}

std::shared_ptr<RobotRuntime>
RobotRuntime::Create(ros::NodeHandle node_handle,
                     const xgc2_ros1_robot_adapter::RobotConfig &config,
                     std::uint64_t spec_revision, EnvelopeEmitter emitter,
                     std::string *error) {
  if (!ValidateNativeProfileContract(error))
    return nullptr;
  if (config.profile_id != contract::kProfileId)
    return fail(error, "unsupported Mocap Rotor profile: " + config.profile_id),
           nullptr;
  const auto robot_namespace = config.parameters.find("namespace");
  const auto robot_id = config.parameters.find("robot_id");
  const auto transport = config.parameters.find("wire_transport");
  const auto listen = config.parameters.find("zenoh_listen");
  if (config.parameters.size() != 4u ||
      robot_namespace == config.parameters.end() ||
      robot_id == config.parameters.end() ||
      transport == config.parameters.end() ||
      listen == config.parameters.end()) {
    return fail(error, "Mocap Rotor requires exactly namespace, robot_id, "
                       "wire_transport, zenoh_listen"),
           nullptr;
  }
  if (robot_id->second != config.robot_id)
    return fail(error, "wire robot_id must equal Adapter Runtime robot id"),
           nullptr;
  if (transport->second != "zenoh")
    return fail(error,
                "Mocap Rotor requires Zenoh and has no fallback transport"),
           nullptr;
  if (!ValidateRobotNamespace(robot_namespace->second, error) ||
      !ValidateZenohListenEndpoint(listen->second, error))
    return nullptr;

  std::set<std::string> enabled;
  for (const auto &channel : config.channels) {
    contract::ChannelMetadata metadata{};
    if (!contract::channelMetadata(config.profile_id, channel.channel_id,
                                   &metadata)) {
      return fail(error, "unknown Mocap Rotor channel: " + channel.channel_id),
             nullptr;
    }
    if (channel.enabled)
      enabled.insert(channel.channel_id);
  }
  static const std::set<std::string> baseline{
      "state.pose",   "state.velocity",  "state.speed",
      "state.imu",    "state.power",     "state.health",
      "state.flight", "diagnostic.link", "diagnostic.stream-health"};
  for (const auto &channel : baseline) {
    if (enabled.count(channel) == 0u)
      return fail(error,
                  "Mocap Rotor baseline channel is disabled: " + channel),
             nullptr;
  }

  auto runtime = std::shared_ptr<RobotRuntime>(
      new RobotRuntime(std::move(node_handle), config.robot_id,
                       config.profile_id, robot_namespace->second,
                       spec_revision, std::move(enabled), std::move(emitter)));
  if (!runtime->install(error)) {
    runtime->Stop();
    return nullptr;
  }
  return runtime;
}

RobotRuntime::RobotRuntime(ros::NodeHandle node_handle, std::string robot_id,
                           std::string profile_id, std::string robot_namespace,
                           std::uint64_t spec_revision,
                           std::set<std::string> enabled_channels,
                           EnvelopeEmitter emitter)
    : node_handle_(std::move(node_handle)), robot_id_(std::move(robot_id)),
      profile_id_(std::move(profile_id)),
      robot_namespace_(std::move(robot_namespace)),
      frame_prefix_(robot_namespace_.substr(1) + "/"),
      spec_revision_(spec_revision),
      enabled_channels_(std::move(enabled_channels)),
      emitter_(std::move(emitter)) {
  for (const auto &channel : enabled_channels_)
    sources_[channel].stale_after_seconds = staleSeconds(profile_id_, channel);
  path_.header.frame_id = "world";
}

RobotRuntime::~RobotRuntime() { Stop(); }

bool RobotRuntime::install(std::string *error) {
  if (!emitter_)
    return fail(error, "Mocap Rotor telemetry emitter is unavailable");
  ros::NodeHandle robot_node(robot_namespace_);
  pose_publisher_ =
      robot_node.advertise<geometry_msgs::PoseStamped>("local_pose", 20);
  velocity_publisher_ =
      robot_node.advertise<geometry_msgs::TwistStamped>("local_velocity", 20);
  odometry_publisher_ = robot_node.advertise<nav_msgs::Odometry>("odom", 20);
  path_publisher_ = robot_node.advertise<nav_msgs::Path>("path", 5, true);
  imu_publisher_ = robot_node.advertise<sensor_msgs::Imu>("imu", 20);
  power_publisher_ =
      robot_node.advertise<sensor_msgs::BatteryState>("power", 10);
  flight_state_publisher_ =
      robot_node.advertise<std_msgs::String>("flight_state_json", 10);
  forwarder_status_publisher_ =
      robot_node.advertise<std_msgs::String>("forwarder_status_json", 10);
  ROS_INFO_STREAM("XGC Mocap Rotor Adapter robot="
                  << robot_id_ << " namespace=" << robot_namespace_
                  << " transport=zenoh revision=" << spec_revision_);
  return true;
}

void RobotRuntime::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_)
    return;
  stopping_ = true;
  pose_publisher_.shutdown();
  velocity_publisher_.shutdown();
  odometry_publisher_.shutdown();
  path_publisher_.shutdown();
  imu_publisher_.shutdown();
  power_publisher_.shutdown();
  flight_state_publisher_.shutdown();
  forwarder_status_publisher_.shutdown();
}

bool RobotRuntime::HandleWireFrame(WireChannel channel,
                                   const std::string &payload,
                                   std::string *error) {
  if (payload.empty() || payload.size() > MaximumPayloadBytes(channel)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++rejected_frames_;
    return fail(error, "wire payload is empty or exceeds its channel bound");
  }
  const ros::WallTime received = ros::WallTime::now();
  try {
    switch (channel) {
    case WireChannel::kLocalPose:
      return handlePose(received, payload, error);
    case WireChannel::kLocalVelocity:
      return handleVelocity(received, payload, error);
    case WireChannel::kImu:
      return handleImu(received, payload, error);
    case WireChannel::kPower:
      return handlePower(received, payload, error);
    case WireChannel::kFlightState:
      return handleFlightState(received, payload, error);
    case WireChannel::kForwarderHeartbeat:
      return handleHeartbeat(received, payload, error);
    }
  } catch (const std::exception &exception) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++decode_errors_;
    return fail(error, exception.what());
  }
  std::lock_guard<std::mutex> lock(mutex_);
  ++decode_errors_;
  return fail(error, "unknown Mocap Rotor wire channel");
}

bool RobotRuntime::handlePose(const ros::WallTime &received,
                              const std::string &payload, std::string *error) {
  const Json body = Json::parse(payload);
  requireVersion(body);
  const auto sequence = wireSequence(body);
  const auto source_ms = sourceTimeMillis(body);
  const std::string parent = qualifyFrame(
      frame_prefix_, required<std::string>(body, "frame_id"), false);
  const std::string child = qualifyFrame(
      frame_prefix_, required<std::string>(body, "child_frame_id"), true);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = rosStamp(source_ms);
  pose.header.frame_id = parent;
  xgc::semantic::common::v1::PoseEstimate semantic;
  semantic.set_frame_id(parent);
  semantic.set_child_frame_id(child);
  copyPoint(body.at("position"), semantic.mutable_position(),
            &pose.pose.position);
  copyQuaternion(body.at("orientation"), semantic.mutable_orientation(),
                 &pose.pose.orientation);

  nav_msgs::Odometry odometry;
  nav_msgs::Path path;
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ ||
        !acceptWireSequenceLocked(WireChannel::kLocalPose, sequence, error))
      return false;
    recordSourceLocked("state.pose", received);
    if (channelEnabled("state.pose") &&
        shouldEmitLocked("state.pose", received)) {
      output.push_back(makeEnvelopeLocked("state.pose", source_ms, semantic));
      recordOutputLocked("state.pose");
    }
    odometry_.header = pose.header;
    odometry_.child_frame_id = child;
    odometry_.pose.pose = pose.pose;
    odometry = odometry_;
    path_.header = pose.header;
    path_.poses.push_back(pose);
    if (path_.poses.size() > kMaximumPathPoses)
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 100u);
    path = path_;
  }

  pose_publisher_.publish(pose);
  odometry_publisher_.publish(odometry);
  path_publisher_.publish(path);
  geometry_msgs::TransformStamped transform;
  transform.header = pose.header;
  transform.child_frame_id = child;
  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation = pose.pose.orientation;
  tf_broadcaster_.sendTransform(transform);
  emit(std::move(output));
  return true;
}

bool RobotRuntime::handleVelocity(const ros::WallTime &received,
                                  const std::string &payload,
                                  std::string *error) {
  const Json body = Json::parse(payload);
  requireVersion(body);
  const auto sequence = wireSequence(body);
  const auto source_ms = sourceTimeMillis(body);
  const std::string frame = qualifyFrame(
      frame_prefix_, required<std::string>(body, "frame_id"), true);
  geometry_msgs::TwistStamped velocity;
  velocity.header.stamp = rosStamp(source_ms);
  velocity.header.frame_id = frame;
  xgc::semantic::common::v1::VelocityEstimate semantic_velocity;
  semantic_velocity.set_frame_id(frame);
  copyVector(body.at("linear"), semantic_velocity.mutable_linear(),
             &velocity.twist.linear);
  copyVector(body.at("angular"), semantic_velocity.mutable_angular(),
             &velocity.twist.angular);
  const double speed =
      std::sqrt(velocity.twist.linear.x * velocity.twist.linear.x +
                velocity.twist.linear.y * velocity.twist.linear.y +
                velocity.twist.linear.z * velocity.twist.linear.z);

  nav_msgs::Odometry odometry;
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ ||
        !acceptWireSequenceLocked(WireChannel::kLocalVelocity, sequence, error))
      return false;
    recordSourceLocked("state.velocity", received);
    recordSourceLocked("state.speed", received);
    if (channelEnabled("state.velocity") &&
        shouldEmitLocked("state.velocity", received)) {
      output.push_back(
          makeEnvelopeLocked("state.velocity", source_ms, semantic_velocity));
      recordOutputLocked("state.velocity");
    }
    if (channelEnabled("state.speed") &&
        shouldEmitLocked("state.speed", received)) {
      xgc::semantic::common::v1::SpeedEstimate semantic_speed;
      semantic_speed.set_frame_id(frame);
      semantic_speed.set_meters_per_second(speed);
      output.push_back(
          makeEnvelopeLocked("state.speed", source_ms, semantic_speed));
      recordOutputLocked("state.speed");
    }
    odometry_.twist.twist = velocity.twist;
    odometry = odometry_;
  }
  velocity_publisher_.publish(velocity);
  if (!odometry.header.frame_id.empty())
    odometry_publisher_.publish(odometry);
  emit(std::move(output));
  return true;
}

bool RobotRuntime::handleImu(const ros::WallTime &received,
                             const std::string &payload, std::string *error) {
  const Json body = Json::parse(payload);
  requireVersion(body);
  const auto sequence = wireSequence(body);
  const auto source_ms = sourceTimeMillis(body);
  const std::string frame = qualifyFrame(
      frame_prefix_, required<std::string>(body, "frame_id"), true);
  sensor_msgs::Imu message;
  message.header.stamp = rosStamp(source_ms);
  message.header.frame_id = frame;
  xgc::semantic::common::v1::ImuEstimate semantic;
  semantic.set_frame_id(frame);
  copyQuaternion(body.at("orientation"), semantic.mutable_orientation(),
                 &message.orientation);
  copyVector(body.at("angular_velocity"), semantic.mutable_angular_velocity(),
             &message.angular_velocity);
  copyVector(body.at("linear_acceleration"),
             semantic.mutable_linear_acceleration(),
             &message.linear_acceleration);
  const auto &covariance = body.at("covariance");
  requireObjectSize(covariance, 3u, "IMU covariance");
  copyCovariance(covariance.at("orientation"),
                 semantic.mutable_orientation_covariance(),
                 &message.orientation_covariance);
  copyCovariance(covariance.at("angular_velocity"),
                 semantic.mutable_angular_velocity_covariance(),
                 &message.angular_velocity_covariance);
  copyCovariance(covariance.at("linear_acceleration"),
                 semantic.mutable_linear_acceleration_covariance(),
                 &message.linear_acceleration_covariance);

  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ ||
        !acceptWireSequenceLocked(WireChannel::kImu, sequence, error))
      return false;
    recordSourceLocked("state.imu", received);
    if (channelEnabled("state.imu") &&
        shouldEmitLocked("state.imu", received)) {
      output.push_back(makeEnvelopeLocked("state.imu", source_ms, semantic));
      recordOutputLocked("state.imu");
    }
  }
  imu_publisher_.publish(message);
  emit(std::move(output));
  return true;
}

bool RobotRuntime::handlePower(const ros::WallTime &received,
                               const std::string &payload, std::string *error) {
  const Json body = Json::parse(payload);
  requireVersion(body);
  const auto sequence = wireSequence(body);
  const auto source_ms = sourceTimeMillis(body);
  const double percentage = finiteNumber(body, "percentage");
  if (percentage < -1.0 || percentage > 1.0)
    throw std::runtime_error("power percentage must be -1 or in 0..1");
  xgc::semantic::common::v1::PowerStatus semantic;
  semantic.set_percentage(percentage);
  semantic.set_voltage_v(nullableFiniteNumber(body, "voltage_v"));
  semantic.set_current_a(nullableFiniteNumber(body, "current_a"));
  semantic.set_temperature_c(nullableFiniteNumber(body, "temperature_c"));
  semantic.set_charging(required<bool>(body, "charging"));
  sensor_msgs::BatteryState message;
  message.header.stamp = rosStamp(source_ms);
  message.percentage = static_cast<float>(semantic.percentage());
  message.voltage = static_cast<float>(semantic.voltage_v());
  message.current = static_cast<float>(semantic.current_a());
  message.temperature = static_cast<float>(semantic.temperature_c());
  message.power_supply_status =
      semantic.charging()
          ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING
          : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ ||
        !acceptWireSequenceLocked(WireChannel::kPower, sequence, error))
      return false;
    recordSourceLocked("state.power", received);
    if (channelEnabled("state.power") &&
        shouldEmitLocked("state.power", received)) {
      output.push_back(makeEnvelopeLocked("state.power", source_ms, semantic));
      recordOutputLocked("state.power");
    }
  }
  power_publisher_.publish(message);
  emit(std::move(output));
  return true;
}

bool RobotRuntime::handleFlightState(const ros::WallTime &received,
                                     const std::string &payload,
                                     std::string *error) {
  const Json body = Json::parse(payload);
  requireVersion(body);
  const auto sequence = wireSequence(body);
  const auto source_ms = sourceTimeMillis(body);
  const bool connected = required<bool>(body, "connected");
  const bool armed = required<bool>(body, "armed");
  (void)required<bool>(body, "guided");
  (void)required<bool>(body, "manual_input");
  const std::string mode = required<std::string>(body, "mode");
  if (mode.size() > 64u)
    throw std::runtime_error("flight mode exceeds 64 bytes");
  const auto system_status = required<std::uint32_t>(body, "system_status");
  const auto landed_state = required<std::uint32_t>(body, "landed_state");
  const auto &faults = body.at("faults");
  if (!faults.is_array() || faults.size() > kMaximumFaults)
    throw std::runtime_error("flight faults must be a bounded array");
  std::vector<std::string> fault_summaries;
  for (const auto &fault : faults) {
    std::string summary;
    if (fault.is_string())
      summary = fault.get<std::string>();
    else if (fault.is_object())
      summary = required<std::string>(fault, "summary");
    else
      throw std::runtime_error("flight fault must be a string or object");
    if (summary.empty() || summary.size() > kMaximumFaultTextBytes)
      throw std::runtime_error("flight fault text is empty or oversized");
    fault_summaries.push_back(std::move(summary));
  }
  xgc::semantic::aerial::v1::FlightStatus semantic;
  semantic.set_connected(connected);
  semantic.set_armed(armed);
  semantic.set_mode(mode);
  semantic.set_system_status(system_status);
  semantic.set_landed_state(landed_state);

  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ ||
        !acceptWireSequenceLocked(WireChannel::kFlightState, sequence, error))
      return false;
    flight_connected_ = connected;
    flight_armed_ = armed;
    flight_mode_ = mode;
    flight_system_status_ = system_status;
    flight_landed_state_ = landed_state;
    flight_faults_ = std::move(fault_summaries);
    recordSourceLocked("state.flight", received);
    recordSourceLocked("state.health", received);
    if (channelEnabled("state.flight") &&
        shouldEmitLocked("state.flight", received)) {
      output.push_back(makeEnvelopeLocked("state.flight", source_ms, semantic));
      recordOutputLocked("state.flight");
    }
  }
  std_msgs::String message;
  message.data = payload;
  flight_state_publisher_.publish(message);
  emit(std::move(output));
  return true;
}

bool RobotRuntime::handleHeartbeat(const ros::WallTime &received,
                                   const std::string &payload,
                                   std::string *error) {
  const Json body = Json::parse(payload);
  requireVersion(body);
  const auto sequence = wireSequence(body);
  const auto source_ms = sourceTimeMillis(body);
  if (required<std::string>(body, "robot_id") != robot_id_)
    throw std::runtime_error("forwarder heartbeat robot_id mismatch");
  if (required<std::string>(body, "transport") != "zenoh")
    throw std::runtime_error("forwarder heartbeat transport is not Zenoh");
  const auto uptime_ms = required<std::int64_t>(body, "uptime_ms");
  if (uptime_ms < 0)
    throw std::runtime_error("forwarder uptime cannot be negative");
  const auto &channels = body.at("channels");
  const auto &stats = body.at("stats");
  if (!channels.is_array() || channels.size() > 64u || !stats.is_object() ||
      stats.size() > 64u) {
    throw std::runtime_error("heartbeat channels/stats are not bounded");
  }

  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || !acceptHeartbeatSequenceLocked(
                         received, sequence, source_ms, uptime_ms, error))
      return false;
    recordSourceLocked("diagnostic.link", received);
    if (channelEnabled("diagnostic.link") &&
        shouldEmitLocked("diagnostic.link", received)) {
      xgc::semantic::common::v1::ChannelHealth semantic;
      semantic.set_channel_id("forwarder_hb");
      semantic.set_source_rate_hz(sources_["diagnostic.link"].source_rate_hz);
      semantic.set_output_rate_hz(sources_["diagnostic.link"].output_rate_hz);
      semantic.set_dropped_samples(decode_errors_ + rejected_frames_);
      semantic.set_source_age_ms(0u);
      semantic.set_stale(false);
      output.push_back(
          makeEnvelopeLocked("diagnostic.link", source_ms, semantic));
      recordOutputLocked("diagnostic.link");
    }
  }
  std_msgs::String message;
  message.data = payload;
  forwarder_status_publisher_.publish(message);
  emit(std::move(output));
  return true;
}

bool RobotRuntime::acceptWireSequenceLocked(WireChannel channel,
                                            std::uint64_t sequence,
                                            std::string *error) {
  auto &last = wire_sequences_[channel];
  if (sequence <= last) {
    ++rejected_frames_;
    return fail(error, std::string(WireChannelLeaf(channel)) +
                           " sequence is duplicate or regressed");
  }
  last = sequence;
  return true;
}

bool RobotRuntime::acceptHeartbeatSequenceLocked(
    const ros::WallTime &received, std::uint64_t sequence,
    std::int64_t source_time_millis, std::int64_t uptime_millis,
    std::string *error) {
  const auto previous = wire_sequences_[WireChannel::kForwarderHeartbeat];
  if (sequence <= previous) {
    const auto found = sources_.find("diagnostic.link");
    double link_age_seconds = 0.0;
    if (found != sources_.end() && !found->second.last_seen.isZero() &&
        received >= found->second.last_seen) {
      link_age_seconds = (received - found->second.last_seen).toSec();
    }
    if (!ShouldResetWireEpoch(
            previous, sequence, last_forwarder_source_time_millis_,
            source_time_millis, last_forwarder_uptime_millis_, uptime_millis,
            link_age_seconds, staleSeconds(profile_id_, "diagnostic.link"))) {
      ++rejected_frames_;
      return fail(error, "forwarder_hb sequence is duplicate or regressed");
    }
    resetWireEpochLocked();
    ++forwarder_restarts_;
    ROS_WARN_STREAM("Mocap Rotor Forwarder epoch changed for robot="
                    << robot_id_ << " restart=" << forwarder_restarts_);
  }
  wire_sequences_[WireChannel::kForwarderHeartbeat] = sequence;
  last_forwarder_source_time_millis_ = source_time_millis;
  last_forwarder_uptime_millis_ = uptime_millis;
  return true;
}

void RobotRuntime::resetWireEpochLocked() {
  wire_sequences_.clear();
  for (auto &entry : sources_)
    entry.second.last_seen = ros::WallTime();
  flight_connected_ = false;
  flight_armed_ = false;
  flight_mode_.clear();
  flight_system_status_ = 0u;
  flight_landed_state_ = 0u;
  flight_faults_.clear();
}

bool RobotRuntime::channelEnabled(const std::string &channel_id) const {
  return enabled_channels_.count(channel_id) != 0u;
}

bool RobotRuntime::shouldEmitLocked(const std::string &channel_id,
                                    const ros::WallTime &now) {
  contract::ChannelMetadata metadata{};
  if (!contract::channelMetadata(profile_id_, channel_id, &metadata) ||
      metadata.output_rate_hz <= 0.0)
    return false;
  auto &last = last_output_[channel_id];
  if (!last.isZero() && now >= last &&
      (now - last).toSec() < 1.0 / metadata.output_rate_hz)
    return false;
  last = now;
  return true;
}

void RobotRuntime::recordSourceLocked(const std::string &channel_id,
                                      const ros::WallTime &now) {
  auto &tracker = sources_[channel_id];
  if (tracker.window_started.isZero())
    tracker.window_started = now;
  tracker.last_seen = now;
  ++tracker.source_samples;
}

void RobotRuntime::recordOutputLocked(const std::string &channel_id) {
  ++sources_[channel_id].output_samples;
}

std::uint64_t
RobotRuntime::sourceAgeMillisLocked(const std::string &channel_id,
                                    const ros::WallTime &now) const {
  const auto found = sources_.find(channel_id);
  if (found == sources_.end() || found->second.last_seen.isZero() ||
      now < found->second.last_seen)
    return 0u;
  return static_cast<std::uint64_t>((now - found->second.last_seen).toSec() *
                                    1000.0);
}

bool RobotRuntime::sourceFreshLocked(const std::string &channel_id,
                                     const ros::WallTime &now) const {
  const auto found = sources_.find(channel_id);
  return found != sources_.end() && !found->second.last_seen.isZero() &&
         now >= found->second.last_seen &&
         found->second.stale_after_seconds > 0.0 &&
         (now - found->second.last_seen).toSec() <=
             found->second.stale_after_seconds;
}

xgc::robot::v1::RobotMessage
RobotRuntime::makeEnvelopeLocked(const std::string &channel_id,
                                 std::int64_t source_time_millis,
                                 const google::protobuf::Message &payload) {
  contract::ChannelMetadata channel{};
  contract::MessageMetadata metadata{};
  if (!contract::channelMetadata(profile_id_, channel_id, &channel) ||
      !contract::messageMetadata(channel.output_message_id, &metadata)) {
    throw std::logic_error("Mocap Rotor semantic schema metadata is missing");
  }
  xgc2_ros1_robot_adapter::MessageSchema schema{
      channel.output_message_id, metadata.type_name, metadata.version,
      metadata.fingerprint};
  xgc2_ros1_robot_adapter::RobotMessageContext context;
  context.robot_id = robot_id_;
  context.channel_id = channel_id;
  context.sequence = ++semantic_sequences_[channel_id];
  if (source_time_millis > 0) {
    context.has_source_time = true;
    context.source_time_nanos = source_time_millis * 1000000;
    context.source_clock_domain = xgc::v1::CLOCK_DOMAIN_NATIVE;
  }
  context.observed_unix_nanos =
      static_cast<std::int64_t>(ros::WallTime::now().toNSec());
  xgc::robot::v1::RobotMessage envelope;
  std::string build_error;
  if (!xgc2_ros1_robot_adapter::BuildRobotMessage(context, schema, payload,
                                                  &envelope, &build_error)) {
    throw std::runtime_error("cannot build Mocap Rotor telemetry envelope: " +
                             build_error);
  }
  return envelope;
}

void RobotRuntime::emit(std::vector<xgc::robot::v1::RobotMessage> messages) {
  for (auto &message : messages) {
    std::string item;
    if (!message.SerializeToString(&item))
      throw std::runtime_error("cannot serialize Mocap Rotor telemetry");
    emitter_(std::move(item));
  }
}

void RobotRuntime::EmitPeriodic(const ros::WallTime &now) {
  std::vector<xgc::robot::v1::RobotMessage> messages;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_)
      return;
    emitHealthLocked(now, &messages);
    emitStreamHealthLocked(now, &messages);
  }
  emit(std::move(messages));
}

void RobotRuntime::emitHealthLocked(
    const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("state.health") || !shouldEmitLocked("state.health", now))
    return;
  const bool flight_fresh = sourceFreshLocked("state.flight", now);
  const bool pose_fresh = sourceFreshLocked("state.pose", now);
  const bool link_fresh = sourceFreshLocked("diagnostic.link", now);
  xgc::semantic::common::v1::VehicleHealth health;
  health.set_online(flight_fresh && flight_connected_ && pose_fresh &&
                    link_fresh);
  if (!flight_fresh) {
    health.set_summary("Mocap Rotor flight state is missing or stale");
    addFault(&health, "mocap-rotor.flight-state.stale",
             "Flight-state uplink exceeded its freshness limit", 2u);
  } else if (!flight_connected_) {
    health.set_summary("Mocap Rotor autopilot is disconnected onboard");
    addFault(&health, "mocap-rotor.autopilot.disconnected",
             "The onboard flight-state source reports disconnected", 2u);
  } else if (!pose_fresh) {
    health.set_summary("Mocap Rotor local pose is missing or stale");
    addFault(&health, "mocap-rotor.local-pose.stale",
             "Local pose exceeded its freshness limit", 2u);
  } else if (!link_fresh) {
    health.set_summary("Mocap Rotor forwarder heartbeat is missing or stale");
    addFault(&health, "mocap-rotor.forwarder.stale",
             "Forwarder heartbeat exceeded its freshness limit", 2u);
  } else {
    health.set_summary("Mocap Rotor read-only telemetry is healthy");
  }
  for (const auto &summary : flight_faults_)
    addFault(&health, "mocap-rotor.flight", summary, 1u);
  messages->push_back(makeEnvelopeLocked("state.health", 0, health));
  recordOutputLocked("state.health");
}

void RobotRuntime::emitStreamHealthLocked(
    const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("diagnostic.stream-health") ||
      !shouldEmitLocked("diagnostic.stream-health", now))
    return;
  contract::ChannelMetadata descriptor{};
  if (!contract::channelMetadata(profile_id_, "diagnostic.stream-health",
                                 &descriptor)) {
    throw std::logic_error("Mocap Rotor stream-health descriptor is missing");
  }
  xgc::semantic::common::v1::StreamHealthReport report;
  for (std::size_t index = 0u; index < descriptor.observes_count; ++index) {
    const std::string channel_id = descriptor.observes[index];
    auto &source = sources_[channel_id];
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
    source.source_samples = 0u;
    source.output_samples = 0u;
    source.window_started = now;
    auto *health = report.add_channels();
    health->set_channel_id(channel_id);
    health->set_source_rate_hz(source.source_rate_hz);
    health->set_output_rate_hz(source.output_rate_hz);
    std::uint64_t dropped = source.dropped_samples;
    if (channel_id == "diagnostic.link")
      dropped += decode_errors_ + rejected_frames_;
    health->set_dropped_samples(dropped);
    health->set_source_age_ms(sourceAgeMillisLocked(channel_id, now));
    health->set_stale(!sourceFreshLocked(channel_id, now));
  }
  messages->push_back(
      makeEnvelopeLocked("diagnostic.stream-health", 0, report));
}

} // namespace xgc_mocap_rotor_ros1_adapter
