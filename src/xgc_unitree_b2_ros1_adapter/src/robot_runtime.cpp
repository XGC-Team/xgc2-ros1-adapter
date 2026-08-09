#include "xgc_unitree_b2_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nlohmann/json.hpp>

#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc/semantic/ground/v1/locomotion.pb.h"
#include "xgc_unitree_b2_ros1_adapter/generated_contract.hpp"

namespace xgc_unitree_b2_ros1_adapter {
namespace {
using Json = nlohmann::json;
constexpr std::size_t kMaximumJoints = 64u;
const std::array<const char *, 5u> kBaseSources{{
    "odom", "joint_states", "power_summary", "driver_status", "forwarder_hb"}};
const std::array<const char *, 12u> kDriverLegNames{{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}};
const std::array<const char *, 12u> kUrdfLegNames{{
    "b2_description_FR_hip_joint", "b2_description_FR_thigh_joint", "b2_description_FR_calf_joint",
    "b2_description_FL_hip_joint", "b2_description_FL_thigh_joint", "b2_description_FL_calf_joint",
    "b2_description_RR_hip_joint", "b2_description_RR_thigh_joint", "b2_description_RR_calf_joint",
    "b2_description_RL_hip_joint", "b2_description_RL_thigh_joint", "b2_description_RL_calf_joint"}};
const std::array<const char *, 8u> kArmNames{{
    "R5a_joint1", "R5a_joint2", "R5a_joint3", "R5a_joint4",
    "R5a_joint5", "R5a_joint6", "R5a_joint7", "R5a_joint8"}};

bool fail(std::string *error, const std::string &message) {
  if (error) *error = message;
  return false;
}

double staleSeconds(const std::string &profile_id, const std::string &channel_id) {
  contract::ChannelMetadata metadata{};
  if (!contract::channelMetadata(profile_id, channel_id, &metadata) ||
      metadata.stale_after_millis == 0u)
    throw std::logic_error("missing B2 freshness metadata for " + channel_id);
  return static_cast<double>(metadata.stale_after_millis) / 1000.0;
}

ros::Time rosStamp(std::int64_t millis) {
  if (millis <= 0) return ros::Time::now();
  ros::Time result;
  result.fromNSec(static_cast<std::uint64_t>(millis) * 1000000u);
  return result;
}

void copyVector(const Json &source, xgc::semantic::common::v1::Vector3 *target) {
  target->set_x(source.at("x").get<double>());
  target->set_y(source.at("y").get<double>());
  target->set_z(source.at("z").get<double>());
}

void copyQuaternion(const Json &source,
                    xgc::semantic::common::v1::Quaternion *target) {
  target->set_x(source.at("x").get<double>());
  target->set_y(source.at("y").get<double>());
  target->set_z(source.at("z").get<double>());
  target->set_w(source.at("w").get<double>());
}

std::string urdfLegName(const std::string &name) {
  for (std::size_t i = 0; i < kDriverLegNames.size(); ++i)
    if (name == kDriverLegNames[i]) return kUrdfLegNames[i];
  return name;
}

template <typename T>
T required(const Json &value, const char *name) { return value.at(name).get<T>(); }

double optionalNumber(const Json &value, const char *name, double fallback = 0.0) {
  const auto found = value.find(name);
  return found == value.end() || found->is_null() ? fallback : found->get<double>();
}

std::int64_t sourceTimeMillis(const Json &value) {
  const auto millis = required<std::int64_t>(value, "t_ms");
  if (millis <= 0 || millis > std::numeric_limits<std::int64_t>::max() / 1000000)
    throw std::runtime_error("source t_ms is out of range");
  return millis;
}
}  // namespace

bool validWireHost(const std::string &value, std::string *error) {
  static const std::regex ipv4("^[0-9]{1,3}(\\.[0-9]{1,3}){3}$");
  if (!std::regex_match(value, ipv4)) return fail(error, "wire_host must be an IPv4 listen address");
  return true;
}

bool parseWirePort(const std::string &value, std::uint16_t *port,
                   std::string *error) {
  if (!port) return fail(error, "wire port output is required");
  try {
    std::size_t consumed = 0;
    const unsigned long parsed = std::stoul(value, &consumed);
    if (consumed != value.size() || parsed == 0 || parsed > 65535u)
      return fail(error, "wire_port must be in 1..65535");
    *port = static_cast<std::uint16_t>(parsed);
    return true;
  } catch (...) { return fail(error, "wire_port must be an integer in 1..65535"); }
}

bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds) {
  return !last_seen.isZero() && now >= last_seen && stale_after_seconds > 0.0 &&
         (now - last_seen).toSec() <= stale_after_seconds;
}

bool validateNativeProfileContract(std::string *error) {
  std::size_t parameter_count = 0;
  const auto *parameters = contract::profileParameters(contract::kProfileId, &parameter_count);
  if (!parameters || parameter_count != 4u)
    return fail(error, "B2 profile must expose robot_id and three wire parameters");
  std::size_t channel_count = 0;
  const auto *channels = contract::profileChannels(contract::kProfileId, &channel_count);
  if (!channels || channel_count != 10u)
    return fail(error, "B2 profile must expose nine baseline channels plus arm joints");
  for (std::size_t i = 0; i < channel_count; ++i) {
    if (channels[i].kind != contract::ChannelKind::kStreamOut ||
        channels[i].output_message_id == 0u || channels[i].input_message_id != 0u ||
        channels[i].operation_timeout_millis != 0u)
      return fail(error, "B2 profile contains a command or incomplete stream channel");
    contract::MessageMetadata message{};
    if (!contract::messageMetadata(channels[i].output_message_id, &message) ||
        !message.type_name || message.version == 0u || message.fingerprint == 0u)
      return fail(error, "B2 profile output schema is absent from registry");
  }
  return true;
}

std::shared_ptr<RobotRuntime> RobotRuntime::Create(
    ros::NodeHandle node_handle,
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    std::uint64_t spec_revision, EnvelopeEmitter emitter,
    std::string *error) {
  if (!validateNativeProfileContract(error)) return nullptr;
  if (config.profile_id != contract::kProfileId)
    return fail(error, "unsupported B2 profile: " + config.profile_id), nullptr;
  const auto robot_id = config.parameters.find("robot_id");
  const auto transport = config.parameters.find("wire_transport");
  const auto host = config.parameters.find("wire_host");
  const auto port_text = config.parameters.find("wire_port");
  if (config.parameters.size() != 4u || robot_id == config.parameters.end() ||
      transport == config.parameters.end() || host == config.parameters.end() ||
      port_text == config.parameters.end())
    return fail(error, "B2 requires exactly robot_id, wire_transport, wire_host, wire_port"), nullptr;
  if (robot_id->second != config.robot_id)
    return fail(error, "wire robot_id must equal Adapter Runtime robot id"), nullptr;
  if (transport->second != "tcp")
    return fail(error, "Zenoh backend is not compiled; explicit TCP is required"), nullptr;
  std::uint16_t port = 0;
  if (!validWireHost(host->second, error) || !parseWirePort(port_text->second, &port, error))
    return nullptr;
  std::set<std::string> enabled;
  for (const auto &channel : config.channels) {
    contract::ChannelMetadata metadata{};
    if (!contract::channelMetadata(config.profile_id, channel.channel_id, &metadata))
      return fail(error, "unknown B2 channel: " + channel.channel_id), nullptr;
    if (channel.enabled) enabled.insert(channel.channel_id);
  }
  auto runtime = std::shared_ptr<RobotRuntime>(new RobotRuntime(
      std::move(node_handle), config.robot_id, config.profile_id, spec_revision,
      std::move(enabled), host->second, port, std::move(emitter)));
  if (!runtime->install(error)) { runtime->Stop(); return nullptr; }
  return runtime;
}

RobotRuntime::RobotRuntime(ros::NodeHandle node_handle, std::string robot_id,
                           std::string profile_id, std::uint64_t spec_revision,
                           std::set<std::string> enabled_channels,
                           std::string wire_host, std::uint16_t wire_port,
                           EnvelopeEmitter emitter)
    : node_handle_(std::move(node_handle)), robot_id_(std::move(robot_id)),
      profile_id_(std::move(profile_id)), spec_revision_(spec_revision),
      enabled_channels_(std::move(enabled_channels)), wire_host_(std::move(wire_host)),
      wire_port_(wire_port), emitter_(std::move(emitter)) {
  sources_["odom"].stale_after_seconds = staleSeconds(profile_id_, "state.pose");
  sources_["joint_states"].stale_after_seconds = staleSeconds(profile_id_, "state.joints");
  sources_["power_summary"].stale_after_seconds = staleSeconds(profile_id_, "state.power");
  sources_["driver_status"].stale_after_seconds = staleSeconds(profile_id_, "state.health");
  sources_["forwarder_hb"].stale_after_seconds = staleSeconds(profile_id_, "diagnostic.link");
  sources_["arm_slave_status"].stale_after_seconds = staleSeconds(profile_id_, "state.arm-joints");
  sources_["arm_forwarder_hb"].stale_after_seconds = staleSeconds(profile_id_, "diagnostic.link");
  path_.header.frame_id = "odom";
}

RobotRuntime::~RobotRuntime() { Stop(); }

bool RobotRuntime::install(std::string *error) {
  odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("/remote/b2/odom", 20);
  leg_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("/remote/b2/joint_states", 20);
  arm_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("/remote/arm/slave_joint_states", 20);
  combined_joint_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 20);
  path_publisher_ = node_handle_.advertise<nav_msgs::Path>("/remote/b2/path", 5, true);
  power_publisher_ = node_handle_.advertise<std_msgs::String>("/remote/b2/power_summary", 10);
  driver_publisher_ = node_handle_.advertise<std_msgs::String>("/remote/b2/driver_status_json", 10);
  const std::weak_ptr<RobotRuntime> weak = shared_from_this();
  wire_.reset(new WireTcpServer(wire_host_, wire_port_,
      [weak](const std::string &key, const std::string &payload) {
        if (const auto self = weak.lock()) self->onWireFrame(key, payload);
      }));
  if (!wire_->Start(error)) return false;
  ROS_INFO_STREAM("XGC Unitree B2 Adapter robot=" << robot_id_ << " TCP="
                  << wire_host_ << ':' << wire_port_ << " revision=" << spec_revision_);
  return true;
}

void RobotRuntime::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    stopping_ = true;
  }
  if (wire_) wire_->Stop();
  odom_publisher_.shutdown(); leg_publisher_.shutdown(); arm_publisher_.shutdown();
  combined_joint_publisher_.shutdown(); path_publisher_.shutdown();
  power_publisher_.shutdown(); driver_publisher_.shutdown();
}

bool RobotRuntime::channelEnabled(const std::string &channel_id) const {
  return enabled_channels_.count(channel_id) != 0u;
}

void RobotRuntime::onWireFrame(const std::string &key, const std::string &payload) {
  std::string channel;
  if (!validWireKey(robot_id_, key, &channel)) {
    std::lock_guard<std::mutex> lock(mutex_); ++dropped_frames_; return;
  }
  const ros::WallTime received = ros::WallTime::now();
  try {
    if (channel == "odom") handleOdom(received, payload);
    else if (channel == "joint_states") handleJoints(received, payload, false);
    else if (channel == "arm_slave_status") handleJoints(received, payload, true);
    else if (channel == "power_summary") handlePower(received, payload);
    else if (channel == "driver_status") handleDriver(received, payload);
    else if (channel == "forwarder_hb") handleHeartbeat(received, payload, false);
    else if (channel == "arm_forwarder_hb") handleHeartbeat(received, payload, true);
  } catch (const std::exception &exception) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++decode_errors_;
    ROS_WARN_STREAM_THROTTLE(2.0, "B2 wire decode rejected " << channel << ": " << exception.what());
  }
}

void RobotRuntime::handleOdom(const ros::WallTime &received,
                              const std::string &payload_text) {
  const Json body = Json::parse(payload_text);
  if (required<int>(body, "v") != 1) throw std::runtime_error("unsupported odom version");
  const auto source_ms = sourceTimeMillis(body);
  const auto frame = required<std::string>(body, "frame_id");
  const auto child = required<std::string>(body, "child_frame_id");
  const auto &position = body.at("position"); const auto &orientation = body.at("orientation");
  const auto &linear = body.at("linear"); const auto &angular = body.at("angular");
  std::vector<xgc::robot::v1::RobotMessage> output;
  nav_msgs::Odometry odom;
  odom.header.stamp = rosStamp(source_ms); odom.header.frame_id = frame; odom.child_frame_id = child;
  odom.pose.pose.position.x = position.at("x").get<double>();
  odom.pose.pose.position.y = position.at("y").get<double>();
  odom.pose.pose.position.z = position.at("z").get<double>();
  odom.pose.pose.orientation.x = orientation.at("x").get<double>();
  odom.pose.pose.orientation.y = orientation.at("y").get<double>();
  odom.pose.pose.orientation.z = orientation.at("z").get<double>();
  odom.pose.pose.orientation.w = orientation.at("w").get<double>();
  odom.twist.twist.linear.x = linear.at("x").get<double>();
  odom.twist.twist.linear.y = linear.at("y").get<double>();
  odom.twist.twist.linear.z = linear.at("z").get<double>();
  odom.twist.twist.angular.x = angular.at("x").get<double>();
  odom.twist.twist.angular.y = angular.at("y").get<double>();
  odom.twist.twist.angular.z = angular.at("z").get<double>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("odom", received);
    if (channelEnabled("state.pose") && shouldEmitLocked("state.pose", received)) {
      xgc::semantic::common::v1::PoseEstimate value; value.set_frame_id(frame); value.set_child_frame_id(child);
      copyVector(position, value.mutable_position()); copyQuaternion(orientation, value.mutable_orientation());
      output.push_back(makeEnvelopeLocked("state.pose", source_ms, value));
    }
    if (channelEnabled("state.velocity") && shouldEmitLocked("state.velocity", received)) {
      xgc::semantic::common::v1::VelocityEstimate value; value.set_frame_id(child);
      copyVector(linear, value.mutable_linear()); copyVector(angular, value.mutable_angular());
      output.push_back(makeEnvelopeLocked("state.velocity", source_ms, value));
    }
    if (channelEnabled("state.speed") && shouldEmitLocked("state.speed", received)) {
      xgc::semantic::common::v1::SpeedEstimate value; value.set_frame_id(child);
      value.set_meters_per_second(std::hypot(linear.at("x").get<double>(), linear.at("y").get<double>()));
      output.push_back(makeEnvelopeLocked("state.speed", source_ms, value));
    }
    if (!output.empty()) recordOutputLocked("odom");
    geometry_msgs::PoseStamped pose; pose.header = odom.header; pose.pose = odom.pose.pose;
    path_.header = odom.header; path_.poses.push_back(pose);
    if (path_.poses.size() > 2000u) path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 100u);
  }
  nav_msgs::Path path_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path_snapshot = path_;
  }
  odom_publisher_.publish(odom); path_publisher_.publish(path_snapshot);
  geometry_msgs::TransformStamped transform;
  transform.header = odom.header; transform.child_frame_id = child;
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.translation.z = odom.pose.pose.position.z;
  transform.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_.sendTransform(transform);
  emit(std::move(output));
}

void RobotRuntime::handleJoints(const ros::WallTime &received,
                                const std::string &payload_text, bool arm) {
  const Json body = Json::parse(payload_text);
  if (required<int>(body, "v") != 1) throw std::runtime_error("unsupported joint version");
  const auto source_ms = sourceTimeMillis(body);
  const auto frame = required<std::string>(body, "frame_id");
  const auto names = required<std::vector<std::string>>(body, "names");
  const auto position = required<std::vector<double>>(body, "positions");
  const auto velocity = required<std::vector<double>>(body, "velocities");
  const auto effort = required<std::vector<double>>(body, "efforts");
  if (names.empty() || names.size() > kMaximumJoints || position.size() != names.size() ||
      velocity.size() != names.size() || effort.size() != names.size())
    throw std::runtime_error("joint arrays are empty, oversized, or unequal");
  sensor_msgs::JointState ros_message; ros_message.header.stamp = rosStamp(source_ms);
  ros_message.header.frame_id = frame; ros_message.velocity = velocity; ros_message.effort = effort;
  xgc::semantic::ground::v1::JointStateSet semantic; semantic.set_frame_id(frame);
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto *joint = semantic.add_joints(); joint->set_name(names[i]);
    joint->set_position(position[i]); joint->set_velocity(velocity[i]); joint->set_effort(effort[i]);
    ros_message.name.push_back(arm ? names[i] : urdfLegName(names[i]));
    ros_message.position.push_back(position[i]);
  }
  const std::string source = arm ? "arm_slave_status" : "joint_states";
  const std::string channel = arm ? "state.arm-joints" : "state.joints";
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked(source, received);
    auto &positions = arm ? arm_positions_ : leg_positions_;
    for (std::size_t i = 0; i < ros_message.name.size(); ++i) positions[ros_message.name[i]] = position[i];
    const bool arm_link_fresh = !arm || sourceIsFresh(
        sources_["arm_forwarder_hb"].last_seen, received,
        sources_["arm_forwarder_hb"].stale_after_seconds);
    if (arm_link_fresh && channelEnabled(channel) && shouldEmitLocked(channel, received)) {
      output.push_back(makeEnvelopeLocked(channel, source_ms, semantic)); recordOutputLocked(source);
    }
    publishCombinedJoints(source_ms);
  }
  if (arm) arm_publisher_.publish(ros_message); else leg_publisher_.publish(ros_message);
  emit(std::move(output));
}

void RobotRuntime::handlePower(const ros::WallTime &received,
                               const std::string &payload_text) {
  const Json body = Json::parse(payload_text);
  if (required<int>(body, "v") != 1) throw std::runtime_error("unsupported power version");
  const auto source_ms = sourceTimeMillis(body);
  xgc::semantic::common::v1::PowerStatus value;
  value.set_percentage(optionalNumber(body, "soc"));
  value.set_voltage_v(optionalNumber(body, "power_v"));
  value.set_current_a(optionalNumber(body, "power_a"));
  value.set_temperature_c((optionalNumber(body, "temperature_ntc1") +
                           optionalNumber(body, "temperature_ntc2")) / 2.0);
  value.set_charging(value.current_a() > 0.0);
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_); recordSourceLocked("power_summary", received);
    if (channelEnabled("state.power") && shouldEmitLocked("state.power", received)) {
      output.push_back(makeEnvelopeLocked("state.power", source_ms, value)); recordOutputLocked("power_summary");
    }
  }
  std_msgs::String ros_message; ros_message.data = payload_text; power_publisher_.publish(ros_message);
  emit(std::move(output));
}

void RobotRuntime::handleDriver(const ros::WallTime &received,
                                const std::string &payload_text) {
  const Json body = Json::parse(payload_text);
  if (required<int>(body, "v") != 1) throw std::runtime_error("unsupported driver version");
  const auto source_ms = sourceTimeMillis(body);
  const int level = required<int>(body, "level");
  const auto summary = required<std::string>(body, "summary");
  const bool motion_enabled = required<bool>(body, "motion_enabled");
  const bool command_stale = required<bool>(body, "command_stale");
  const auto faults = required<std::vector<std::string>>(body, "faults");
  if (summary.size() > 512u || faults.size() > 32u ||
      std::any_of(faults.begin(), faults.end(),
                  [](const std::string &fault) { return fault.size() > 512u; }))
    throw std::runtime_error("driver status strings or fault list exceed bounds");
  xgc::semantic::ground::v1::LocomotionStatus locomotion;
  locomotion.set_mode(motion_enabled ? "enabled" : "idle");
  locomotion.set_motion_enabled(motion_enabled); locomotion.set_command_stale(command_stale);
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_); recordSourceLocked("driver_status", received);
    driver_level_ = level; driver_summary_ = summary; driver_faults_ = faults;
    if (channelEnabled("state.locomotion") && shouldEmitLocked("state.locomotion", received)) {
      output.push_back(makeEnvelopeLocked("state.locomotion", source_ms, locomotion));
      recordOutputLocked("driver_status");
    }
  }
  std_msgs::String ros_message; ros_message.data = payload_text; driver_publisher_.publish(ros_message);
  emit(std::move(output));
}

void RobotRuntime::handleHeartbeat(const ros::WallTime &received,
                                   const std::string &payload_text, bool arm) {
  const Json body = Json::parse(payload_text);
  if (required<int>(body, "v") != 1 || required<std::string>(body, "robot_id") != robot_id_)
    throw std::runtime_error("heartbeat identity/version mismatch");
  const auto source_ms = sourceTimeMillis(body);
  const int domain = required<int>(body, "domain");
  (void)required<std::string>(body, "transport");
  const auto &channels = body.at("channels");
  const auto &stats = body.at("stats");
  (void)required<std::int64_t>(body, "uptime_ms");
  if (!channels.is_array() || channels.size() > 64u || !stats.is_object() || stats.size() > 64u)
    throw std::runtime_error("heartbeat channels/stats exceed bounds or have wrong type");
  for (const auto &entry : stats.items()) {
    const auto &value = entry.value();
    (void)required<std::uint64_t>(value, "rx_count");
    (void)required<std::uint64_t>(value, "tx_count");
    (void)required<std::int64_t>(value, "last_rx_ms");
    (void)required<std::int64_t>(value, "last_tx_ms");
    (void)required<double>(value, "effective_hz");
  }
  if ((!arm && domain != 0) || (arm && domain != 17)) throw std::runtime_error("heartbeat ROS domain mismatch");
  const std::string source = arm ? "arm_forwarder_hb" : "forwarder_hb";
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_); recordSourceLocked(source, received);
    if (!arm && channelEnabled("diagnostic.link") && shouldEmitLocked("diagnostic.link", received)) {
      xgc::semantic::common::v1::ChannelHealth link;
      link.set_channel_id("forwarder_hb"); link.set_source_rate_hz(sources_[source].source_rate_hz);
      link.set_output_rate_hz(sources_[source].output_rate_hz);
      link.set_dropped_samples(decode_errors_ + dropped_frames_);
      link.set_source_age_ms(0); link.set_stale(false);
      output.push_back(makeEnvelopeLocked("diagnostic.link", source_ms, link)); recordOutputLocked(source);
    }
  }
  emit(std::move(output));
}

bool RobotRuntime::baseOnlineLocked(const ros::WallTime &now) const {
  if (driver_level_ >= 2) return false;
  for (const char *name : kBaseSources) {
    const auto found = sources_.find(name);
    if (found == sources_.end() || !sourceIsFresh(found->second.last_seen, now,
                                                   found->second.stale_after_seconds)) return false;
  }
  return true;
}

void RobotRuntime::emitPeriodic(const ros::WallTime &now) {
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    emitHealthLocked(now, &output); emitStreamHealthLocked(now, &output);
  }
  emit(std::move(output));
}

void RobotRuntime::emitHealthLocked(
    const ros::WallTime &now, std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("state.health") || !shouldEmitLocked("state.health", now)) return;
  xgc::semantic::common::v1::VehicleHealth health;
  health.set_online(baseOnlineLocked(now));
  if (!has_online_state_ || last_online_state_ != health.online()) {
    ROS_INFO_STREAM("B2 data readiness robot=" << robot_id_ << " state="
                    << (health.online() ? "LIVE" : "OFFLINE_OR_STALE")
                    << " decode_errors=" << decode_errors_
                    << " dropped_frames=" << dropped_frames_);
    has_online_state_ = true;
    last_online_state_ = health.online();
  }
  health.set_summary(health.online() ? driver_summary_ : "B2 required source stale or driver error");
  for (const auto &text : driver_faults_) {
    auto *fault = health.add_faults(); fault->set_code("b2-driver");
    fault->set_summary(text); fault->set_severity(static_cast<std::uint32_t>(std::max(0, driver_level_)));
  }
  messages->push_back(makeEnvelopeLocked("state.health", 0, health));
  recordOutputLocked("driver_status");
}

void RobotRuntime::emitStreamHealthLocked(
    const ros::WallTime &now, std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("diagnostic.stream-health") ||
      !shouldEmitLocked("diagnostic.stream-health", now)) return;
  for (auto &entry : sources_) {
    auto &source = entry.second;
    if (source.window_started.isZero()) source.window_started = now;
    const double elapsed = now >= source.window_started ? (now - source.window_started).toSec() : 0.0;
    if (elapsed > 0.0) {
      source.source_rate_hz = static_cast<double>(source.source_samples) / elapsed;
      source.output_rate_hz = static_cast<double>(source.output_samples) / elapsed;
    }
    source.source_samples = 0; source.output_samples = 0; source.window_started = now;
  }
  static const std::array<std::pair<const char *, const char *>, 9u> mapping{{
      {"odom", "state.pose"}, {"odom", "state.velocity"}, {"odom", "state.speed"},
      {"power_summary", "state.power"}, {"driver_status", "state.health"},
      {"driver_status", "state.locomotion"}, {"joint_states", "state.joints"},
      {"arm_slave_status", "state.arm-joints"}, {"forwarder_hb", "diagnostic.link"}}};
  xgc::semantic::common::v1::StreamHealthReport report;
  for (const auto &entry : mapping) {
    auto &source = sources_[entry.first];
    auto *health = report.add_channels(); health->set_channel_id(entry.second);
    health->set_source_rate_hz(source.source_rate_hz); health->set_output_rate_hz(source.output_rate_hz);
    std::uint64_t dropped = source.dropped_samples;
    if (std::string(entry.second) == "diagnostic.link")
      dropped += decode_errors_ + dropped_frames_;
    health->set_dropped_samples(dropped); health->set_source_age_ms(sourceAgeMillisLocked(entry.first, now));
    bool stale = !sourceIsFresh(source.last_seen, now, source.stale_after_seconds);
    if (std::string(entry.second) == "state.arm-joints") {
      const auto &heartbeat = sources_["arm_forwarder_hb"];
      stale = stale || !sourceIsFresh(heartbeat.last_seen, now,
                                      heartbeat.stale_after_seconds);
    }
    health->set_stale(stale);
  }
  messages->push_back(makeEnvelopeLocked("diagnostic.stream-health", 0, report));
}

bool RobotRuntime::shouldEmitLocked(const std::string &channel_id,
                                    const ros::WallTime &now) {
  contract::ChannelMetadata metadata{};
  if (!contract::channelMetadata(profile_id_, channel_id, &metadata) || metadata.output_rate_hz <= 0.0)
    return false;
  auto &last = last_output_[channel_id];
  if (!last.isZero() && now >= last && (now - last).toSec() < 1.0 / metadata.output_rate_hz) return false;
  last = now; return true;
}

xgc::robot::v1::RobotMessage RobotRuntime::makeEnvelopeLocked(
    const std::string &channel_id, std::int64_t source_time_millis,
    const google::protobuf::Message &payload) {
  contract::ChannelMetadata channel{}; contract::MessageMetadata metadata{};
  if (!contract::channelMetadata(profile_id_, channel_id, &channel) ||
      !contract::messageMetadata(channel.output_message_id, &metadata))
    throw std::logic_error("B2 semantic schema metadata is missing");
  xgc2_ros1_robot_adapter::MessageSchema schema{channel.output_message_id,
      metadata.type_name, metadata.version, metadata.fingerprint};
  xgc2_ros1_robot_adapter::RobotMessageContext context;
  context.robot_id = robot_id_; context.channel_id = channel_id;
  context.sequence = ++sequences_[channel_id];
  if (source_time_millis > 0) {
    context.has_source_time = true; context.source_time_nanos = source_time_millis * 1000000;
    context.source_clock_domain = xgc::v1::CLOCK_DOMAIN_NATIVE;
  }
  context.observed_unix_nanos = static_cast<std::int64_t>(ros::WallTime::now().toNSec());
  xgc::robot::v1::RobotMessage envelope; std::string error;
  if (!xgc2_ros1_robot_adapter::BuildRobotMessage(context, schema, payload, &envelope, &error))
    throw std::runtime_error("cannot build B2 telemetry envelope: " + error);
  return envelope;
}

void RobotRuntime::emit(std::vector<xgc::robot::v1::RobotMessage> messages) {
  for (auto &message : messages) {
    std::string item;
    if (!message.SerializeToString(&item)) throw std::runtime_error("cannot serialize B2 telemetry");
    emitter_(std::move(item));
  }
}

void RobotRuntime::recordSourceLocked(const std::string &source, const ros::WallTime &now) {
  auto &tracker = sources_[source]; if (tracker.window_started.isZero()) tracker.window_started = now;
  tracker.last_seen = now; ++tracker.source_samples;
}
void RobotRuntime::recordOutputLocked(const std::string &source) { ++sources_[source].output_samples; }
std::uint64_t RobotRuntime::sourceAgeMillisLocked(const std::string &source,
                                                  const ros::WallTime &now) const {
  const auto found = sources_.find(source);
  if (found == sources_.end() || found->second.last_seen.isZero() || now < found->second.last_seen) return 0;
  return static_cast<std::uint64_t>((now - found->second.last_seen).toSec() * 1000.0);
}

void RobotRuntime::publishCombinedJoints(std::int64_t source_time_millis) {
  sensor_msgs::JointState combined; combined.header.stamp = rosStamp(source_time_millis);
  for (const char *name : kUrdfLegNames) {
    const auto found = leg_positions_.find(name);
    if (found != leg_positions_.end()) { combined.name.push_back(name); combined.position.push_back(found->second); }
  }
  for (const char *name : kArmNames) {
    const auto found = arm_positions_.find(name);
    if (found != arm_positions_.end()) { combined.name.push_back(name); combined.position.push_back(found->second); }
  }
  combined_joint_publisher_.publish(combined);
}

}  // namespace xgc_unitree_b2_ros1_adapter
