#include "xgc_scout_mini_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <ros/names.h>

#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"

namespace xgc_scout_mini_ros1_adapter {
namespace {

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

template <typename Handle>
bool requireRosRegistration(const Handle &handle, const std::string &endpoint,
                            std::string *error) {
  if (handle)
    return true;
  if (error != nullptr)
    *error = "ROS master did not accept native endpoint registration: " +
             endpoint;
  return false;
}

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

struct NativeChannelBinding {
  const char *channel_id;
  const char *processor;
  const char *endpoint_role;
  const char *ros_type;
  const char *output_type;
  bool observes_channels;
};

const std::array<NativeChannelBinding, 6u> kNativeBindings{{
    {"state.pose", "scout-mini.pose-estimate", "odometry",
     "nav_msgs/Odometry", "xgc.semantic.common.v1.PoseEstimate", false},
    {"state.velocity", "scout-mini.velocity-estimate", "odometry",
     "nav_msgs/Odometry", "xgc.semantic.common.v1.VelocityEstimate", false},
    {"state.imu", "scout-mini.imu-estimate", "imu", "sensor_msgs/Imu",
     "xgc.semantic.common.v1.ImuEstimate", false},
    {"state.power", "scout-mini.power-status", "chassis_status",
     "scout_msgs/ScoutStatus", "xgc.semantic.common.v1.PowerStatus", false},
    {"state.health", "scout-mini.vehicle-health", "chassis_status",
     "scout_msgs/ScoutStatus", "xgc.semantic.common.v1.VehicleHealth", false},
    {"diagnostic.channel-health", "common.channel-health", nullptr, nullptr,
     "xgc.semantic.common.v1.ChannelHealth", true},
}};

bool resolveEndpointTemplate(
    const contract::EndpointMetadata &endpoint,
    const xgc2_ros1_robot_adapter::RobotConfig &config, std::string *resolved,
    std::string *error) {
  if (resolved == nullptr)
    return fail(error, "resolved endpoint output is required");
  std::string value(endpoint.name_template);
  std::size_t begin = value.find('{');
  while (begin != std::string::npos) {
    const std::size_t end = value.find('}', begin + 1u);
    if (end == std::string::npos)
      return fail(error, "generated endpoint template is malformed");
    const std::string parameter = value.substr(begin + 1u, end - begin - 1u);
    const auto found = config.parameters.find(parameter);
    if (found == config.parameters.end())
      return fail(error, "endpoint template requires parameter " + parameter);
    value.replace(begin, end - begin + 1u, found->second);
    begin = value.find('{', begin + found->second.size());
  }
  if (value.find('}') != std::string::npos)
    return fail(error, "generated endpoint template is malformed");
  if (endpoint.scope == contract::EndpointScope::kGlobal) {
    *resolved = topicName("", value);
  } else {
    const auto namespace_value =
        config.parameters.find("namespace");
    if (namespace_value == config.parameters.end())
      return fail(error, "robot namespace parameter is missing");
    *resolved = topicName(namespace_value->second, value);
  }
  return true;
}

bool resolveInputEndpoint(
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    const std::string &channel_id, const std::string &role,
    std::string *resolved, std::string *error) {
  contract::ChannelMetadata channel{};
  if (!contract::channelMetadata(config.profile_id, channel_id, &channel))
    return fail(error, "generated channel is missing: " + channel_id);
  const auto *endpoint = contract::channelEndpoint(
      channel, contract::EndpointKind::kInput, role);
  if (endpoint == nullptr)
    return fail(error, "generated input endpoint is missing: " + channel_id);
  return resolveEndpointTemplate(*endpoint, config, resolved, error);
}

double channelStaleAfterSeconds(const std::string &profile_id,
                                const std::string &channel_id) {
  contract::ChannelMetadata channel{};
  if (!contract::channelMetadata(profile_id, channel_id, &channel) ||
      channel.stale_after_millis == 0u) {
    throw std::logic_error("generated channel stale policy is invalid");
  }
  return static_cast<double>(channel.stale_after_millis) / 1000.0;
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

bool scoutIsOnline(bool status_fresh) { return status_fresh; }

bool validateNativeProfileContract(std::string *error) {
  std::size_t parameter_count = 0u;
  const auto *parameters =
      contract::profileParameters(contract::kProfileId, &parameter_count);
  if (parameters == nullptr || parameter_count != 1u ||
      std::string(parameters[0].name) != "namespace" ||
      parameters[0].type != contract::ParameterType::kString ||
      !parameters[0].required) {
    return fail(error, "Scout native parameter binding is incomplete");
  }

  std::size_t channel_count = 0u;
  const auto *channels =
      contract::profileChannels(contract::kProfileId, &channel_count);
  if (channels == nullptr || channel_count != kNativeBindings.size())
    return fail(error, "Scout native channel binding is not exhaustive");

  for (const auto &binding : kNativeBindings) {
    contract::ChannelMetadata channel{};
    if (!contract::channelMetadata(contract::kProfileId, binding.channel_id,
                                   &channel) ||
        channel.kind != contract::ChannelKind::kStreamOut ||
        std::string(channel.processor) != binding.processor ||
        channel.input_message_id != 0u || channel.output_message_id == 0u ||
        channel.output_rate_hz <= 0.0 ||
        channel.operation_timeout_millis != 0u ||
        channel.stale_after_millis == 0u || channel.policy_count != 0u ||
        std::string(channel.operation_id).size() != 0u ||
        std::string(channel.operation_contract.side_effect).size() != 0u ||
        std::string(channel.operation_contract.idempotency).size() != 0u ||
        channel.operation_contract.cancellation_supported ||
        channel.operation_contract.deadline_required) {
      return fail(error, std::string("Scout native channel binding drifted: ") +
                             binding.channel_id);
    }
    contract::MessageMetadata message{};
    if (!contract::messageMetadata(channel.output_message_id, &message) ||
        std::string(message.type_name) != binding.output_type) {
      return fail(error, std::string("Scout output schema binding drifted: ") +
                             binding.channel_id);
    }
    if (binding.observes_channels) {
      if (channel.endpoint_count != 0u || channel.observes_count == 0u)
        return fail(error, "Scout diagnostic observes binding is empty");
      for (std::size_t index = 0u; index < channel.observes_count; ++index) {
        contract::ChannelMetadata observed{};
        if (!contract::channelMetadata(contract::kProfileId,
                                       channel.observes[index], &observed) ||
            observed.kind != contract::ChannelKind::kStreamOut ||
            std::string(observed.channel_id) == binding.channel_id) {
          return fail(error, "Scout diagnostic observes binding is invalid");
        }
      }
    } else {
      if (channel.endpoint_count != 1u || channel.observes_count != 0u) {
        return fail(error, std::string("Scout endpoint binding drifted: ") +
                               binding.channel_id);
      }
      const auto &endpoint = channel.endpoints[0];
      if (endpoint.kind != contract::EndpointKind::kInput ||
          std::string(endpoint.role) != binding.endpoint_role ||
          std::string(endpoint.ros_type) != binding.ros_type ||
          std::string(endpoint.name_template).empty()) {
        return fail(error, std::string("Scout ROS endpoint binding drifted: ") +
                               binding.channel_id);
      }
    }
  }
  if (error != nullptr)
    error->clear();
  return true;
}

std::shared_ptr<RobotRuntime> RobotRuntime::Create(
    ros::NodeHandle node_handle,
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    std::uint64_t spec_revision, EnvelopeEmitter emitter, std::string *error) {
  if (!validateNativeProfileContract(error))
    return nullptr;
  if (config.profile_id != contract::kProfileId) {
    if (error != nullptr) {
      *error = "unsupported profile: " + config.profile_id;
    }
    return nullptr;
  }
  const auto namespace_it =
      config.parameters.find("namespace");
  if (namespace_it == config.parameters.end()) {
    if (error != nullptr) {
      *error = "robot configuration is missing required namespace parameter";
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
  for (const auto &channel : config.channels) {
    if (channel.enabled) {
      enabled_channels.insert(channel.channel_id);
    }
  }

  std::set<std::string> required_channels = enabled_channels;
  bool expanded = true;
  while (expanded) {
    expanded = false;
    const std::vector<std::string> snapshot(required_channels.begin(),
                                            required_channels.end());
    for (const auto &channel_id : snapshot) {
      contract::ChannelMetadata channel{};
      if (!contract::channelMetadata(config.profile_id, channel_id, &channel))
        continue;
      for (std::size_t index = 0u; index < channel.observes_count; ++index) {
        if (required_channels.insert(channel.observes[index]).second)
          expanded = true;
      }
    }
  }

  std::string odometry_endpoint;
  std::string velocity_endpoint;
  std::string imu_endpoint;
  std::string status_endpoint;
  std::string health_endpoint;
  if (!resolveInputEndpoint(config, "state.pose", "odometry",
                            &odometry_endpoint, error) ||
      !resolveInputEndpoint(config, "state.velocity", "odometry",
                            &velocity_endpoint, error) ||
      !resolveInputEndpoint(config, "state.imu", "imu", &imu_endpoint,
                            error) ||
      !resolveInputEndpoint(config, "state.power", "chassis_status",
                            &status_endpoint, error) ||
      !resolveInputEndpoint(config, "state.health", "chassis_status",
                            &health_endpoint, error)) {
    return nullptr;
  }
  if (odometry_endpoint != velocity_endpoint ||
      status_endpoint != health_endpoint) {
    fail(error,
         "Scout processors sharing a subscription resolved to different "
         "native endpoints");
    return nullptr;
  }

  auto runtime = std::shared_ptr<RobotRuntime>(
      new RobotRuntime(std::move(node_handle), config.robot_id,
                       config.profile_id, namespace_it->second, spec_revision,
                       std::move(enabled_channels),
                       std::move(required_channels),
                       std::move(odometry_endpoint), std::move(imu_endpoint),
                       std::move(status_endpoint), std::move(emitter)));
  try {
    if (!runtime->install(error)) {
      runtime->Stop();
      return nullptr;
    }
  } catch (...) {
    runtime->Stop();
    throw;
  }
  return runtime;
}

RobotRuntime::RobotRuntime(ros::NodeHandle node_handle, std::string robot_id,
                           std::string profile_id, std::string robot_namespace,
                           std::uint64_t spec_revision,
                           std::set<std::string> enabled_channels,
                           std::set<std::string> required_channels,
                           std::string odometry_endpoint,
                           std::string imu_endpoint,
                           std::string status_endpoint,
                           EnvelopeEmitter emitter)
    : node_handle_(std::move(node_handle)), robot_id_(std::move(robot_id)),
      profile_id_(std::move(profile_id)),
      robot_namespace_(cleanTopicPart(robot_namespace)),
      spec_revision_(spec_revision),
      enabled_channels_(std::move(enabled_channels)),
      required_channels_(std::move(required_channels)),
      emitter_(std::move(emitter)),
      odometry_endpoint_(std::move(odometry_endpoint)),
      imu_endpoint_(std::move(imu_endpoint)),
      status_endpoint_(std::move(status_endpoint)) {}

RobotRuntime::~RobotRuntime() { Stop(); }

RobotRuntime::CallbackGuard::CallbackGuard(RobotRuntime *runtime)
    : runtime_(runtime), active_(runtime_ != nullptr && runtime_->beginCallback()) {}

RobotRuntime::CallbackGuard::~CallbackGuard() {
  if (active_)
    runtime_->endCallback();
}

bool RobotRuntime::beginCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_)
    return false;
  ++active_callbacks_;
  return true;
}

void RobotRuntime::endCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_callbacks_ > 0)
    --active_callbacks_;
  if (active_callbacks_ == 0)
    callbacks_idle_.notify_all();
}

void RobotRuntime::Stop() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopping_) {
      callbacks_idle_.wait(lock, [this] { return stop_complete_; });
      return;
    }
    stopping_ = true;
  }

  odometry_subscriber_.shutdown();
  imu_subscriber_.shutdown();
  status_subscriber_.shutdown();

  {
    std::unique_lock<std::mutex> lock(mutex_);
    callbacks_idle_.wait(lock, [this] { return active_callbacks_ == 0; });
    stop_complete_ = true;
  }
  callbacks_idle_.notify_all();
}

bool RobotRuntime::channelEnabled(const std::string &channel_id) const {
  return enabled_channels_.count(channel_id) != 0;
}

bool RobotRuntime::install(std::string *error) {
  if (profile_id_ != contract::kProfileId) {
    if (error != nullptr) {
      *error = "unsupported profile: " + profile_id_;
    }
    return false;
  }

  const std::weak_ptr<RobotRuntime> weak_self = shared_from_this();
  {
    // AsyncSpinner callbacks may start immediately after subscribe(). Keep
    // tracking state and subscriber registration behind the callback mutex.
    std::lock_guard<std::mutex> lock(mutex_);
    if (required_channels_.count("state.pose") != 0u ||
        required_channels_.count("state.velocity") != 0u) {
      if (required_channels_.count("state.pose") != 0u)
        ensureSourceLocked(
            "state.pose",
            channelStaleAfterSeconds(profile_id_, "state.pose"));
      if (required_channels_.count("state.velocity") != 0u)
        ensureSourceLocked(
            "state.velocity",
            channelStaleAfterSeconds(profile_id_, "state.velocity"));
      odometry_subscriber_ = node_handle_.subscribe<nav_msgs::Odometry>(
          odometry_endpoint_, 20,
          [weak_self](const nav_msgs::Odometry::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->odometryCallback(message);
            }
          });
      if (!requireRosRegistration(odometry_subscriber_, odometry_endpoint_,
                                  error))
        return false;
    }
    if (required_channels_.count("state.imu") != 0u) {
      ensureSourceLocked("state.imu",
                         channelStaleAfterSeconds(profile_id_, "state.imu"));
      imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
          imu_endpoint_, 20,
          [weak_self](const sensor_msgs::Imu::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->imuCallback(message);
            }
          });
      if (!requireRosRegistration(imu_subscriber_, imu_endpoint_, error))
        return false;
    }
    if (required_channels_.count("state.power") != 0u ||
        required_channels_.count("state.health") != 0u) {
      if (required_channels_.count("state.power") != 0u)
        ensureSourceLocked(
            "state.power",
            channelStaleAfterSeconds(profile_id_, "state.power"));
      if (required_channels_.count("state.health") != 0u)
        ensureSourceLocked(
            "state.health",
            channelStaleAfterSeconds(profile_id_, "state.health"));
      status_subscriber_ = node_handle_.subscribe<scout_msgs::ScoutStatus>(
          status_endpoint_, 10,
          [weak_self](const scout_msgs::ScoutStatus::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->statusCallback(message);
            }
          });
      if (!requireRosRegistration(status_subscriber_, status_endpoint_, error))
        return false;
    }
  }

  ROS_INFO_STREAM("XGC Scout Mini Robot Adapter robot="
                  << robot_id_ << " namespace=/" << robot_namespace_
                  << " revision=" << spec_revision_);
  return true;
}

bool RobotRuntime::shouldEmitLocked(const std::string &channel_id,
                                    const ros::WallTime &now) {
  contract::ChannelMetadata metadata{};
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

xgc::robot::v1::RobotMessage RobotRuntime::makeEnvelopeLocked(
    const std::string &channel_id, const ros::Time &source_stamp,
    const google::protobuf::Message &payload) {
  contract::ChannelMetadata channel{};
  if (!contract::channelMetadata(profile_id_, channel_id, &channel) ||
      channel.output_message_id == 0u) {
    throw std::logic_error(
        "channel output is absent from generated XGC2 contract metadata");
  }
  contract::MessageMetadata metadata{};
  if (!contract::messageMetadata(channel.output_message_id, &metadata)) {
    throw std::logic_error(
        "message ID is absent from generated XGC2 contract metadata");
  }
  xgc2_ros1_robot_adapter::MessageSchema schema;
  schema.message_id = channel.output_message_id;
  schema.type_name = metadata.type_name;
  schema.version = metadata.version;
  schema.fingerprint = metadata.fingerprint;

  xgc2_ros1_robot_adapter::RobotMessageContext context;
  context.robot_id = robot_id_;
  context.channel_id = channel_id;
  context.sequence = ++sequences_[channel_id];
  if (!source_stamp.isZero()) {
    context.has_source_time = true;
    context.source_time_nanos =
        static_cast<std::int64_t>(source_stamp.toNSec());
    context.source_clock_domain = ros::Time::isSimTime()
                                      ? xgc::v1::CLOCK_DOMAIN_SIMULATION
                                      : xgc::v1::CLOCK_DOMAIN_NATIVE;
  }
  context.observed_unix_nanos =
      static_cast<std::int64_t>(ros::WallTime::now().toNSec());

  xgc::robot::v1::RobotMessage envelope;
  std::string error;
  if (!xgc2_ros1_robot_adapter::BuildRobotMessage(
          context, schema, payload, &envelope, &error)) {
    throw std::runtime_error("failed to build robot telemetry item: " + error);
  }
  return envelope;
}

void RobotRuntime::emit(std::vector<xgc::robot::v1::RobotMessage> messages) {
  for (auto &message : messages) {
    std::string item;
    if (!message.SerializeToString(&item)) {
      throw std::runtime_error("failed to serialize robot telemetry item");
    }
    emitter_(std::move(item));
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
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (required_channels_.count("state.pose") != 0u)
      recordSourceLocked("state.pose", now);
    if (required_channels_.count("state.velocity") != 0u)
      recordSourceLocked("state.velocity", now);
    if (channelEnabled("state.pose") && shouldEmitLocked("state.pose", now)) {
      xgc::semantic::common::v1::PoseEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      payload.set_child_frame_id(message->child_frame_id);
      copyVector(message->pose.pose.position, payload.mutable_position());
      copyQuaternion(message->pose.pose.orientation,
                     payload.mutable_orientation());
      copyCovariance(message->pose.covariance, payload.mutable_covariance());
      output.push_back(makeEnvelopeLocked("state.pose",
                                          message->header.stamp, payload));
      recordOutputLocked("state.pose");
    } else if (channelEnabled("state.pose")) {
      ++sources_["state.pose"].dropped_samples;
    }
    if (channelEnabled("state.velocity") &&
        shouldEmitLocked("state.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      payload.set_frame_id(message->child_frame_id);
      copyVector(message->twist.twist.linear, payload.mutable_linear());
      copyVector(message->twist.twist.angular, payload.mutable_angular());
      copyCovariance(message->twist.covariance, payload.mutable_covariance());
      output.push_back(makeEnvelopeLocked("state.velocity",
                                          message->header.stamp, payload));
      recordOutputLocked("state.velocity");
    } else if (channelEnabled("state.velocity")) {
      ++sources_["state.velocity"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::imuCallback(const sensor_msgs::Imu::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("state.imu", now);
    if (channelEnabled("state.imu") && shouldEmitLocked("state.imu", now)) {
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
      output.push_back(makeEnvelopeLocked("state.imu",
                                          message->header.stamp, payload));
      recordOutputLocked("state.imu");
    } else if (channelEnabled("state.imu")) {
      ++sources_["state.imu"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::statusCallback(
    const scout_msgs::ScoutStatus::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scout_status_ = *message;
    has_status_ = true;
    if (required_channels_.count("state.power") != 0u)
      recordSourceLocked("state.power", now);
    if (required_channels_.count("state.health") != 0u)
      recordSourceLocked("state.health", now);
    if (channelEnabled("state.power") && shouldEmitLocked("state.power", now)) {
      xgc::semantic::common::v1::PowerStatus payload;
      if (std::isfinite(message->battery_voltage)) {
        payload.set_voltage_v(message->battery_voltage);
      }
      output.push_back(makeEnvelopeLocked("state.power",
                                          message->header.stamp, payload));
      recordOutputLocked("state.power");
    } else if (channelEnabled("state.power")) {
      ++sources_["state.power"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::emitPeriodic(const ros::WallTime &now) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> messages;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    emitHealthLocked(now, &messages);
    emitChannelHealthLocked(now, &messages);
  }
  emit(std::move(messages));
}

void RobotRuntime::emitHealthLocked(const ros::WallTime &now,
                                    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("state.health") ||
      !shouldEmitLocked("state.health", now)) {
    return;
  }

  const bool status_fresh =
      has_status_ && sourceFreshLocked("state.health", now);
  xgc::semantic::common::v1::VehicleHealth payload;
  payload.set_online(scoutIsOnline(status_fresh));
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
    payload.set_summary("Scout chassis telemetry is healthy");
  } else if (has_status_ && scout_status_.fault_code != 0) {
    payload.set_summary("Scout chassis reports a fault");
  } else {
    payload.set_summary("Scout chassis telemetry is incomplete or stale");
  }
  const ros::Time stamp =
      has_status_ ? scout_status_.header.stamp : ros::Time::now();
  messages->push_back(makeEnvelopeLocked("state.health", stamp, payload));
  recordOutputLocked("state.health");
}

void RobotRuntime::emitChannelHealthLocked(
    const ros::WallTime &now, std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("diagnostic.channel-health") ||
      !shouldEmitLocked("diagnostic.channel-health", now)) {
    return;
  }
  contract::ChannelMetadata diagnostic{};
  if (!contract::channelMetadata(profile_id_, "diagnostic.channel-health",
                                 &diagnostic)) {
    throw std::logic_error("generated Scout diagnostic binding is missing");
  }
  for (std::size_t index = 0u; index < diagnostic.observes_count; ++index) {
    const std::string channel_id(diagnostic.observes[index]);
    const auto found = sources_.find(channel_id);
    if (found == sources_.end())
      throw std::logic_error("observed Scout channel has no native tracker");
    auto &source = found->second;
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
    payload.set_channel_id(channel_id);
    payload.set_source_rate_hz(source.source_rate_hz);
    payload.set_output_rate_hz(source.output_rate_hz);
    payload.set_dropped_samples(source.dropped_samples);
    payload.set_source_age_ms(sourceAgeMillisLocked(channel_id, now));
    payload.set_stale(
        !sourceIsFresh(source.last_seen, now, source.stale_after_seconds));
    messages->push_back(makeEnvelopeLocked("diagnostic.channel-health",
                                           ros::Time::now(), payload));
  }
}

} // namespace xgc_scout_mini_ros1_adapter
