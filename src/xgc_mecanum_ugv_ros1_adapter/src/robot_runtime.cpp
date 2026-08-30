#include "xgc_mecanum_ugv_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <utility>

#include <ros/names.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc_mecanum_ugv_ros1_adapter/generated_contract.hpp"

namespace xgc_mecanum_ugv_ros1_adapter {
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

// Covariance arrays arrive as a fixed-size boost::array on the ROS message and
// leave as a repeated field, so the copy is the same shape as the Scout Mini's.
template <typename Covariance>
void copyCovariance(const Covariance &source,
                    google::protobuf::RepeatedField<double> *target) {
  target->Reserve(static_cast<int>(source.size()));
  for (const double value : source) {
    target->Add(value);
  }
}

template <typename RosQuaternion>
void copyQuaternion(const RosQuaternion &source,
                    xgc::semantic::common::v1::Quaternion *target) {
  target->set_x(source.x);
  target->set_y(source.y);
  target->set_z(source.z);
  target->set_w(source.w);
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

const std::array<NativeChannelBinding, 9u> kNativeBindings{{
    {"vrpn.position", "mecanum-ugv.vrpn-pose", "pose",
     "geometry_msgs/PoseStamped", "xgc.semantic.common.v1.PoseEstimate", false},
    {"vrpn.velocity", "mecanum-ugv.vrpn-velocity", "velocity",
     "geometry_msgs/TwistStamped", "xgc.semantic.common.v1.VelocityEstimate",
     false},
    {"vrpn.acceleration", "mecanum-ugv.vrpn-acceleration", "acceleration",
     "geometry_msgs/TwistStamped",
     "xgc.semantic.common.v1.AccelerationEstimate", false},
    {"vrpn.speed", "mecanum-ugv.vrpn-speed", "velocity",
     "geometry_msgs/TwistStamped", "xgc.semantic.common.v1.SpeedEstimate",
     false},
    {"command.velocity", "mecanum-ugv.velocity-command-observation", "command",
     "geometry_msgs/Twist", "xgc.semantic.common.v1.VelocityEstimate", false},
    {"state.imu", "mecanum-ugv.imu-estimate", "imu", "sensor_msgs/Imu",
     "xgc.semantic.common.v1.ImuEstimate", false},
    {"state.power", "mecanum-ugv.power-status", "battery", "std_msgs/Float32",
     "xgc.semantic.common.v1.PowerStatus", false},
    {"state.health", "mecanum-ugv.vehicle-health", nullptr, nullptr,
     "xgc.semantic.common.v1.VehicleHealth", true},
    {"diagnostic.stream-health", "common.stream-health-report", nullptr,
     nullptr, "xgc.semantic.common.v1.StreamHealthReport", true},
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

bool resolveOutputEndpoint(
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    const std::string &channel_id, std::string *resolved, std::string *error) {
  contract::ChannelMetadata channel{};
  if (!contract::channelMetadata(config.profile_id, channel_id, &channel))
    return fail(error, "generated channel is missing: " + channel_id);
  const auto *endpoint = contract::channelEndpoint(
      channel, contract::EndpointKind::kOutput, "output");
  if (endpoint == nullptr)
    return fail(error, "generated output endpoint is missing: " + channel_id);
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

bool loadPositioningHealthConfig(
    const xgc2_ros1_robot_adapter::RobotConfig &robot,
    xgc2_ros1_robot_adapter::PositioningHealthConfig *config,
    std::string *error) {
  return xgc2_ros1_robot_adapter::parsePositioningHealthConfig(
      robot.parameters, config, error);
}

bool loadBatteryCurve(
    const std::string &profile_id,
    std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> *curve,
    std::string *error) {
  contract::ChannelMetadata power{};
  if (!contract::channelMetadata(profile_id, "state.power", &power))
    return fail(error, "Mecanum power channel is missing");
  const auto *policy = contract::channelPolicy(
      power, "battery_voltage_percentage_curve");
  if (policy == nullptr) {
    curve->clear();
    return true;
  }
  const char *const *entries = nullptr;
  std::size_t count = 0u;
  if (!contract::channelPolicyStringArray(
          power, "battery_voltage_percentage_curve", &entries, &count)) {
    return fail(error, "Mecanum battery curve policy must be a string array");
  }
  return xgc2_ros1_robot_adapter::parseBatteryCurve(entries, count, curve,
                                                     error);
}

void setPositioningHealth(
    const xgc2_ros1_robot_adapter::PositioningHealthResult &result,
    xgc::semantic::common::v1::PositioningHealth *payload) {
  using Health = xgc::semantic::common::v1::PositioningHealth;
  using Reason = xgc2_ros1_robot_adapter::PositioningHealthReason;
  using State = xgc2_ros1_robot_adapter::PositioningHealthState;
  switch (result.state) {
  case State::kWarmingUp:
    payload->set_state(Health::POSITIONING_STATE_WARMING_UP);
    break;
  case State::kStable:
    payload->set_state(Health::POSITIONING_STATE_STABLE);
    break;
  case State::kJittering:
    payload->set_state(Health::POSITIONING_STATE_JITTERING);
    break;
  case State::kMoving:
    payload->set_state(Health::POSITIONING_STATE_MOVING);
    break;
  case State::kTimedOut:
    payload->set_state(Health::POSITIONING_STATE_TIMED_OUT);
    break;
  case State::kActive:
    payload->set_state(Health::POSITIONING_STATE_ACTIVE);
    break;
  case State::kFrozen:
    payload->set_state(Health::POSITIONING_STATE_FROZEN);
    break;
  case State::kUnspecified:
    payload->set_state(Health::POSITIONING_STATE_UNSPECIFIED);
    break;
  }
  switch (result.reason) {
  case Reason::kInsufficientSamples:
    payload->set_reason(Health::POSITIONING_REASON_INSUFFICIENT_SAMPLES);
    break;
  case Reason::kStationaryWindowStable:
    payload->set_reason(Health::POSITIONING_REASON_STATIONARY_WINDOW_STABLE);
    break;
  case Reason::kStationaryJitterExceeded:
    payload->set_reason(
        Health::POSITIONING_REASON_STATIONARY_JITTER_EXCEEDED);
    break;
  case Reason::kRobotMoving:
    payload->set_reason(Health::POSITIONING_REASON_ROBOT_MOVING);
    break;
  case Reason::kVrpnTimeout:
    payload->set_reason(Health::POSITIONING_REASON_VRPN_TIMEOUT);
    break;
  case Reason::kWindowVariationObserved:
    payload->set_reason(
        Health::POSITIONING_REASON_WINDOW_VARIATION_OBSERVED);
    break;
  case Reason::kRepeatFrameWindowFrozen:
    payload->set_reason(
        Health::POSITIONING_REASON_REPEAT_FRAME_WINDOW_FROZEN);
    break;
  case Reason::kUnspecified:
    payload->set_reason(Health::POSITIONING_REASON_UNSPECIFIED);
    break;
  }
  payload->set_observed_age_ms(result.observed_age_ms);
  payload->set_window_spread_m(result.comparison_metric_m);
  payload->set_sample_count(static_cast<std::uint32_t>(result.sample_count));
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

bool validMocapRigidBodyName(const std::string &value, std::string *error) {
  static const std::regex pattern("^[A-Za-z][A-Za-z0-9_]*$");
  if (!std::regex_match(value, pattern)) {
    if (error != nullptr)
      *error = "mocap rigid-body name must match ^[A-Za-z][A-Za-z0-9_]*$";
    return false;
  }
  return true;
}

bool resolveMotionCommandTopic(
    const xgc2_ros1_robot_adapter::RobotConfig &config, std::string *topic,
    std::string *error) {
  return resolveOutputEndpoint(config, "operation.motion-intent", topic,
                               error);
}

bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds) {
  if (last_seen.isZero() || stale_after_seconds <= 0.0 || now < last_seen) {
    return false;
  }
  return (now - last_seen).toSec() <= stale_after_seconds;
}

double vrpnForwardSpeedMetersPerSecond(
    double velocity_x, double velocity_y, double velocity_z,
    double orientation_x, double orientation_y, double orientation_z,
    double orientation_w) {
  const double norm = std::hypot(
      std::hypot(orientation_x, orientation_y),
      std::hypot(orientation_z, orientation_w));
  if (!std::isfinite(velocity_x) || !std::isfinite(velocity_y) ||
      !std::isfinite(velocity_z) || !std::isfinite(norm) || norm <= 1e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double x = orientation_x / norm;
  const double y = orientation_y / norm;
  const double z = orientation_z / norm;
  const double w = orientation_w / norm;

  // Pose orientation rotates body-frame vectors into the VRPN world frame.
  // Its first rotation-matrix column is therefore the body X axis expressed
  // in world coordinates. Projecting world velocity onto that axis rejects
  // lateral slip and preserves reverse-motion sign.
  const double body_x_world_x = 1.0 - 2.0 * (y * y + z * z);
  const double body_x_world_y = 2.0 * (x * y + w * z);
  const double body_x_world_z = 2.0 * (x * z - w * y);
  return body_x_world_x * velocity_x + body_x_world_y * velocity_y +
         body_x_world_z * velocity_z;
}

xgc::semantic::common::v1::AccelerationEstimate
vrpnAccelerationEstimate(const geometry_msgs::TwistStamped &message) {
  xgc::semantic::common::v1::AccelerationEstimate payload;
  payload.set_frame_id(message.header.frame_id);
  copyVector(message.twist.linear, payload.mutable_linear());
  copyVector(message.twist.angular, payload.mutable_angular());
  return payload;
}

bool validateNativeProfileContract(std::string *error) {
  std::size_t parameter_count = 0u;
  const auto *parameters =
      contract::profileParameters(contract::kProfileId, &parameter_count);
  contract::ParameterMetadata mocap{};
  contract::ParameterMetadata robot_namespace{};
  contract::ParameterMetadata frames{};
  contract::ParameterMetadata threshold{};
  if (parameters == nullptr || parameter_count != 4u ||
      !contract::parameterMetadata(contract::kProfileId, "mocap_rigid_body", &mocap) ||
      mocap.type != contract::ParameterType::kString || !mocap.required ||
      !contract::parameterMetadata(contract::kProfileId, "namespace", &robot_namespace) ||
      robot_namespace.type != contract::ParameterType::kString || !robot_namespace.required ||
      !contract::parameterMetadata(contract::kProfileId, "positioning_frame_number", &frames) ||
      frames.type != contract::ParameterType::kInteger || !frames.required ||
      !contract::parameterMetadata(contract::kProfileId, "positioning_comparison_threshold_m", &threshold) ||
      threshold.type != contract::ParameterType::kNumber || !threshold.required) {
    return fail(error, "Mecanum UGV native parameter binding is incomplete");
  }

  std::size_t channel_count = 0u;
  const auto *channels =
      contract::profileChannels(contract::kProfileId, &channel_count);
  if (channels == nullptr || channel_count != kNativeBindings.size() + 1u)
    return fail(error, "Mecanum UGV native channel binding is not exhaustive");

  for (const auto &binding : kNativeBindings) {
    contract::ChannelMetadata channel{};
    if (!contract::channelMetadata(contract::kProfileId, binding.channel_id,
                                   &channel)) {
      return fail(error, std::string("Mecanum native channel is missing: ") +
                             binding.channel_id);
    }
    const std::string channel_id(binding.channel_id);
    const bool positioning_health = channel_id == "state.health";
    const bool power = channel_id == "state.power";
    bool invalid_policy = power ? channel.policy_count > 1u
                                : channel.policy_count != 0u;
    if (power && channel.policy_count == 1u) {
      const auto *curve = contract::channelPolicy(
          channel, "battery_voltage_percentage_curve");
      invalid_policy =
          curve == nullptr ||
          curve->kind != contract::PolicyValueKind::kStringArray;
    }
    if (channel.kind != contract::ChannelKind::kStreamOut ||
        std::string(channel.processor) != binding.processor ||
        channel.input_message_id != 0u || channel.output_message_id == 0u ||
        channel.output_rate_hz <= 0.0 ||
        channel.operation_timeout_millis != 0u ||
        channel.stale_after_millis == 0u || invalid_policy ||
        std::string(channel.operation_id).size() != 0u ||
        std::string(channel.operation_contract.side_effect).size() != 0u ||
        std::string(channel.operation_contract.idempotency).size() != 0u ||
        channel.operation_contract.cancellation_supported ||
        channel.operation_contract.deadline_required) {
      return fail(error, std::string("Mecanum native channel binding drifted: ") +
                             binding.channel_id);
    }
    contract::MessageMetadata message{};
    if (!contract::messageMetadata(channel.output_message_id, &message) ||
        std::string(message.type_name) != binding.output_type) {
      return fail(error, std::string("Mecanum output schema binding drifted: ") +
                             binding.channel_id);
    }
    if (binding.observes_channels) {
      if (channel.endpoint_count != 0u || channel.observes_count == 0u)
        return fail(error, "Mecanum diagnostic observes binding is empty");
      for (std::size_t index = 0u; index < channel.observes_count; ++index) {
        contract::ChannelMetadata observed{};
        if (!contract::channelMetadata(contract::kProfileId,
                                       channel.observes[index], &observed) ||
            observed.kind != contract::ChannelKind::kStreamOut ||
            std::string(observed.channel_id) == binding.channel_id) {
          return fail(error, "Mecanum diagnostic observes binding is invalid");
        }
      }
      if (positioning_health) {
        const std::set<std::string> expected{
            "state.imu", "vrpn.position", "vrpn.velocity"};
        const std::set<std::string> observed(
            channel.observes, channel.observes + channel.observes_count);
        if (observed != expected) {
          return fail(error,
                      "Mecanum positioning health observes binding drifted");
        }
      }
    } else {
      const bool fused_speed = std::string(binding.channel_id) == "vrpn.speed";
      if (channel.endpoint_count != (fused_speed ? 2u : 1u) ||
          channel.observes_count != 0u) {
        return fail(error, std::string("Mecanum endpoint binding drifted: ") +
                               binding.channel_id);
      }
      const auto *endpoint = contract::channelEndpoint(
          channel, contract::EndpointKind::kInput, binding.endpoint_role);
      if (endpoint == nullptr ||
          std::string(endpoint->ros_type) != binding.ros_type ||
          std::string(endpoint->name_template).empty()) {
        return fail(error, std::string("Mecanum ROS endpoint binding drifted: ") +
                               binding.channel_id);
      }
      if (fused_speed) {
        const auto *pose_endpoint = contract::channelEndpoint(
            channel, contract::EndpointKind::kInput, "pose");
        if (pose_endpoint == nullptr ||
            std::string(pose_endpoint->ros_type) !=
                "geometry_msgs/PoseStamped" ||
            std::string(pose_endpoint->name_template).empty()) {
          return fail(error,
                      "Mecanum VRPN speed pose binding drifted");
        }
      }
    }
  }

  contract::ChannelMetadata motion{};
  contract::MessageMetadata motion_input{};
  contract::MessageMetadata motion_output{};
  std::int64_t operation_timeout_millis = 0;
  if (!contract::channelMetadata(contract::kProfileId,
                                 "operation.motion-intent", &motion) ||
      motion.kind != contract::ChannelKind::kOperation ||
      std::string(motion.processor) != "mecanum-ugv.set-motion-intent" ||
      std::string(motion.operation_id) != "set-motion-intent" ||
      motion.input_message_id != 3205u || motion.output_message_id != 1u ||
      motion.output_rate_hz != 0.0 ||
      motion.operation_timeout_millis != 1000u ||
      motion.stale_after_millis != 0u || motion.endpoint_count != 1u ||
      motion.observes_count != 0u || motion.policy_count != 1u ||
      std::string(motion.operation_contract.side_effect) != "idempotent" ||
      std::string(motion.operation_contract.idempotency) != "required" ||
      motion.operation_contract.cancellation_supported ||
      !motion.operation_contract.deadline_required ||
      !contract::channelPolicyInteger(motion, "timeout_ms",
                                      &operation_timeout_millis) ||
      operation_timeout_millis != 1000 ||
      !contract::messageMetadata(motion.input_message_id, &motion_input) ||
      std::string(motion_input.type_name) !=
          "xgc.semantic.common.v1.RemoteControlIntentRequest" ||
      !contract::messageMetadata(motion.output_message_id, &motion_output) ||
      std::string(motion_output.type_name) != "xgc.v1.Empty") {
    return fail(error, "Mecanum motion-intent operation binding drifted");
  }
  const auto *motion_endpoint = contract::channelEndpoint(
      motion, contract::EndpointKind::kOutput, "output");
  if (motion_endpoint == nullptr ||
      std::string(motion_endpoint->name_template) != "cmd_vel" ||
      std::string(motion_endpoint->ros_type) != "geometry_msgs/Twist" ||
      motion_endpoint->scope != contract::EndpointScope::kRobotNamespace) {
    return fail(error, "Mecanum motion-intent output topic binding drifted");
  }
  std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> battery_curve;
  if (!loadBatteryCurve(contract::kProfileId, &battery_curve, error)) {
    return false;
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
  const auto mocap_it = config.parameters.find("mocap_rigid_body");
  std::string mocap_error;
  if (mocap_it == config.parameters.end() ||
      !validMocapRigidBodyName(mocap_it->second, &mocap_error)) {
    if (error != nullptr) {
      *error = mocap_it == config.parameters.end()
                   ? "robot configuration is missing required "
                     "mocap_rigid_body parameter"
                   : "invalid mocap rigid-body name: " + mocap_error;
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

  std::string pose_endpoint;
  std::string vrpn_velocity_endpoint;
  std::string vrpn_acceleration_endpoint;
  std::string speed_endpoint;
  std::string speed_pose_endpoint;
  std::string command_velocity_endpoint;
  std::string imu_endpoint;
  std::string voltage_endpoint;
  if (!resolveInputEndpoint(config, "vrpn.position", "pose",
                            &pose_endpoint, error) ||
      !resolveInputEndpoint(config, "vrpn.velocity", "velocity",
                            &vrpn_velocity_endpoint, error) ||
      !resolveInputEndpoint(config, "vrpn.acceleration", "acceleration",
                            &vrpn_acceleration_endpoint, error) ||
      !resolveInputEndpoint(config, "vrpn.speed", "velocity",
                            &speed_endpoint, error) ||
      !resolveInputEndpoint(config, "vrpn.speed", "pose",
                            &speed_pose_endpoint, error) ||
      !resolveInputEndpoint(config, "command.velocity", "command",
                            &command_velocity_endpoint, error) ||
      !resolveInputEndpoint(config, "state.imu", "imu", &imu_endpoint,
                            error) ||
      !resolveInputEndpoint(config, "state.power", "battery",
                            &voltage_endpoint, error)) {
    return nullptr;
  }
  if (vrpn_velocity_endpoint != speed_endpoint ||
      pose_endpoint != speed_pose_endpoint) {
    fail(error,
         "Mecanum UGV processors sharing a subscription resolved to different "
         "native endpoints");
    return nullptr;
  }
  xgc2_ros1_robot_adapter::PositioningHealthConfig positioning_config;
  std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> battery_curve;
  if (!loadPositioningHealthConfig(config, &positioning_config,
                                   error) ||
      !loadBatteryCurve(config.profile_id, &battery_curve, error)) {
    return nullptr;
  }

  auto runtime = std::shared_ptr<RobotRuntime>(
      new RobotRuntime(std::move(node_handle), config.robot_id,
                       config.profile_id, namespace_it->second, spec_revision,
                       std::move(enabled_channels),
                       std::move(required_channels),
                       mocap_it->second, std::move(pose_endpoint),
                       std::move(vrpn_velocity_endpoint),
                       std::move(vrpn_acceleration_endpoint),
                       std::move(command_velocity_endpoint),
                       std::move(imu_endpoint), std::move(voltage_endpoint),
                       positioning_config, std::move(battery_curve),
                       std::move(emitter)));
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
                           std::string mocap_rigid_body,
                           std::string pose_endpoint,
                           std::string vrpn_velocity_endpoint,
                           std::string vrpn_acceleration_endpoint,
                           std::string command_velocity_endpoint,
                           std::string imu_endpoint,
                           std::string voltage_endpoint,
                           xgc2_ros1_robot_adapter::PositioningHealthConfig
                               positioning_config,
                           std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint>
                               battery_curve,
                           EnvelopeEmitter emitter)
    : node_handle_(std::move(node_handle)), robot_id_(std::move(robot_id)),
      profile_id_(std::move(profile_id)),
      robot_namespace_(cleanTopicPart(robot_namespace)),
      mocap_rigid_body_(std::move(mocap_rigid_body)),
      spec_revision_(spec_revision),
      enabled_channels_(std::move(enabled_channels)),
      required_channels_(std::move(required_channels)),
      emitter_(std::move(emitter)),
      pose_endpoint_(std::move(pose_endpoint)),
      vrpn_velocity_endpoint_(std::move(vrpn_velocity_endpoint)),
      vrpn_acceleration_endpoint_(std::move(vrpn_acceleration_endpoint)),
      command_velocity_endpoint_(std::move(command_velocity_endpoint)),
      imu_endpoint_(std::move(imu_endpoint)),
      voltage_endpoint_(std::move(voltage_endpoint)),
      positioning_health_(positioning_config),
      battery_curve_(std::move(battery_curve)) {}

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

  pose_subscriber_.shutdown();
  vrpn_velocity_subscriber_.shutdown();
  vrpn_acceleration_subscriber_.shutdown();
  command_velocity_subscriber_.shutdown();
  imu_subscriber_.shutdown();
  voltage_subscriber_.shutdown();

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
    if (required_channels_.count("vrpn.position") != 0u ||
        required_channels_.count("vrpn.speed") != 0u) {
      ensureSourceLocked(
          "vrpn.position",
          channelStaleAfterSeconds(profile_id_, "vrpn.position"));
      pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>(
          pose_endpoint_, 20,
          [weak_self](const geometry_msgs::PoseStamped::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->poseCallback(message);
            }
          });
      if (!requireRosRegistration(pose_subscriber_, pose_endpoint_, error))
        return false;
    }
    if (required_channels_.count("vrpn.velocity") != 0u ||
        required_channels_.count("vrpn.speed") != 0u) {
      if (required_channels_.count("vrpn.velocity") != 0u)
        ensureSourceLocked(
            "vrpn.velocity",
            channelStaleAfterSeconds(profile_id_, "vrpn.velocity"));
      if (required_channels_.count("vrpn.speed") != 0u)
        ensureSourceLocked(
            "vrpn.speed", channelStaleAfterSeconds(profile_id_, "vrpn.speed"));
      vrpn_velocity_subscriber_ =
          node_handle_.subscribe<geometry_msgs::TwistStamped>(
              vrpn_velocity_endpoint_, 20,
              [weak_self](const geometry_msgs::TwistStamped::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->vrpnVelocityCallback(message);
                }
              });
      if (!requireRosRegistration(vrpn_velocity_subscriber_,
                                  vrpn_velocity_endpoint_, error))
        return false;
    }
    if (required_channels_.count("vrpn.acceleration") != 0u) {
      ensureSourceLocked(
          "vrpn.acceleration",
          channelStaleAfterSeconds(profile_id_, "vrpn.acceleration"));
      vrpn_acceleration_subscriber_ =
          node_handle_.subscribe<geometry_msgs::TwistStamped>(
              vrpn_acceleration_endpoint_, 20,
              [weak_self](const geometry_msgs::TwistStamped::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->vrpnAccelerationCallback(message);
                }
              });
      if (!requireRosRegistration(vrpn_acceleration_subscriber_,
                                  vrpn_acceleration_endpoint_, error))
        return false;
    }
    if (required_channels_.count("command.velocity") != 0u) {
      ensureSourceLocked(
          "command.velocity",
          channelStaleAfterSeconds(profile_id_, "command.velocity"));
      command_velocity_subscriber_ =
          node_handle_.subscribe<geometry_msgs::Twist>(
              command_velocity_endpoint_, 20,
              [weak_self](const geometry_msgs::Twist::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->commandVelocityCallback(message);
                }
              });
      if (!requireRosRegistration(command_velocity_subscriber_,
                                  command_velocity_endpoint_, error))
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
    if (required_channels_.count("state.power") != 0u) {
      ensureSourceLocked(
          "state.power",
          channelStaleAfterSeconds(profile_id_, "state.power"));
      voltage_subscriber_ = node_handle_.subscribe<std_msgs::Float32>(
          voltage_endpoint_, 10,
          [weak_self](const std_msgs::Float32::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->voltageCallback(message);
            }
          });
      if (!requireRosRegistration(voltage_subscriber_, voltage_endpoint_,
                                  error))
        return false;
    }
  }

  ROS_INFO_STREAM("XGC Mecanum UGV Robot Adapter robot="
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

void RobotRuntime::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vrpn_orientation_ = message->pose.orientation;
    has_vrpn_orientation_ = true;
    positioning_health_.recordPose(
        now.toSec(), message->pose.position.x, message->pose.position.y,
        message->pose.position.z);
    recordSourceLocked("vrpn.position", now);
    if (channelEnabled("vrpn.position") &&
        shouldEmitLocked("vrpn.position", now)) {
      xgc::semantic::common::v1::PoseEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      payload.set_child_frame_id(mocap_rigid_body_);
      copyVector(message->pose.position, payload.mutable_position());
      copyQuaternion(message->pose.orientation,
                     payload.mutable_orientation());
      output.push_back(makeEnvelopeLocked("vrpn.position",
                                          message->header.stamp, payload));
      recordOutputLocked("vrpn.position");
    } else if (channelEnabled("vrpn.position")) {
      ++sources_["vrpn.position"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::commandVelocityCallback(
    const geometry_msgs::Twist::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("command.velocity", now);
    if (channelEnabled("command.velocity") &&
        shouldEmitLocked("command.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      copyVector(message->linear, payload.mutable_linear());
      copyVector(message->angular, payload.mutable_angular());
      output.push_back(
          makeEnvelopeLocked("command.velocity", ros::Time(), payload));
      recordOutputLocked("command.velocity");
    } else if (channelEnabled("command.velocity")) {
      ++sources_["command.velocity"].dropped_samples;
    }
  }
  emit(std::move(output));
}

// The on-board IMU is this chassis's liveness signal: it is the only stream the
// vehicle publishes without being commanded, so the Robot Profile gates "online"
// on it and leaves VRPN to gate operational readiness.
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
      output.push_back(
          makeEnvelopeLocked("state.imu", message->header.stamp, payload));
      recordOutputLocked("state.imu");
    } else if (channelEnabled("state.imu")) {
      ++sources_["state.imu"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::voltageCallback(const std_msgs::Float32::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  const double voltage = static_cast<double>(message->data);
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  const ros::Time stamp = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("state.power", now);
    if (channelEnabled("state.power") && shouldEmitLocked("state.power", now)) {
      xgc::semantic::common::v1::PowerStatus payload;
      payload.set_percentage_state(
          xgc::semantic::common::v1::PowerStatus::PERCENTAGE_STATE_UNAVAILABLE);
      if (std::isfinite(voltage)) {
        payload.set_voltage_v(voltage);
        double percentage = 0.0;
        if (xgc2_ros1_robot_adapter::batteryPercentage(
                battery_curve_, voltage, &percentage)) {
          payload.set_percentage(percentage);
          payload.set_percentage_state(
              xgc::semantic::common::v1::PowerStatus::
                  PERCENTAGE_STATE_AVAILABLE);
        }
      }
      output.push_back(makeEnvelopeLocked("state.power", stamp, payload));
      recordOutputLocked("state.power");
    } else if (channelEnabled("state.power")) {
      ++sources_["state.power"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::vrpnVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (required_channels_.count("vrpn.velocity") != 0u)
      recordSourceLocked("vrpn.velocity", now);
    if (required_channels_.count("vrpn.speed") != 0u)
      recordSourceLocked("vrpn.speed", now);

    if (channelEnabled("vrpn.velocity") &&
        shouldEmitLocked("vrpn.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyVector(message->twist.linear, payload.mutable_linear());
      copyVector(message->twist.angular, payload.mutable_angular());
      output.push_back(makeEnvelopeLocked("vrpn.velocity",
                                          message->header.stamp, payload));
      recordOutputLocked("vrpn.velocity");
    } else if (channelEnabled("vrpn.velocity")) {
      ++sources_["vrpn.velocity"].dropped_samples;
    }

    const double speed = has_vrpn_orientation_ &&
                                 sourceFreshLocked("vrpn.position", now)
                             ? vrpnForwardSpeedMetersPerSecond(
                                   message->twist.linear.x,
                                   message->twist.linear.y,
                                   message->twist.linear.z,
                                   vrpn_orientation_.x, vrpn_orientation_.y,
                                   vrpn_orientation_.z, vrpn_orientation_.w)
                             : std::numeric_limits<double>::quiet_NaN();
    if (channelEnabled("vrpn.speed") && std::isfinite(speed) &&
        shouldEmitLocked("vrpn.speed", now)) {
      xgc::semantic::common::v1::SpeedEstimate payload;
      payload.set_frame_id(mocap_rigid_body_);
      payload.set_meters_per_second(speed);
      output.push_back(makeEnvelopeLocked("vrpn.speed", message->header.stamp,
                                          payload));
      recordOutputLocked("vrpn.speed");
    } else if (channelEnabled("vrpn.speed")) {
      ++sources_["vrpn.speed"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::vrpnAccelerationCallback(
    const geometry_msgs::TwistStamped::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("vrpn.acceleration", now);
    if (channelEnabled("vrpn.acceleration") &&
        shouldEmitLocked("vrpn.acceleration", now)) {
      output.push_back(makeEnvelopeLocked(
          "vrpn.acceleration", message->header.stamp,
          vrpnAccelerationEstimate(*message)));
      recordOutputLocked("vrpn.acceleration");
    } else if (channelEnabled("vrpn.acceleration")) {
      ++sources_["vrpn.acceleration"].dropped_samples;
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
    emitStreamHealthLocked(now, &messages);
  }
  emit(std::move(messages));
}

void RobotRuntime::emitHealthLocked(
    const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("state.health") ||
      !shouldEmitLocked("state.health", now)) {
    return;
  }
  const auto imu = sources_.find("state.imu");
  const bool imu_observed =
      imu != sources_.end() && !imu->second.last_seen.isZero();
  const bool imu_fresh = sourceFreshLocked("state.imu", now);
  xgc::semantic::common::v1::VehicleHealth payload;
  payload.set_online(imu_fresh);
  const auto positioning = positioning_health_.evaluate(now.toSec());
  setPositioningHealth(positioning, payload.mutable_positioning());
  if (positioning.state ==
      xgc2_ros1_robot_adapter::PositioningHealthState::kFrozen) {
    addFault(&payload, "mocap.position.frozen",
             "VRPN is publishing a repeated frozen pose", 2);
  } else if (positioning.state ==
             xgc2_ros1_robot_adapter::PositioningHealthState::kTimedOut) {
    addFault(&payload, "mocap.position.timeout",
             "VRPN pose exceeded the XGC1 100 ms freshness boundary", 2);
  }
  if (!imu_observed) {
    addFault(&payload, "mecanum.imu.missing",
             "Mecanum IMU has not been observed", 2);
  } else if (!imu_fresh) {
    addFault(&payload, "mecanum.imu.stale",
             "Mecanum IMU exceeded its freshness limit", 2);
  }
  if (imu_fresh && positioning.state ==
                       xgc2_ros1_robot_adapter::PositioningHealthState::kFrozen) {
    payload.set_summary("Mecanum VRPN positioning data is frozen");
  } else if (imu_fresh && positioning.state ==
                              xgc2_ros1_robot_adapter::PositioningHealthState::kTimedOut) {
    payload.set_summary("Mecanum VRPN positioning is unavailable");
  } else {
    payload.set_summary(imu_fresh ? "Mecanum on-board telemetry is healthy"
                                  : "Mecanum on-board telemetry is missing or stale");
  }
  messages->push_back(
      makeEnvelopeLocked("state.health", ros::Time::now(), payload));
  recordOutputLocked("state.health");
}

void RobotRuntime::emitStreamHealthLocked(
    const ros::WallTime &now, std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("diagnostic.stream-health") ||
      !shouldEmitLocked("diagnostic.stream-health", now)) {
    return;
  }
  contract::ChannelMetadata diagnostic{};
  if (!contract::channelMetadata(profile_id_, "diagnostic.stream-health",
                                 &diagnostic)) {
    throw std::logic_error("generated Mecanum diagnostic binding is missing");
  }
  xgc::semantic::common::v1::StreamHealthReport report;
  for (std::size_t index = 0u; index < diagnostic.observes_count; ++index) {
    const std::string channel_id(diagnostic.observes[index]);
    const auto found = sources_.find(channel_id);
    if (found == sources_.end())
      throw std::logic_error("observed Mecanum channel has no native tracker");
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

    auto *payload = report.add_channels();
    payload->set_channel_id(channel_id);
    payload->set_source_rate_hz(source.source_rate_hz);
    payload->set_output_rate_hz(source.output_rate_hz);
    payload->set_dropped_samples(source.dropped_samples);
    payload->set_source_age_ms(sourceAgeMillisLocked(channel_id, now));
    payload->set_stale(
        !sourceIsFresh(source.last_seen, now, source.stale_after_seconds));
  }
  messages->push_back(makeEnvelopeLocked("diagnostic.stream-health",
                                         ros::Time::now(), report));
}

} // namespace xgc_mecanum_ugv_ros1_adapter
