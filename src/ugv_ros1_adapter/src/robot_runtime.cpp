#include "xgc_ugv_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <cctype>
#include <array>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <utility>

#include <ros/names.h>

#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc/semantic/ground/v1/chassis.pb.h"
#include "xgc/semantic/ground/v1/reset.pb.h"

namespace xgc_ugv_ros1_adapter {
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
  std::string channel_id;
  std::string processor;
  const char *endpoint_role;
  const char *ros_type;
  const char *output_type;
  bool observes_channels;
};

// The channels the runtime implements for this robot type, in profile order.
std::vector<NativeChannelBinding> nativeBindings(const UgvVariant &variant) {
  const std::string prefix = variant.processor_prefix;
  std::vector<NativeChannelBinding> bindings{
      {"vrpn.position", prefix + ".vrpn-pose", "pose",
       "geometry_msgs/PoseStamped", "xgc.semantic.common.v1.PoseEstimate",
       false},
      {"vrpn.velocity", prefix + ".vrpn-velocity", "velocity",
       "geometry_msgs/TwistStamped", "xgc.semantic.common.v1.VelocityEstimate",
       false},
      {"vrpn.acceleration", prefix + ".vrpn-acceleration", "acceleration",
       "geometry_msgs/TwistStamped",
       "xgc.semantic.common.v1.AccelerationEstimate", false},
      {"vrpn.speed", prefix + ".vrpn-speed", "velocity",
       "geometry_msgs/TwistStamped", "xgc.semantic.common.v1.SpeedEstimate",
       false},
      {"command.velocity", prefix + ".velocity-command-observation", "command",
       "geometry_msgs/Twist", "xgc.semantic.common.v1.VelocityEstimate", false},
      {"state.imu", prefix + ".imu-estimate", "imu", "sensor_msgs/Imu",
       "xgc.semantic.common.v1.ImuEstimate", false},
      {"state.power", prefix + ".power-status", "battery", "std_msgs/Float32",
       "xgc.semantic.common.v1.PowerStatus", false},
  };
  if (variant.health == HealthSource::kChassisState) {
    bindings.push_back({"state.health", prefix + ".vehicle-health",
                        "chassis_state", "std_msgs/UInt32",
                        "xgc.semantic.common.v1.VehicleHealth", false});
    bindings.push_back({"state.chassis", prefix + ".chassis-status",
                        "chassis_state", "std_msgs/UInt32",
                        "xgc.semantic.ground.v1.ChassisStatus", false});
  } else {
    bindings.push_back({"state.health", prefix + ".vehicle-health", nullptr,
                        nullptr, "xgc.semantic.common.v1.VehicleHealth", true});
  }
  bindings.push_back({"state.controller", prefix + ".controller-status",
                      "status", "std_msgs/String",
                      "xgc.semantic.common.v1.ControllerStatus", false});
  bindings.push_back({"state.reset", prefix + ".reset-status", "session",
                      "ugv_reset_msgs/ResetSession",
                      "xgc.semantic.ground.v1.ResetStatus", false});
  bindings.push_back({"diagnostic.stream-health", "common.stream-health-report",
                      nullptr, nullptr,
                      "xgc.semantic.common.v1.StreamHealthReport", true});
  return bindings;
}

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
    const contract::Contract &installed,
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    const std::string &channel_id, const std::string &role,
    std::string *resolved, std::string *error) {
  contract::ChannelMetadata channel{};
  if (!installed.channelMetadata(config.profile_id, channel_id, &channel))
    return fail(error, "generated channel is missing: " + channel_id);
  const auto *endpoint = contract::channelEndpoint(
      channel, contract::EndpointKind::kInput, role);
  if (endpoint == nullptr)
    return fail(error, "generated input endpoint is missing: " + channel_id);
  return resolveEndpointTemplate(*endpoint, config, resolved, error);
}

bool resolveOutputEndpoint(
    const contract::Contract &installed,
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    const std::string &channel_id, std::string *resolved, std::string *error) {
  contract::ChannelMetadata channel{};
  if (!installed.channelMetadata(config.profile_id, channel_id, &channel))
    return fail(error, "generated channel is missing: " + channel_id);
  const auto *endpoint = contract::channelEndpoint(
      channel, contract::EndpointKind::kOutput, "output");
  if (endpoint == nullptr)
    return fail(error, "generated output endpoint is missing: " + channel_id);
  return resolveEndpointTemplate(*endpoint, config, resolved, error);
}

double channelStaleAfterSeconds(const contract::Contract &installed,
                                const std::string &profile_id,
                                const std::string &channel_id) {
  contract::ChannelMetadata channel{};
  if (!installed.channelMetadata(profile_id, channel_id, &channel) ||
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
    const UgvVariant &variant, const std::string &profile_id,
    std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> *curve,
    std::string *error) {
  contract::ChannelMetadata power{};
  if (!variant.contract.channelMetadata(profile_id, "state.power", &power))
    return fail(error, std::string(variant.name) + " power channel is missing");
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
    return fail(error, std::string(variant.name) +
                           " battery curve policy must be a string array");
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
    const UgvVariant &variant,
    const xgc2_ros1_robot_adapter::RobotConfig &config, std::string *topic,
    std::string *error) {
  return resolveOutputEndpoint(variant.contract, config,
                               "operation.motion-intent", topic, error);
}

bool sourceIsFresh(const ros::WallTime &last_seen, const ros::WallTime &now,
                   double stale_after_seconds) {
  if (last_seen.isZero() || stale_after_seconds <= 0.0 || now < last_seen) {
    return false;
  }
  return (now - last_seen).toSec() <= stale_after_seconds;
}

StreamRateEstimate updateStreamRateEstimate(
    const StreamRateEstimate &previous, std::uint64_t source_samples,
    std::uint64_t output_samples, double elapsed_seconds, bool source_fresh) {
  if (!source_fresh)
    return {};

  StreamRateEstimate next = previous;
  if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0)
    return next;
  if (source_samples > 0u) {
    next.source_rate_hz =
        static_cast<double>(source_samples) / elapsed_seconds;
  }
  if (output_samples > 0u) {
    next.output_rate_hz =
        static_cast<double>(output_samples) / elapsed_seconds;
  }
  return next;
}

StreamRateWindow updateStreamRateWindow(
    const StreamRateEstimate &previous, std::uint64_t source_samples,
    std::uint64_t output_samples, double elapsed_seconds, bool source_fresh,
    double window_seconds) {
  if (!source_fresh)
    return {{}, true};
  if (!std::isfinite(elapsed_seconds) || elapsed_seconds < window_seconds)
    return {previous, false};
  return {updateStreamRateEstimate(previous, source_samples, output_samples,
                                   elapsed_seconds, true),
          true};
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
vrpnAccelerationEstimate(const geometry_msgs::AccelStamped &message) {
  xgc::semantic::common::v1::AccelerationEstimate payload;
  payload.set_frame_id(message.header.frame_id);
  copyVector(message.accel.linear, payload.mutable_linear());
  copyVector(message.accel.angular, payload.mutable_angular());
  return payload;
}

std::uint32_t packChassisState(unsigned control_mode, unsigned base_state,
                               unsigned fault_code) {
  return (control_mode & 0xFFu) | ((base_state & 0xFFu) << 8) |
         ((fault_code & 0xFFFFu) << 16);
}

bool unpackChassisState(std::uint32_t word, ChassisState *out) {
  if (out == nullptr)
    return false;
  out->control_mode = word & 0xFFu;
  out->base_state = (word >> 8) & 0xFFu;
  out->fault_code = (word >> 16) & 0xFFFFu;
  return true;
}

xgc::semantic::ground::v1::ChassisStatus::ControlMode
chassisControlMode(std::uint8_t native_mode) {
  using Status = xgc::semantic::ground::v1::ChassisStatus;
  switch (native_mode) {
  case 0u:
  case 1u:
    return Status::CONTROL_MODE_COMMAND_CAN;
  case 2u:
    return Status::CONTROL_MODE_COMMAND_UART;
  case 3u:
    return Status::CONTROL_MODE_REMOTE;
  default:
    return Status::CONTROL_MODE_UNSPECIFIED;
  }
}

bool validateNativeProfileContract(const UgvVariant &variant,
                                   std::string *error) {
  const contract::Contract &installed = variant.contract;
  const std::string primary = installed.primaryProfile().profile_id;
  const std::string name = variant.name;
  contract::ParameterMetadata mocap{};
  contract::ParameterMetadata robot_namespace{};
  contract::ParameterMetadata frames{};
  contract::ParameterMetadata threshold{};
  contract::ParameterMetadata source_root{};
  contract::ParameterMetadata offset_x{};
  contract::ParameterMetadata offset_y{};
  contract::ParameterMetadata offset_z{};
  if (!installed.parameterMetadata(primary, "mocap_rigid_body", &mocap) ||
      mocap.type != contract::ParameterType::kString || !mocap.required ||
      !installed.parameterMetadata(primary, "namespace", &robot_namespace) ||
      robot_namespace.type != contract::ParameterType::kString || !robot_namespace.required ||
      !installed.parameterMetadata(primary, "mocap_source_root", &source_root) ||
      source_root.type != contract::ParameterType::kString || source_root.required ||
      !installed.parameterMetadata(primary, "localization_offset_x", &offset_x) ||
      offset_x.type != contract::ParameterType::kNumber || offset_x.required ||
      !installed.parameterMetadata(primary, "localization_offset_y", &offset_y) ||
      offset_y.type != contract::ParameterType::kNumber || offset_y.required ||
      !installed.parameterMetadata(primary, "localization_offset_z", &offset_z) ||
      offset_z.type != contract::ParameterType::kNumber || offset_z.required ||
      !installed.parameterMetadata(primary, "positioning_frame_number", &frames) ||
      frames.type != contract::ParameterType::kInteger || !frames.required ||
      !installed.parameterMetadata(primary, "positioning_comparison_threshold_m", &threshold) ||
      threshold.type != contract::ParameterType::kNumber || !threshold.required) {
    return fail(error, name + " native parameter binding is incomplete");
  }

  const auto bindings = nativeBindings(variant);
  std::size_t channel_count = 0u;
  const auto *channels = installed.profileChannels(primary, &channel_count);
  if (channels == nullptr || channel_count != bindings.size() + 2u)
    return fail(error, name + " native channel binding is not exhaustive");

  for (const auto &binding : bindings) {
    contract::ChannelMetadata channel{};
    if (!installed.channelMetadata(primary, binding.channel_id, &channel)) {
      return fail(error,
                  name + " native channel is missing: " + binding.channel_id);
    }
    const std::string &channel_id = binding.channel_id;
    const bool positioning_health = channel_id == "state.health";
    const bool power = channel_id == "state.power";
    const bool pose = channel_id == "vrpn.position";
    bool apply_offset = false;
    bool invalid_policy = pose
        ? channel.policy_count != 1u || !contract::channelPolicyBoolean(channel, "apply_world_offset", &apply_offset)
        : power ? channel.policy_count > 1u : channel.policy_count != 0u;
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
        std::string(channel.operation_contract.safety).size() != 0u ||
        channel.operation_contract.cancellation_supported ||
        channel.operation_contract.deadline_required) {
      return fail(error, name + " native channel binding drifted: " +
                             binding.channel_id);
    }
    contract::MessageMetadata message{};
    if (!installed.messageMetadata(channel.output_message_id, &message) ||
        std::string(message.type_name) != binding.output_type) {
      return fail(error, name + " output schema binding drifted: " +
                             binding.channel_id);
    }
    if (binding.observes_channels) {
      if (channel.endpoint_count != 0u || channel.observes_count == 0u)
        return fail(error, name + " observes binding is empty: " +
                               binding.channel_id);
      for (std::size_t index = 0u; index < channel.observes_count; ++index) {
        contract::ChannelMetadata observed{};
        if (!installed.channelMetadata(primary, channel.observes[index],
                                       &observed) ||
            observed.kind != contract::ChannelKind::kStreamOut ||
            std::string(observed.channel_id) == binding.channel_id) {
          return fail(error, name + " observes binding is invalid: " +
                                 binding.channel_id);
        }
      }
    } else {
      const bool fused_speed = channel_id == "vrpn.speed";
      const bool localization_projection = channel_id == "vrpn.position" ||
                                           channel_id == "vrpn.velocity" ||
                                           channel_id == "vrpn.acceleration";
      const bool reset_observation = channel_id == "state.reset";
      if (channel.endpoint_count !=
              ((fused_speed || localization_projection || reset_observation) ? 2u : 1u) ||
          channel.observes_count != (positioning_health ? 3u : 0u)) {
        return fail(error, name + " endpoint binding drifted: " +
                               binding.channel_id);
      }
      if (localization_projection) {
        const char *projection_ros_type = channel_id == "vrpn.acceleration"
            ? "geometry_msgs/AccelStamped" : binding.ros_type;
        const auto *projection_output = contract::channelEndpoint(
            channel, contract::EndpointKind::kOutput, "output");
        if (projection_output == nullptr ||
            projection_output->scope != contract::EndpointScope::kRobotNamespace ||
            std::string(projection_output->ros_type) != projection_ros_type) {
          return fail(error, name + " localization output binding drifted");
        }
      }
      const auto *endpoint = contract::channelEndpoint(
          channel, contract::EndpointKind::kInput, binding.endpoint_role);
      if (endpoint == nullptr ||
          std::string(endpoint->ros_type) != binding.ros_type ||
          std::string(endpoint->name_template).empty()) {
        return fail(error, name + " ROS endpoint binding drifted: " +
                               binding.channel_id);
      }
      if (fused_speed) {
        const auto *pose_endpoint = contract::channelEndpoint(
            channel, contract::EndpointKind::kInput, "pose");
        if (pose_endpoint == nullptr ||
            std::string(pose_endpoint->ros_type) !=
                "geometry_msgs/PoseStamped" ||
            std::string(pose_endpoint->name_template).empty()) {
          return fail(error, name + " VRPN speed pose binding drifted");
        }
      }
      if (reset_observation) {
        const auto *result_endpoint = contract::channelEndpoint(
            channel, contract::EndpointKind::kInput, "result");
        if (result_endpoint == nullptr ||
            std::string(result_endpoint->ros_type) != "ugv_reset_msgs/ResetResult" ||
            std::string(result_endpoint->name_template) != "reset/result" ||
            result_endpoint->scope != contract::EndpointScope::kRobotNamespace ||
            endpoint->scope != contract::EndpointScope::kRobotNamespace ||
            std::string(endpoint->name_template) != "reset/session") {
          return fail(error, name + " reset observation binding drifted");
        }
      }
    }
    if (positioning_health) {
      const std::set<std::string> expected{
          "state.imu", "vrpn.position", "vrpn.velocity"};
      const std::set<std::string> observed(
          channel.observes, channel.observes + channel.observes_count);
      if (observed != expected) {
        return fail(error, name + " positioning health observes binding drifted");
      }
    }
  }

  const std::string prefix = variant.processor_prefix;
  contract::ChannelMetadata motion{};
  contract::MessageMetadata motion_input{};
  contract::MessageMetadata motion_output{};
  std::int64_t operation_timeout_millis = 0;
  std::int64_t core_silence_millis = 0;
  if (!installed.channelMetadata(primary, "operation.motion-intent", &motion) ||
      motion.kind != contract::ChannelKind::kOperation ||
      std::string(motion.processor) != prefix + ".set-motion-intent" ||
      std::string(motion.operation_id) != "set-motion-intent" ||
      motion.input_message_id != 3205u || motion.output_message_id != 1u ||
      motion.output_rate_hz != 0.0 ||
      motion.operation_timeout_millis != 1000u ||
      motion.stale_after_millis != 0u || motion.endpoint_count != 1u ||
      motion.observes_count != 0u || motion.policy_count != 2u ||
      std::string(motion.operation_contract.side_effect) != "idempotent" ||
      std::string(motion.operation_contract.safety) != "none" ||
      std::string(motion.operation_contract.idempotency) != "required" ||
      motion.operation_contract.cancellation_supported ||
      !motion.operation_contract.deadline_required ||
      !contract::channelPolicyInteger(motion, "timeout_ms",
                                      &operation_timeout_millis) ||
      operation_timeout_millis != 1000 ||
      !contract::channelPolicyInteger(motion, "core_silence_ms",
                                      &core_silence_millis) ||
      core_silence_millis < 0 || core_silence_millis > 60000 ||
      !installed.messageMetadata(motion.input_message_id, &motion_input) ||
      std::string(motion_input.type_name) !=
          "xgc.semantic.common.v1.RemoteControlIntentRequest" ||
      !installed.messageMetadata(motion.output_message_id, &motion_output) ||
      std::string(motion_output.type_name) != "xgc.v1.Empty") {
    return fail(error, name + " motion-intent operation binding drifted");
  }
  const auto *motion_endpoint = contract::channelEndpoint(
      motion, contract::EndpointKind::kOutput, "output");
  if (motion_endpoint == nullptr ||
      std::string(motion_endpoint->name_template) != "cmd_vel" ||
      std::string(motion_endpoint->ros_type) != "geometry_msgs/Twist" ||
      motion_endpoint->scope != contract::EndpointScope::kRobotNamespace) {
    return fail(error, name + " motion-intent output topic binding drifted");
  }
  contract::ChannelMetadata release{};
  contract::MessageMetadata release_input{};
  if (!installed.channelMetadata(primary, "operation.motion-intent-release",
                                 &release) ||
      release.kind != contract::ChannelKind::kOperation ||
      std::string(release.processor) != prefix + ".release-motion-intent" ||
      std::string(release.operation_id) != "release-motion-intent" ||
      release.input_message_id != 3205u || release.output_message_id != 1u ||
      release.output_rate_hz != 0.0 ||
      release.operation_timeout_millis != 1000u ||
      release.stale_after_millis != 0u || release.endpoint_count != 1u ||
      release.observes_count != 0u || release.policy_count != 1u ||
      std::string(release.operation_contract.side_effect) != "idempotent" ||
      std::string(release.operation_contract.safety) != "release" ||
      std::string(release.operation_contract.idempotency) != "required" ||
      release.operation_contract.cancellation_supported ||
      !release.operation_contract.deadline_required ||
      !installed.messageMetadata(release.input_message_id, &release_input) ||
      std::string(release_input.type_name) !=
          "xgc.semantic.common.v1.RemoteControlIntentRequest") {
    return fail(error, name + " motion-intent-release operation binding drifted");
  }
  const auto *release_endpoint = contract::channelEndpoint(
      release, contract::EndpointKind::kOutput, "output");
  if (release_endpoint == nullptr ||
      std::string(release_endpoint->name_template) != "cmd_vel" ||
      std::string(release_endpoint->ros_type) != "geometry_msgs/Twist" ||
      release_endpoint->scope != contract::EndpointScope::kRobotNamespace) {
    return fail(error, name + " motion-intent-release output topic binding drifted");
  }
  std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> battery_curve;
  if (!loadBatteryCurve(variant, primary, &battery_curve, error)) {
    return false;
  }
  if (error != nullptr)
    error->clear();
  return true;
}

std::shared_ptr<RobotRuntime> RobotRuntime::Create(
    const UgvVariant &variant, ros::NodeHandle node_handle,
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    EnvelopeEmitter emitter, std::string *error) {
  const contract::Contract &installed = variant.contract;
  if (!validateNativeProfileContract(variant, error))
    return nullptr;
  if (!installed.profileDigest(config.profile_id)) {
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
  contract::ParameterMetadata mocap_parameter{};
  const bool canonical_input = !installed.parameterMetadata(config.profile_id, "mocap_rigid_body", &mocap_parameter);
  const auto mocap_it = config.parameters.find("mocap_rigid_body");
  std::string mocap_error;
  if (!canonical_input && (mocap_it == config.parameters.end() ||
      !validMocapRigidBodyName(mocap_it->second, &mocap_error))) {
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
      if (!installed.channelMetadata(config.profile_id, channel_id, &channel))
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
  std::string canonical_pose_endpoint;
  std::string canonical_velocity_endpoint;
  std::string canonical_acceleration_endpoint;
  std::string speed_endpoint;
  std::string speed_pose_endpoint;
  std::string command_velocity_endpoint;
  std::string imu_endpoint;
  std::string voltage_endpoint;
  std::string health_endpoint;
  std::string chassis_endpoint;
  std::string controller_status_endpoint;
  std::string reset_session_endpoint;
  std::string reset_result_endpoint;
  if (!resolveInputEndpoint(installed, config, "vrpn.position", "pose",
                            &pose_endpoint, error) ||
      !resolveInputEndpoint(installed, config, "vrpn.velocity", "velocity",
                            &vrpn_velocity_endpoint, error) ||
      (!canonical_input && !resolveInputEndpoint(installed, config, "vrpn.acceleration", "acceleration",
                            &vrpn_acceleration_endpoint, error)) ||
      (!canonical_input && !resolveOutputEndpoint(installed, config, "vrpn.position", &canonical_pose_endpoint,
                             error)) ||
      (!canonical_input && !resolveOutputEndpoint(installed, config, "vrpn.velocity", &canonical_velocity_endpoint,
                             error)) ||
      (!canonical_input && !resolveOutputEndpoint(installed, config, "vrpn.acceleration",
                             &canonical_acceleration_endpoint, error)) ||
      !resolveInputEndpoint(installed, config, "vrpn.speed", "velocity",
                            &speed_endpoint, error) ||
      !resolveInputEndpoint(installed, config, "vrpn.speed", "pose",
                            &speed_pose_endpoint, error) ||
      !resolveInputEndpoint(installed, config, "command.velocity", "command",
                            &command_velocity_endpoint, error) ||
      !resolveInputEndpoint(installed, config, "state.imu", "imu", &imu_endpoint,
                            error) ||
      !resolveInputEndpoint(installed, config, "state.power", "battery",
                            &voltage_endpoint, error) ||
      (variant.health == HealthSource::kChassisState &&
       (!resolveInputEndpoint(installed, config, "state.health", "chassis_state",
                              &health_endpoint, error) ||
        !resolveInputEndpoint(installed, config, "state.chassis", "chassis_state",
                              &chassis_endpoint, error))) ||
      !resolveInputEndpoint(installed, config, "state.controller", "status",
                            &controller_status_endpoint, error) ||
      !resolveInputEndpoint(installed, config, "state.reset", "session",
                            &reset_session_endpoint, error) ||
      !resolveInputEndpoint(installed, config, "state.reset", "result",
                            &reset_result_endpoint, error)) {
    return nullptr;
  }
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization;
  if (!canonical_input && !xgc2_ros1_robot_adapter::parseLocalizationProjectionConfig(
          config.parameters, &localization, error))
    return nullptr;
  contract::ChannelMetadata pose_channel{};
  bool apply_world_offset = false;
  if (!installed.channelMetadata(config.profile_id, "vrpn.position", &pose_channel) ||
      !contract::channelPolicyBoolean(pose_channel, "apply_world_offset", &apply_world_offset)) {
    fail(error, "localization offset policy is missing");
    return nullptr;
  }
  if (!apply_world_offset)
    localization.offset_x = localization.offset_y = localization.offset_z = 0.0;
  if (!canonical_input) {
    pose_endpoint = topicName(localization.source_root, pose_endpoint);
    vrpn_velocity_endpoint = topicName(localization.source_root, vrpn_velocity_endpoint);
    vrpn_acceleration_endpoint = topicName(localization.source_root, vrpn_acceleration_endpoint);
    speed_endpoint = topicName(localization.source_root, speed_endpoint);
    speed_pose_endpoint = topicName(localization.source_root, speed_pose_endpoint);
  }
  if (vrpn_velocity_endpoint != speed_endpoint ||
      pose_endpoint != speed_pose_endpoint ||
      health_endpoint != chassis_endpoint) {
    fail(error, std::string(variant.name) +
                    " processors sharing a subscription resolved to different "
                    "native endpoints");
    return nullptr;
  }
  xgc2_ros1_robot_adapter::PositioningHealthConfig positioning_config;
  std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> battery_curve;
  if (!loadPositioningHealthConfig(config, &positioning_config,
                                   error) ||
      !loadBatteryCurve(variant, config.profile_id, &battery_curve, error)) {
    return nullptr;
  }

  auto runtime = std::shared_ptr<RobotRuntime>(
      new RobotRuntime(variant, std::move(node_handle), config.robot_id,
                       config.profile_id, namespace_it->second,
                       std::move(enabled_channels),
                       std::move(required_channels),
                       canonical_input ? std::string() : mocap_it->second, std::move(pose_endpoint),
                       std::move(vrpn_velocity_endpoint),
                       std::move(vrpn_acceleration_endpoint),
                       std::move(canonical_pose_endpoint),
                       std::move(canonical_velocity_endpoint),
                       std::move(canonical_acceleration_endpoint),
                       localization,
                       std::move(command_velocity_endpoint),
                       std::move(imu_endpoint),
                       std::move(voltage_endpoint), std::move(chassis_endpoint),
                       std::move(controller_status_endpoint),
                       std::move(reset_session_endpoint),
                       std::move(reset_result_endpoint),
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

RobotRuntime::RobotRuntime(const UgvVariant &variant,
                           ros::NodeHandle node_handle,
                           std::string robot_id,
                           std::string profile_id, std::string robot_namespace,
                           std::set<std::string> enabled_channels,
                           std::set<std::string> required_channels,
                           std::string mocap_rigid_body,
                           std::string pose_endpoint,
                           std::string vrpn_velocity_endpoint,
                           std::string vrpn_acceleration_endpoint,
                           std::string canonical_pose_endpoint,
                           std::string canonical_velocity_endpoint,
                           std::string canonical_acceleration_endpoint,
                           xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization,
                           std::string command_velocity_endpoint,
                           std::string imu_endpoint,
                           std::string voltage_endpoint,
                           std::string chassis_state_endpoint,
                           std::string controller_status_endpoint,
                           std::string reset_session_endpoint,
                           std::string reset_result_endpoint,
                           xgc2_ros1_robot_adapter::PositioningHealthConfig
                               positioning_config,
                           std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint>
                               battery_curve,
                           EnvelopeEmitter emitter)
    : variant_(variant), node_handle_(std::move(node_handle)),
      robot_id_(std::move(robot_id)),
      profile_id_(std::move(profile_id)),
      robot_namespace_(cleanTopicPart(robot_namespace)),
      mocap_rigid_body_(std::move(mocap_rigid_body)),
      enabled_channels_(std::move(enabled_channels)),
      required_channels_(std::move(required_channels)),
      emitter_(std::move(emitter)),
      pose_endpoint_(std::move(pose_endpoint)),
      vrpn_velocity_endpoint_(std::move(vrpn_velocity_endpoint)),
      vrpn_acceleration_endpoint_(std::move(vrpn_acceleration_endpoint)),
      canonical_pose_endpoint_(std::move(canonical_pose_endpoint)),
      canonical_velocity_endpoint_(std::move(canonical_velocity_endpoint)),
      canonical_acceleration_endpoint_(std::move(canonical_acceleration_endpoint)),
      command_velocity_endpoint_(std::move(command_velocity_endpoint)),
      imu_endpoint_(std::move(imu_endpoint)),
      voltage_endpoint_(std::move(voltage_endpoint)),
      chassis_state_endpoint_(std::move(chassis_state_endpoint)),
      controller_status_endpoint_(std::move(controller_status_endpoint)),
      reset_session_endpoint_(std::move(reset_session_endpoint)),
      reset_result_endpoint_(std::move(reset_result_endpoint)),
      localization_(std::move(localization)),
      positioning_health_(positioning_config),
      battery_curve_(std::move(battery_curve)) {}

RobotRuntime::~RobotRuntime() { Stop(); }

RobotRuntime::CallbackGuard::CallbackGuard(RobotRuntime *runtime)
    : runtime_(runtime), active_(runtime_ != nullptr && runtime_->beginCallback(&epoch_)) {}

RobotRuntime::CallbackGuard::~CallbackGuard() {
  if (active_)
    runtime_->endCallback(epoch_);
}

bool RobotRuntime::beginCallback(std::size_t *epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_)
    return false;
  *epoch = callback_epoch_;
  ++active_callback_epochs_[*epoch];
  ++active_callbacks_;
  return true;
}

void RobotRuntime::endCallback(std::size_t epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  --active_callback_epochs_[epoch];
  if (active_callbacks_ > 0)
    --active_callbacks_;
  if (active_callback_epochs_[epoch] == 0)
    callbacks_idle_.notify_all();
}

RobotRuntime::StatusUpdateGuard::StatusUpdateGuard(RobotRuntime *runtime)
    : runtime_(runtime), active_(runtime_ != nullptr && runtime_->beginStatusUpdate()) {}

RobotRuntime::StatusUpdateGuard::~StatusUpdateGuard() {
  if (active_) runtime_->endStatusUpdate();
}

bool RobotRuntime::beginStatusUpdate() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_) return false;
  ++active_status_updates_;
  return true;
}

void RobotRuntime::endStatusUpdate() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (--active_status_updates_ == 0) callbacks_idle_.notify_all();
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
  chassis_state_subscriber_.shutdown();
  controller_status_subscriber_.shutdown();
  reset_session_subscriber_.shutdown();
  reset_result_subscriber_.shutdown();

  {
    std::unique_lock<std::mutex> lock(mutex_);
    callbacks_idle_.wait(lock, [this] { return active_callbacks_ == 0 && active_status_updates_ == 0; });
  }
  canonical_pose_publisher_.shutdown();
  canonical_velocity_publisher_.shutdown();
  canonical_acceleration_publisher_.shutdown();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_complete_ = true;
  }
  callbacks_idle_.notify_all();
}

bool RobotRuntime::channelEnabled(const std::string &channel_id) const {
  return enabled_channels_.count(channel_id) != 0;
}

bool RobotRuntime::install(std::string *error) {
  if (!variant_.contract.profileDigest(profile_id_)) {
    if (error != nullptr) {
      *error = "unsupported profile: " + profile_id_;
    }
    return false;
  }

  const std::weak_ptr<RobotRuntime> weak_self = shared_from_this();
  if (!canonical_pose_endpoint_.empty()) {
    canonical_pose_publisher_ =
        node_handle_.advertise<geometry_msgs::PoseStamped>(canonical_pose_endpoint_, 20, false);
    canonical_velocity_publisher_ =
        node_handle_.advertise<geometry_msgs::TwistStamped>(canonical_velocity_endpoint_, 20, false);
    canonical_acceleration_publisher_ =
        node_handle_.advertise<geometry_msgs::AccelStamped>(canonical_acceleration_endpoint_, 20, false);
    if (!requireRosRegistration(canonical_pose_publisher_, canonical_pose_endpoint_, error) ||
        !requireRosRegistration(canonical_velocity_publisher_, canonical_velocity_endpoint_, error) ||
        !requireRosRegistration(canonical_acceleration_publisher_, canonical_acceleration_endpoint_, error))
      return false;
  }
  {
    // Shared ROS callbacks may start immediately after subscribe(). Keep
    // tracking state and subscriber registration behind the callback mutex.
    std::lock_guard<std::mutex> lock(mutex_);
    if (required_channels_.count("vrpn.position") != 0u ||
        required_channels_.count("vrpn.speed") != 0u) {
      ensureSourceLocked(
          "vrpn.position",
          channelStaleAfterSeconds(variant_.contract, profile_id_, "vrpn.position"));
      pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>(
          pose_endpoint_, 20,
          [weak_self](const geometry_msgs::PoseStamped::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->poseCallback(message);
            }
          }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(pose_subscriber_, pose_endpoint_, error))
        return false;
    }
    if (required_channels_.count("vrpn.velocity") != 0u ||
        required_channels_.count("vrpn.speed") != 0u) {
      if (required_channels_.count("vrpn.velocity") != 0u)
        ensureSourceLocked(
            "vrpn.velocity",
            channelStaleAfterSeconds(variant_.contract, profile_id_, "vrpn.velocity"));
      if (required_channels_.count("vrpn.speed") != 0u)
        ensureSourceLocked(
            "vrpn.speed", channelStaleAfterSeconds(variant_.contract, profile_id_, "vrpn.speed"));
      vrpn_velocity_subscriber_ =
          node_handle_.subscribe<geometry_msgs::TwistStamped>(
              vrpn_velocity_endpoint_, 20,
              [weak_self](const geometry_msgs::TwistStamped::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->vrpnVelocityCallback(message);
                }
              }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(vrpn_velocity_subscriber_,
                                  vrpn_velocity_endpoint_, error))
        return false;
    }
    if (required_channels_.count("vrpn.acceleration") != 0u) {
      ensureSourceLocked(
          "vrpn.acceleration",
          channelStaleAfterSeconds(variant_.contract, profile_id_, "vrpn.acceleration"));
      vrpn_acceleration_subscriber_ =
          node_handle_.subscribe<geometry_msgs::TwistStamped>(
              vrpn_acceleration_endpoint_, 20,
              [weak_self](const geometry_msgs::TwistStamped::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->vrpnAccelerationCallback(
                      geometry_msgs::AccelStamped::Ptr(new geometry_msgs::AccelStamped(
                          xgc2_ros1_robot_adapter::projectVrpnAcceleration(*message))));
                }
              }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(vrpn_acceleration_subscriber_,
                                  vrpn_acceleration_endpoint_, error))
        return false;
    }
    if (required_channels_.count("command.velocity") != 0u) {
      ensureSourceLocked(
          "command.velocity",
          channelStaleAfterSeconds(variant_.contract, profile_id_, "command.velocity"));
      command_velocity_subscriber_ =
          node_handle_.subscribe<geometry_msgs::Twist>(
              command_velocity_endpoint_, 20,
              [weak_self](const geometry_msgs::Twist::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->commandVelocityCallback(message);
                }
              }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(command_velocity_subscriber_,
                                  command_velocity_endpoint_, error))
        return false;
    }
    if (required_channels_.count("state.imu") != 0u) {
      ensureSourceLocked("state.imu",
                         channelStaleAfterSeconds(variant_.contract, profile_id_, "state.imu"));
      imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
          imu_endpoint_, 20,
          [weak_self](const sensor_msgs::Imu::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->imuCallback(message);
            }
          }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(imu_subscriber_, imu_endpoint_, error))
        return false;
    }
    if (required_channels_.count("state.power") != 0u) {
      ensureSourceLocked(
          "state.power",
          channelStaleAfterSeconds(variant_.contract, profile_id_, "state.power"));
      voltage_subscriber_ = node_handle_.subscribe<std_msgs::Float32>(
          voltage_endpoint_, 10,
          [weak_self](const std_msgs::Float32::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->voltageCallback(message);
            }
          }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(voltage_subscriber_, voltage_endpoint_,
                                  error))
        return false;
    }
    if (variant_.health == HealthSource::kChassisState &&
        (required_channels_.count("state.health") != 0u ||
         required_channels_.count("state.chassis") != 0u)) {
      if (required_channels_.count("state.health") != 0u)
        ensureSourceLocked(
            "state.health",
            channelStaleAfterSeconds(variant_.contract, profile_id_, "state.health"));
      if (required_channels_.count("state.chassis") != 0u)
        ensureSourceLocked(
            "state.chassis",
            channelStaleAfterSeconds(variant_.contract, profile_id_, "state.chassis"));
      chassis_state_subscriber_ = node_handle_.subscribe<std_msgs::UInt32>(
          chassis_state_endpoint_, 10,
          [weak_self](const std_msgs::UInt32::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->chassisStateCallback(message);
            }
          }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(chassis_state_subscriber_,
                                  chassis_state_endpoint_, error))
        return false;
    }
    if (required_channels_.count("state.controller") != 0u) {
      ensureSourceLocked(
          "state.controller",
          channelStaleAfterSeconds(variant_.contract, profile_id_, "state.controller"));
      controller_status_subscriber_ = node_handle_.subscribe<std_msgs::String>(
          controller_status_endpoint_, 10,
          [weak_self](const std_msgs::String::ConstPtr &message) {
            if (const auto self = weak_self.lock()) {
              self->controllerStatusCallback(message);
            }
          }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(controller_status_subscriber_,
                                    controller_status_endpoint_, error))
        return false;
    }
    if (required_channels_.count("state.reset") != 0u) {
      ensureSourceLocked(
          "state.reset",
          channelStaleAfterSeconds(variant_.contract, profile_id_, "state.reset"));
      reset_session_subscriber_ =
          node_handle_.subscribe<ugv_reset_msgs::ResetSession>(
              reset_session_endpoint_, 10,
              [weak_self](const ugv_reset_msgs::ResetSession::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->resetSessionCallback(message);
                }
              },
              ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      reset_result_subscriber_ =
          node_handle_.subscribe<ugv_reset_msgs::ResetResult>(
              reset_result_endpoint_, 10,
              [weak_self](const ugv_reset_msgs::ResetResult::ConstPtr &message) {
                if (const auto self = weak_self.lock()) {
                  self->resetResultCallback(message);
                }
              },
              ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
      if (!requireRosRegistration(reset_session_subscriber_,
                                  reset_session_endpoint_, error) ||
          !requireRosRegistration(reset_result_subscriber_,
                                  reset_result_endpoint_, error))
        return false;
    }
  }

  ROS_INFO_STREAM("XGC " << variant_.name << " Robot Adapter robot=" << robot_id_
                  << " namespace=/" << robot_namespace_);
  return true;
}

bool RobotRuntime::shouldEmitLocked(const std::string &channel_id,
                                    const ros::WallTime &now) {
  if (!status_demand_.count(channel_id) ||
      (!forced_channel_.empty() && forced_channel_ != channel_id))
    return false;
  contract::ChannelMetadata metadata{};
  if (!variant_.contract.channelMetadata(profile_id_, channel_id, &metadata) ||
      metadata.output_rate_hz <= 0.0) {
    return false;
  }
  auto &last = last_output_[channel_id];
  if (forced_channel_.empty() && !last.isZero() && now >= last &&
      (now - last).toSec() < (1.0 / metadata.output_rate_hz)) {
    return false;
  }
  last = now;
  return true;
}

xgc::robot::v1::RobotMessage RobotRuntime::makeEnvelopeLocked(
    const std::string &channel_id, const ros::Time &source_stamp,
    const google::protobuf::Message &payload) {
  auto cached = message_schemas_.find(channel_id);
  if (cached == message_schemas_.end()) {
    contract::ChannelMetadata channel{};
    if (!variant_.contract.channelMetadata(profile_id_, channel_id, &channel) ||
        channel.output_message_id == 0u) {
      throw std::logic_error(
          "channel output is absent from generated XGC2 contract metadata");
    }
    contract::MessageMetadata metadata{};
    if (!variant_.contract.messageMetadata(channel.output_message_id, &metadata)) {
      throw std::logic_error(
          "message ID is absent from generated XGC2 contract metadata");
    }
    xgc2_ros1_robot_adapter::MessageSchema schema;
    schema.message_id = channel.output_message_id;
    schema.type_name = metadata.type_name;
    schema.version = metadata.version;
    schema.fingerprint = metadata.fingerprint;

    cached = message_schemas_.emplace(channel_id, std::move(schema)).first;
  }
  const auto &schema = cached->second;

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
  ros::WallTime observed = ros::WallTime::now();
  if (!forced_channel_.empty() && channel_id != "state.health" &&
      channel_id != "diagnostic.stream-health") {
    const auto source = sources_.find(channel_id);
    if (source != sources_.end() && !source->second.last_seen.isZero())
      observed = source->second.last_seen;
  }
  context.observed_unix_nanos = static_cast<std::int64_t>(observed.toNSec());

  xgc::robot::v1::RobotMessage envelope;
  std::string error;
  if (!xgc2_ros1_robot_adapter::EncodeRobotMessage(
          context, schema, payload, &envelope, &error)) {
    throw std::runtime_error("failed to build robot telemetry item: " + error);
  }
  return envelope;
}

void RobotRuntime::emit(std::vector<xgc::robot::v1::RobotMessage> messages) {
  for (auto &message : messages) emitter_(std::move(message));
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
  geometry_msgs::PoseStamped projected;
  if (!xgc2_ros1_robot_adapter::projectLocalizationPose(
          *message, localization_, &projected)) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_pose_.reset();
    has_vrpn_orientation_ = false;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channelEnabled("vrpn.position") &&
        message->header.frame_id.size() <= xgc2_ros1_robot_adapter::kMaximumTelemetryItemBytes)
      latest_pose_ = message;
    else
      latest_pose_.reset();
    vrpn_orientation_ = projected.pose.orientation;
    has_vrpn_orientation_ = true;
    positioning_health_.recordPose(
        now.toSec(), projected.pose.position.x, projected.pose.position.y,
        projected.pose.position.z);
    recordSourceLocked("vrpn.position", now);
    if (latest_pose_ && channelEnabled("vrpn.position") &&
        shouldEmitLocked("vrpn.position", now)) {
      xgc::semantic::common::v1::PoseEstimate payload;
      payload.set_frame_id(projected.header.frame_id);
      payload.set_child_frame_id(mocap_rigid_body_);
      copyVector(projected.pose.position, payload.mutable_position());
      copyQuaternion(projected.pose.orientation,
                     payload.mutable_orientation());
      output.push_back(makeEnvelopeLocked("vrpn.position",
                                          projected.header.stamp, payload));
      recordOutputLocked("vrpn.position");
    } else if (status_demand_.count("vrpn.position") && channelEnabled("vrpn.position")) {
      ++sources_["vrpn.position"].dropped_samples;
    }
  }
  if (canonical_pose_publisher_) canonical_pose_publisher_.publish(projected);
  emit(std::move(output));
}

void RobotRuntime::commandVelocityCallback(
    const geometry_msgs::Twist::ConstPtr &message) {
  static const std::string command_velocity_channel = "command.velocity";
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channelEnabled(command_velocity_channel))
      latest_command_velocity_ = message;
    recordSourceLocked(command_velocity_channel, now);
    if (channelEnabled(command_velocity_channel) &&
        shouldEmitLocked(command_velocity_channel, now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      copyVector(message->linear, payload.mutable_linear());
      copyVector(message->angular, payload.mutable_angular());
      output.push_back(
          makeEnvelopeLocked(command_velocity_channel, ros::Time(), payload));
      recordOutputLocked(command_velocity_channel);
    } else if (status_demand_.count(command_velocity_channel) && channelEnabled(command_velocity_channel)) {
      ++sources_[command_velocity_channel].dropped_samples;
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
  if (!xgc2_ros1_robot_adapter::validLocalizationTwist(*message)) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_velocity_.reset();
    has_velocity_ = false;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channelEnabled("vrpn.speed")) {
      latest_velocity_value_ = message->twist;
      latest_velocity_stamp_ = message->header.stamp;
      has_velocity_ = true;
    }
    if (channelEnabled("vrpn.velocity") &&
        message->header.frame_id.size() <= xgc2_ros1_robot_adapter::kMaximumTelemetryItemBytes)
      latest_velocity_ = message;
    else
      latest_velocity_.reset();
    if (required_channels_.count("vrpn.velocity") != 0u)
      recordSourceLocked("vrpn.velocity", now);
    if (required_channels_.count("vrpn.speed") != 0u)
      recordSourceLocked("vrpn.speed", now);

    if (latest_velocity_ && channelEnabled("vrpn.velocity") &&
        shouldEmitLocked("vrpn.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyVector(message->twist.linear, payload.mutable_linear());
      copyVector(message->twist.angular, payload.mutable_angular());
      output.push_back(makeEnvelopeLocked("vrpn.velocity",
                                          message->header.stamp, payload));
      recordOutputLocked("vrpn.velocity");
    } else if (status_demand_.count("vrpn.velocity") && channelEnabled("vrpn.velocity")) {
      ++sources_["vrpn.velocity"].dropped_samples;
    }

    if (status_demand_.count("vrpn.speed") && channelEnabled("vrpn.speed")) {
      const double speed = has_vrpn_orientation_ && sourceFreshLocked("vrpn.position", now)
                               ? vrpnForwardSpeedMetersPerSecond(
                                     message->twist.linear.x, message->twist.linear.y,
                                     message->twist.linear.z, vrpn_orientation_.x,
                                     vrpn_orientation_.y, vrpn_orientation_.z,
                                     vrpn_orientation_.w)
                               : std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(speed) && shouldEmitLocked("vrpn.speed", now)) {
        xgc::semantic::common::v1::SpeedEstimate payload;
        payload.set_frame_id(mocap_rigid_body_);
        payload.set_meters_per_second(speed);
        output.push_back(makeEnvelopeLocked("vrpn.speed", message->header.stamp, payload));
        recordOutputLocked("vrpn.speed");
      } else {
        ++sources_["vrpn.speed"].dropped_samples;
      }
    }
  }
  if (canonical_velocity_publisher_) canonical_velocity_publisher_.publish(*message);
  emit(std::move(output));
}

void RobotRuntime::vrpnAccelerationCallback(
    const geometry_msgs::AccelStamped::ConstPtr &message) {
  static const std::string vrpn_acceleration_channel = "vrpn.acceleration";
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  if (!xgc2_ros1_robot_adapter::validLocalizationAcceleration(*message)) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_acceleration_.reset();
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channelEnabled(vrpn_acceleration_channel) &&
        message->header.frame_id.size() <= xgc2_ros1_robot_adapter::kMaximumTelemetryItemBytes)
      latest_acceleration_ = message;
    else
      latest_acceleration_.reset();
    recordSourceLocked(vrpn_acceleration_channel, now);
    if (latest_acceleration_ && channelEnabled(vrpn_acceleration_channel) &&
        shouldEmitLocked(vrpn_acceleration_channel, now)) {
      output.push_back(makeEnvelopeLocked(
          vrpn_acceleration_channel, message->header.stamp,
          vrpnAccelerationEstimate(*message)));
      recordOutputLocked(vrpn_acceleration_channel);
    } else if (status_demand_.count(vrpn_acceleration_channel) && channelEnabled(vrpn_acceleration_channel)) {
      ++sources_[vrpn_acceleration_channel].dropped_samples;
    }
  }
  if (canonical_acceleration_publisher_) canonical_acceleration_publisher_.publish(*message);
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
    if (channelEnabled("state.imu") &&
        message->header.frame_id.size() <= xgc2_ros1_robot_adapter::kMaximumTelemetryItemBytes)
      latest_imu_ = message;
    else
      latest_imu_.reset();
    recordSourceLocked("state.imu", now);
    if (latest_imu_ && channelEnabled("state.imu") && shouldEmitLocked("state.imu", now)) {
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
    } else if (status_demand_.count("state.imu") && channelEnabled("state.imu")) {
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
    double battery_voltage = voltage;
    bool estimate_valid = true;
    if (channelEnabled("state.power") && variant_.filter_battery)
      estimate_valid = battery_voltage_filter_.update(
          voltage, ros::SteadyTime::now().toSec(), &battery_voltage);
    if (channelEnabled("state.power")) {
      has_power_ = true;
      latest_power_voltage_ = voltage;
      latest_power_filtered_ = battery_voltage;
      latest_power_valid_ = estimate_valid;
      latest_power_stamp_ = stamp;
    }
    if (channelEnabled("state.power") && shouldEmitLocked("state.power", now)) {
      xgc::semantic::common::v1::PowerStatus payload;
      payload.set_percentage_state(
          xgc::semantic::common::v1::PowerStatus::PERCENTAGE_STATE_UNAVAILABLE);
      if (std::isfinite(voltage)) {
        payload.set_voltage_v(voltage);
        double percentage = 0.0;
        if (estimate_valid && xgc2_ros1_robot_adapter::batteryPercentage(
                battery_curve_, battery_voltage, &percentage)) {
          payload.set_percentage(percentage);
          payload.set_percentage_state(
              xgc::semantic::common::v1::PowerStatus::
                  PERCENTAGE_STATE_AVAILABLE);
        }
      }
      output.push_back(makeEnvelopeLocked("state.power", stamp, payload));
      recordOutputLocked("state.power");
    } else if (status_demand_.count("state.power") && channelEnabled("state.power")) {
      ++sources_["state.power"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::chassisStateCallback(
    const std_msgs::UInt32::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  ChassisState parsed;
  if (!unpackChassisState(message->data, &parsed))
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  const ros::Time stamp = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    chassis_status_ = parsed;
    status_stamp_ = stamp;
    has_chassis_ = true;
    if (required_channels_.count("state.health") != 0u)
      recordSourceLocked("state.health", now);
    if (required_channels_.count("state.chassis") != 0u)
      recordSourceLocked("state.chassis", now);
    if (channelEnabled("state.chassis") &&
        shouldEmitLocked("state.chassis", now)) {
      xgc::semantic::ground::v1::ChassisStatus payload;
      payload.set_control_mode(chassisControlMode(
          static_cast<std::uint8_t>(parsed.control_mode)));
      payload.set_native_control_mode(
          static_cast<std::uint32_t>(parsed.control_mode));
      output.push_back(makeEnvelopeLocked("state.chassis", stamp, payload));
      recordOutputLocked("state.chassis");
    } else if (status_demand_.count("state.chassis") && channelEnabled("state.chassis")) {
      ++sources_["state.chassis"].dropped_samples;
    }
  }
  emit(std::move(output));
}

namespace {

xgc::semantic::ground::v1::ResetPhase resetPhase(std::uint8_t native_phase) {
  switch (native_phase) {
  case 0:
    return xgc::semantic::ground::v1::RESET_PHASE_IDLE;
  case 1:
    return xgc::semantic::ground::v1::RESET_PHASE_REQUESTING;
  case 2:
    return xgc::semantic::ground::v1::RESET_PHASE_WAITING;
  case 3:
    return xgc::semantic::ground::v1::RESET_PHASE_MOVING;
  case 4:
    return xgc::semantic::ground::v1::RESET_PHASE_SETTLING;
  case 5:
    return xgc::semantic::ground::v1::RESET_PHASE_DONE;
  default:
    return xgc::semantic::ground::v1::RESET_PHASE_UNSPECIFIED;
  }
}

xgc::semantic::ground::v1::ResetOutcome resetOutcome(std::uint32_t native_outcome) {
  return static_cast<xgc::semantic::ground::v1::ResetOutcome>(native_outcome);
}

xgc::semantic::ground::v1::ResetReason resetReason(std::uint32_t native_reason) {
  return static_cast<xgc::semantic::ground::v1::ResetReason>(native_reason);
}

} // namespace

void RobotRuntime::resetSessionCallback(
    const ugv_reset_msgs::ResetSession::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback || !message)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    noteResetSessionLocked(*message, ros::WallTime::now(), &output);
  }
  emit(std::move(output));
}

void RobotRuntime::resetResultCallback(
    const ugv_reset_msgs::ResetResult::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback || !message)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    noteResetResultLocked(*message, ros::WallTime::now(), &output);
  }
  emit(std::move(output));
}

void RobotRuntime::noteResetSessionLocked(
    const ugv_reset_msgs::ResetSession &message, const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *output) {
  recordSourceLocked("state.reset", now);
  reset_session_.present = true;
  reset_session_.session = message.session;
  reset_session_.phase = message.phase;
  reset_session_.source_stamp = message.header.stamp;
  publishResetLocked(now, output);
}

void RobotRuntime::noteResetResultLocked(
    const ugv_reset_msgs::ResetResult &message, const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *output) {
  if (message.session == 0u || message.outcome == 0u)
    return;
  if (message.detail.size() > xgc2_ros1_robot_adapter::kMaximumTelemetryItemBytes) {
    for (auto &slot : reset_results_)
      if (slot.present && slot.session == message.session)
        slot = {};
    publishResetLocked(now, output);
    return;
  }
  auto fill = [&](ResetResultSlot *slot) {
    slot->present = true;
    slot->session = message.session;
    slot->outcome = message.outcome;
    slot->reason = message.reason;
    slot->final_planar_error_m = message.final_planar_error;
    slot->final_yaw_error_rad = message.final_yaw_error;
    slot->duration_s = message.duration;
    slot->detail = message.detail;
    slot->source_stamp = message.header.stamp;
  };
  // [0] is the earlier receipt. A new or updated result becomes [1].
  auto note_newest = [&](int index) {
    if (index == 0)
      std::swap(reset_results_[0], reset_results_[1]);
  };
  int found = -1;
  for (int index = 0; index < 2; ++index) {
    if (reset_results_[static_cast<std::size_t>(index)].present &&
        reset_results_[static_cast<std::size_t>(index)].session == message.session) {
      found = index;
      break;
    }
  }
  if (found >= 0) {
    fill(&reset_results_[static_cast<std::size_t>(found)]);
    note_newest(found);
  } else {
    int dest = -1;
    for (int index = 0; index < 2; ++index) {
      if (!reset_results_[static_cast<std::size_t>(index)].present) {
        dest = index;
        break;
      }
    }
    if (dest < 0) {
      const std::uint32_t current =
          reset_session_.present ? reset_session_.session : 0u;
      for (int index = 0; index < 2; ++index) {
        const ResetResultSlot &slot = reset_results_[static_cast<std::size_t>(index)];
        const bool protected_match =
            current != 0u && slot.present && slot.session == current;
        if (!protected_match) {
          dest = index;
          break;
        }
      }
    }
    if (dest >= 0) {
      fill(&reset_results_[static_cast<std::size_t>(dest)]);
      note_newest(dest);
    }
  }
  publishResetLocked(now, output);
}

void RobotRuntime::publishResetLocked(
    const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *output) {
  static const std::string channel = "state.reset";
  if (!reset_session_.present || !channelEnabled(channel) ||
      !status_demand_.count(channel) ||
      (!forced_channel_.empty() && forced_channel_ != channel) ||
      !sourceFreshLocked(channel, now))
    return;
  const ResetResultSlot *matched_slot = nullptr;
  if (reset_session_.session != 0u) {
    for (const ResetResultSlot &slot : reset_results_) {
      if (slot.present && slot.outcome != 0u &&
          slot.session == reset_session_.session) {
        matched_slot = &slot;
        break;
      }
    }
  }
  const bool matched = matched_slot != nullptr;
  const xgc::semantic::ground::v1::ResetPhase phase =
      resetPhase(reset_session_.phase);
  ResetTuple tuple;
  tuple.valid = true;
  tuple.generation = reset_session_.session;
  tuple.phase = static_cast<int>(phase);
  tuple.matched = matched;
  tuple.outcome = matched ? static_cast<int>(matched_slot->outcome) : 0;
  tuple.reason = matched ? static_cast<int>(matched_slot->reason) : 0;
  const bool discrete =
      !last_reset_tuple_.valid ||
      last_reset_tuple_.generation != tuple.generation ||
      last_reset_tuple_.phase != tuple.phase ||
      last_reset_tuple_.matched != tuple.matched ||
      last_reset_tuple_.outcome != tuple.outcome ||
      last_reset_tuple_.reason != tuple.reason;
  if (!discrete) {
    if (!shouldEmitLocked(channel, now)) {
      ++sources_[channel].dropped_samples;
      return;
    }
  } else {
    last_output_[channel] = now;
  }
  xgc::semantic::ground::v1::ResetStatus payload;
  payload.set_generation(reset_session_.session);
  payload.set_phase(phase);
  if (matched) {
    auto *result = payload.mutable_result();
    result->set_outcome(resetOutcome(matched_slot->outcome));
    result->set_reason(resetReason(matched_slot->reason));
    result->set_final_planar_error_m(matched_slot->final_planar_error_m);
    result->set_final_yaw_error_rad(matched_slot->final_yaw_error_rad);
    result->set_duration_s(matched_slot->duration_s);
    result->set_detail(matched_slot->detail);
    result->set_source_stamp_ns(matched_slot->source_stamp.isZero()
                                    ? 0
                                    : static_cast<std::int64_t>(
                                          matched_slot->source_stamp.toNSec()));
  }
  output->push_back(makeEnvelopeLocked(channel, reset_session_.source_stamp, payload));
  recordOutputLocked(channel);
  last_reset_tuple_ = tuple;
}

void RobotRuntime::controllerStatusCallback(
    const std_msgs::String::ConstPtr &message) {
  static const std::string state_controller_channel = "state.controller";
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  const ros::Time stamp = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channelEnabled(state_controller_channel) &&
        message->data.size() <= xgc2_ros1_robot_adapter::kMaximumTelemetryItemBytes) {
      latest_controller_ = message;
      latest_controller_stamp_ = stamp;
    } else {
      latest_controller_.reset();
    }
    recordSourceLocked(state_controller_channel, now);
    if (latest_controller_ && channelEnabled(state_controller_channel) &&
        shouldEmitLocked(state_controller_channel, now)) {
      xgc::semantic::common::v1::ControllerStatus payload;
      payload.set_text(message->data);
      output.push_back(makeEnvelopeLocked(state_controller_channel, stamp, payload));
      recordOutputLocked(state_controller_channel);
    } else if (status_demand_.count(state_controller_channel) && channelEnabled(state_controller_channel)) {
      ++sources_[state_controller_channel].dropped_samples;
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

void RobotRuntime::SetStatusDemand(const std::set<std::string> &channels) {
  StatusUpdateGuard update(this);
  if (!update) return;
  std::vector<xgc::robot::v1::RobotMessage> messages;
  std::unique_lock<std::mutex> lock(mutex_);
  if (stopping_)
    return;
  std::vector<std::string> added;
  bool removed = false;
  for (const auto &channel : channels)
    if (channelEnabled(channel) && !status_demand_.count(channel))
      added.push_back(channel);
  for (const auto &channel : status_demand_)
    if (!channels.count(channel))
      removed = true;
  status_demand_ = channels;
  if (removed) {
    const auto old_epoch = callback_epoch_;
    callback_epoch_ ^= 1u;
    callbacks_idle_.wait(lock, [&] { return active_callback_epochs_[old_epoch] == 0; });
    if (stopping_)
      return;
  }
  try {
    const auto now = ros::WallTime::now();
    for (const auto &channel : added) {
      forced_channel_ = channel;
      emitSnapshotLocked(channel, now, &messages);
    }
    forced_channel_.clear();
  } catch (...) {
    forced_channel_.clear();
    throw;
  }
  lock.unlock();
  emit(std::move(messages));
}

void RobotRuntime::emitSnapshotLocked(
    const std::string &channel, const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (channel == "state.health") {
    emitHealthLocked(now, messages);
    return;
  }
  if (channel == "diagnostic.stream-health") {
    emitStreamHealthLocked(now, messages);
    return;
  }
  if (channel == "state.reset") {
    publishResetLocked(now, messages);
    return;
  }
  if (channel == "vrpn.position" && latest_pose_) {
    geometry_msgs::PoseStamped projected;
    if (!xgc2_ros1_robot_adapter::projectLocalizationPose(*latest_pose_, localization_, &projected) ||
        !shouldEmitLocked(channel, now))
      return;
    xgc::semantic::common::v1::PoseEstimate payload;
    payload.set_frame_id(projected.header.frame_id);
    payload.set_child_frame_id(mocap_rigid_body_);
    copyVector(projected.pose.position, payload.mutable_position());
    copyQuaternion(projected.pose.orientation, payload.mutable_orientation());
    messages->push_back(makeEnvelopeLocked(channel, projected.header.stamp, payload));
  } else if (channel == "command.velocity" && latest_command_velocity_) {
    if (!shouldEmitLocked(channel, now)) return;
    xgc::semantic::common::v1::VelocityEstimate payload;
    copyVector(latest_command_velocity_->linear, payload.mutable_linear());
    copyVector(latest_command_velocity_->angular, payload.mutable_angular());
    messages->push_back(makeEnvelopeLocked(channel, ros::Time(), payload));
  } else if (channel == "vrpn.velocity" && latest_velocity_) {
    if (!shouldEmitLocked(channel, now)) return;
    xgc::semantic::common::v1::VelocityEstimate payload;
    payload.set_frame_id(latest_velocity_->header.frame_id);
    copyVector(latest_velocity_->twist.linear, payload.mutable_linear());
    copyVector(latest_velocity_->twist.angular, payload.mutable_angular());
    messages->push_back(makeEnvelopeLocked(channel, latest_velocity_->header.stamp, payload));
  } else if (channel == "vrpn.speed" && has_velocity_ && has_vrpn_orientation_ &&
             sourceFreshLocked("vrpn.position", now)) {
    const double speed = vrpnForwardSpeedMetersPerSecond(
        latest_velocity_value_.linear.x, latest_velocity_value_.linear.y,
        latest_velocity_value_.linear.z, vrpn_orientation_.x, vrpn_orientation_.y,
        vrpn_orientation_.z, vrpn_orientation_.w);
    if (!std::isfinite(speed) || !shouldEmitLocked(channel, now)) return;
    xgc::semantic::common::v1::SpeedEstimate payload;
    payload.set_frame_id(mocap_rigid_body_);
    payload.set_meters_per_second(speed);
    messages->push_back(makeEnvelopeLocked(channel, latest_velocity_stamp_, payload));
  } else if (channel == "vrpn.acceleration" && latest_acceleration_) {
    if (!shouldEmitLocked(channel, now)) return;
    messages->push_back(makeEnvelopeLocked(channel, latest_acceleration_->header.stamp,
                                            vrpnAccelerationEstimate(*latest_acceleration_)));
  } else if (channel == "state.imu" && latest_imu_) {
    if (!shouldEmitLocked(channel, now)) return;
    xgc::semantic::common::v1::ImuEstimate payload;
    payload.set_frame_id(latest_imu_->header.frame_id);
    copyQuaternion(latest_imu_->orientation, payload.mutable_orientation());
    copyVector(latest_imu_->angular_velocity, payload.mutable_angular_velocity());
    copyVector(latest_imu_->linear_acceleration, payload.mutable_linear_acceleration());
    copyCovariance(latest_imu_->orientation_covariance, payload.mutable_orientation_covariance());
    copyCovariance(latest_imu_->angular_velocity_covariance, payload.mutable_angular_velocity_covariance());
    copyCovariance(latest_imu_->linear_acceleration_covariance, payload.mutable_linear_acceleration_covariance());
    messages->push_back(makeEnvelopeLocked(channel, latest_imu_->header.stamp, payload));
  } else if (channel == "state.power" && has_power_) {
    if (!shouldEmitLocked(channel, now)) return;
    xgc::semantic::common::v1::PowerStatus payload;
    payload.set_percentage_state(xgc::semantic::common::v1::PowerStatus::PERCENTAGE_STATE_UNAVAILABLE);
    if (std::isfinite(latest_power_voltage_)) {
      payload.set_voltage_v(latest_power_voltage_);
      double percentage = 0;
      if (latest_power_valid_ && xgc2_ros1_robot_adapter::batteryPercentage(
              battery_curve_, latest_power_filtered_, &percentage)) {
        payload.set_percentage(percentage);
        payload.set_percentage_state(xgc::semantic::common::v1::PowerStatus::PERCENTAGE_STATE_AVAILABLE);
      }
    }
    messages->push_back(makeEnvelopeLocked(channel, latest_power_stamp_, payload));
  } else if (channel == "state.chassis" && has_chassis_) {
    if (!shouldEmitLocked(channel, now)) return;
    xgc::semantic::ground::v1::ChassisStatus payload;
    payload.set_control_mode(chassisControlMode(static_cast<std::uint8_t>(chassis_status_.control_mode)));
    payload.set_native_control_mode(static_cast<std::uint32_t>(chassis_status_.control_mode));
    messages->push_back(makeEnvelopeLocked(channel, status_stamp_, payload));
  } else if (channel == "state.controller" && latest_controller_) {
    if (!shouldEmitLocked(channel, now)) return;
    xgc::semantic::common::v1::ControllerStatus payload;
    payload.set_text(latest_controller_->data);
    messages->push_back(makeEnvelopeLocked(channel, latest_controller_stamp_, payload));
  } else {
    return;
  }
  recordOutputLocked(channel);
}

xgc::semantic::common::v1::VehicleHealth
RobotRuntime::healthLocked(const ros::WallTime &now) const {
  const std::string name = variant_.short_name;
  const std::string code_prefix = [&] {
    std::string lower = name;
    for (auto &character : lower)
      character = static_cast<char>(std::tolower(character));
    return lower;
  }();
  const bool chassis_reporting = variant_.health == HealthSource::kChassisState;
  // The input the vehicle's "online" fact follows.
  const char *liveness_channel = chassis_reporting ? "state.health" : "state.imu";
  const auto liveness = sources_.find(liveness_channel);
  const bool observed = chassis_reporting
                            ? has_chassis_
                            : liveness != sources_.end() &&
                                  !liveness->second.last_seen.isZero();
  const bool fresh = (!chassis_reporting || has_chassis_) &&
                     sourceFreshLocked(liveness_channel, now);
  xgc::semantic::common::v1::VehicleHealth payload;
  payload.set_online(fresh);
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
  const std::string source_name = chassis_reporting ? " chassis status" : " IMU";
  const std::string fault_source = chassis_reporting ? ".status" : ".imu";
  if (!observed) {
    addFault(&payload, code_prefix + fault_source + ".missing",
             name + source_name + " has not been observed", 2);
  } else if (!fresh) {
    addFault(&payload, code_prefix + fault_source + ".stale",
             name + source_name + " exceeded its freshness limit", 2);
  } else if (chassis_reporting && chassis_status_.fault_code != 0) {
    addFault(&payload, code_prefix + ".chassis.fault",
             name + " chassis reports fault code " +
                 std::to_string(chassis_status_.fault_code),
             2);
  }
  if (fresh && positioning.state ==
                   xgc2_ros1_robot_adapter::PositioningHealthState::kFrozen) {
    payload.set_summary(name + " VRPN positioning data is frozen");
  } else if (fresh && positioning.state ==
                          xgc2_ros1_robot_adapter::PositioningHealthState::kTimedOut) {
    payload.set_summary(name + " VRPN positioning is unavailable");
  } else if (chassis_reporting) {
    if (fresh && payload.faults_size() == 0)
      payload.set_summary(name + " chassis telemetry is healthy");
    else if (has_chassis_ && chassis_status_.fault_code != 0)
      payload.set_summary(name + " chassis reports a fault");
    else
      payload.set_summary(name + " chassis telemetry is incomplete or stale");
  } else {
    payload.set_summary(name + (fresh ? " on-board telemetry is healthy"
                                      : " on-board telemetry is missing or stale"));
  }
  return payload;
}

std::pair<bool, std::string> RobotRuntime::DescribeHealth() {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto health = healthLocked(ros::WallTime::now());
  return {health.online(), health.summary()};
}

void RobotRuntime::emitHealthLocked(const ros::WallTime &now,
                                    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("state.health") ||
      !shouldEmitLocked("state.health", now)) {
    return;
  }

  auto payload = healthLocked(now);
  const bool chassis_reporting = variant_.health == HealthSource::kChassisState;
  // The chassis report is stamped by the vehicle's own state word; an IMU-only
  // vehicle has no source time of its own for the health fact.
  const ros::Time stamp =
      chassis_reporting && has_chassis_ ? status_stamp_ : ros::Time::now();
  messages->push_back(makeEnvelopeLocked("state.health", stamp, payload));
  recordOutputLocked("state.health");
}

void RobotRuntime::emitStreamHealthLocked(
    const ros::WallTime &now, std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("diagnostic.stream-health") ||
      !shouldEmitLocked("diagnostic.stream-health", now)) {
    return;
  }
  contract::ChannelMetadata diagnostic{};
  if (!variant_.contract.channelMetadata(profile_id_, "diagnostic.stream-health",
                                 &diagnostic)) {
    throw std::logic_error("generated diagnostic binding is missing");
  }
  xgc::semantic::common::v1::StreamHealthReport report;
  for (std::size_t index = 0u; index < diagnostic.observes_count; ++index) {
    const std::string channel_id(diagnostic.observes[index]);
    const auto found = sources_.find(channel_id);
    if (found == sources_.end())
      throw std::logic_error("observed channel has no native tracker");
    auto &source = found->second;
    if (source.window_started.isZero()) {
      source.window_started = now;
    }
    const double elapsed = now >= source.window_started
                               ? (now - source.window_started).toSec()
                               : 0.0;
    const bool fresh =
        sourceIsFresh(source.last_seen, now, source.stale_after_seconds);
    const auto window = updateStreamRateWindow(
        {source.source_rate_hz, source.output_rate_hz}, source.source_samples,
        source.output_samples, elapsed, fresh,
        variant_.stream_rate_window_seconds);
    source.source_rate_hz = window.rates.source_rate_hz;
    source.output_rate_hz = window.rates.output_rate_hz;
    if (window.close) {
      source.source_samples = 0;
      source.output_samples = 0;
      source.window_started = now;
    }

    auto *payload = report.add_channels();
    payload->set_channel_id(channel_id);
    payload->set_source_rate_hz(source.source_rate_hz);
    payload->set_output_rate_hz(source.output_rate_hz);
    payload->set_dropped_samples(source.dropped_samples);
    payload->set_source_age_ms(sourceAgeMillisLocked(channel_id, now));
    payload->set_stale(!fresh);
  }
  messages->push_back(makeEnvelopeLocked("diagnostic.stream-health",
                                         ros::Time::now(), report));
}

} // namespace xgc_ugv_ros1_adapter
