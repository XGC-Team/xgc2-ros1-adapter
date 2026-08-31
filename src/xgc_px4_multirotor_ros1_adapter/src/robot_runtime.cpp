#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>

#include <ros/names.h>

#include "xgc/semantic/aerial/v1/diagnostic.pb.h"
#include "xgc/semantic/aerial/v1/flight.pb.h"
#include "xgc/semantic/aerial/v1/setpoint.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
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

template <typename RosVector>
void copySelectedVector(const RosVector &source,
                        xgc::semantic::common::v1::Vector3 *target,
                        std::uint32_t fields, std::uint32_t first_bit) {
  if ((fields & (1u << first_bit)) != 0)
    target->set_x(source.x);
  if ((fields & (1u << (first_bit + 1))) != 0)
    target->set_y(source.y);
  if ((fields & (1u << (first_bit + 2))) != 0)
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

void addBlocker(xgc::semantic::aerial::v1::OffboardInputStatus *status,
                const std::string &code, const std::string &summary,
                std::uint32_t severity) {
  auto *fault = status->add_blockers();
  fault->set_code(code);
  fault->set_summary(summary);
  fault->set_severity(severity);
}

xgc::semantic::aerial::v1::LocalCoordinateFrame
localCoordinateFrame(std::uint8_t frame) {
  switch (frame) {
  case mavros_msgs::PositionTarget::FRAME_LOCAL_NED:
    return xgc::semantic::aerial::v1::LOCAL_COORDINATE_FRAME_NED;
  case mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED:
    return xgc::semantic::aerial::v1::LOCAL_COORDINATE_FRAME_OFFSET_NED;
  case mavros_msgs::PositionTarget::FRAME_BODY_NED:
    return xgc::semantic::aerial::v1::LOCAL_COORDINATE_FRAME_BODY_NED;
  case mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED:
    return xgc::semantic::aerial::v1::LOCAL_COORDINATE_FRAME_BODY_OFFSET_NED;
  default:
    return xgc::semantic::aerial::v1::LOCAL_COORDINATE_FRAME_UNSPECIFIED;
  }
}

std::uint32_t localValidFieldsImpl(std::uint16_t mask) {
  std::uint32_t fields = 0;
  const std::uint16_t ignore[] = {
      mavros_msgs::PositionTarget::IGNORE_PX,
      mavros_msgs::PositionTarget::IGNORE_PY,
      mavros_msgs::PositionTarget::IGNORE_PZ,
      mavros_msgs::PositionTarget::IGNORE_VX,
      mavros_msgs::PositionTarget::IGNORE_VY,
      mavros_msgs::PositionTarget::IGNORE_VZ,
      mavros_msgs::PositionTarget::IGNORE_AFX,
      mavros_msgs::PositionTarget::IGNORE_AFY,
      mavros_msgs::PositionTarget::IGNORE_AFZ,
      mavros_msgs::PositionTarget::IGNORE_YAW,
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE,
  };
  for (std::size_t index = 0; index < sizeof(ignore) / sizeof(ignore[0]);
       ++index) {
    if ((mask & ignore[index]) == 0)
      fields |= (1u << index);
  }
  return fields;
}

std::uint32_t attitudeValidFieldsImpl(std::uint8_t mask) {
  std::uint32_t fields = 0;
  if ((mask & mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE) == 0)
    fields |= 1u << 0;
  if ((mask & mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE) == 0)
    fields |= 1u << 1;
  if ((mask & mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE) == 0)
    fields |= 1u << 2;
  if ((mask & mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE) == 0)
    fields |= 1u << 3;
  if ((mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) == 0)
    fields |= 1u << 4;
  return fields;
}

bool finiteLocalSetpoint(const mavros_msgs::PositionTarget &message,
                         std::uint32_t fields) {
  const double values[] = {
      message.position.x,
      message.position.y,
      message.position.z,
      message.velocity.x,
      message.velocity.y,
      message.velocity.z,
      message.acceleration_or_force.x,
      message.acceleration_or_force.y,
      message.acceleration_or_force.z,
      message.yaw,
      message.yaw_rate,
  };
  for (std::size_t index = 0; index < sizeof(values) / sizeof(values[0]);
       ++index) {
    if ((fields & (1u << index)) != 0 && !std::isfinite(values[index]))
      return false;
  }
  return true;
}

bool finiteAttitudeSetpoint(const mavros_msgs::AttitudeTarget &message,
                            std::uint32_t fields) {
  if ((fields & (1u << 4)) != 0 && !std::isfinite(message.thrust))
    return false;
  if ((fields & (1u << 0)) != 0) {
    const auto &q = message.orientation;
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm_squared) || norm_squared <= 1e-12)
      return false;
  }
  const double rates[] = {message.body_rate.x, message.body_rate.y,
                          message.body_rate.z};
  for (std::size_t index = 0; index < 3; ++index) {
    if ((fields & (1u << (index + 1))) != 0 && !std::isfinite(rates[index]))
      return false;
  }
  return true;
}

template <typename Handle>
bool requireRosRegistration(const Handle &handle, const std::string &endpoint,
                            std::string *error) {
  if (handle)
    return true;
  if (error != nullptr)
    *error =
        "ROS master did not accept native endpoint registration: " + endpoint;
  return false;
}

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

struct NativeChannelBinding {
  const char *id;
  const char *processor;
  contract::ChannelKind kind;
  const char *output_type;
  std::size_t endpoint_count;
  std::size_t policy_count;
  bool observes;
};

const std::array<NativeChannelBinding, 19u> kNativeBindings{{
    {"state.pose", "px4.pose-estimate", contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.PoseEstimate", 1u, 0u, false},
    {"state.mocap.pose", "px4.mocap-pose",
     contract::ChannelKind::kStreamOut, "xgc.semantic.common.v1.PoseEstimate",
     1u, 0u, false},
    {"state.velocity", "px4.velocity-estimate",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.VelocityEstimate", 1u, 0u, false},
    {"state.mocap.velocity", "px4.mocap-velocity",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.VelocityEstimate", 1u, 0u, false},
    {"state.mocap.speed", "px4.mocap-speed",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.SpeedEstimate", 1u, 0u, false},
    {"state.localization.error", "px4.localization-position-error",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.DistanceEstimate", 0u, 0u, true},
    {"state.imu", "px4.imu-estimate", contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.ImuEstimate", 1u, 0u, false},
    {"state.power", "px4.power-status", contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.PowerStatus", 1u, 0u, false},
    {"state.health", "px4.vehicle-health", contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.VehicleHealth", 2u, 0u, true},
    {"state.flight", "px4.flight-status", contract::ChannelKind::kStreamOut,
     "xgc.semantic.aerial.v1.FlightStatus", 2u, 0u, false},
    {"setpoint.local", "px4.local-trajectory-setpoint",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.aerial.v1.LocalTrajectorySetpoint", 1u, 0u, false},
    {"setpoint.attitude", "px4.attitude-setpoint",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.aerial.v1.AttitudeSetpoint", 1u, 0u, false},
    {"diagnostic.fcu-link", "px4.fcu-link-status",
     contract::ChannelKind::kStreamOut, "xgc.semantic.aerial.v1.FcuLinkStatus",
     1u, 0u, false},
    {"diagnostic.offboard-input", "px4.offboard-input-status",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.aerial.v1.OffboardInputStatus", 0u, 2u, true},
    {"diagnostic.stream-health", "common.stream-health-report",
     contract::ChannelKind::kStreamOut,
     "xgc.semantic.common.v1.StreamHealthReport", 0u, 0u, true},
    {"operation.arm", "px4.arm", contract::ChannelKind::kOperation,
     "xgc.v1.Empty", 1u, 1u, false},
    {"operation.mode", "px4.mode", contract::ChannelKind::kOperation,
     "xgc.v1.Empty", 1u, 2u, false},
    {"operation.autopilot-reboot", "px4.autopilot-reboot",
     contract::ChannelKind::kOperation, "xgc.v1.Empty", 1u, 8u, false},
    {"operation.motion-intent", "px4.set-motion-intent",
     contract::ChannelKind::kOperation, "xgc.v1.Empty", 1u, 5u, false},
}};

bool resolveEndpointTemplate(const contract::EndpointMetadata &endpoint,
                             const xgc2_ros1_robot_adapter::RobotConfig &config,
                             std::string *resolved, std::string *error) {
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
  std::string ros_error;
  if (*resolved == "/" || !ros::names::validate(*resolved, ros_error)) {
    return fail(error, "generated native endpoint is invalid: " + ros_error);
  }
  return true;
}

bool resolveEndpoint(const xgc2_ros1_robot_adapter::RobotConfig &config,
                     const std::string &channel_id, contract::EndpointKind kind,
                     const std::string &role, const std::string &ros_type,
                     std::string *resolved, std::string *error) {
  contract::ChannelMetadata channel{};
  if (!contract::channelMetadata(config.profile_id, channel_id, &channel))
    return fail(error, "generated channel is missing: " + channel_id);
  const auto *endpoint = contract::channelEndpoint(channel, kind, role);
  if (endpoint == nullptr || endpoint->ros_type != ros_type)
    return fail(error, "native ROS endpoint binding drifted: " + channel_id);
  return resolveEndpointTemplate(*endpoint, config, resolved, error);
}

double channelStaleAfterSeconds(const std::string &profile_id,
                                const std::string &channel_id) {
  contract::ChannelMetadata channel{};
  if (!contract::channelMetadata(profile_id, channel_id, &channel) ||
      channel.stale_after_millis == 0u)
    throw std::logic_error("generated channel stale policy is invalid");
  return static_cast<double>(channel.stale_after_millis) / 1000.0;
}

} // namespace

std::uint32_t localSetpointValidFields(std::uint16_t type_mask) {
  return localValidFieldsImpl(type_mask);
}

std::uint32_t attitudeSetpointValidFields(std::uint8_t type_mask) {
  return attitudeValidFieldsImpl(type_mask);
}

std::string topicName(const std::string &robot_namespace,
                      const std::string &relative_name) {
  const std::string clean_namespace = cleanTopicPart(robot_namespace);
  const std::string clean_name = cleanTopicPart(relative_name);
  if (clean_namespace.empty()) {
    return "/" + clean_name;
  }
  return "/" + clean_namespace + "/" + clean_name;
}

bool resolveRemoteControlTopic(
    const xgc2_ros1_robot_adapter::RobotConfig &config, std::string *topic,
    std::string *error) {
  return resolveEndpoint(config, "operation.motion-intent",
                         contract::EndpointKind::kOutput, "output",
                         "mavros_msgs/PositionTarget", topic, error);
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

bool loadPositioningLivenessConfig(
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    xgc2_ros1_robot_adapter::PositioningHealthConfig *output,
    std::string *error) {
  return xgc2_ros1_robot_adapter::parsePositioningHealthConfig(
      config.parameters, output, error);
}

bool validVisionPose(const geometry_msgs::PoseStamped &message) {
  const auto &position = message.pose.position;
  const auto &orientation = message.pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w)) {
    return false;
  }
  const double norm = std::hypot(std::hypot(orientation.x, orientation.y),
                                 std::hypot(orientation.z, orientation.w));
  return std::isfinite(norm) && norm > 1e-6;
}

bool normalizeVisionPose(geometry_msgs::PoseStamped *message) {
  if (message == nullptr || !validVisionPose(*message))
    return false;
  auto &orientation = message->pose.orientation;
  const double norm = std::hypot(std::hypot(orientation.x, orientation.y),
                                 std::hypot(orientation.z, orientation.w));
  orientation.x /= norm;
  orientation.y /= norm;
  orientation.z /= norm;
  orientation.w /= norm;
  return std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w);
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

bool BuildNativeProfileConfig(
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    NativeProfileConfig *output, std::string *error) {
  if (output == nullptr)
    return fail(error, "PX4 native profile output is required");
  if (config.profile_id != contract::kProfileId)
    return fail(error, "PX4 native profile identity is unsupported");

  std::size_t parameter_count = 0u;
  const auto *parameters =
      contract::profileParameters(config.profile_id, &parameter_count);
  if (parameters == nullptr || parameter_count != 4u ||
      config.parameters.size() != parameter_count)
    return fail(error, "PX4 native parameter binding is not exhaustive");
  contract::ParameterMetadata namespace_descriptor{};
  contract::ParameterMetadata mocap_parameter{};
  contract::ParameterMetadata positioning_frames{};
  contract::ParameterMetadata positioning_threshold{};
  if (!contract::parameterMetadata(config.profile_id, "namespace",
                                   &namespace_descriptor) ||
      namespace_descriptor.type != contract::ParameterType::kString ||
      !namespace_descriptor.required ||
      !contract::parameterMetadata(config.profile_id, "mocap_rigid_body",
                                   &mocap_parameter) ||
      mocap_parameter.type != contract::ParameterType::kString ||
      !mocap_parameter.required ||
      std::string(mocap_parameter.pattern) != "^[A-Za-z][A-Za-z0-9_]*$" ||
      !contract::parameterMetadata(config.profile_id, "positioning_frame_number", &positioning_frames) ||
      positioning_frames.type != contract::ParameterType::kInteger || !positioning_frames.required ||
      !contract::parameterMetadata(config.profile_id, "positioning_comparison_threshold_m", &positioning_threshold) ||
      positioning_threshold.type != contract::ParameterType::kNumber || !positioning_threshold.required) {
    return fail(error, "PX4 native parameter descriptors drifted");
  }
  const auto namespace_value =
      config.parameters.find("namespace");
  const auto mocap_value = config.parameters.find("mocap_rigid_body");
  std::string parameter_error;
  if (namespace_value == config.parameters.end() ||
      !validRobotNamespace(namespace_value->second, &parameter_error)) {
    return fail(error,
                "PX4 namespace parameter is invalid: " + parameter_error);
  }
  if (mocap_value == config.parameters.end() ||
      !validMocapRigidBodyName(mocap_value->second, &parameter_error)) {
    return fail(error, "PX4 mocap parameter is invalid: " + parameter_error);
  }
  xgc2_ros1_robot_adapter::PositioningHealthConfig positioning_config;
  if (!loadPositioningLivenessConfig(config, &positioning_config,
                                     &parameter_error))
    return fail(error, "PX4 positioning parameters are invalid: " + parameter_error);

  std::size_t channel_count = 0u;
  const auto *channels =
      contract::profileChannels(config.profile_id, &channel_count);
  if (channels == nullptr || channel_count != kNativeBindings.size())
    return fail(error, "PX4 native channel binding is not exhaustive");
  for (const auto &binding : kNativeBindings) {
    contract::ChannelMetadata channel{};
    if (!contract::channelMetadata(config.profile_id, binding.id, &channel) ||
        channel.kind != binding.kind ||
        std::string(channel.processor) != binding.processor ||
        channel.output_message_id == 0u ||
        channel.endpoint_count != binding.endpoint_count ||
        channel.policy_count != binding.policy_count ||
        (channel.observes_count > 0u) != binding.observes) {
      return fail(error, std::string("PX4 native channel binding drifted: ") +
                             binding.id);
    }
    contract::MessageMetadata message{};
    if (!contract::messageMetadata(channel.output_message_id, &message) ||
        std::string(message.type_name) != binding.output_type) {
      return fail(error, std::string("PX4 output schema binding drifted: ") +
                             binding.id);
    }
    if (binding.kind == contract::ChannelKind::kOperation) {
      if (channel.input_message_id == 0u ||
          channel.operation_timeout_millis == 0u ||
          channel.operation_timeout_millis > 5000u ||
          channel.stale_after_millis != 0u ||
          std::string(channel.operation_id).empty() ||
          std::string(channel.operation_contract.idempotency) != "required" ||
          channel.operation_contract.cancellation_supported ||
          !channel.operation_contract.deadline_required) {
        return fail(error, std::string("PX4 operation binding drifted: ") +
                               binding.id);
      }
      const std::string expected_side_effect =
          std::string(binding.id) == "operation.autopilot-reboot"
              ? "non-idempotent"
              : "idempotent";
      if (channel.operation_contract.side_effect != expected_side_effect)
        return fail(error, "PX4 operation side-effect contract drifted");
    } else if (channel.input_message_id != 0u ||
               channel.output_rate_hz <= 0.0 ||
               channel.operation_timeout_millis != 0u ||
               channel.stale_after_millis == 0u ||
               !std::string(channel.operation_id).empty() ||
               !std::string(channel.operation_contract.side_effect).empty()) {
      return fail(error,
                  std::string("PX4 stream binding drifted: ") + binding.id);
    }
    for (std::size_t index = 0u; index < channel.observes_count; ++index) {
      contract::ChannelMetadata observed{};
      if (!contract::channelMetadata(config.profile_id, channel.observes[index],
                                     &observed) ||
          observed.kind != contract::ChannelKind::kStreamOut)
        return fail(error, "PX4 observes binding references an invalid stream");
    }
  }

  NativeProfileConfig candidate;
  std::string flight_state_endpoint;
  std::string flight_extended_state_endpoint;
  std::string mocap_speed_endpoint;
  const auto input = contract::EndpointKind::kInput;
  const auto service = contract::EndpointKind::kService;
  if (!resolveEndpoint(config, "state.pose", input, "pose",
                       "geometry_msgs/PoseStamped", &candidate.pose_endpoint,
                       error) ||
      !resolveEndpoint(config, "state.mocap.pose", input, "pose",
                       "geometry_msgs/PoseStamped", &candidate.mocap_endpoint,
                       error) ||
      !resolveEndpoint(config, "state.mocap.velocity", input, "velocity",
                       "geometry_msgs/TwistStamped",
                       &candidate.mocap_velocity_endpoint, error) ||
      !resolveEndpoint(config, "state.mocap.speed", input, "velocity",
                       "geometry_msgs/TwistStamped", &mocap_speed_endpoint,
                       error) ||
      !resolveEndpoint(config, "state.velocity", input, "velocity",
                       "geometry_msgs/TwistStamped",
                       &candidate.velocity_endpoint, error) ||
      !resolveEndpoint(config, "state.imu", input, "imu", "sensor_msgs/Imu",
                       &candidate.imu_endpoint, error) ||
      !resolveEndpoint(config, "state.power", input, "battery",
                       "sensor_msgs/BatteryState", &candidate.power_endpoint,
                       error) ||
      !resolveEndpoint(config, "state.health", input, "state",
                       "mavros_msgs/State", &candidate.state_endpoint, error) ||
      !resolveEndpoint(config, "state.health", input, "extended_state",
                       "mavros_msgs/ExtendedState",
                       &candidate.extended_state_endpoint, error) ||
      !resolveEndpoint(config, "state.flight", input, "state",
                       "mavros_msgs/State", &flight_state_endpoint, error) ||
      !resolveEndpoint(config, "state.flight", input, "extended_state",
                       "mavros_msgs/ExtendedState",
                       &flight_extended_state_endpoint, error) ||
      !resolveEndpoint(config, "setpoint.local", input, "setpoint",
                       "mavros_msgs/PositionTarget",
                       &candidate.local_setpoint_endpoint, error) ||
      !resolveEndpoint(config, "setpoint.attitude", input, "setpoint",
                       "mavros_msgs/AttitudeTarget",
                       &candidate.attitude_setpoint_endpoint, error) ||
      !resolveEndpoint(config, "diagnostic.fcu-link", input, "timesync",
                       "mavros_msgs/TimesyncStatus",
                       &candidate.timesync_endpoint, error) ||
      !resolveEndpoint(config, "operation.arm", service, "service",
                       "mavros_msgs/CommandBool",
                       &candidate.arm_service_endpoint, error) ||
      !resolveEndpoint(config, "operation.mode", service, "service",
                       "mavros_msgs/SetMode", &candidate.mode_service_endpoint,
                       error) ||
      !resolveEndpoint(config, "operation.autopilot-reboot", service, "service",
                       "mavros_msgs/CommandLong",
                       &candidate.reboot_service_endpoint, error) ||
      !resolveRemoteControlTopic(config, &candidate.remote_control_endpoint,
                                 error)) {
    return false;
  }
  if (flight_state_endpoint != candidate.state_endpoint ||
      flight_extended_state_endpoint != candidate.extended_state_endpoint) {
    return fail(error,
                "PX4 health and flight channels must share state inputs");
  }
  if (candidate.mocap_velocity_endpoint != mocap_speed_endpoint) {
    return fail(error,
                "PX4 mocap velocity and speed must share the canonical twist input");
  }

  contract::ChannelMetadata offboard{};
  contract::ChannelMetadata arm{};
  contract::ChannelMetadata mode{};
  contract::ChannelMetadata reboot{};
  contract::ChannelMetadata remote{};
  contract::channelMetadata(config.profile_id, "diagnostic.offboard-input",
                            &offboard);
  contract::channelMetadata(config.profile_id, "operation.arm", &arm);
  contract::channelMetadata(config.profile_id, "operation.mode", &mode);
  contract::channelMetadata(config.profile_id, "operation.autopilot-reboot",
                            &reboot);
  contract::channelMetadata(config.profile_id, "operation.motion-intent",
                            &remote);
  double offboard_rate = 0.0;
  std::int64_t offboard_timeout = 0;
  const char *const *allowed_modes = nullptr;
  std::size_t allowed_mode_count = 0u;
  std::int64_t arm_timeout = 0;
  std::int64_t mode_timeout = 0;
  std::int64_t reboot_timeout = 0;
  std::int64_t state_timeout = 0;
  std::int64_t mav_command = 0;
  std::int64_t reboot_param = 0;
  bool require_known = false;
  bool require_fresh = false;
  bool require_connected = false;
  bool require_disarmed = false;
  double remote_altitude = 0.0;
  double remote_linear = 0.0;
  double remote_yaw = 0.0;
  std::int64_t remote_publish_rate = 0;
  std::int64_t remote_timeout = 0;
  if (!contract::channelPolicyNumber(offboard, "minimum_rate_hz",
                                     &offboard_rate) ||
      !contract::channelPolicyInteger(offboard, "source_timeout_ms",
                                      &offboard_timeout) ||
      offboard_rate <= 0.0 || offboard_timeout <= 0 ||
      !contract::channelPolicyInteger(arm, "timeout_ms", &arm_timeout) ||
      !contract::channelPolicyInteger(mode, "timeout_ms", &mode_timeout) ||
      !contract::channelPolicyStringArray(mode, "allowed_modes", &allowed_modes,
                                          &allowed_mode_count) ||
      allowed_mode_count == 0u ||
      !contract::channelPolicyInteger(reboot, "timeout_ms", &reboot_timeout) ||
      !contract::channelPolicyInteger(reboot, "state_timeout_ms",
                                      &state_timeout) ||
      !contract::channelPolicyInteger(reboot, "mav_command", &mav_command) ||
      !contract::channelPolicyInteger(reboot, "normal_reboot_param1",
                                      &reboot_param) ||
      !contract::channelPolicyBoolean(reboot, "require_state_known",
                                      &require_known) ||
      !contract::channelPolicyBoolean(reboot, "require_state_fresh",
                                      &require_fresh) ||
      !contract::channelPolicyBoolean(reboot, "require_connected",
                                      &require_connected) ||
      !contract::channelPolicyBoolean(reboot, "require_disarmed",
                                      &require_disarmed) ||
      !contract::channelPolicyNumber(remote, "altitude_meters",
                                     &remote_altitude) ||
      !contract::channelPolicyNumber(remote, "maximum_linear_velocity_mps",
                                     &remote_linear) ||
      !contract::channelPolicyNumber(remote, "maximum_yaw_rate_rps",
                                     &remote_yaw) ||
      !contract::channelPolicyInteger(remote, "publish_rate_hz",
                                      &remote_publish_rate) ||
      !contract::channelPolicyInteger(remote, "timeout_ms", &remote_timeout) ||
      mav_command != 246 || reboot_param != 1 || !require_known ||
      !require_fresh || !require_connected || !require_disarmed ||
      remote_altitude != 1.0 || remote_linear <= 0.0 || remote_yaw <= 0.0 ||
      remote_publish_rate != 10 || remote_timeout != 1000) {
    return fail(error, "PX4 native policy binding is incomplete or unsafe");
  }
  candidate.offboard_source_timeout_seconds =
      static_cast<double>(offboard_timeout) / 1000.0;
  candidate.offboard_minimum_rate_hz = offboard_rate;
  candidate.reboot_state_timeout_seconds =
      static_cast<double>(state_timeout) / 1000.0;
  const std::int64_t maximum_timeout =
      std::max(std::max(arm_timeout, mode_timeout),
               std::max(reboot_timeout, remote_timeout));
  if (maximum_timeout <= 0 || maximum_timeout > 5000)
    return fail(error, "PX4 operation timeout exceeds the native limit");
  candidate.maximum_operation_timeout_seconds =
      static_cast<double>(maximum_timeout) / 1000.0;
  candidate.remote_control_altitude_meters = remote_altitude;
  candidate.remote_control_maximum_linear_velocity_mps = remote_linear;
  candidate.remote_control_maximum_yaw_rate_rps = remote_yaw;
  candidate.allowed_modes.reserve(allowed_mode_count);
  std::set<std::string> unique_modes;
  for (std::size_t index = 0u; index < allowed_mode_count; ++index) {
    if (allowed_modes[index] == nullptr || allowed_modes[index][0] == '\0' ||
        !unique_modes.insert(allowed_modes[index]).second) {
      return fail(error, "PX4 allowed-mode policy is empty or duplicated");
    }
    candidate.allowed_modes.emplace_back(allowed_modes[index]);
  }

  *output = std::move(candidate);
  if (error != nullptr)
    error->clear();
  return true;
}

double positionDistanceMeters(const geometry_msgs::Point &left,
                              const geometry_msgs::Point &right) {
  return std::hypot(std::hypot(left.x - right.x, left.y - right.y),
                    left.z - right.z);
}

std::shared_ptr<RobotRuntime>
RobotRuntime::Create(ros::NodeHandle node_handle,
                     const xgc2_ros1_robot_adapter::RobotConfig &config,
                     std::uint64_t spec_revision, EnvelopeEmitter emitter,
                     std::string *error) {
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
  if (mocap_it == config.parameters.end()) {
    if (error != nullptr)
      *error =
          "robot configuration is missing required mocap_rigid_body parameter";
    return nullptr;
  }
  std::string mocap_error;
  if (!validMocapRigidBodyName(mocap_it->second, &mocap_error)) {
    if (error != nullptr)
      *error = "invalid mocap rigid-body name: " + mocap_error;
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

  NativeProfileConfig native_profile;
  if (!BuildNativeProfileConfig(config, &native_profile, error))
    return nullptr;
  xgc2_ros1_robot_adapter::PositioningHealthConfig positioning_health_config;
  if (!loadPositioningLivenessConfig(config, &positioning_health_config,
                                     error))
    return nullptr;

  auto runtime = std::shared_ptr<RobotRuntime>(new RobotRuntime(
      std::move(node_handle), config.robot_id, config.profile_id,
      namespace_it->second, spec_revision, std::move(enabled_channels),
      std::move(required_channels), std::move(native_profile),
      positioning_health_config, std::move(emitter)));
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
                           NativeProfileConfig native_profile,
                           xgc2_ros1_robot_adapter::PositioningHealthConfig
                               positioning_health_config,
                           EnvelopeEmitter emitter)
    : node_handle_(std::move(node_handle)), robot_id_(std::move(robot_id)),
      profile_id_(std::move(profile_id)),
      robot_namespace_(cleanTopicPart(robot_namespace)),
      spec_revision_(spec_revision),
      enabled_channels_(std::move(enabled_channels)),
      required_channels_(std::move(required_channels)),
      emitter_(std::move(emitter)),
      offboard_source_timeout_seconds_(
          native_profile.offboard_source_timeout_seconds),
      offboard_minimum_rate_hz_(native_profile.offboard_minimum_rate_hz),
      positioning_health_(positioning_health_config),
      pose_endpoint_(std::move(native_profile.pose_endpoint)),
      velocity_endpoint_(std::move(native_profile.velocity_endpoint)),
      imu_endpoint_(std::move(native_profile.imu_endpoint)),
      power_endpoint_(std::move(native_profile.power_endpoint)),
      state_endpoint_(std::move(native_profile.state_endpoint)),
      extended_state_endpoint_(
          std::move(native_profile.extended_state_endpoint)),
      mocap_endpoint_(std::move(native_profile.mocap_endpoint)),
      mocap_velocity_endpoint_(
          std::move(native_profile.mocap_velocity_endpoint)),
      local_setpoint_endpoint_(
          std::move(native_profile.local_setpoint_endpoint)),
      attitude_setpoint_endpoint_(
          std::move(native_profile.attitude_setpoint_endpoint)),
      timesync_endpoint_(std::move(native_profile.timesync_endpoint)) {}

RobotRuntime::~RobotRuntime() { Stop(); }

void RobotRuntime::Activate() noexcept {}

void RobotRuntime::Deactivate() noexcept {}

RobotRuntime::CallbackGuard::CallbackGuard(RobotRuntime *runtime)
    : runtime_(runtime),
      active_(runtime_ != nullptr && runtime_->beginCallback()) {}

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
  Deactivate();
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopping_) {
      callbacks_idle_.wait(lock, [this] { return stop_complete_; });
      return;
    }
    stopping_ = true;
  }

  pose_subscriber_.shutdown();
  mocap_subscriber_.shutdown();
  mocap_velocity_subscriber_.shutdown();
  velocity_subscriber_.shutdown();
  imu_subscriber_.shutdown();
  power_subscriber_.shutdown();
  state_subscriber_.shutdown();
  extended_state_subscriber_.shutdown();
  local_setpoint_subscriber_.shutdown();
  attitude_setpoint_subscriber_.shutdown();
  timesync_subscriber_.shutdown();

  {
    std::unique_lock<std::mutex> lock(mutex_);
    callbacks_idle_.wait(lock, [this] { return active_callbacks_ == 0; });
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_complete_ = true;
  }
  callbacks_idle_.notify_all();
}

bool RobotRuntime::channelEnabled(const std::string &channel_id) const {
  return enabled_channels_.count(channel_id) != 0;
}

bool RobotRuntime::channelRequired(const std::string &channel_id) const {
  return required_channels_.count(channel_id) != 0;
}

bool RobotRuntime::install(std::string *error) {
  if (profile_id_ != contract::kProfileId) {
    if (error != nullptr) {
      *error = "unsupported profile: " + profile_id_;
    }
    return false;
  }
  {
    // Build all tracking state before any subscriber callback can observe it.
    // AsyncSpinner callbacks may start as soon as the first subscription is
    // registered, so installation holds the same mutex used by callbacks.
    std::lock_guard<std::mutex> lock(mutex_);
    if (!installPx4(error))
      return false;
  }

  ROS_INFO_STREAM("XGC Robot Adapter robot="
                  << robot_id_ << " profile=" << profile_id_ << " namespace=/"
                  << robot_namespace_ << " revision=" << spec_revision_);
  return true;
}

bool RobotRuntime::installPx4(std::string *error) {
  const std::weak_ptr<RobotRuntime> weak_self = shared_from_this();
  if (channelRequired("state.localization.error")) {
    ensureSourceLocked(
        "state.localization.error",
        channelStaleAfterSeconds(profile_id_, "state.localization.error"));
  }
  if (channelRequired("state.pose")) {
    ensureSourceLocked("state.pose",
                       channelStaleAfterSeconds(profile_id_, "state.pose"));
    pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>(
        pose_endpoint_, 20,
        [weak_self](const geometry_msgs::PoseStamped::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->px4PoseCallback(message);
        });
    if (!requireRosRegistration(pose_subscriber_, pose_endpoint_, error))
      return false;
  }
  if (channelRequired("state.mocap.pose")) {
    ensureSourceLocked(
        "state.mocap.pose",
        channelStaleAfterSeconds(profile_id_, "state.mocap.pose"));
    mocap_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>(
        mocap_endpoint_, 50,
        [weak_self](const geometry_msgs::PoseStamped::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->mocapPoseCallback(message);
        });
    if (!requireRosRegistration(mocap_subscriber_, mocap_endpoint_, error))
      return false;
  }
  if (channelRequired("state.mocap.velocity") ||
      channelRequired("state.mocap.speed")) {
    if (channelRequired("state.mocap.velocity")) {
      ensureSourceLocked(
          "state.mocap.velocity",
          channelStaleAfterSeconds(profile_id_, "state.mocap.velocity"));
    }
    if (channelRequired("state.mocap.speed")) {
      ensureSourceLocked(
          "state.mocap.speed",
          channelStaleAfterSeconds(profile_id_, "state.mocap.speed"));
    }
    mocap_velocity_subscriber_ =
        node_handle_.subscribe<geometry_msgs::TwistStamped>(
            mocap_velocity_endpoint_, 20,
            [weak_self](const geometry_msgs::TwistStamped::ConstPtr &message) {
              if (const auto self = weak_self.lock())
                self->mocapVelocityCallback(message);
            });
    if (!requireRosRegistration(mocap_velocity_subscriber_,
                                mocap_velocity_endpoint_, error))
      return false;
  }
  if (channelRequired("state.velocity")) {
    ensureSourceLocked("state.velocity",
                       channelStaleAfterSeconds(profile_id_, "state.velocity"));
    velocity_subscriber_ = node_handle_.subscribe<geometry_msgs::TwistStamped>(
        velocity_endpoint_, 20,
        [weak_self](const geometry_msgs::TwistStamped::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->px4VelocityCallback(message);
        });
    if (!requireRosRegistration(velocity_subscriber_, velocity_endpoint_,
                                error))
      return false;
  }
  if (channelRequired("state.imu")) {
    ensureSourceLocked("state.imu",
                       channelStaleAfterSeconds(profile_id_, "state.imu"));
    imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
        imu_endpoint_, 20,
        [weak_self](const sensor_msgs::Imu::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->imuCallback(message);
        });
    if (!requireRosRegistration(imu_subscriber_, imu_endpoint_, error))
      return false;
  }
  if (channelRequired("state.power")) {
    ensureSourceLocked("state.power",
                       channelStaleAfterSeconds(profile_id_, "state.power"));
    power_subscriber_ = node_handle_.subscribe<sensor_msgs::BatteryState>(
        power_endpoint_, 10,
        [weak_self](const sensor_msgs::BatteryState::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->batteryCallback(message);
        });
    if (!requireRosRegistration(power_subscriber_, power_endpoint_, error))
      return false;
  }
  if (channelRequired("state.health")) {
    ensureSourceLocked("state.health",
                       channelStaleAfterSeconds(profile_id_, "state.health"));
  }
  if (channelRequired("state.flight")) {
    ensureSourceLocked("state.flight",
                       channelStaleAfterSeconds(profile_id_, "state.flight"));
  }
  if (channelRequired("state.health") || channelRequired("state.flight")) {
    state_subscriber_ = node_handle_.subscribe<mavros_msgs::State>(
        state_endpoint_, 10,
        [weak_self](const mavros_msgs::State::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->mavrosStateCallback(message);
        });
    if (!requireRosRegistration(state_subscriber_, state_endpoint_, error))
      return false;
    extended_state_subscriber_ =
        node_handle_.subscribe<mavros_msgs::ExtendedState>(
            extended_state_endpoint_, 10,
            [weak_self](const mavros_msgs::ExtendedState::ConstPtr &message) {
              if (const auto self = weak_self.lock())
                self->mavrosExtendedStateCallback(message);
            });
    if (!requireRosRegistration(extended_state_subscriber_,
                                extended_state_endpoint_, error))
      return false;
  }
  if (channelRequired("setpoint.local")) {
    ensureSourceLocked("setpoint.local",
                       channelStaleAfterSeconds(profile_id_, "setpoint.local"));
    local_setpoint_subscriber_ =
        node_handle_.subscribe<mavros_msgs::PositionTarget>(
            local_setpoint_endpoint_, 50,
            [weak_self](const mavros_msgs::PositionTarget::ConstPtr &message) {
              if (const auto self = weak_self.lock())
                self->localSetpointCallback(message);
            });
    if (!requireRosRegistration(local_setpoint_subscriber_,
                                local_setpoint_endpoint_, error))
      return false;
  }
  if (channelRequired("setpoint.attitude")) {
    ensureSourceLocked(
        "setpoint.attitude",
        channelStaleAfterSeconds(profile_id_, "setpoint.attitude"));
    attitude_setpoint_subscriber_ =
        node_handle_.subscribe<mavros_msgs::AttitudeTarget>(
            attitude_setpoint_endpoint_, 50,
            [weak_self](const mavros_msgs::AttitudeTarget::ConstPtr &message) {
              if (const auto self = weak_self.lock())
                self->attitudeSetpointCallback(message);
            });
    if (!requireRosRegistration(attitude_setpoint_subscriber_,
                                attitude_setpoint_endpoint_, error))
      return false;
  }
  if (channelRequired("diagnostic.fcu-link")) {
    ensureSourceLocked(
        "diagnostic.fcu-link",
        channelStaleAfterSeconds(profile_id_, "diagnostic.fcu-link"));
    timesync_subscriber_ = node_handle_.subscribe<mavros_msgs::TimesyncStatus>(
        timesync_endpoint_, 20,
        [weak_self](const mavros_msgs::TimesyncStatus::ConstPtr &message) {
          if (const auto self = weak_self.lock())
            self->timesyncStatusCallback(message);
        });
    if (!requireRosRegistration(timesync_subscriber_, timesync_endpoint_,
                                error))
      return false;
  }
  if (channelRequired("diagnostic.offboard-input")) {
    ensureSourceLocked(
        "diagnostic.offboard-input",
        channelStaleAfterSeconds(profile_id_, "diagnostic.offboard-input"));
  }
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

xgc::robot::v1::RobotMessage
RobotRuntime::makeEnvelopeLocked(const std::string &channel_id,
                                 const ros::Time &source_stamp,
                                 const google::protobuf::Message &payload) {
  contract::ChannelMetadata channel;
  if (!contract::channelMetadata(profile_id_, channel_id, &channel) ||
      channel.output_message_id == 0u) {
    throw std::logic_error(
        "channel output is absent from generated XGC2 contract metadata");
  }
  contract::MessageMetadata metadata;
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
  if (!xgc2_ros1_robot_adapter::BuildRobotMessage(context, schema, payload,
                                                  &envelope, &error)) {
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

void RobotRuntime::ensureSourceLocked(const std::string &channel_id,
                                      double stale_after_seconds) {
  if (stale_after_seconds <= 0.0)
    throw std::logic_error("semantic source freshness must be positive");
  auto &source = sources_[channel_id];
  source.stale_after_seconds = stale_after_seconds;
}

void RobotRuntime::recordSourceLocked(const std::string &channel_id,
                                      const ros::WallTime &now) {
  auto source_it = sources_.find(channel_id);
  if (source_it == sources_.end())
    throw std::logic_error("semantic source was not installed: " + channel_id);
  auto &source = source_it->second;
  if (source.window_started.isZero())
    source.window_started = now;
  if (!source.last_seen.isZero() && now > source.last_seen) {
    const double instantaneous_rate = 1.0 / (now - source.last_seen).toSec();
    source.source_rate_hz =
        source.source_rate_hz <= 0.0
            ? instantaneous_rate
            : 0.8 * source.source_rate_hz + 0.2 * instantaneous_rate;
  }
  source.last_seen = now;
  ++source.source_samples;
}

void RobotRuntime::recordStateSourceLocked(const std::string &channel_id,
                                           bool count_sample) {
  auto source_it = sources_.find(channel_id);
  if (source_it == sources_.end())
    throw std::logic_error("PX4 state source was not installed: " + channel_id);
  auto &source = source_it->second;
  if (source.window_started.isZero()) {
    source.window_started = !mavros_state_last_seen_.isZero()
                                ? mavros_state_last_seen_
                                : mavros_extended_state_last_seen_;
  }
  if (count_sample)
    ++source.source_samples;
  if (mavros_state_last_seen_.isZero() ||
      mavros_extended_state_last_seen_.isZero()) {
    return;
  }
  const ros::WallTime complete_input_time =
      std::min(mavros_state_last_seen_, mavros_extended_state_last_seen_);
  if (!source.last_seen.isZero() && complete_input_time > source.last_seen) {
    const double instantaneous_rate =
        1.0 / (complete_input_time - source.last_seen).toSec();
    source.source_rate_hz =
        source.source_rate_hz <= 0.0
            ? instantaneous_rate
            : 0.8 * source.source_rate_hz + 0.2 * instantaneous_rate;
  }
  source.last_seen = complete_input_time;
}

void RobotRuntime::recordOutputLocked(const std::string &channel_id) {
  auto source_it = sources_.find(channel_id);
  if (source_it == sources_.end())
    throw std::logic_error("semantic source was not installed: " + channel_id);
  ++source_it->second.output_samples;
}

void RobotRuntime::emitPositionErrorLocked(
    const ros::Time &source_stamp, const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!has_local_position_ || !has_mocap_position_ || messages == nullptr)
    return;
  if (channelRequired("state.localization.error"))
    recordSourceLocked("state.localization.error", now);
  const double distance =
      positionDistanceMeters(local_position_, mocap_position_);
  if (channelEnabled("state.localization.error") && std::isfinite(distance) &&
      shouldEmitLocked("state.localization.error", now)) {
    xgc::semantic::common::v1::DistanceEstimate payload;
    payload.set_frame_id(local_position_frame_id_);
    payload.set_meters(distance);
    messages->push_back(makeEnvelopeLocked("state.localization.error",
                                           source_stamp, payload));
    recordOutputLocked("state.localization.error");
  } else if (channelEnabled("state.localization.error")) {
    ++sources_["state.localization.error"].dropped_samples;
  }
}

void RobotRuntime::setPositioningHealthLocked(
    const ros::WallTime &now,
    xgc::semantic::common::v1::VehicleHealth *payload) const {
  if (payload == nullptr)
    return;
  const auto result = positioning_health_.evaluate(now.toSec());
  auto *positioning = payload->mutable_positioning();
  using Health = xgc::semantic::common::v1::PositioningHealth;
  using Reason = xgc2_ros1_robot_adapter::PositioningHealthReason;
  using State = xgc2_ros1_robot_adapter::PositioningHealthState;
  switch (result.state) {
  case State::kActive:
    positioning->set_state(Health::POSITIONING_STATE_ACTIVE);
    break;
  case State::kFrozen:
    positioning->set_state(Health::POSITIONING_STATE_FROZEN);
    addFault(payload, "mocap.position.frozen",
             "VRPN is publishing a repeated frozen pose", 2);
    break;
  case State::kTimedOut:
    positioning->set_state(Health::POSITIONING_STATE_TIMED_OUT);
    addFault(payload, "mocap.position.timeout",
             "VRPN pose exceeded the XGC1 100 ms freshness boundary", 2);
    break;
  default:
    positioning->set_state(Health::POSITIONING_STATE_UNSPECIFIED);
    break;
  }
  switch (result.reason) {
  case Reason::kWindowVariationObserved:
    positioning->set_reason(
        Health::POSITIONING_REASON_WINDOW_VARIATION_OBSERVED);
    break;
  case Reason::kRepeatFrameWindowFrozen:
    positioning->set_reason(
        Health::POSITIONING_REASON_REPEAT_FRAME_WINDOW_FROZEN);
    break;
  case Reason::kVrpnTimeout:
    positioning->set_reason(Health::POSITIONING_REASON_VRPN_TIMEOUT);
    break;
  default:
    positioning->set_reason(Health::POSITIONING_REASON_UNSPECIFIED);
    break;
  }
  positioning->set_observed_age_ms(result.observed_age_ms);
  positioning->set_window_spread_m(result.comparison_metric_m);
  positioning->set_sample_count(
      static_cast<std::uint32_t>(result.sample_count));
}

std::uint64_t
RobotRuntime::sourceAgeMillisLocked(const std::string &channel_id,
                                    const ros::WallTime &now) const {
  const auto it = sources_.find(channel_id);
  if (it == sources_.end() || it->second.last_seen.isZero() ||
      now < it->second.last_seen) {
    return 0;
  }
  const double milliseconds = (now - it->second.last_seen).toSec() * 1000.0;
  return static_cast<std::uint64_t>(std::max(0.0, milliseconds));
}

void RobotRuntime::px4PoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("state.pose", now);
    if (std::isfinite(message->pose.position.x) &&
        std::isfinite(message->pose.position.y) &&
        std::isfinite(message->pose.position.z)) {
      local_position_ = message->pose.position;
      local_position_frame_id_ = message->header.frame_id;
      has_local_position_ = true;
    }
    if (channelEnabled("state.pose") && shouldEmitLocked("state.pose", now)) {
      xgc::semantic::common::v1::PoseEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyVector(message->pose.position, payload.mutable_position());
      copyQuaternion(message->pose.orientation, payload.mutable_orientation());
      output.push_back(
          makeEnvelopeLocked("state.pose", message->header.stamp, payload));
      recordOutputLocked("state.pose");
    } else if (channelEnabled("state.pose")) {
      ++sources_["state.pose"].dropped_samples;
    }
    emitPositionErrorLocked(message->header.stamp, now, &output);
  }
  emit(std::move(output));
}

void RobotRuntime::mocapPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("state.mocap.pose", now);
    geometry_msgs::PoseStamped normalized_message = *message;
    if (!normalizeVisionPose(&normalized_message)) {
      ++sources_["state.mocap.pose"].dropped_samples;
      return;
    }
    mocap_position_ = normalized_message.pose.position;
    has_mocap_position_ = true;
    positioning_health_.recordPose(
        now.toSec(), normalized_message.pose.position.x,
        normalized_message.pose.position.y, normalized_message.pose.position.z);

    if (channelEnabled("state.mocap.pose") &&
        shouldEmitLocked("state.mocap.pose", now)) {
      xgc::semantic::common::v1::PoseEstimate payload;
      payload.set_frame_id(normalized_message.header.frame_id);
      copyVector(normalized_message.pose.position, payload.mutable_position());
      copyQuaternion(normalized_message.pose.orientation,
                     payload.mutable_orientation());
      output.push_back(makeEnvelopeLocked(
          "state.mocap.pose", normalized_message.header.stamp, payload));
      recordOutputLocked("state.mocap.pose");
    } else if (channelEnabled("state.mocap.pose")) {
      ++sources_["state.mocap.pose"].dropped_samples;
    }
    emitPositionErrorLocked(normalized_message.header.stamp, now, &output);
  }
  emit(std::move(output));
}

void RobotRuntime::mocapVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channelRequired("state.mocap.velocity"))
      recordSourceLocked("state.mocap.velocity", now);
    if (channelRequired("state.mocap.speed"))
      recordSourceLocked("state.mocap.speed", now);

    if (channelEnabled("state.mocap.velocity") &&
        shouldEmitLocked("state.mocap.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyVector(message->twist.linear, payload.mutable_linear());
      copyVector(message->twist.angular, payload.mutable_angular());
      output.push_back(makeEnvelopeLocked(
          "state.mocap.velocity", message->header.stamp, payload));
      recordOutputLocked("state.mocap.velocity");
    } else if (channelEnabled("state.mocap.velocity")) {
      ++sources_["state.mocap.velocity"].dropped_samples;
    }

    const double speed = std::hypot(
        std::hypot(message->twist.linear.x, message->twist.linear.y),
        message->twist.linear.z);
    if (channelEnabled("state.mocap.speed") && std::isfinite(speed) &&
        shouldEmitLocked("state.mocap.speed", now)) {
      xgc::semantic::common::v1::SpeedEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      payload.set_meters_per_second(speed);
      output.push_back(makeEnvelopeLocked(
          "state.mocap.speed", message->header.stamp, payload));
      recordOutputLocked("state.mocap.speed");
    } else if (channelEnabled("state.mocap.speed")) {
      ++sources_["state.mocap.speed"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::px4VelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("state.velocity", now);
    if (channelEnabled("state.velocity") &&
        shouldEmitLocked("state.velocity", now)) {
      xgc::semantic::common::v1::VelocityEstimate payload;
      payload.set_frame_id(message->header.frame_id);
      copyVector(message->twist.linear, payload.mutable_linear());
      copyVector(message->twist.angular, payload.mutable_angular());
      output.push_back(
          makeEnvelopeLocked("state.velocity", message->header.stamp, payload));
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
      output.push_back(
          makeEnvelopeLocked("state.imu", message->header.stamp, payload));
      recordOutputLocked("state.imu");
    } else if (channelEnabled("state.imu")) {
      ++sources_["state.imu"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::batteryCallback(
    const sensor_msgs::BatteryState::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("state.power", now);
    if (channelEnabled("state.power") && shouldEmitLocked("state.power", now)) {
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
      output.push_back(
          makeEnvelopeLocked("state.power", message->header.stamp, payload));
      recordOutputLocked("state.power");
    } else if (channelEnabled("state.power")) {
      ++sources_["state.power"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::mavrosStateCallback(
    const mavros_msgs::State::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  mavros_state_ = *message;
  has_mavros_state_ = true;
  mavros_state_last_seen_ = ros::WallTime::now();
  if (channelRequired("state.health"))
    recordStateSourceLocked("state.health", true);
  if (channelRequired("state.flight"))
    recordStateSourceLocked("state.flight", true);
}

void RobotRuntime::mavrosExtendedStateCallback(
    const mavros_msgs::ExtendedState::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  mavros_extended_state_ = *message;
  has_mavros_extended_state_ = true;
  mavros_extended_state_last_seen_ = ros::WallTime::now();
  if (channelRequired("state.health"))
    recordStateSourceLocked("state.health", false);
  if (channelRequired("state.flight"))
    recordStateSourceLocked("state.flight", false);
}

void RobotRuntime::localSetpointCallback(
    const mavros_msgs::PositionTarget::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  const std::uint32_t fields = localSetpointValidFields(message->type_mask);
  const auto frame = localCoordinateFrame(message->coordinate_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_setpoint_ = *message;
    has_local_setpoint_ = true;
    valid_local_setpoint_ =
        frame !=
            xgc::semantic::aerial::v1::LOCAL_COORDINATE_FRAME_UNSPECIFIED &&
        fields != 0 && finiteLocalSetpoint(*message, fields);
    recordSourceLocked("setpoint.local", now);
    if (channelEnabled("setpoint.local") &&
        shouldEmitLocked("setpoint.local", now)) {
      xgc::semantic::aerial::v1::LocalTrajectorySetpoint payload;
      payload.set_frame_id(message->header.frame_id);
      payload.set_coordinate_frame(frame);
      payload.set_valid_fields(fields);
      copySelectedVector(message->position, payload.mutable_position(), fields,
                         0);
      copySelectedVector(message->velocity, payload.mutable_velocity(), fields,
                         3);
      copySelectedVector(message->acceleration_or_force,
                         payload.mutable_acceleration_or_force(), fields, 6);
      if ((fields & (1u << 9)) != 0 && std::isfinite(message->yaw))
        payload.set_yaw_rad(message->yaw);
      if ((fields & (1u << 10)) != 0 && std::isfinite(message->yaw_rate))
        payload.set_yaw_rate_rad_s(message->yaw_rate);
      payload.set_acceleration_is_force(
          (message->type_mask & mavros_msgs::PositionTarget::FORCE) != 0);
      output.push_back(
          makeEnvelopeLocked("setpoint.local", message->header.stamp, payload));
      recordOutputLocked("setpoint.local");
    } else if (channelEnabled("setpoint.local")) {
      ++sources_["setpoint.local"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::attitudeSetpointCallback(
    const mavros_msgs::AttitudeTarget::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  const std::uint32_t fields = attitudeSetpointValidFields(message->type_mask);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    attitude_setpoint_ = *message;
    has_attitude_setpoint_ = true;
    valid_attitude_setpoint_ =
        fields != 0 && finiteAttitudeSetpoint(*message, fields);
    recordSourceLocked("setpoint.attitude", now);
    if (channelEnabled("setpoint.attitude") &&
        shouldEmitLocked("setpoint.attitude", now)) {
      xgc::semantic::aerial::v1::AttitudeSetpoint payload;
      payload.set_frame_id(message->header.frame_id);
      payload.set_valid_fields(fields);
      if ((fields & (1u << 0)) != 0)
        copyQuaternion(message->orientation, payload.mutable_orientation());
      copySelectedVector(message->body_rate, payload.mutable_body_rate_rad_s(),
                         fields, 1);
      if ((fields & (1u << 4)) != 0 && std::isfinite(message->thrust))
        payload.set_thrust(message->thrust);
      output.push_back(makeEnvelopeLocked("setpoint.attitude",
                                          message->header.stamp, payload));
      recordOutputLocked("setpoint.attitude");
    } else if (channelEnabled("setpoint.attitude")) {
      ++sources_["setpoint.attitude"].dropped_samples;
    }
  }
  emit(std::move(output));
}

void RobotRuntime::timesyncStatusCallback(
    const mavros_msgs::TimesyncStatus::ConstPtr &message) {
  CallbackGuard callback(this);
  if (!callback)
    return;
  std::vector<xgc::robot::v1::RobotMessage> output;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recordSourceLocked("diagnostic.fcu-link", now);
    if (channelEnabled("diagnostic.fcu-link") &&
        shouldEmitLocked("diagnostic.fcu-link", now)) {
      xgc::semantic::aerial::v1::FcuLinkStatus payload;
      payload.set_remote_timestamp_ns(
          static_cast<std::int64_t>(std::min<std::uint64_t>(
              message->remote_timestamp_ns,
              static_cast<std::uint64_t>(
                  std::numeric_limits<std::int64_t>::max()))));
      payload.set_observed_offset_ns(message->observed_offset_ns);
      payload.set_estimated_offset_ns(message->estimated_offset_ns);
      if (std::isfinite(message->round_trip_time_ms))
        payload.set_round_trip_time_ms(message->round_trip_time_ms);
      output.push_back(makeEnvelopeLocked("diagnostic.fcu-link",
                                          message->header.stamp, payload));
      recordOutputLocked("diagnostic.fcu-link");
    } else if (channelEnabled("diagnostic.fcu-link")) {
      ++sources_["diagnostic.fcu-link"].dropped_samples;
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
    emitPx4PeriodicLocked(now, &messages);
    emitStreamHealthLocked(now, &messages);
  }
  emit(std::move(messages));
}

void RobotRuntime::emitPx4PeriodicLocked(
    const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  const double health_stale_after =
      channelStaleAfterSeconds(profile_id_, "state.health");
  const double flight_stale_after =
      channelStaleAfterSeconds(profile_id_, "state.flight");
  const bool health_state_fresh =
      has_mavros_state_ &&
      sourceIsFresh(mavros_state_last_seen_, now, health_stale_after);
  const bool health_extended_fresh =
      has_mavros_extended_state_ &&
      sourceIsFresh(mavros_extended_state_last_seen_, now, health_stale_after);
  const bool flight_state_fresh =
      has_mavros_state_ &&
      sourceIsFresh(mavros_state_last_seen_, now, flight_stale_after);
  const bool flight_extended_fresh =
      has_mavros_extended_state_ &&
      sourceIsFresh(mavros_extended_state_last_seen_, now, flight_stale_after);

  if (channelEnabled("state.health") && shouldEmitLocked("state.health", now)) {
    xgc::semantic::common::v1::VehicleHealth payload;
    const bool online = px4IsOnline(has_mavros_state_, health_state_fresh,
                                    mavros_state_.connected);
    payload.set_online(online);
    setPositioningHealthLocked(now, &payload);
    if (!has_mavros_state_) {
      payload.set_summary("MAVROS state has not been observed");
      addFault(&payload, "mavros.state.missing",
               "MAVROS state has not been observed", 2);
    } else if (!health_state_fresh) {
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
    } else if (!health_extended_fresh) {
      addFault(&payload, "mavros.extended-state.stale",
               "MAVROS extended state exceeded its freshness limit", 1);
    }
    if (online && payload.positioning().state() ==
                      xgc::semantic::common::v1::PositioningHealth::
                          POSITIONING_STATE_FROZEN) {
      payload.set_summary("VRPN positioning data is frozen");
    } else if (online && payload.positioning().state() ==
                             xgc::semantic::common::v1::PositioningHealth::
                                 POSITIONING_STATE_TIMED_OUT) {
      payload.set_summary("VRPN positioning is unavailable");
    }
    const ros::Time stamp =
        has_mavros_state_ ? mavros_state_.header.stamp : ros::Time();
    messages->push_back(makeEnvelopeLocked("state.health", stamp, payload));
    recordOutputLocked("state.health");
  }

  if (channelEnabled("state.flight") && shouldEmitLocked("state.flight", now)) {
    xgc::semantic::aerial::v1::FlightStatus payload;
    payload.set_connected(flight_state_fresh && mavros_state_.connected);
    payload.set_armed(flight_state_fresh && mavros_state_.armed);
    payload.set_mode(flight_state_fresh ? mavros_state_.mode : std::string());
    payload.set_system_status(flight_state_fresh ? mavros_state_.system_status
                                                 : 0u);
    payload.set_landed_state(
        flight_extended_fresh ? mavros_extended_state_.landed_state : 0u);
    const ros::Time stamp =
        has_mavros_state_ ? mavros_state_.header.stamp : ros::Time();
    messages->push_back(makeEnvelopeLocked("state.flight", stamp, payload));
    recordOutputLocked("state.flight");
  }

  if (channelRequired("diagnostic.offboard-input")) {
    recordSourceLocked("diagnostic.offboard-input", now);
  }
  if (channelEnabled("diagnostic.offboard-input") &&
      shouldEmitLocked("diagnostic.offboard-input", now)) {
    xgc::semantic::aerial::v1::OffboardInputStatus payload;
    const auto local_it = sources_.find("setpoint.local");
    const auto attitude_it = sources_.find("setpoint.attitude");
    const bool local_seen = has_local_setpoint_ && local_it != sources_.end() &&
                            !local_it->second.last_seen.isZero();
    const bool attitude_seen = has_attitude_setpoint_ &&
                               attitude_it != sources_.end() &&
                               !attitude_it->second.last_seen.isZero();
    const bool use_local =
        local_seen && (!attitude_seen || local_it->second.last_seen >=
                                             attitude_it->second.last_seen);
    const bool has_setpoint = local_seen || attitude_seen;
    const std::string active_channel =
        use_local ? "setpoint.local"
                  : (attitude_seen ? "setpoint.attitude" : std::string());
    const bool setpoint_valid =
        use_local ? valid_local_setpoint_
                  : (attitude_seen && valid_attitude_setpoint_);
    const auto active_it = use_local ? local_it : attitude_it;
    const bool setpoint_fresh =
        has_setpoint && sourceIsFresh(active_it->second.last_seen, now,
                                      offboard_source_timeout_seconds_);
    const double source_rate =
        has_setpoint ? active_it->second.source_rate_hz : 0.0;

    payload.set_active_setpoint_channel(active_channel);
    payload.set_source_rate_hz(source_rate);
    if (has_setpoint)
      payload.set_source_age_ms(sourceAgeMillisLocked(active_channel, now));

    if (!has_mavros_state_) {
      addBlocker(&payload, "mavros.state.missing",
                 "MAVROS state has not been observed", 2);
    } else if (!flight_state_fresh) {
      addBlocker(&payload, "mavros.state.stale",
                 "MAVROS state exceeded its freshness limit", 2);
    } else if (!mavros_state_.connected) {
      addBlocker(&payload, "mavros.autopilot.disconnected",
                 "MAVROS is not connected to the autopilot", 2);
    }
    if (!has_setpoint) {
      addBlocker(&payload, "offboard.setpoint.missing",
                 "No local or attitude setpoint has been observed", 2);
    } else if (!setpoint_valid) {
      addBlocker(&payload, "offboard.setpoint.invalid",
                 "Latest setpoint has invalid fields or coordinate frame", 2);
    } else if (!setpoint_fresh) {
      addBlocker(&payload, "offboard.setpoint.stale",
                 "Latest setpoint exceeded the configured source timeout", 2);
    } else if (source_rate < offboard_minimum_rate_hz_) {
      addBlocker(&payload, "offboard.setpoint.rate-low",
                 "Setpoint rate is below the configured minimum", 2);
    }
    payload.set_ready(has_mavros_state_ && flight_state_fresh &&
                      mavros_state_.connected && has_setpoint &&
                      setpoint_valid && setpoint_fresh &&
                      source_rate >= offboard_minimum_rate_hz_);
    const ros::Time stamp =
        use_local
            ? local_setpoint_.header.stamp
            : (attitude_seen ? attitude_setpoint_.header.stamp : ros::Time());
    messages->push_back(
        makeEnvelopeLocked("diagnostic.offboard-input", stamp, payload));
    recordOutputLocked("diagnostic.offboard-input");
  }
}

void RobotRuntime::emitStreamHealthLocked(
    const ros::WallTime &now,
    std::vector<xgc::robot::v1::RobotMessage> *messages) {
  if (!channelEnabled("diagnostic.stream-health") ||
      !shouldEmitLocked("diagnostic.stream-health", now)) {
    return;
  }
  contract::ChannelMetadata health_channel{};
  if (!contract::channelMetadata(profile_id_, "diagnostic.stream-health",
                                 &health_channel)) {
    throw std::logic_error("stream-health descriptor is missing");
  }
  xgc::semantic::common::v1::StreamHealthReport report;
  for (std::size_t index = 0u; index < health_channel.observes_count; ++index) {
    const std::string channel_id = health_channel.observes[index];
    auto source_it = sources_.find(channel_id);
    if (source_it == sources_.end()) {
      throw std::logic_error("observed semantic source was not installed: " +
                             channel_id);
    }
    auto &source = source_it->second;
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

    auto *payload = report.add_channels();
    payload->set_channel_id(channel_id);
    payload->set_source_rate_hz(source.source_rate_hz);
    payload->set_output_rate_hz(source.output_rate_hz);
    payload->set_dropped_samples(source.dropped_samples);
    payload->set_source_age_ms(sourceAgeMillisLocked(channel_id, now));
    payload->set_stale(
        !sourceIsFresh(source.last_seen, now, source.stale_after_seconds));
  }
  messages->push_back(
      makeEnvelopeLocked("diagnostic.stream-health", ros::Time(), report));
}

} // namespace xgc_px4_multirotor_ros1_adapter
