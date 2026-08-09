#include "xgc_mocap_rotor_zenoh_forwarder/wire_encoder.hpp"

#include <cmath>
#include <regex>

#include <nlohmann/json.hpp>

#include "xgc_mocap_rotor_zenoh_forwarder/forwarder_contract.hpp"

namespace xgc_mocap_rotor_zenoh_forwarder {
namespace {

using Json = nlohmann::json;

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

bool finite(double value) { return std::isfinite(value); }

bool validateEnvelope(std::uint64_t sequence, std::int64_t timestamp_ms,
                      std::string *payload, std::string *error) {
  if (payload == nullptr)
    return fail(error, "wire payload output is required");
  if (sequence == 0u)
    return fail(error, "wire sequence must be positive");
  if (timestamp_ms <= 0)
    return fail(error, "wire timestamp must be positive");
  return true;
}

bool validateFrame(const std::string &frame, std::string *error) {
  static const std::regex pattern(
      "^[A-Za-z][A-Za-z0-9_.\\/-]{0,126}[A-Za-z0-9_]$");
  if (frame.size() < 2u || !std::regex_match(frame, pattern) ||
      frame.find("//") != std::string::npos ||
      frame.find("..") != std::string::npos) {
    return fail(error, "source frame_id is not canonical");
  }
  return true;
}

Json vector3(double x, double y, double z) {
  return Json{{"x", x}, {"y", y}, {"z", z}};
}

Json quaternion(double x, double y, double z, double w) {
  return Json{{"x", x}, {"y", y}, {"z", z}, {"w", w}};
}

Json finiteOrNull(double value) {
  return finite(value) ? Json(value) : Json(nullptr);
}

template <typename Array>
Json covariance(const Array &values) {
  Json result = Json::array();
  for (double value : values)
    result.push_back(value);
  return result;
}

bool serialize(const Json &body, std::string *payload, std::string *error) {
  try {
    *payload = body.dump();
    return true;
  } catch (const std::exception &exception) {
    return fail(error, std::string("cannot encode wire JSON: ") +
                           exception.what());
  }
}

bool validateQuaternion(double x, double y, double z, double w,
                        std::string *error) {
  if (!finite(x) || !finite(y) || !finite(z) || !finite(w))
    return fail(error, "orientation contains a non-finite value");
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!finite(norm) || norm < 0.5 || norm > 1.5)
    return fail(error, "orientation quaternion norm is outside the safe range");
  return true;
}

bool validateVector(double x, double y, double z, const char *label,
                    std::string *error) {
  if (!finite(x) || !finite(y) || !finite(z))
    return fail(error, std::string(label) + " contains a non-finite value");
  return true;
}

} // namespace

bool EncodePose(const geometry_msgs::PoseStamped &message,
                const std::string &child_frame_id, std::uint64_t sequence,
                std::int64_t timestamp_ms, std::string *payload,
                std::string *error) {
  if (!validateEnvelope(sequence, timestamp_ms, payload, error) ||
      !validateFrame(message.header.frame_id, error) ||
      !validateFrame(child_frame_id, error) ||
      !validateVector(message.pose.position.x, message.pose.position.y,
                      message.pose.position.z, "position", error) ||
      !validateQuaternion(message.pose.orientation.x,
                          message.pose.orientation.y,
                          message.pose.orientation.z,
                          message.pose.orientation.w, error)) {
    return false;
  }
  return serialize(
      Json{{"v", 1},
           {"sequence", sequence},
           {"t_ms", timestamp_ms},
           {"frame_id", message.header.frame_id},
           {"child_frame_id", child_frame_id},
           {"position", vector3(message.pose.position.x,
                                message.pose.position.y,
                                message.pose.position.z)},
           {"orientation", quaternion(message.pose.orientation.x,
                                      message.pose.orientation.y,
                                      message.pose.orientation.z,
                                      message.pose.orientation.w)}},
      payload, error);
}

bool EncodeVelocity(const geometry_msgs::TwistStamped &message,
                    std::uint64_t sequence, std::int64_t timestamp_ms,
                    std::string *payload, std::string *error) {
  if (!validateEnvelope(sequence, timestamp_ms, payload, error) ||
      !validateFrame(message.header.frame_id, error) ||
      !validateVector(message.twist.linear.x, message.twist.linear.y,
                      message.twist.linear.z, "linear velocity", error) ||
      !validateVector(message.twist.angular.x, message.twist.angular.y,
                      message.twist.angular.z, "angular velocity", error)) {
    return false;
  }
  return serialize(
      Json{{"v", 1},
           {"sequence", sequence},
           {"t_ms", timestamp_ms},
           {"frame_id", message.header.frame_id},
           {"linear", vector3(message.twist.linear.x, message.twist.linear.y,
                              message.twist.linear.z)},
           {"angular", vector3(message.twist.angular.x,
                               message.twist.angular.y,
                               message.twist.angular.z)}},
      payload, error);
}

bool EncodeImu(const sensor_msgs::Imu &message, std::uint64_t sequence,
               std::int64_t timestamp_ms, std::string *payload,
               std::string *error) {
  if (!validateEnvelope(sequence, timestamp_ms, payload, error) ||
      !validateFrame(message.header.frame_id, error) ||
      !validateQuaternion(message.orientation.x, message.orientation.y,
                          message.orientation.z, message.orientation.w,
                          error) ||
      !validateVector(message.angular_velocity.x, message.angular_velocity.y,
                      message.angular_velocity.z, "angular velocity", error) ||
      !validateVector(message.linear_acceleration.x,
                      message.linear_acceleration.y,
                      message.linear_acceleration.z, "linear acceleration",
                      error)) {
    return false;
  }
  for (double value : message.orientation_covariance) {
    if (!finite(value))
      return fail(error, "orientation covariance contains a non-finite value");
  }
  for (double value : message.angular_velocity_covariance) {
    if (!finite(value))
      return fail(error,
                  "angular velocity covariance contains a non-finite value");
  }
  for (double value : message.linear_acceleration_covariance) {
    if (!finite(value))
      return fail(error,
                  "linear acceleration covariance contains a non-finite value");
  }
  return serialize(
      Json{{"v", 1},
           {"sequence", sequence},
           {"t_ms", timestamp_ms},
           {"frame_id", message.header.frame_id},
           {"orientation", quaternion(message.orientation.x,
                                      message.orientation.y,
                                      message.orientation.z,
                                      message.orientation.w)},
           {"angular_velocity", vector3(message.angular_velocity.x,
                                        message.angular_velocity.y,
                                        message.angular_velocity.z)},
           {"linear_acceleration", vector3(message.linear_acceleration.x,
                                            message.linear_acceleration.y,
                                            message.linear_acceleration.z)},
           {"covariance",
            Json{{"orientation", covariance(message.orientation_covariance)},
                 {"angular_velocity",
                  covariance(message.angular_velocity_covariance)},
                 {"linear_acceleration",
                  covariance(message.linear_acceleration_covariance)}}}},
      payload, error);
}

bool EncodePower(const sensor_msgs::BatteryState &message,
                 std::uint64_t sequence, std::int64_t timestamp_ms,
                 std::string *payload, std::string *error) {
  if (!validateEnvelope(sequence, timestamp_ms, payload, error))
    return false;
  double percentage = message.percentage;
  if (!finite(percentage))
    percentage = -1.0;
  if (percentage < -1.0 || percentage > 1.0)
    return fail(error, "battery percentage must be unknown or in 0..1");
  const bool charging =
      message.power_supply_status ==
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  return serialize(Json{{"v", 1},
                        {"sequence", sequence},
                        {"t_ms", timestamp_ms},
                        {"percentage", percentage},
                        {"voltage_v", finiteOrNull(message.voltage)},
                        {"current_a", finiteOrNull(message.current)},
                        {"temperature_c", finiteOrNull(message.temperature)},
                        {"charging", charging}},
                   payload, error);
}

bool EncodeFlightState(const mavros_msgs::State &state,
                       const mavros_msgs::ExtendedState &extended_state,
                       std::uint64_t sequence, std::int64_t timestamp_ms,
                       std::string *payload, std::string *error) {
  if (!validateEnvelope(sequence, timestamp_ms, payload, error))
    return false;
  if (state.mode.size() > 64u)
    return fail(error, "flight mode exceeds 64 bytes");
  return serialize(Json{{"v", 1},
                        {"sequence", sequence},
                        {"t_ms", timestamp_ms},
                        {"connected", static_cast<bool>(state.connected)},
                        {"armed", static_cast<bool>(state.armed)},
                        {"guided", static_cast<bool>(state.guided)},
                        {"manual_input", static_cast<bool>(state.manual_input)},
                        {"mode", state.mode},
                        {"system_status", state.system_status},
                        {"landed_state", extended_state.landed_state},
                        {"faults", Json::array()}},
                   payload, error);
}

bool EncodeHeartbeat(const std::string &robot_id, std::uint64_t sequence,
                     std::int64_t timestamp_ms, std::int64_t uptime_ms,
                     const std::vector<HeartbeatChannel> &channels,
                     const HeartbeatStats &stats, std::string *payload,
                     std::string *error) {
  if (!validateEnvelope(sequence, timestamp_ms, payload, error) ||
      !ValidateRobotId(robot_id, error)) {
    return false;
  }
  if (uptime_ms < 0)
    return fail(error, "forwarder uptime must not be negative");
  if (channels.size() > 64u)
    return fail(error, "heartbeat channel list exceeds its bound");
  Json channel_values = Json::array();
  for (const auto &channel : channels) {
    if (channel.id.empty() || channel.id.size() > 64u ||
        channel.source_age_ms < -1) {
      return fail(error, "heartbeat channel metadata is invalid");
    }
    channel_values.push_back(Json{{"id", channel.id},
                                  {"source_samples", channel.source_samples},
                                  {"source_age_ms", channel.source_age_ms},
                                  {"ready", channel.ready}});
  }
  const Json stats_value{{"publish_success", stats.publish_success},
                         {"publish_failure", stats.publish_failure},
                         {"throttled", stats.throttled},
                         {"rejected_source", stats.rejected_source}};
  return serialize(Json{{"v", 1},
                        {"sequence", sequence},
                        {"t_ms", timestamp_ms},
                        {"robot_id", robot_id},
                        {"transport", "zenoh"},
                        {"uptime_ms", uptime_ms},
                        {"channels", channel_values},
                        {"stats", stats_value}},
                   payload, error);
}

} // namespace xgc_mocap_rotor_zenoh_forwarder
