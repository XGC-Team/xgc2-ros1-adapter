#include "xgc_mocap_rotor_zenoh_forwarder/forwarder_contract.hpp"

#include <array>
#include <regex>
#include <set>
#include <stdexcept>

namespace xgc_mocap_rotor_zenoh_forwarder {
namespace {

struct ChannelDefinition {
  const char *leaf;
  UplinkChannel channel;
  std::size_t maximum_payload_bytes;
  double maximum_rate_hz;
};

const std::array<ChannelDefinition, 6u> kChannels{{
    {"local_pose", UplinkChannel::kLocalPose, 4096u, 15.0},
    {"local_velocity", UplinkChannel::kLocalVelocity, 4096u, 15.0},
    {"imu", UplinkChannel::kImu, 8192u, 10.0},
    {"power", UplinkChannel::kPower, 2048u, 2.0},
    {"flight_state", UplinkChannel::kFlightState, 4096u, 2.0},
    {"forwarder_hb", UplinkChannel::kForwarderHeartbeat, 8192u, 1.0},
}};

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

const ChannelDefinition &definition(UplinkChannel channel) {
  for (const auto &candidate : kChannels) {
    if (candidate.channel == channel)
      return candidate;
  }
  throw std::logic_error("unknown Mocap Rotor uplink channel");
}

bool validateAbsoluteTopic(const std::string &name, const std::string &value,
                           std::string *error) {
  static const std::regex pattern("^/[A-Za-z][A-Za-z0-9_/]*$");
  if (value.empty() || value.size() > 256u ||
      !std::regex_match(value, pattern) ||
      value.find("//") != std::string::npos) {
    return fail(error, name + " must be one explicit absolute ROS1 topic");
  }
  return true;
}

bool validateFrame(const std::string &value, std::string *error) {
  static const std::regex pattern(
      "^[A-Za-z][A-Za-z0-9_.\\/-]{0,126}[A-Za-z0-9_]$");
  if (value.size() < 2u || !std::regex_match(value, pattern) ||
      value.find("//") != std::string::npos ||
      value.find("..") != std::string::npos) {
    return fail(error, "pose_child_frame_id must be one canonical frame");
  }
  return true;
}

} // namespace

const char *UplinkLeaf(UplinkChannel channel) { return definition(channel).leaf; }

std::size_t MaximumPayloadBytes(UplinkChannel channel) {
  return definition(channel).maximum_payload_bytes;
}

double MaximumRateHz(UplinkChannel channel) {
  return definition(channel).maximum_rate_hz;
}

std::string UplinkKey(const std::string &robot_id, UplinkChannel channel) {
  return "xgc2/" + robot_id + "/up/" + UplinkLeaf(channel);
}

bool ValidateRobotId(const std::string &robot_id, std::string *error) {
  static const std::regex pattern("^[a-z][a-z0-9-]{1,126}[a-z0-9]$");
  if (!std::regex_match(robot_id, pattern))
    return fail(error, "robot_id is not a canonical Mocap Rotor wire identity");
  return true;
}

bool ValidateZenohConnectEndpoint(const std::string &endpoint,
                                  std::string *error) {
  static const std::regex pattern(
      "^tcp/([A-Za-z0-9][A-Za-z0-9.-]{0,252}):([0-9]{1,5})$");
  std::smatch match;
  if (!std::regex_match(endpoint, match, pattern))
    return fail(error, "zenoh_connect must be one tcp/host:port endpoint");
  try {
    const unsigned long port = std::stoul(match[2].str());
    if (port == 0u || port > 65535u)
      return fail(error, "zenoh_connect port must be in 1..65535");
  } catch (...) {
    return fail(error, "zenoh_connect port must be in 1..65535");
  }
  return true;
}

bool ValidateForwarderConfig(const ForwarderConfig &config,
                             std::string *error) {
  if (!ValidateRobotId(config.robot_id, error) ||
      !ValidateZenohConnectEndpoint(config.zenoh_connect, error) ||
      !validateFrame(config.pose_child_frame_id, error)) {
    return false;
  }
  const std::array<std::pair<const char *, const std::string *>, 6u> topics{{
      {"local_pose_topic", &config.local_pose_topic},
      {"local_velocity_topic", &config.local_velocity_topic},
      {"imu_topic", &config.imu_topic},
      {"power_topic", &config.power_topic},
      {"flight_state_topic", &config.flight_state_topic},
      {"extended_state_topic", &config.extended_state_topic},
  }};
  std::set<std::string> unique;
  for (const auto &topic : topics) {
    if (!validateAbsoluteTopic(topic.first, *topic.second, error))
      return false;
    if (!unique.insert(*topic.second).second)
      return fail(error, "Mocap Rotor source topic bindings must be unique");
  }
  return true;
}

} // namespace xgc_mocap_rotor_zenoh_forwarder
