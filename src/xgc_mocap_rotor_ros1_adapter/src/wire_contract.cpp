#include "xgc_mocap_rotor_ros1_adapter/wire_contract.hpp"

#include <array>
#include <cstdint>
#include <regex>
#include <stdexcept>
#include <utility>

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

struct ChannelDefinition {
  const char *leaf;
  WireChannel channel;
  std::size_t maximum_payload_bytes;
};

const std::array<ChannelDefinition, 6u> kChannels{{
    {"local_pose", WireChannel::kLocalPose, 4096u},
    {"local_velocity", WireChannel::kLocalVelocity, 4096u},
    {"imu", WireChannel::kImu, 8192u},
    {"power", WireChannel::kPower, 2048u},
    {"flight_state", WireChannel::kFlightState, 4096u},
    {"forwarder_hb", WireChannel::kForwarderHeartbeat, 8192u},
}};

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

const ChannelDefinition &definition(WireChannel channel) {
  for (const auto &candidate : kChannels) {
    if (candidate.channel == channel)
      return candidate;
  }
  throw std::logic_error("unknown Mocap Rotor wire channel");
}

} // namespace

bool ParseWireKey(const std::string &key, ParsedWireKey *output,
                  std::string *error) {
  if (output == nullptr)
    return fail(error, "wire key output is required");
  static const std::regex pattern(
      "^xgc2/([a-z][a-z0-9-]{1,126}[a-z0-9])/up/([a-z][a-z0-9_]*)$");
  std::smatch match;
  if (!std::regex_match(key, match, pattern)) {
    return fail(error,
                "wire key must be xgc2/{robot_id}/up/{read-only-leaf}");
  }
  const std::string leaf = match[2].str();
  for (const auto &candidate : kChannels) {
    if (leaf == candidate.leaf) {
      ParsedWireKey parsed;
      parsed.robot_id = match[1].str();
      parsed.channel = candidate.channel;
      *output = std::move(parsed);
      return true;
    }
  }
  return fail(error, "wire key leaf is not in the Mocap Rotor v1 allowlist");
}

bool ValidateZenohListenEndpoint(const std::string &endpoint,
                                 std::string *error) {
  // Keep the configuration injectable only as one canonical TCP listen
  // endpoint. Discovery, connect lists, JSON fragments, and fallback
  // transports are not accepted through the Robot asset.
  static const std::regex pattern(
      "^tcp/([A-Za-z0-9][A-Za-z0-9.-]{0,252}):([0-9]{1,5})$");
  std::smatch match;
  if (!std::regex_match(endpoint, match, pattern))
    return fail(error, "zenoh_listen must be one tcp/host:port endpoint");
  try {
    const unsigned long port = std::stoul(match[2].str());
    if (port == 0u || port > 65535u)
      return fail(error, "zenoh_listen port must be in 1..65535");
  } catch (...) {
    return fail(error, "zenoh_listen port must be in 1..65535");
  }
  return true;
}

const char *WireChannelLeaf(WireChannel channel) {
  return definition(channel).leaf;
}

std::size_t MaximumPayloadBytes(WireChannel channel) {
  return definition(channel).maximum_payload_bytes;
}

} // namespace xgc_mocap_rotor_ros1_adapter
