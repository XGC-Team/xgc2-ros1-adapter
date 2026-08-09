#pragma once

#include <cstddef>
#include <string>

namespace xgc_mocap_rotor_ros1_adapter {

enum class WireChannel {
  kLocalPose,
  kLocalVelocity,
  kImu,
  kPower,
  kFlightState,
  kForwarderHeartbeat,
};

struct ParsedWireKey {
  std::string robot_id;
  WireChannel channel = WireChannel::kLocalPose;
};

// The Mocap Rotor data plane accepts only the six read-only v1 uplink leaves.
// GPS, command/downlink, and FS150 keys are absent by construction.
bool ParseWireKey(const std::string &key, ParsedWireKey *output,
                  std::string *error);

bool ValidateZenohListenEndpoint(const std::string &endpoint,
                                 std::string *error);

const char *WireChannelLeaf(WireChannel channel);
std::size_t MaximumPayloadBytes(WireChannel channel);

} // namespace xgc_mocap_rotor_ros1_adapter
