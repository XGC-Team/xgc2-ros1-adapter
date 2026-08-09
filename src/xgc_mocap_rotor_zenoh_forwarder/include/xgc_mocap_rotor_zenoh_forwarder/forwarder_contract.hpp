#pragma once

#include <cstddef>
#include <string>

namespace xgc_mocap_rotor_zenoh_forwarder {

enum class UplinkChannel {
  kLocalPose,
  kLocalVelocity,
  kImu,
  kPower,
  kFlightState,
  kForwarderHeartbeat,
};

struct ForwarderConfig {
  std::string robot_id;
  std::string zenoh_connect;
  std::string local_pose_topic;
  std::string local_velocity_topic;
  std::string imu_topic;
  std::string power_topic;
  std::string flight_state_topic;
  std::string extended_state_topic;
  std::string pose_child_frame_id;
};

const char *UplinkLeaf(UplinkChannel channel);
std::size_t MaximumPayloadBytes(UplinkChannel channel);
double MaximumRateHz(UplinkChannel channel);
std::string UplinkKey(const std::string &robot_id, UplinkChannel channel);

bool ValidateRobotId(const std::string &robot_id, std::string *error);
bool ValidateZenohConnectEndpoint(const std::string &endpoint,
                                  std::string *error);
bool ValidateForwarderConfig(const ForwarderConfig &config,
                             std::string *error);

} // namespace xgc_mocap_rotor_zenoh_forwarder
