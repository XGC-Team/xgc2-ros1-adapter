#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

namespace xgc_mocap_rotor_zenoh_forwarder {

struct HeartbeatChannel {
  std::string id;
  std::uint64_t source_samples = 0u;
  std::int64_t source_age_ms = -1;
  bool ready = false;
};

struct HeartbeatStats {
  std::uint64_t publish_success = 0u;
  std::uint64_t publish_failure = 0u;
  std::uint64_t throttled = 0u;
  std::uint64_t rejected_source = 0u;
};

bool EncodePose(const geometry_msgs::PoseStamped &message,
                const std::string &child_frame_id, std::uint64_t sequence,
                std::int64_t timestamp_ms, std::string *payload,
                std::string *error);
bool EncodeVelocity(const geometry_msgs::TwistStamped &message,
                    std::uint64_t sequence, std::int64_t timestamp_ms,
                    std::string *payload, std::string *error);
bool EncodeImu(const sensor_msgs::Imu &message, std::uint64_t sequence,
               std::int64_t timestamp_ms, std::string *payload,
               std::string *error);
bool EncodePower(const sensor_msgs::BatteryState &message,
                 std::uint64_t sequence, std::int64_t timestamp_ms,
                 std::string *payload, std::string *error);
bool EncodeFlightState(const mavros_msgs::State &state,
                       const mavros_msgs::ExtendedState &extended_state,
                       std::uint64_t sequence, std::int64_t timestamp_ms,
                       std::string *payload, std::string *error);
bool EncodeHeartbeat(const std::string &robot_id, std::uint64_t sequence,
                     std::int64_t timestamp_ms, std::int64_t uptime_ms,
                     const std::vector<HeartbeatChannel> &channels,
                     const HeartbeatStats &stats, std::string *payload,
                     std::string *error);

} // namespace xgc_mocap_rotor_zenoh_forwarder
