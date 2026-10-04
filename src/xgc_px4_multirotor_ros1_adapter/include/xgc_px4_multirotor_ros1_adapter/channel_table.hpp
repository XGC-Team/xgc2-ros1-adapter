#pragma once

// Per-robot semantic channel state, resolved once.
//
// The generated profile contract is immutable for the life of a runtime: a
// channel's output rate, message schema and freshness bound never change after
// the runtime is created. Resolving them on every ROS callback (a linear scan
// of the profile, a struct copy and several string-keyed map lookups) repeated
// that constant work at the source rate (about 100 Hz per topic) although the
// 10 Hz output gate dropped most samples. ChannelTable performs each lookup
// once at construction and gives every channel a fixed array slot, so a
// callback indexes an array and compares a steady-time interval before it
// builds any protobuf.
//
// Semantics are those of the string-keyed implementation it replaces: the same
// gate decision (first sample, clock rollback, interval compare), the same
// per-channel sequence, the same source accounting (including the lazily
// created tracker of a channel that is enabled but not required), and the same
// std::logic_error text at the same moment. Callers hold the owning runtime's
// mutex for every member except the constructor.

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <google/protobuf/message.h>
#include <ros/time.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"

namespace xgc_px4_multirotor_ros1_adapter {

struct SourceTracker {
  ros::WallTime last_seen;
  ros::WallTime window_started;
  std::uint64_t source_samples = 0;
  std::uint64_t output_samples = 0;
  std::uint64_t dropped_samples = 0;
  double source_rate_hz = 0.0;
  double output_rate_hz = 0.0;
  double stale_after_seconds = 1.0;
};

// Every semantic channel the PX4 runtime emits, in a fixed order. The order
// is the array index of ChannelTable and must match channelIdName().
enum class ChannelId : std::size_t {
  kLocalizationError = 0,
  kPose,
  kMocapPose,
  kVisionPose,
  kMocapVelocity,
  kMocapSpeed,
  kMocapAcceleration,
  kVelocity,
  kImu,
  kPower,
  kController,
  kHealth,
  kFlight,
  kSetpointLocal,
  kSetpointAttitude,
  kFcuLink,
  kOffboardInput,
  kStreamHealth,
  kCount
};

constexpr std::size_t kChannelCount =
    static_cast<std::size_t>(ChannelId::kCount);

inline const char *channelIdName(ChannelId id) {
  static const char *const names[kChannelCount] = {
      "state.localization.error",
      "state.pose",
      "state.mocap.pose",
      "state.vision.pose",
      "state.mocap.velocity",
      "state.mocap.speed",
      "state.mocap.acceleration",
      "state.velocity",
      "state.imu",
      "state.power",
      "state.controller",
      "state.health",
      "state.flight",
      "setpoint.local",
      "setpoint.attitude",
      "diagnostic.fcu-link",
      "diagnostic.offboard-input",
      "diagnostic.stream-health",
  };
  return names[static_cast<std::size_t>(id)];
}

class ChannelTable {
public:
  struct Channel {
    enum class Envelope { kReady, kChannelAbsent, kMessageAbsent };

    std::string id;
    bool enabled = false;
    bool required = false;

    // Resolved from the generated contract at construction.
    bool metadata_found = false;
    double output_rate_hz = 0.0;
    // 1.0 / output_rate_hz when the rate is positive, else 0.0 (unused).
    double emit_interval_seconds = 0.0;
    std::uint32_t stale_after_millis = 0u;
    std::vector<std::string> observes;
    Envelope envelope = Envelope::kChannelAbsent;
    xgc2_ros1_robot_adapter::MessageSchema schema;

    // Mutable per-channel state, previously keyed by channel id in maps.
    ros::WallTime last_output;
    std::uint64_t sequence = 0;
    // Points into sources_ (std::map nodes are stable). Non-null exactly when
    // sources_ holds an entry for this channel.
    SourceTracker *source = nullptr;
  };

  ChannelTable(const std::string &profile_id,
               const std::set<std::string> &enabled_channels,
               const std::set<std::string> &required_channels) {
    for (std::size_t index = 0u; index < kChannelCount; ++index) {
      Channel &channel = channels_[index];
      channel.id = channelIdName(static_cast<ChannelId>(index));
      channel.enabled = enabled_channels.count(channel.id) != 0;
      channel.required = required_channels.count(channel.id) != 0;
      resolve(profile_id, &channel);
    }
  }

  ChannelTable(const ChannelTable &) = delete;
  ChannelTable &operator=(const ChannelTable &) = delete;

  Channel &operator[](ChannelId id) {
    return channels_[static_cast<std::size_t>(id)];
  }
  const Channel &operator[](ChannelId id) const {
    return channels_[static_cast<std::size_t>(id)];
  }

  // Installation-time lookup by wire id. Returns nullptr for an id that the
  // PX4 runtime does not emit.
  Channel *find(const std::string &channel_id) {
    for (auto &channel : channels_) {
      if (channel.id == channel_id)
        return &channel;
    }
    return nullptr;
  }

  std::map<std::string, SourceTracker> &sources() { return sources_; }
  const std::map<std::string, SourceTracker> &sources() const {
    return sources_;
  }

  // Number of generated-contract lookups performed since construction. The
  // value is fixed after construction; every member below reads only the
  // resolved fields.
  std::uint64_t contractLookups() const { return contract_lookups_; }

  void ensureSource(Channel *channel, double stale_after_seconds) {
    if (stale_after_seconds <= 0.0)
      throw std::logic_error("semantic source freshness must be positive");
    SourceTracker &source = sources_[channel->id];
    source.stale_after_seconds = stale_after_seconds;
    channel->source = &source;
  }

  // The tracker for a dropped sample. A channel that is enabled but was never
  // installed as a source still gets a default tracker here, as before.
  SourceTracker &droppedSource(Channel *channel) {
    if (channel->source == nullptr)
      channel->source = &sources_[channel->id];
    return *channel->source;
  }

  void recordSource(Channel *channel, const ros::WallTime &now) {
    if (channel->source == nullptr) {
      throw std::logic_error("semantic source was not installed: " +
                             channel->id);
    }
    SourceTracker &source = *channel->source;
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

  void recordOutput(Channel *channel) {
    if (channel->source == nullptr) {
      throw std::logic_error("semantic source was not installed: " +
                             channel->id);
    }
    ++channel->source->output_samples;
  }

  bool shouldEmit(Channel *channel, const ros::WallTime &now) {
    if (!channel->metadata_found || channel->output_rate_hz <= 0.0)
      return false;
    if (!channel->last_output.isZero() && now >= channel->last_output &&
        (now - channel->last_output).toSec() < channel->emit_interval_seconds) {
      return false;
    }
    channel->last_output = now;
    return true;
  }

  // observed_unix_nanos is the wall-clock read taken by the caller when the
  // envelope is built.
  xgc::robot::v1::RobotMessage
  makeEnvelope(Channel *channel, const std::string &robot_id,
               const ros::Time &source_stamp,
               const google::protobuf::Message &payload,
               std::int64_t observed_unix_nanos) {
    if (channel->envelope == Channel::Envelope::kChannelAbsent) {
      throw std::logic_error(
          "channel output is absent from generated XGC2 contract metadata");
    }
    if (channel->envelope == Channel::Envelope::kMessageAbsent) {
      throw std::logic_error(
          "message ID is absent from generated XGC2 contract metadata");
    }
    xgc2_ros1_robot_adapter::RobotMessageContext context;
    context.robot_id = robot_id;
    context.channel_id = channel->id;
    context.sequence = ++channel->sequence;
    if (!source_stamp.isZero()) {
      context.has_source_time = true;
      context.source_time_nanos =
          static_cast<std::int64_t>(source_stamp.toNSec());
      context.source_clock_domain = ros::Time::isSimTime()
                                        ? xgc::v1::CLOCK_DOMAIN_SIMULATION
                                        : xgc::v1::CLOCK_DOMAIN_NATIVE;
    }
    context.observed_unix_nanos = observed_unix_nanos;

    xgc::robot::v1::RobotMessage envelope;
    std::string error;
    if (!xgc2_ros1_robot_adapter::BuildRobotMessage(
            context, channel->schema, payload, &envelope, &error)) {
      throw std::runtime_error("failed to build robot telemetry item: " +
                               error);
    }
    return envelope;
  }

private:
  void resolve(const std::string &profile_id, Channel *channel) {
    contract::ChannelMetadata metadata{};
    ++contract_lookups_;
    if (!contract::channelMetadata(profile_id, channel->id, &metadata))
      return;
    channel->metadata_found = true;
    channel->output_rate_hz = metadata.output_rate_hz;
    if (metadata.output_rate_hz > 0.0)
      channel->emit_interval_seconds = 1.0 / metadata.output_rate_hz;
    channel->stale_after_millis = metadata.stale_after_millis;
    for (std::size_t index = 0u; index < metadata.observes_count; ++index)
      channel->observes.emplace_back(metadata.observes[index]);
    if (metadata.output_message_id == 0u)
      return;
    contract::MessageMetadata message{};
    ++contract_lookups_;
    if (!contract::messageMetadata(metadata.output_message_id, &message)) {
      channel->envelope = Channel::Envelope::kMessageAbsent;
      return;
    }
    channel->schema.message_id = metadata.output_message_id;
    channel->schema.type_name = message.type_name;
    channel->schema.version = message.version;
    channel->schema.fingerprint = message.fingerprint;
    channel->envelope = Channel::Envelope::kReady;
  }

  std::array<Channel, kChannelCount> channels_;
  std::map<std::string, SourceTracker> sources_;
  std::uint64_t contract_lookups_ = 0u;
};

} // namespace xgc_px4_multirotor_ros1_adapter
