#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

// Publication cadence of the simulated MAVROS `state` and `extended_state`.
//
// A real MAVROS node publishes `state` once per FCU HEARTBEAT (1 Hz) and
// `extended_state` once per EXTENDED_SYS_STATE message, not once per model
// output. The lightweight plant produces both records at every output period.
// This gate lets a deployment publish them at the physical period instead, and
// additionally at once whenever a content field changes (so an arming, a mode
// change or a landed-state transition is never delayed by the period).
//
// Scope:
//   * Only the ROS publication is gated. The caller keeps feeding every plant
//     sample to the FCU request facade (xgc_sim_fcu::Rpc::state) and keeps its
//     own monotonic stamp checks; those do not depend on this header.
//   * The gate never looks at subscribers. It is a function of the content and
//     of steady (wall) time only.
//   * Steady time, not the Session clock and not the sample stamp, is the
//     period reference: the consumers' freshness windows (controller topic
//     statistics, Adapter source freshness, visualizer timeout) are measured on
//     wall receipt time. A paused Session produces no samples, so nothing is
//     published; the first sample after the pause is published at once because
//     the elapsed steady time then exceeds the period.
//   * A period of zero keeps the previous behavior: every sample is published.
//   * The first sample after construction or reset() is always published.
namespace xgc_sim_publish {

using SteadyClock = std::chrono::steady_clock;

// Largest accepted period. A state record older than this is no longer a
// plausible heartbeat for any consumer window in the deployment.
constexpr double kMaxPeriodMs = 60000.0;

// Converts a configured whole number of milliseconds into a steady duration.
// Returns false (leaving *out unchanged) for NaN, infinity, negative, over
// kMaxPeriodMs or non-integral values. Zero means "publish every sample".
inline bool period_from_ms(double ms, SteadyClock::duration* out) {
  if (out == nullptr || !std::isfinite(ms) || ms < 0.0 || ms > kMaxPeriodMs ||
      std::floor(ms) != ms)
    return false;
  *out = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::milliseconds(static_cast<int64_t>(ms)));
  return true;
}

// Content fields of mavros_msgs/State (the header stamp is not content).
struct FcuStateContent {
  bool connected{false};
  bool armed{false};
  bool guided{false};
  bool manual_input{false};
  uint32_t system_status{0};
  std::string mode;
  bool operator==(const FcuStateContent& other) const {
    return connected == other.connected && armed == other.armed && guided == other.guided &&
           manual_input == other.manual_input && system_status == other.system_status &&
           mode == other.mode;
  }
};

// Content fields of mavros_msgs/ExtendedState (the header stamp is not content).
struct ExtendedStateContent {
  uint32_t landed_state{0};
  uint32_t vtol_state{0};
  bool operator==(const ExtendedStateContent& other) const {
    return landed_state == other.landed_state && vtol_state == other.vtol_state;
  }
};

template <class Content>
class ChangeOrPeriodGate {
 public:
  ChangeOrPeriodGate() = default;
  explicit ChangeOrPeriodGate(SteadyClock::duration period) : period_(period) {}

  bool every_sample() const { return period_ <= SteadyClock::duration::zero(); }

  // The next sample is published unconditionally (activation, reopened output).
  void reset() { have_published_ = false; }

  // True when this sample must be published now: the first sample, a content
  // change, or a full period since the previous publication. A clock reading
  // earlier than the previous publication cannot happen on a steady clock; it
  // is treated as due rather than silently held back.
  bool admit(const Content& content, SteadyClock::time_point now) {
    if (every_sample()) return true;
    if (have_published_ && content == last_content_ && now >= last_publish_ &&
        now - last_publish_ < period_)
      return false;
    have_published_ = true;
    last_content_ = content;
    last_publish_ = now;
    return true;
  }

 private:
  SteadyClock::duration period_{SteadyClock::duration::zero()};
  bool have_published_{false};
  Content last_content_{};
  SteadyClock::time_point last_publish_{};
};

}  // namespace xgc_sim_publish
