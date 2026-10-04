#include "sim_publish_gate.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace {

using xgc_sim_publish::ChangeOrPeriodGate;
using xgc_sim_publish::ExtendedStateContent;
using xgc_sim_publish::FcuStateContent;
using xgc_sim_publish::SteadyClock;

const SteadyClock::time_point kOrigin = SteadyClock::time_point(std::chrono::seconds(1000));

SteadyClock::time_point at_ms(int64_t ms) { return kOrigin + std::chrono::milliseconds(ms); }

FcuStateContent standby() {
  FcuStateContent state;
  state.connected = true;
  state.guided = true;
  state.system_status = 3;
  state.mode = "AUTO.LOITER";
  return state;
}

// One plant output every 10 ms for `seconds`, constant content.
template <class Content>
int count_published(ChangeOrPeriodGate<Content>& gate, const Content& content, int seconds) {
  int published = 0;
  for (int64_t ms = 0; ms < seconds * 1000; ms += 10)
    if (gate.admit(content, at_ms(ms))) ++published;
  return published;
}

void check_period_conversion() {
  SteadyClock::duration period{};
  assert(xgc_sim_publish::period_from_ms(0.0, &period) && period == SteadyClock::duration::zero());
  assert(xgc_sim_publish::period_from_ms(1000.0, &period) &&
         period == std::chrono::milliseconds(1000));
  assert(xgc_sim_publish::period_from_ms(xgc_sim_publish::kMaxPeriodMs, &period));
  const SteadyClock::duration kept = period;
  for (double bad : {-1.0, 0.5, 1000.25, xgc_sim_publish::kMaxPeriodMs + 1.0,
                     std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()}) {
    assert(!xgc_sim_publish::period_from_ms(bad, &period));
    assert(period == kept);  // rejected input never writes the output
  }
  assert(!xgc_sim_publish::period_from_ms(1000.0, nullptr));
}

// The previous behavior (zero period) and an unset gate publish every sample.
void check_zero_period_publishes_every_sample() {
  ChangeOrPeriodGate<FcuStateContent> unset;
  assert(count_published(unset, standby(), 3) == 300);
  ChangeOrPeriodGate<FcuStateContent> zero{SteadyClock::duration::zero()};
  assert(zero.every_sample());
  assert(count_published(zero, standby(), 3) == 300);
}

// 100 Hz plant output, unchanged content: the first sample, then one per period.
void check_physical_period_replaces_mechanical_rate() {
  ChangeOrPeriodGate<FcuStateContent> state{std::chrono::milliseconds(1000)};
  assert(count_published(state, standby(), 10) == 10);  // t = 0, 1, ..., 9 s (not 1000)
  ChangeOrPeriodGate<ExtendedStateContent> extended{std::chrono::milliseconds(1000)};
  assert(count_published(extended, ExtendedStateContent{1, 0}, 10) == 10);
}

void check_first_sample_and_reset() {
  ChangeOrPeriodGate<FcuStateContent> gate{std::chrono::milliseconds(1000)};
  assert(gate.admit(standby(), at_ms(0)));
  assert(!gate.admit(standby(), at_ms(10)));
  gate.reset();
  assert(gate.admit(standby(), at_ms(20)));  // reopened output publishes at once
  assert(!gate.admit(standby(), at_ms(30)));
}

// Every content field is a change trigger; the header stamp is not content.
void check_each_content_field_publishes_immediately() {
  const FcuStateContent base = standby();
  FcuStateContent armed = base;
  armed.armed = true;
  FcuStateContent manual = base;
  manual.manual_input = true;
  FcuStateContent unguided = base;
  unguided.guided = false;
  FcuStateContent disconnected = base;
  disconnected.connected = false;
  FcuStateContent status = base;
  status.system_status = 4;
  FcuStateContent mode = base;
  mode.mode = "OFFBOARD";
  for (const FcuStateContent& changed : {armed, manual, unguided, disconnected, status, mode}) {
    ChangeOrPeriodGate<FcuStateContent> gate{std::chrono::milliseconds(1000)};
    assert(gate.admit(base, at_ms(0)));
    assert(!gate.admit(base, at_ms(10)));
    assert(gate.admit(changed, at_ms(20)));    // 20 ms after the previous publish, not 1 s
    assert(!gate.admit(changed, at_ms(30)));   // and the new value is now the held one
    assert(gate.admit(base, at_ms(40)));       // changing back is also a change
  }
  ChangeOrPeriodGate<ExtendedStateContent> extended{std::chrono::milliseconds(1000)};
  assert(extended.admit(ExtendedStateContent{1, 0}, at_ms(0)));
  assert(!extended.admit(ExtendedStateContent{1, 0}, at_ms(10)));
  assert(extended.admit(ExtendedStateContent{2, 0}, at_ms(20)));  // landed -> in air
  assert(extended.admit(ExtendedStateContent{2, 3}, at_ms(30)));  // vtol field
  assert(!extended.admit(ExtendedStateContent{2, 3}, at_ms(40)));
}

// A change restarts the period, so a heartbeat never follows a change within
// a few milliseconds and the steady-state rate stays at one per period.
void check_change_restarts_period() {
  ChangeOrPeriodGate<FcuStateContent> gate{std::chrono::milliseconds(1000)};
  FcuStateContent armed = standby();
  armed.armed = true;
  assert(gate.admit(standby(), at_ms(0)));
  assert(gate.admit(armed, at_ms(500)));
  assert(!gate.admit(armed, at_ms(1000)));  // 500 ms after the change
  assert(!gate.admit(armed, at_ms(1499)));
  assert(gate.admit(armed, at_ms(1500)));
}

// A long gap (paused Session) makes the next sample due at once.
void check_gap_publishes_next_sample() {
  ChangeOrPeriodGate<FcuStateContent> gate{std::chrono::milliseconds(1000)};
  assert(gate.admit(standby(), at_ms(0)));
  assert(gate.admit(standby(), at_ms(60000)));
  assert(!gate.admit(standby(), at_ms(60010)));
}

// A clock reading before the last publication is treated as due.
void check_backwards_clock_fails_open() {
  ChangeOrPeriodGate<FcuStateContent> gate{std::chrono::milliseconds(1000)};
  assert(gate.admit(standby(), at_ms(5000)));
  assert(gate.admit(standby(), at_ms(4000)));
}

// The audit bound for consumers: with a 1 s period and any 10 ms sample grid,
// the gap between publications of an unchanged state never exceeds the period
// plus one sample step, well inside the 2 s windows of the Adapter, the
// controller topic statistics (2.5 s) and the visualizer.
void check_gap_bound() {
  ChangeOrPeriodGate<FcuStateContent> gate{std::chrono::milliseconds(1000)};
  int64_t last = -1;
  int64_t worst = 0;
  for (int64_t ms = 0; ms < 20000; ms += 10) {
    if (gate.admit(standby(), at_ms(ms))) {
      if (last >= 0) worst = worst > ms - last ? worst : ms - last;
      last = ms;
    }
  }
  assert(worst >= 1000 && worst <= 1010);
}

}  // namespace

int main() {
  check_period_conversion();
  check_zero_period_publishes_every_sample();
  check_physical_period_replaces_mechanical_rate();
  check_first_sample_and_reset();
  check_each_content_field_publishes_immediately();
  check_change_restarts_period();
  check_gap_publishes_next_sample();
  check_backwards_clock_fails_open();
  check_gap_bound();
  std::cout << "sim_publish_gate checks passed\n";
  return 0;
}
