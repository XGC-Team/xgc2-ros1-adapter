#!/usr/bin/env bash
# ROS-free checks of the simulated MAVROS state publication gate
# (sim_publish_gate.hpp) and of its wiring in ros_io.cpp.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$source_dir" \
  "$source_dir/sim_publish_gate_test.cpp" -o "$output/publish-gate-test"
"$output/publish-gate-test"

# Wiring contract. The gate sits between the facade update and the ROS
# publication of both records: every plant sample still reaches the FCU request
# facade (sim_rpc->state) and the monotonic extended-state stamp check, and only
# the publication is gated. A publication that bypasses the gate fails here.
python3 - "$source_dir/ros_io.cpp" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()

def body(start_marker, end_marker):
    start = source.index(start_marker)
    return source[start:source.index(end_marker, start)]

state = body("while (host->next(host->host, kSimFcuState, &v) == XGC_OK) {",
             "while (host->next(host->host, kSimFcuResult, &v) == XGC_OK) {")
extended = body("while (host->next(host->host, kSimExtendedState, &v) == XGC_OK) {",
                "while (host->next(host->host, kPlanarPva, &v) == XGC_OK) {")

def ordered(text, *needles):
    positions = [text.find(needle) for needle in needles]
    return all(position >= 0 for position in positions) and positions == sorted(positions)

assert ordered(state, "sim_rpc->state(s.stamp", "sim_state_gate.admit(",
               "pubs[kSimFcuState].publish(m)"), "state publication must follow facade update and gate"
assert state.count("pubs[kSimFcuState].publish(") == 1
assert ordered(extended, "state.stamp <= last_sim_extended_stamp", "last_sim_extended_stamp = state.stamp",
               "sim_extended_gate.admit(", "pubs[kSimExtendedState].publish(message)"), \
    "extended publication must follow the stamp update and gate"
assert extended.count("pubs[kSimExtendedState].publish(") == 1
# Reopened output starts a new publication period.
discard = body("void discard_pending_output() {", "// --- module inputs -> ROS")
assert "sim_state_gate.reset()" in discard and "sim_extended_gate.reset()" in discard
# Defaults keep the previous behavior: no period key means every sample.
assert re.search(r'cfg::number\(t, "sim_fcu_state_period_ms"', source)
assert re.search(r'cfg::number\(t, "sim_extended_state_period_ms"', source)
print("ros_io state publication wiring checks passed")
PY
