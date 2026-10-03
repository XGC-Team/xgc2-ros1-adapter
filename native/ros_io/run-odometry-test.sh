#!/usr/bin/env bash
# ROS-free coordinate and sampling checks; the ROS/Host check is
# lightweight_vehicle_live.py against an independently prepared ROS master.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
simulation_prefix="${XGC_LIGHTWEIGHT_SIM_INTERFACES_PREFIX:-/usr}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
for source in sim_odometry_test sim_imu_test; do
  "${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
    -I "$sdk_include" -I "$robotics_prefix/include" -I "$simulation_prefix/include" -I "$source_dir" \
    "$source_dir/$source.cpp" -o "$output/$source"
  "$output/$source"
done
