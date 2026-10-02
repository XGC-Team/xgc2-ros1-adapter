#!/usr/bin/env bash
# ROS-free coordinate and sampling checks; the ROS/Host check is
# lightweight_vehicle_live.py against an independently prepared ROS master.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-${XGC_RUNTIME_SDK_SOURCE_ROOT:+$XGC_RUNTIME_SDK_SOURCE_ROOT/abi/include}}"
sdk_include="${sdk_include:-/usr/include/xgc-runtime}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
for source in sim_odometry_test sim_imu_test; do
  "${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
    -I "$sdk_include" -I "$source_dir" \
    "$source_dir/$source.cpp" -o "$output/$source"
  "$output/$source"
done
