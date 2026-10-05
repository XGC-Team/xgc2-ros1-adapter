#!/usr/bin/env bash
# Exercise the versioned wire conversion with the real MAVROS message type.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$sdk_include" -I "$robotics_prefix/include" -I "${XGC_ROS_RUNTIME_EDGE_PREFIX:-/usr}/include" -I "$source_dir" -isystem "$prefix/include" \
  "$source_dir/attitude_target_full_test.cpp" -o "$output/attitude-test"
"$output/attitude-test"
