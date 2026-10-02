#!/usr/bin/env bash
# Exercise the versioned wire conversion with the real MAVROS message type.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-${XGC_RUNTIME_SDK_SOURCE_ROOT:+$XGC_RUNTIME_SDK_SOURCE_ROOT/abi/include}}"
sdk_include="${sdk_include:-/usr/include/xgc-runtime}"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$sdk_include" -I "$source_dir" -isystem "$prefix/include" \
  "$source_dir/attitude_target_full_test.cpp" -o "$output/attitude-test"
"$output/attitude-test"
