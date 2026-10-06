#!/usr/bin/env bash
# ROS-free checks of ros_io's per-step ROS service arithmetic (ros_slice.hpp).
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$source_dir" \
  "$source_dir/ros_slice_test.cpp" -o "$output/slice-test"
"$output/slice-test"
