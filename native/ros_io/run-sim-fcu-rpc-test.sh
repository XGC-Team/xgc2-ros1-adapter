#!/usr/bin/env bash
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
simulation_prefix="${XGC_LIGHTWEIGHT_SIM_INTERFACES_PREFIX:-/usr}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
  -I "$sdk_include" -I "$robotics_prefix/include" -I "$simulation_prefix/include" -I "$source_dir" \
  "$source_dir/sim_fcu_rpc_test.cpp" -o "$output/fcu-rpc-test"
"$output/fcu-rpc-test"
