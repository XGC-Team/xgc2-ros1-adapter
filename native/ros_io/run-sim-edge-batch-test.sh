#!/usr/bin/env bash
# ROS-free checks of the batched simulation edge (sim_edge_batch.hpp): port
# layout, config split, plant-input tee and per-robot host.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$source_dir" -I "$sdk_include" \
  "$source_dir/sim_edge_batch_test.cpp" -o "$output/sim-edge-batch-test"
"$output/sim-edge-batch-test"
