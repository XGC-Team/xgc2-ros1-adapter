#!/usr/bin/env bash
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-${XGC_RUNTIME_SDK_SOURCE_ROOT:+$XGC_RUNTIME_SDK_SOURCE_ROOT/abi/include}}"
sdk_include="${sdk_include:-/usr/include/xgc-runtime}"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
  -I "$sdk_include" -I "$source_dir" \
  "$source_dir/sim_fcu_rpc_test.cpp" -o "$output/fcu-rpc-test"
"$output/fcu-rpc-test"
