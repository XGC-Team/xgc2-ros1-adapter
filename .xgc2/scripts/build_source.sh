#!/usr/bin/env bash
# Install selected robot server packages from source into a supplied prefix.
# Caller supplies dependencies; CI's APT resolution and packaging stay separate.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "${1:-}" == --workspace ]]; then
  work="${2:?workspace}"
  stage="${3:?install root}"
  prefix="${4:?ROS prefix}"
  jobs="${BUILD_JOBS:-2}"
  [[ "$jobs" =~ ^[1-9][0-9]*$ ]] || exit 2
  cd "$work"
  DESTDIR="$stage" catkin_make -j"$jobs" -l"$jobs" install \
    -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_PREFIX_PATH="$prefix" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"
  exit 0
fi
work="${1:?usage: build_source.sh BUILD_DIR INSTALL_ROOT ROS_PREFIX common|px4|ugv}"
stage="${2:?install root}"
prefix="${3:?ROS prefix}"
case "${4:?package selection}" in
  common) package=common ;;
  px4) package=px4_multirotor_ros1_adapter ;;
  ugv) package=ugv_ros1_adapter ;;
  *) echo 'select common, px4 or ugv' >&2; exit 2 ;;
esac
jobs="${BUILD_JOBS:-2}"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || exit 2
cmake -S "$root/src/$package" -B "$work/$package" \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
  -DCATKIN_DEVEL_PREFIX="$work/devel/$package" \
  -DXGC2_PROTOBUF_PROTO_ROOT=/usr/share/xgc2-protobuf/proto \
  -DXGC2_PROTOBUF_REGISTRY_JSON=/usr/share/xgc2-protobuf/registry/registry.json \
  -DCMAKE_INSTALL_PREFIX="$prefix"
cmake --build "$work/$package" --parallel "$jobs"
DESTDIR="$stage" cmake --install "$work/$package"
