#!/usr/bin/env bash
# Private Noetic master for the ros_io clock-source adapter. Not the station.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
simulation_prefix="${XGC_LIGHTWEIGHT_SIM_INTERFACES_PREFIX:-/usr}"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
port="${ROS_CLOCK_TEST_PORT:-11531}"
work="${ROS_CLOCK_TEST_OUTPUT:-$(mktemp -d /tmp/ros-clock-test.XXXXXX)}"
mkdir -p "$work"
cleanup() {
  if [[ -n "${core_pid:-}" ]]; then kill "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; fi
  if [[ -z "${ROS_CLOCK_TEST_OUTPUT:-}" ]]; then rm -rf "$work"; fi
}
trap cleanup EXIT
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_HOME="$work/ros-home"
mkdir -p "$ROS_HOME"
if ss -ltn | awk -v p=":${port}\$" 'NR>1 && $4 ~ p { found=1 } END { exit found ? 0 : 1 }'; then
  echo "ros-clock-test: port ${port} is busy" >&2
  exit 2
fi
"$prefix/bin/roscore" -p "$port" >"$work/roscore.log" 2>&1 &
core_pid=$!
for _ in $(seq 1 50); do
  if "$prefix/bin/rosparam" list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
"$prefix/bin/rosparam" list >/dev/null
ros_lib="${ROS_IO_LIB:-$work/libros_io.so}"
if [[ -z "${ROS_IO_LIB:-}" ]]; then "$source_dir/build.sh" "$ros_lib"; fi
cxx="${CXX:-c++}"
"$cxx" -std=c++17 -O2 -Wall -Wextra \
  -I "$sdk_include" -I "$robotics_prefix/include" -I "$simulation_prefix/include" -I "$source_dir" -isystem "$prefix/include" \
  -o "$work/ros_clock_source_test" "$source_dir/ros_clock_source_test.cpp" \
  -L "${XGC_ROS_IO_HELPERS_PREFIX:-/usr}/lib" -Wl,-rpath,"${XGC_ROS_IO_HELPERS_PREFIX:-/usr}/lib" -lxgc_ros_edge \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization -lrosconsole -lrostime -lcpp_common -ldl
"$work/ros_clock_source_test" "$ros_lib"
