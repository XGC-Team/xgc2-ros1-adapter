#!/usr/bin/env bash
# roscpp's TopicManager::unadvertise is not safe against a concurrent
# advertise/unadvertise in one process (see ros_publisher_lifecycle_test.cpp).
# Private Noetic master; not the station.
#   guarded  must survive every run (the edges' lifecycle mutex)
#   raw      informational: counts the runs that fault without the mutex
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
port="${ROS_LIFECYCLE_TEST_PORT:-11541}"
runs="${ROS_LIFECYCLE_TEST_RUNS:-20}"
work="$(mktemp -d /tmp/ros-lifecycle-test.XXXXXX)"
cleanup() {
  if [[ -n "${core_pid:-}" ]]; then kill "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; fi
  rm -rf "$work"
}
trap cleanup EXIT
export ROS_MASTER_URI="http://127.0.0.1:${port}" ROS_HOME="$work/ros-home" ROS_LOG_DIR="$work/ros-log"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
"$prefix/bin/roscore" -p "$port" >"$work/roscore.log" 2>&1 &
core_pid=$!
for _ in $(seq 1 50); do
  if "$prefix/bin/rosparam" list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
"$prefix/bin/rosparam" list >/dev/null
helpers="${XGC_ROS_IO_HELPERS_PREFIX:-/usr}"
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra \
  -I "$source_dir" -I "${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}" -isystem "$prefix/include" \
  -o "$work/lifecycle-test" "$source_dir/ros_publisher_lifecycle_test.cpp" \
  -L "$helpers/lib" -Wl,-rpath,"$helpers/lib" -lxgc_ros_edge \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization -lrosconsole -lrostime -lcpp_common -lpthread
faults() {
  local mode="$1" count=0
  for _ in $(seq 1 "$runs"); do
    "$work/lifecycle-test" "$mode" 12 12 >/dev/null 2>&1 || count=$((count + 1))
  done
  echo "$count"
}
guarded="$(faults guarded)"
raw="$(faults raw)"
echo "publisher lifecycle: guarded ${guarded}/${runs} runs faulted, raw ${raw}/${runs} runs faulted"
[[ "$guarded" == 0 ]] || { echo "publisher lifecycle: the guarded teardown must never fault" >&2; exit 1; }
