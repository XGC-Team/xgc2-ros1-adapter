#!/usr/bin/env bash
# Run in a 1 CPU, network-none validation container, never the station master.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
provider_prefix="${XGC_LIGHTWEIGHT_SIM_MSGS_PREFIX:-$prefix}"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
simulation_prefix="${XGC_LIGHTWEIGHT_SIM_INTERFACES_PREFIX:-/usr}"
work="${FCU_EPOCH_OUTPUT:-$(mktemp -d)}"
mkdir -p "$work"
set +u
source "$prefix/setup.bash"
set -u
ros_lib="${ROS_IO_LIB:-$work/libros_io.so}"
if [[ -z "${ROS_IO_LIB:-}" ]]; then bash "$source_dir/build.sh" "$ros_lib"; fi
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -pthread \
  -I "$sdk_include" -I "$robotics_prefix/include" -I "$simulation_prefix/include" -I "$source_dir" -I "$(dirname -- "$ros_lib")/ros-io-gen" -I "$provider_prefix/include" \
  -I "${XGC_RIGID_STATE_WIRE_PREFIX:-/usr}/include" \
  -I "${XGC_HOVER_THRUST_WIRE_PREFIX:-/usr}/include" \
  -I "${XGC_REFERENCE_WIRE_PREFIX:-/usr}/include" -isystem "$prefix/include" \
  "$source_dir/sim_fcu_epoch_queue_test.cpp" "$source_dir/ros_clock_source.cpp" \
  -L "${XGC_ROS_IO_HELPERS_PREFIX:-/usr}/lib" -Wl,-rpath,"${XGC_ROS_IO_HELPERS_PREFIX:-/usr}/lib" -lxgc_ros_edge \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization \
  -lrosconsole -lrostime -lcpp_common -o "$work/fcu-epoch-queue-test"
port="$(python3 - <<'PY'
import socket
with socket.socket() as s:
    s.bind(('127.0.0.1', 0))
    print(s.getsockname()[1])
PY
)"
export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 ROS_MASTER_URI="http://127.0.0.1:$port"
export ROS_HOME="$work/ros-home" ROS_LOG_DIR="$work/ros-log"
roscore -p "$port" >"$work/roscore.log" 2>&1 & pid=$!
trap 'kill -INT "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true' EXIT
python3 - <<'PY'
import os,time,xmlrpc.client
end=time.monotonic()+5
m=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
while time.monotonic()<end:
    try:
        if m.getPid('/epoch_probe')[0]==1: break
    except OSError: pass
    time.sleep(.05)
else: raise SystemExit('private master unavailable')
PY
timeout 10s "$work/fcu-epoch-queue-test"
