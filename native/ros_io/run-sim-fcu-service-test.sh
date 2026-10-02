#!/usr/bin/env bash
# Private ROS master and real ros_io/native-plant ABI service test.
# Run in a resource-limited, network-none container; never use the station master.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")" && pwd)"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-${XGC_RUNTIME_SDK_SOURCE_ROOT:+$XGC_RUNTIME_SDK_SOURCE_ROOT/abi/include}}"
sdk_include="${sdk_include:-/usr/include/xgc-runtime}"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
plant_lib="${PLANT_LIB:?set PLANT_LIB to the owning native plant product plugin}"
[[ -f "$plant_lib" ]] || { echo "native plant plugin does not exist: $plant_lib" >&2; exit 2; }
output="${FCU_TEST_OUTPUT:-$(mktemp -d)}"
mkdir -p "$output"
core_pid=""
cleanup() {
  if [[ -n "$core_pid" ]]; then kill -INT "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
set +u
source "$prefix/setup.bash"
set -u
export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1
export ROS_HOME="$output/ros-home" ROS_LOG_DIR="$output/ros-log"
port="$(python3 - <<'PY'
import socket
with socket.socket() as s:
    s.bind(('127.0.0.1', 0))
    print(s.getsockname()[1])
PY
)"
export ROS_MASTER_URI="http://127.0.0.1:$port"
roscore -p "$port" >"$output/roscore.log" 2>&1 & core_pid=$!
python3 - <<'PY'
import os,time,xmlrpc.client
end=time.monotonic()+8
master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
while time.monotonic()<end:
    try:
        if master.getPid('/fcu_service_validation')[0]==1: break
    except OSError: pass
    time.sleep(.05)
else: raise SystemExit('private ROS master did not become ready')
PY
ros_lib="${ROS_IO_LIB:-$output/libros_io.so}"
if [[ -z "${ROS_IO_LIB:-}" ]]; then bash "$source_dir/build.sh" "$ros_lib"; fi
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
  -I "$sdk_include" -isystem "$prefix/include" \
  "$source_dir/sim_fcu_service_test.cpp" -o "$output/fcu-service-test" \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization \
  -lrosconsole -lrostime -lcpp_common -ldl
timeout --signal=TERM 15s "$output/fcu-service-test" "$ros_lib" "$plant_lib"
