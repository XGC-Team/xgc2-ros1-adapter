#!/usr/bin/env bash
# Owning ROS edge build. Runtime is a header-only SDK dependency, not a source
# location for this generic physical/control facade. Simulation owners are not
# built or exported from this product; init/gate links the sole lower Edge DSO.
set -euo pipefail
[[ $# == 1 ]] || { echo "usage: build.sh OUTPUT.so" >&2; exit 2; }
source_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
rigid_prefix="${XGC_RIGID_STATE_WIRE_PREFIX:-/usr}"
hte_prefix="${XGC_HOVER_THRUST_WIRE_PREFIX:-/usr}"
reference_prefix="${XGC_REFERENCE_WIRE_PREFIX:-/usr}"
edge_prefix="${XGC_ROS_RUNTIME_EDGE_PREFIX:-/usr}"
set +u
source "$prefix/setup.bash"
set -u
output="$1"
mkdir -p "$(dirname -- "$output")"
gen="$(dirname -- "$output")/ros-io-gen"
mkdir -p "$gen"
for msg in rigid_state_estimator_msgs/RigidStateEstimate \
  multirotor_reference_trajectory_msgs/{AnalyticReference,SampledReference,FlatReferencePoint,ReferenceStatus} \
  hover_thrust_estimator_msgs/HoverThrustEstimate \
  unicycle_reference_trajectory_msgs/PlanarPvaReference; do
  pkg="${msg%/*}"
  python3 "$prefix/lib/gencpp/gen_cpp.py" "$source_dir/msg/$msg.msg" -p "$pkg" \
    -Istd_msgs:"$prefix/share/std_msgs/msg" -Igeometry_msgs:"$prefix/share/geometry_msgs/msg" \
    -I"$pkg:$source_dir/msg/$pkg" \
    -o "$gen/$pkg" -e "$prefix/share/gencpp" >/dev/null
done
"${CXX:-c++}" -std=c++17 -O2 -fPIC -Wall -Wextra -shared -fvisibility=hidden \
  -I "$sdk_include" -I "$source_dir" -I "$gen" \
  -I "$edge_prefix/include" -I "$robotics_prefix/include" -I "$rigid_prefix/include" \
  -I "$hte_prefix/include" -I "$reference_prefix/include" -isystem "$prefix/include" \
  "$source_dir/ros_io.cpp" "$source_dir/ros_clock_source.cpp" \
  -L "$edge_prefix/lib" -Wl,-rpath,"$edge_prefix/lib" -lxgc_ros_edge \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization \
  -lrosconsole -lrostime -lcpp_common -o "$output"
