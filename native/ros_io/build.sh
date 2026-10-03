#!/usr/bin/env bash
# Owning ROS edge build. Runtime is a header-only SDK dependency, not a source
# location for this ROS/domain facade. SetProvider comes from the installed lightweight-sim interface package.
set -euo pipefail
[[ $# == 1 ]] || { echo "usage: build.sh OUTPUT.so" >&2; exit 2; }
source_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-/usr/include/xgc-runtime}"
provider_prefix="${XGC_LIGHTWEIGHT_SIM_MSGS_PREFIX:-$prefix}"
robotics_prefix="${XGC_ROBOTICS_INTERFACES_PREFIX:-/usr}"
simulation_prefix="${XGC_LIGHTWEIGHT_SIM_INTERFACES_PREFIX:-/usr}"
rigid_prefix="${XGC_RIGID_STATE_WIRE_PREFIX:-/usr}"
hte_prefix="${XGC_HOVER_THRUST_WIRE_PREFIX:-/usr}"
reference_prefix="${XGC_REFERENCE_WIRE_PREFIX:-/usr}"
edge_prefix="${XGC_ROS_IO_HELPERS_PREFIX:-/usr}"
for installed_header in \
  "$robotics_prefix/include/xgc-robotics-interfaces/robotics_interfaces_v1.h" \
  "$robotics_prefix/include/xgc-robotics-interfaces/control_records_v1.h" \
  "$robotics_prefix/include/xgc-robotics-interfaces/paired_state_v1.h" \
  "$simulation_prefix/include/xgc-lightweight-sim/simulation_records_v1.h" \
  "$rigid_prefix/include/estimator_vrpn_px4_rotor_state/native/rigid_state_wire_v1.h" \
  "$hte_prefix/include/hover_thrust_estimator/native/hover_thrust_wire.h" \
  "$reference_prefix/include/multirotor_reference_trajectory/reference_wire_v1.h"; do
  [[ -f "$installed_header" ]] || { echo "required installed DTO header missing: $installed_header" >&2; exit 2; }
done
[[ -f "$edge_prefix/lib/libxgc_ros_edge.so" && -f "$edge_prefix/include/xgc-ros-io/ros_edge.hpp" ]] || {
  echo "required installed XgcRosIo::Edge utility missing at $edge_prefix" >&2; exit 2;
}
for sdk_header in xgc_rt.h xgc_clock_source.h flat_config.hpp; do
  [[ -f "$sdk_include/$sdk_header" ]] || { echo "required installed Runtime SDK header missing: $sdk_include/$sdk_header" >&2; exit 2; }
done
[[ -f "$provider_prefix/include/xgc2_lightweight_sim_msgs/SetProvider.h" &&
   -f "$provider_prefix/share/xgc2_lightweight_sim_msgs/srv/SetProvider.srv" ]] || {
  echo "installed xgc2_lightweight_sim_msgs interface missing at $provider_prefix" >&2; exit 2;
}
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
  -I "$sdk_include" -I "$source_dir" -I "$gen" -I "$provider_prefix/include" \
  -I "$robotics_prefix/include" -I "$simulation_prefix/include" -I "$rigid_prefix/include" \
  -I "$hte_prefix/include" -I "$reference_prefix/include" -isystem "$prefix/include" \
  "$source_dir/ros_io.cpp" "$source_dir/ros_clock_source.cpp" \
  -L "$edge_prefix/lib" -Wl,-rpath,"$edge_prefix/lib" -lxgc_ros_edge \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization \
  -lrosconsole -lrostime -lcpp_common -o "$output"
