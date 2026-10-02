#!/usr/bin/env bash
# Owning ROS edge build. Runtime is a header-only SDK dependency, not a source
# location for this ROS/domain facade. SetProvider comes from owning SSS.
set -euo pipefail
[[ $# == 1 ]] || { echo "usage: build.sh OUTPUT.so" >&2; exit 2; }
source_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
sdk_include="${XGC_RUNTIME_SDK_INCLUDE:-${XGC_RUNTIME_SDK_SOURCE_ROOT:+$XGC_RUNTIME_SDK_SOURCE_ROOT/abi/include}}"
sdk_include="${sdk_include:-/usr/include/xgc-runtime}"
sss_source="${SSS_SOURCE_ROOT:?set owning SSS source root}"
[[ -f "$sdk_include/xgc_rt.h" && -f "$sss_source/src/sss_sim_env/srv/SetProvider.srv" ]]
set +u
source "$prefix/setup.bash"
set -u
output="$1"
mkdir -p "$(dirname -- "$output")"
gen="$(dirname -- "$output")/ros-io-gen"
mkdir -p "$gen"
for msg in formation_generator/AssumedTrajectory formation_generator/FormationTick periodic_sync/SyncTrigger rigid_state_estimator_msgs/RigidStateEstimate \
  multirotor_reference_trajectory_msgs/{AnalyticReference,SampledReference,FlatReferencePoint,ReferenceStatus} \
  hover_thrust_estimator_msgs/HoverThrustEstimate \
  xgc2_geometry_msgs/{SceneGeometry,ScenePart,SceneObstacle,SceneObstacleState,SceneSnapshot,SceneState} \
  unicycle_reference_trajectory_msgs/PlanarPvaReference; do
  pkg="${msg%/*}"
  python3 "$prefix/lib/gencpp/gen_cpp.py" "$source_dir/msg/$msg.msg" -p "$pkg" \
    -Istd_msgs:"$prefix/share/std_msgs/msg" -Igeometry_msgs:"$prefix/share/geometry_msgs/msg" \
    -Iperiodic_sync:"$source_dir/msg/periodic_sync" -I"$pkg:$source_dir/msg/$pkg" \
    -o "$gen/$pkg" -e "$prefix/share/gencpp" >/dev/null
done
python3 "$prefix/lib/gencpp/gen_cpp.py" "$sss_source/src/sss_sim_env/srv/SetProvider.srv" \
  -p sss_sim_env -Isss_sim_env:"$sss_source/src/sss_sim_env/srv" \
  -o "$gen/sss_sim_env" -e "$prefix/share/gencpp" >/dev/null
"${CXX:-c++}" -std=c++17 -O2 -fPIC -Wall -Wextra -shared -fvisibility=hidden \
  -I "$sdk_include" -I "$source_dir" -I "$gen" -isystem "$prefix/include" \
  "$source_dir/ros_io.cpp" "$source_dir/ros_clock_source.cpp" "$source_dir/ros_dmpc_edge.cpp" \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization \
  -lrosconsole -lrostime -lcpp_common -o "$output"
