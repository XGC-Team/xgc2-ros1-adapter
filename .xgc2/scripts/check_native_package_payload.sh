#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo 'usage: check_native_package_payload.sh INSTALL_ROOT DEB_DIR' >&2; exit 2; }
install_root="$1"
deb_dir="$2"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
prefix="/opt/ros/${ROS_DISTRO:-noetic}"
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
shopt -s nullglob
debs=("$deb_dir"/ros-"${ROS_DISTRO:-noetic}"-xgc2-ros1-native-bridge_*.deb)
[[ ${#debs[@]} == 1 ]] || { echo 'expected exactly one native bridge Deb' >&2; exit 1; }
dpkg-deb --extract "${debs[0]}" "$work/payload"
path="$prefix/lib/libros_io.so"
cmp "$install_root$path" "$work/payload$path"
file -b "$work/payload$path" | grep -q '^ELF'
nm -D --defined-only "$work/payload$path" | awk '$3 == "xgc_rt_plugin_v1" {found=1} END {exit !found}'
NATIVE_UTILITY_PATHS=(
  "/usr/lib/libxgc_ros_edge.so"
  "/usr/include/xgc-ros-io/ros_edge.hpp"
  "/usr/include/xgc-ros-io/ros_slice.hpp"
  "/usr/include/xgc-ros-io/sim_odometry.hpp"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoHelpersConfig.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoHelpersConfigVersion.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoHelpersTargets.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoEdgeTargets.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoEdgeTargets-release.cmake"
)
for utility in "${NATIVE_UTILITY_PATHS[@]}"; do
  cmp "$install_root$utility" "$work/payload$utility"
done
file -b "$work/payload/usr/lib/libxgc_ros_edge.so" | grep -q '^ELF'
# Reject each missing Edge/header/export before producing any Deb.
for utility in "${NATIVE_UTILITY_PATHS[@]}"; do
  missing="$work/missing-utility"
  rm -rf -- "$missing" "$work/utility-out"
  mkdir -p "$missing"
  cp -al "$install_root/." "$missing/"
  rm -- "$missing$utility"
  if "$script_dir/package_debs.sh" --install-root "$missing" --output-dir "$work/utility-out" >"$work/utility-negative.log" 2>&1; then
    echo "packager accepted missing native utility: $utility" >&2; exit 1
  fi
  grep -Fq "missing required installed native utility: $utility" "$work/utility-negative.log"
  [[ ! -d "$work/utility-out" ]] || [[ -z "$(find "$work/utility-out" -name '*.deb' -print -quit)" ]]
done
mkdir -p "$work/missing"
if "$script_dir/package_debs.sh" --install-root "$work/missing" --output-dir "$work/out" >"$work/negative.log" 2>&1; then
  echo 'packager accepted missing native ROS bridge' >&2; exit 1
fi
grep -Fq "missing required installed native bridge: $path" "$work/negative.log"
[[ ! -d "$work/out" ]] || [[ -z "$(find "$work/out" -name '*.deb' -print -quit)" ]]
echo 'PASS: real native bridge/Edge/Helpers Deb and missing-payload refusals'
