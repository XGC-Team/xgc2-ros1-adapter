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
# This bridge may not redistribute the sole lower Edge or retired Helpers.
for retired in /usr/lib/libxgc_ros_edge.so /usr/include/xgc-ros-io /usr/share/cmake/XgcRosIoHelpers; do
  [[ ! -e "$work/payload$retired" ]] || { echo "bridge shipped retired/foreign payload: $retired" >&2; exit 1; }
done
mkdir -p "$work/missing"
if "$script_dir/package_debs.sh" --install-root "$work/missing" --output-dir "$work/out" >"$work/negative.log" 2>&1; then
  echo 'packager accepted missing native ROS bridge' >&2; exit 1
fi
grep -Fq "missing required installed native bridge: $path" "$work/negative.log"
[[ ! -d "$work/out" ]] || [[ -z "$(find "$work/out" -name '*.deb' -print -quit)" ]]
echo 'PASS: owning generic native bridge, foreign export refusal and missing native payload refusal'
