#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
PX4_PACKAGE="ros-${ROS_DISTRO}-xgc2-px4-multirotor-adapter"
PX4_ROS_PACKAGE="xgc_px4_multirotor_ros1_adapter"
SCOUT_PACKAGE="ros-${ROS_DISTRO}-xgc2-scout-mini-adapter"
SCOUT_ROS_PACKAGE="xgc_scout_mini_ros1_adapter"

dpkg -s "${PX4_PACKAGE}" >/dev/null
dpkg -s "${SCOUT_PACKAGE}" >/dev/null
dpkg -s libxgc2-adapter-link-client-dev >/dev/null
if dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-adapter" >/dev/null 2>&1; then
  echo "removed generic ROS1 adapter package is still installed" >&2
  exit 1
fi

px4_depends="$(dpkg-query -W -f='${Depends}' "${PX4_PACKAGE}")"
scout_depends="$(dpkg-query -W -f='${Depends}' "${SCOUT_PACKAGE}")"
grep -q "ros-${ROS_DISTRO}-mavros-msgs" <<<"${px4_depends}"
if grep -q 'scout-msgs' <<<"${px4_depends}"; then
  echo "PX4 adapter package must not depend on Scout messages" >&2
  exit 1
fi
grep -q "ros-${ROS_DISTRO}-scout-msgs" <<<"${scout_depends}"
if grep -q 'mavros-msgs' <<<"${scout_depends}"; then
  echo "Scout adapter package must not depend on MAVROS messages" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${PREFIX}/setup.bash"
set -u

check_ros_package() {
  local ros_package="$1"
  local launch_file="$2"
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"

  rospack find "${ros_package}" >/dev/null
  test -f "${PREFIX}/share/${ros_package}/package.xml"
  test -f "${PREFIX}/share/${ros_package}/launch/${launch_file}"
  test -d "${PREFIX}/include/${ros_package}"
  test -x "${executable}"
  ldd "${executable}" | grep -q 'libxgc2_adapter_link_client'
}

check_ros_package \
  "${PX4_ROS_PACKAGE}" \
  "px4_multirotor_ros1_adapter.launch"
check_ros_package \
  "${SCOUT_ROS_PACKAGE}" \
  "scout_mini_ros1_adapter.launch"

test ! -e "${PREFIX}/share/xgc_ros1_adapter"
test ! -e "${PREFIX}/lib/xgc_ros1_adapter/xgc_ros1_adapter_node"

echo "Installed split ROS1 adapter package checks passed"
