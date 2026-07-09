#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
PACKAGE="ros-${ROS_DISTRO}-xgc2-ros1-adapter"
ROS_PACKAGE="xgc_ros1_adapter"

dpkg -s "${PACKAGE}" >/dev/null

set +u
source "${PREFIX}/setup.bash"
set -u

rospack find "${ROS_PACKAGE}" >/dev/null
test -f "${PREFIX}/share/${ROS_PACKAGE}/package.xml"
test -d "${PREFIX}/include/${ROS_PACKAGE}"
test -x "${PREFIX}/lib/${ROS_PACKAGE}/${ROS_PACKAGE}_node"
ldd "${PREFIX}/lib/${ROS_PACKAGE}/${ROS_PACKAGE}_node" | grep -q "libgrpc++"
ldd "${PREFIX}/lib/${ROS_PACKAGE}/${ROS_PACKAGE}_node" | grep -q "libprotobuf"

echo "Installed package check passed"
