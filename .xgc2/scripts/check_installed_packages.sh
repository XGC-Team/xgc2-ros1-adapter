#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
PX4_PACKAGE="ros-${ROS_DISTRO}-xgc2-px4-multirotor-adapter"
PX4_ROS_PACKAGE="xgc_px4_multirotor_ros1_adapter"
SCOUT_PACKAGE="ros-${ROS_DISTRO}-xgc2-scout-mini-adapter"
SCOUT_ROS_PACKAGE="xgc_scout_mini_ros1_adapter"
ADAPTER_RUNTIME_CLIENT_DEB_VERSION="${ADAPTER_RUNTIME_CLIENT_DEB_VERSION:-0.5.0-2~focal}"
PROTOBUF_REGISTRY="/usr/share/xgc2-protobuf/registry/registry.json"

dpkg -s "${PX4_PACKAGE}" >/dev/null
dpkg -s "${SCOUT_PACKAGE}" >/dev/null
dpkg -s libxgc2-adapter-runtime-client1 >/dev/null
test "$(dpkg-query -W -f='${Version}' libxgc2-adapter-runtime-client1)" = \
  "${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}"
test -f "${PROTOBUF_REGISTRY}"
if dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-adapter" >/dev/null 2>&1; then
  echo "removed generic ROS1 adapter package is still installed" >&2
  exit 1
fi

px4_depends="$(dpkg-query -W -f='${Depends}' "${PX4_PACKAGE}")"
scout_depends="$(dpkg-query -W -f='${Depends}' "${SCOUT_PACKAGE}")"
for depends in "${px4_depends}" "${scout_depends}"; do
  grep -Eq '(^|, )libxgc2-adapter-runtime-client1( |[(])' <<<"${depends}"
  if grep -Eq '(^|, )(libxgc2-adapter-runtime-client-dev|xgc2-protobuf-dev)( |[(,]|$)' \
      <<<"${depends}"; then
    echo "Adapter runtime dependencies leaked SDK/schema packages" >&2
    exit 1
  fi
done
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
  if [[ "$#" -ne 4 ]]; then
    echo "check_ros_package requires package, launch, profile, and definition" >&2
    exit 1
  fi
  local ros_package="$1"
  local launch_file="$2"
  local profile_file="$3"
  local definition_id="$4"
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"

  rospack find "${ros_package}" >/dev/null
  test -f "${PREFIX}/share/${ros_package}/package.xml"
  test -f "${PREFIX}/share/${ros_package}/launch/${launch_file}"
  test -f "${PREFIX}/share/${ros_package}/profiles/ros1/${profile_file}"
  test -f "${PREFIX}/share/${ros_package}/profiles/schema/robot-adapter-profile-v3.schema.json"
  test ! -e "${PREFIX}/include/${ros_package}"
  test -x "${executable}"
  ldd "${executable}" | grep -q 'libxgc2_adapter_runtime_client'
  test -f "/usr/share/xgc2/adapter-definitions/${definition_id}.json"
  test -f "/usr/share/xgc2/process-definitions/${definition_id}.json"
  test -f "/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json"
  python3 "$(dirname "$0")/../../tools/verify_runtime_manifests.py" \
    --executable "${executable}" \
    --artifact-path "${executable}" \
    --definition-id "${definition_id}" \
    --registry "${PROTOBUF_REGISTRY}" \
    --profile-file "${PREFIX}/share/${ros_package}/profiles/ros1/${profile_file}" \
    --profile-schema "${PREFIX}/share/${ros_package}/profiles/schema/robot-adapter-profile-v3.schema.json" \
    --adapter-manifest "/usr/share/xgc2/adapter-definitions/${definition_id}.json" \
    --process-manifest "/usr/share/xgc2/process-definitions/${definition_id}.json" \
    --profile-catalog "/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json"
}

check_ros_package \
  "${PX4_ROS_PACKAGE}" \
  "px4_multirotor_ros1_adapter.launch" \
  "px4-multirotor-ros1-v6.yaml" \
  "xgc2-px4-multirotor-ros1-adapter"
PX4_SERVICE_HELPER="${PREFIX}/lib/${PX4_ROS_PACKAGE}/${PX4_ROS_PACKAGE}_service_helper"
test -x "${PX4_SERVICE_HELPER}"
ldd "${PX4_SERVICE_HELPER}" | grep -q 'libroscpp'
check_ros_package \
  "${SCOUT_ROS_PACKAGE}" \
  "scout_mini_ros1_adapter.launch" \
  "scout-mini-ros1-v4.yaml" \
  "xgc2-scout-mini-ros1-adapter"

test ! -e "${PREFIX}/share/xgc_ros1_adapter"
test ! -e "${PREFIX}/lib/xgc_ros1_adapter/xgc_ros1_adapter_node"

echo "Installed robot Adapter package checks passed"
