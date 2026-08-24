#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
PX4_PACKAGE="ros-${ROS_DISTRO}-xgc2-px4-multirotor-adapter"
PX4_ROS_PACKAGE="xgc_px4_multirotor_ros1_adapter"
SCOUT_PACKAGE="ros-${ROS_DISTRO}-xgc2-scout-mini-adapter"
SCOUT_ROS_PACKAGE="xgc_scout_mini_ros1_adapter"
MECANUM_PACKAGE="ros-${ROS_DISTRO}-xgc2-mecanum-ugv-adapter"
MECANUM_ROS_PACKAGE="xgc_mecanum_ugv_ros1_adapter"
B2_PACKAGE="ros-${ROS_DISTRO}-xgc2-unitree-b2-adapter"
B2_ROS_PACKAGE="xgc_unitree_b2_ros1_adapter"
MOCAP_PACKAGE="ros-${ROS_DISTRO}-xgc2-mocap-rotor-adapter"
MOCAP_ROS_PACKAGE="xgc_mocap_rotor_ros1_adapter"
MOCAP_FORWARDER_PACKAGE="ros-${ROS_DISTRO}-xgc2-mocap-rotor-forwarder"
MOCAP_FORWARDER_ROS_PACKAGE="xgc_mocap_rotor_zenoh_forwarder"
ADAPTER_RUNTIME_CLIENT_DEB_VERSION="${ADAPTER_RUNTIME_CLIENT_DEB_VERSION:-0.6.0-12~focal}"
EXPECTED_PRODUCT_VERSION="${EXPECTED_PRODUCT_VERSION:-$(
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' \
    "$(dirname "$0")/../product.yml"
)}"
EXPECTED_MOCAP_ADAPTER_VERSION="${EXPECTED_MOCAP_ADAPTER_VERSION:-${MOCAP_ADAPTER_PACKAGE_VERSION:-${EXPECTED_PRODUCT_VERSION}}}"
EXPECTED_MOCAP_ADAPTER_SOURCE_DIGEST="${EXPECTED_MOCAP_ADAPTER_SOURCE_DIGEST:-${MOCAP_ADAPTER_SOURCE_DIGEST:-}}"
if [[ -n "${EXPECTED_MOCAP_ADAPTER_SOURCE_DIGEST}" &&
      ! "${EXPECTED_MOCAP_ADAPTER_SOURCE_DIGEST}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "expected Mocap Adapter source digest must be 64 lowercase hex characters" >&2
  exit 1
fi
PROTOBUF_REGISTRY="/usr/share/xgc2-protobuf/registry/registry.json"

dpkg -s "${PX4_PACKAGE}" >/dev/null
dpkg -s "${SCOUT_PACKAGE}" >/dev/null
dpkg -s "${MECANUM_PACKAGE}" >/dev/null
dpkg -s "${B2_PACKAGE}" >/dev/null
dpkg -s "${MOCAP_PACKAGE}" >/dev/null
dpkg -s "${MOCAP_FORWARDER_PACKAGE}" >/dev/null
for base_version_package in \
  "${PX4_PACKAGE}" "${SCOUT_PACKAGE}" "${MECANUM_PACKAGE}" \
  "${B2_PACKAGE}" "${MOCAP_FORWARDER_PACKAGE}"; do
  test "$(dpkg-query -W -f='${Version}' "${base_version_package}")" = \
    "${EXPECTED_PRODUCT_VERSION}"
done
test "$(dpkg-query -W -f='${Version}' "${MOCAP_PACKAGE}")" = \
  "${EXPECTED_MOCAP_ADAPTER_VERSION}"
if [[ -n "${EXPECTED_MOCAP_ADAPTER_SOURCE_DIGEST}" ]]; then
  test "${EXPECTED_MOCAP_ADAPTER_SOURCE_DIGEST}" = \
    "$(dpkg-query -W -f='${X-XGC2-Source-Digest}' "${MOCAP_PACKAGE}")"
fi
dpkg -s libxgc2-adapter-runtime-client2 >/dev/null
test "$(dpkg-query -W -f='${Version}' libxgc2-adapter-runtime-client2)" = \
  "${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}"
test -f "${PROTOBUF_REGISTRY}"
if dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-adapter" >/dev/null 2>&1; then
  echo "removed generic ROS1 adapter package is still installed" >&2
  exit 1
fi

px4_depends="$(dpkg-query -W -f='${Depends}' "${PX4_PACKAGE}")"
scout_depends="$(dpkg-query -W -f='${Depends}' "${SCOUT_PACKAGE}")"
mecanum_depends="$(dpkg-query -W -f='${Depends}' "${MECANUM_PACKAGE}")"
b2_depends="$(dpkg-query -W -f='${Depends}' "${B2_PACKAGE}")"
mocap_depends="$(dpkg-query -W -f='${Depends}' "${MOCAP_PACKAGE}")"
mocap_forwarder_depends="$(dpkg-query -W -f='${Depends}' "${MOCAP_FORWARDER_PACKAGE}")"
for depends in "${px4_depends}" "${scout_depends}" "${mecanum_depends}" "${b2_depends}" "${mocap_depends}"; do
  grep -Eq '(^|, )libxgc2-adapter-runtime-client2( |[(])' <<<"${depends}"
  if grep -Eq '(^|, )(libxgc2-adapter-runtime-client-dev|xgc2-protobuf-dev)( |[(,]|$)' \
      <<<"${depends}"; then
    echo "Adapter runtime dependencies leaked SDK/schema packages" >&2
    exit 1
  fi
done
if grep -Eq 'libxgc2-adapter-runtime|xgc2-protobuf|scout-msgs|libzenohc' \
    <<<"${mocap_forwarder_depends}"; then
  echo "Mocap Rotor onboard Forwarder leaked ground, Scout, schema, or dynamic Zenoh dependencies" >&2
  exit 1
fi
for dependency in geometry-msgs mavros-msgs roscpp sensor-msgs; do
  grep -q "ros-${ROS_DISTRO}-${dependency}" <<<"${mocap_forwarder_depends}"
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
if grep -Eq 'mavros-msgs|scout-msgs|sensor-msgs' <<<"${mecanum_depends}"; then
  echo "Mecanum adapter package leaked unrelated robot message dependencies" >&2
  exit 1
fi
if grep -Eq 'mavros-msgs|scout-msgs' <<<"${b2_depends}"; then
  echo "Unitree B2 adapter leaked unrelated robot message dependencies" >&2
  exit 1
fi
if grep -Eq 'mavros-msgs|scout-msgs|libzenohc' <<<"${mocap_depends}"; then
  echo "Mocap Rotor Adapter leaked MAVROS, Scout, or dynamic Zenoh runtime dependencies" >&2
  exit 1
fi
for dependency in geometry-msgs nav-msgs sensor-msgs std-msgs tf2-ros; do
  grep -q "ros-${ROS_DISTRO}-${dependency}" <<<"${mocap_depends}"
done
for dependency in diagnostic-msgs nav-msgs sensor-msgs std-msgs tf2-ros; do
  grep -q "ros-${ROS_DISTRO}-${dependency}" <<<"${b2_depends}"
done
if grep -Eq 'robot-state-publisher|xgc2-b2arx-description' <<<"${b2_depends}"; then
  echo "Unitree B2 Adapter must not own the generic description/RSP runtime" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${PREFIX}/setup.bash"
set -u

check_ros_package() {
  if [[ "$#" -ne 5 ]]; then
    echo "check_ros_package requires package, launch, profile, schema, and definition" >&2
    exit 1
  fi
  local ros_package="$1"
  local launch_file="$2"
  local profile_file="$3"
  local profile_schema_file="$4"
  local definition_id="$5"
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"

  rospack find "${ros_package}" >/dev/null
  test -f "${PREFIX}/share/${ros_package}/package.xml"
  test -f "${PREFIX}/share/${ros_package}/launch/${launch_file}"
  test -f "${PREFIX}/share/${ros_package}/profiles/ros1/${profile_file}"
  test -f "${PREFIX}/share/${ros_package}/profiles/schema/${profile_schema_file}"
  test ! -e "${PREFIX}/include/${ros_package}"
  test -x "${executable}"
  ldd "${executable}" | grep -q 'libxgc2_adapter_runtime_client'
  test -f "/usr/share/xgc2/adapter-definitions/${definition_id}.json"
  test -f "/usr/share/xgc2/process-definitions/${definition_id}.json"
  test -f "/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json"
  python3 "$(dirname "$0")/../../tools/verify_runtime_manifests.py" \
    --executable "${executable}" \
    --ros-package "${ros_package}" \
    --ros-executable "${ros_package}_node" \
    --definition-id "${definition_id}" \
    --registry "${PROTOBUF_REGISTRY}" \
    --profile-file "${PREFIX}/share/${ros_package}/profiles/ros1/${profile_file}" \
    --profile-schema "${PREFIX}/share/${ros_package}/profiles/schema/${profile_schema_file}" \
    --adapter-manifest "/usr/share/xgc2/adapter-definitions/${definition_id}.json" \
    --process-manifest "/usr/share/xgc2/process-definitions/${definition_id}.json" \
    --profile-catalog "/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json"
}

check_ros_package \
  "${PX4_ROS_PACKAGE}" \
  "px4_multirotor_ros1_adapter.launch" \
  "px4-multirotor-ros1-v7.yaml" \
  "robot-adapter-profile-v4.schema.json" \
  "xgc2-px4-multirotor-ros1-adapter"
PX4_SERVICE_HELPER="${PREFIX}/lib/${PX4_ROS_PACKAGE}/${PX4_ROS_PACKAGE}_service_helper"
test -x "${PX4_SERVICE_HELPER}"
ldd "${PX4_SERVICE_HELPER}" | grep -q 'libroscpp'
check_ros_package \
  "${SCOUT_ROS_PACKAGE}" \
  "scout_mini_ros1_adapter.launch" \
  "scout-mini-ros1-v7.yaml" \
  "robot-adapter-profile-v4.schema.json" \
  "xgc2-scout-mini-ros1-adapter"
check_ros_package \
  "${MECANUM_ROS_PACKAGE}" \
  "mecanum_ugv_ros1_adapter.launch" \
  "mecanum-ugv-ros1-v4.yaml" \
  "robot-adapter-profile-v4.schema.json" \
  "xgc2-mecanum-ugv-ros1-adapter"
check_ros_package \
  "${B2_ROS_PACKAGE}" \
  "unitree_b2_ros1_adapter.launch" \
  "unitree-b2-v1.yaml" \
  "robot-adapter-profile-v4.schema.json" \
  "xgc2-unitree-b2-ros1-adapter"
check_ros_package \
  "${MOCAP_ROS_PACKAGE}" \
  "mocap_rotor_ros1_adapter.launch" \
  "mocap-rotor-ros1-v1.yaml" \
  "robot-adapter-profile-v4.schema.json" \
  "xgc2-mocap-rotor-ros1-adapter"
MOCAP_EXECUTABLE="${PREFIX}/lib/${MOCAP_ROS_PACKAGE}/${MOCAP_ROS_PACKAGE}_node"
if ldd "${MOCAP_EXECUTABLE}" | grep -q 'libzenohc'; then
  echo "Mocap Rotor Adapter must carry its Focal-built Zenoh C dependency statically" >&2
  exit 1
fi
test -f "/usr/share/doc/${MOCAP_PACKAGE}/third-party/zenoh-c/LICENSE"
test -f "/usr/share/doc/${MOCAP_PACKAGE}/third-party/zenoh-c/NOTICE.md"
MOCAP_FORWARDER_EXECUTABLE="${PREFIX}/lib/${MOCAP_FORWARDER_ROS_PACKAGE}/${MOCAP_FORWARDER_ROS_PACKAGE}_node"
rospack find "${MOCAP_FORWARDER_ROS_PACKAGE}" >/dev/null
test -f "${PREFIX}/share/${MOCAP_FORWARDER_ROS_PACKAGE}/package.xml"
test -f "${PREFIX}/share/${MOCAP_FORWARDER_ROS_PACKAGE}/launch/mocap_rotor_zenoh_forwarder.launch"
test -x "${MOCAP_FORWARDER_EXECUTABLE}"
ldd "${MOCAP_FORWARDER_EXECUTABLE}" | grep -q 'libroscpp'
if ldd "${MOCAP_FORWARDER_EXECUTABLE}" | grep -q 'libzenohc'; then
  echo "Mocap Rotor Forwarder must carry its Focal-built Zenoh C dependency statically" >&2
  exit 1
fi
test -f "/usr/share/xgc2/process-definitions/xgc2-mocap-rotor-link.json"
test -f "/usr/share/doc/${MOCAP_FORWARDER_PACKAGE}/third-party/zenoh-c/LICENSE"
test -f "/usr/share/doc/${MOCAP_FORWARDER_PACKAGE}/third-party/zenoh-c/NOTICE.md"
python3 - <<'PY'
import json
from pathlib import Path

path = Path("/usr/share/xgc2/process-definitions/xgc2-mocap-rotor-link.json")
document = json.loads(path.read_text(encoding="utf-8"))
assert document["apiVersion"] == "xgc.execution.process/v1"
assert len(document["definitions"]) == 1
definition = document["definitions"][0]
assert definition["id"] == "xgc2-mocap-rotor-link"
assert definition["internal"] is True
assert definition["command"]["executable"] == (
    "/opt/ros/noetic/lib/xgc_mocap_rotor_zenoh_forwarder/"
    "xgc_mocap_rotor_zenoh_forwarder_node"
)
required = set(definition["parameters"]["required"])
assert {"rosMasterUri", "rosIp", "flightStateTopic", "extendedStateTopic"} <= required
command = json.dumps(definition["command"]).lower()
assert "mavros_node" not in command
assert "fs150" not in command
assert "gps" not in command
assert "down/" not in command
PY
test ! -e "${PREFIX}/share/${B2_ROS_PACKAGE}/launch/unitree_b2_visualization_runtime.launch"
if grep -n -E '/tmp/|/home/|\.worktrees/' \
  /usr/share/xgc2/adapter-definitions/xgc2-unitree-b2-ros1-adapter.json \
  /usr/share/xgc2/process-definitions/xgc2-unitree-b2-ros1-adapter.json \
  /usr/share/xgc2/robot-adapter-profiles/xgc2-unitree-b2-ros1-adapter.json; then
  echo "installed Unitree B2 manifests contain a source-development path" >&2
  exit 1
fi

test ! -e "${PREFIX}/share/xgc_ros1_adapter"
test ! -e "${PREFIX}/lib/xgc_ros1_adapter/xgc_ros1_adapter_node"

echo "Installed robot Adapter package checks passed"
