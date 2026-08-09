#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
PACKAGE_NAME="ros-${ROS_DISTRO}-xgc2-unitree-b2-adapter"
ROS_PACKAGE="xgc_unitree_b2_ros1_adapter"
DEFINITION_ID="xgc2-unitree-b2-ros1-adapter"
EXPECTED_PRODUCT_VERSION="${EXPECTED_PRODUCT_VERSION:-$(
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' \
    "${REPO_ROOT}/.xgc2/product.yml"
)}"
EXECUTABLE="${PREFIX}/lib/${ROS_PACKAGE}/${ROS_PACKAGE}_node"
PROFILE="${PREFIX}/share/${ROS_PACKAGE}/profiles/ros1/unitree-b2-v1.yaml"
PROFILE_SCHEMA="${PREFIX}/share/${ROS_PACKAGE}/profiles/schema/robot-adapter-profile-v4.schema.json"
REGISTRY="/usr/share/xgc2-protobuf/registry/registry.json"
ADAPTER_MANIFEST="/usr/share/xgc2/adapter-definitions/${DEFINITION_ID}.json"
PROCESS_MANIFEST="/usr/share/xgc2/process-definitions/${DEFINITION_ID}.json"
PROFILE_CATALOG="/usr/share/xgc2/robot-adapter-profiles/${DEFINITION_ID}.json"

dpkg-query -W -f='${Status}' "${PACKAGE_NAME}" | grep -qx 'install ok installed'
installed_version="$(dpkg-query -W -f='${Version}' "${PACKAGE_NAME}")"
if [[ "${installed_version}" != "${EXPECTED_PRODUCT_VERSION}" ]]; then
  echo "installed B2 Adapter version mismatch: expected ${EXPECTED_PRODUCT_VERSION}, got ${installed_version}" >&2
  exit 1
fi

test -x "${EXECUTABLE}"
ldd "${EXECUTABLE}" | grep -q 'libxgc2_adapter_runtime_client'
test -f "${PREFIX}/share/${ROS_PACKAGE}/launch/unitree_b2_ros1_adapter.launch"
test ! -e "${PREFIX}/share/${ROS_PACKAGE}/launch/unitree_b2_visualization_runtime.launch"
test -f "${PROFILE}"
test -f "${PROFILE_SCHEMA}"
test -f "${REGISTRY}"
test -f "${ADAPTER_MANIFEST}"
test -f "${PROCESS_MANIFEST}"
test -f "${PROFILE_CATALOG}"

if grep -n -E '/tmp/|/home/|\.worktrees/' \
  "${ADAPTER_MANIFEST}" "${PROCESS_MANIFEST}" "${PROFILE_CATALOG}"; then
  echo "installed Unitree B2 manifests contain a source-development path" >&2
  exit 1
fi

python3 "${REPO_ROOT}/tools/verify_runtime_manifests.py" \
  --executable "${EXECUTABLE}" \
  --ros-package "${ROS_PACKAGE}" \
  --ros-executable "${ROS_PACKAGE}_node" \
  --definition-id "${DEFINITION_ID}" \
  --registry "${REGISTRY}" \
  --profile-file "${PROFILE}" \
  --profile-schema "${PROFILE_SCHEMA}" \
  --adapter-manifest "${ADAPTER_MANIFEST}" \
  --process-manifest "${PROCESS_MANIFEST}" \
  --profile-catalog "${PROFILE_CATALOG}"

echo "Installed B2 Adapter gate passed: ${PACKAGE_NAME}=${installed_version}"
