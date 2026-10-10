#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PX4_PACKAGE="ros-${ROS_DISTRO}-xgc2-px4-multirotor-adapter"
PX4_ROS_PACKAGE="xgc_px4_multirotor_ros1_adapter"
SCOUT_PACKAGE="ros-${ROS_DISTRO}-xgc2-scout-mini-adapter"
SCOUT_ROS_PACKAGE="xgc_scout_mini_ros1_adapter"
MECANUM_PACKAGE="ros-${ROS_DISTRO}-xgc2-mecanum-ugv-adapter"
MECANUM_ROS_PACKAGE="xgc_mecanum_ugv_ros1_adapter"
XGC2_SOURCE_DIGEST="${XGC2_SOURCE_DIGEST:-}"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' \
    "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi
if [[ -n "${XGC2_SOURCE_DIGEST}" && ! "${XGC2_SOURCE_DIGEST}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "XGC2_SOURCE_DIGEST must be empty or 64 lowercase hex characters" >&2
  exit 1
fi

append_source_digest() {
  local control_file="$1"
  local source_digest="$2"
  [[ -z "${source_digest}" ]] \
    || printf 'X-XGC2-Source-Digest: %s\n' "${source_digest}" >>"${control_file}"
}

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
NATIVE_BRIDGE="${PREFIX}/lib/libros_io.so"
if [[ ! -f "${INSTALL_ROOT}${NATIVE_BRIDGE}" ]]; then
  echo "missing required installed native bridge: ${NATIVE_BRIDGE}" >&2
  exit 1
fi
file -b "${INSTALL_ROOT}${NATIVE_BRIDGE}" | grep -q '^ELF'
nm -D --defined-only "${INSTALL_ROOT}${NATIVE_BRIDGE}" | awk '$3 == "xgc_rt_plugin_v1" {found=1} END {exit !found}'
NATIVE_UTILITY_PATHS=(
  "/usr/lib/libxgc_ros_edge.so"
  "/usr/include/xgc-ros-io/ros_edge.hpp"
  "/usr/include/xgc-ros-io/ros_slice.hpp"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoHelpersConfig.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoHelpersConfigVersion.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoHelpersTargets.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoEdgeTargets.cmake"
  "/usr/share/cmake/XgcRosIoHelpers/XgcRosIoEdgeTargets-release.cmake"
)
for path in "${NATIVE_UTILITY_PATHS[@]}"; do
  [[ -f "${INSTALL_ROOT}${path}" ]] || {
    echo "missing required installed native utility: ${path}" >&2
    exit 1
  }
done
file -b "${INSTALL_ROOT}/usr/lib/libxgc_ros_edge.so" | grep -q '^ELF'
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f \
  "${OUTPUT_DIR}/${PX4_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/ros-${ROS_DISTRO}-xgc2-ros1-native-bridge_"*.deb \
  "${OUTPUT_DIR}/${SCOUT_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${MECANUM_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/ros-${ROS_DISTRO}-xgc2-ros1-tools-adapter_"*.deb

mkdir -p "${BUILD_DIR}/debian"
cat > "${BUILD_DIR}/debian/control" <<EOF
Source: xgc2-ros1-adapter
Section: misc
Priority: optional
Maintainer: XGC2 <apt@example.com>

Package: ${PX4_PACKAGE}
Architecture: any

Package: ros-${ROS_DISTRO}-xgc2-ros1-native-bridge
Architecture: any

Package: ${SCOUT_PACKAGE}
Architecture: any

Package: ${MECANUM_PACKAGE}
Architecture: any

Package: ros-${ROS_DISTRO}-xgc2-ros1-tools-adapter
Architecture: any

EOF

binary_dependencies() {
  local -a binaries=("$@")
  local -a options=()
  if [[ -n "${BRIDGE_PRIVATE_ROOT:-}" ]]; then
    options+=("-S${BRIDGE_PRIVATE_ROOT}" "-l${BRIDGE_PRIVATE_ROOT}/usr/lib"
      "-xros-${ROS_DISTRO}-xgc2-ros1-native-bridge")
  fi
  local binary
  local output
  local dependencies
  for binary in "${binaries[@]}"; do
    options+=("-e${binary}")
  done
  output="$(cd "${BUILD_DIR}" && dpkg-shlibdeps -O "${options[@]}")"
  dependencies="${output#shlibs:Depends=}"
  if [[ "${dependencies}" == "${output}" || -z "${dependencies}" ]]; then
    echo "dpkg-shlibdeps did not produce executable dependencies" >&2
    exit 1
  fi
  if grep -Eq '(^|, )(libxgc2-adapter-runtime-client-dev|xgc2-protobuf-dev)( |[(,]|$)' \
      <<<"${dependencies}"; then
    echo "shlibs dependencies leaked a build-only XGC2 package" >&2
    exit 1
  fi
  printf '%s\n' "${dependencies}"
}

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

package_adapter() {
  local package="$1"
  local ros_package="$2"
  local extra_depends="$3"
  local summary="$4"
  local detail="$5"
  local profile_file="$6"
  local definition_id="$7"
  local profile_schema_file="$8"
  local package_version="${VERSION}"
  local package_source_digest="${XGC2_SOURCE_DIGEST}"
  local pkg_root="${BUILD_DIR}/${package}"
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_package}" "${pkg_root}"
  if [[ "${package}" == "${PX4_PACKAGE}" ]]; then
    copy_path "${PREFIX_ROOT}/include/${ros_package}" "${pkg_root}"
    copy_path "${PREFIX_ROOT}/lib/lib${ros_package}_operations.a" "${pkg_root}"
    copy_path "${PREFIX_ROOT}/lib/pkgconfig/${ros_package}.pc" "${pkg_root}"
    test -f "${pkg_root}${PREFIX}/include/${ros_package}/px4_operations.hpp"
    test -f "${pkg_root}${PREFIX}/lib/lib${ros_package}_operations.a"
  fi
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/adapter-definitions/${definition_id}.json" "${pkg_root}"
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/${definition_id}.json" "${pkg_root}"
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json" "${pkg_root}"

  if [[ ! -x "${pkg_root}${executable}" ]]; then
    echo "missing installed ${ros_package}_node executable" >&2
    exit 1
  fi
  if [[ ! -f "${pkg_root}${PREFIX}/share/${ros_package}/profiles/ros1/${profile_file}" ]]; then
    echo "missing installed ${ros_package} native profile" >&2
    exit 1
  fi
  if [[ ! -f "${pkg_root}${PREFIX}/share/${ros_package}/profiles/schema/${profile_schema_file}" ]]; then
    echo "missing installed ${ros_package} profile schema" >&2
    exit 1
  fi
  for manifest in \
    "/usr/share/xgc2/adapter-definitions/${definition_id}.json" \
    "/usr/share/xgc2/process-definitions/${definition_id}.json" \
    "/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json"; do
    if [[ ! -f "${pkg_root}${manifest}" ]]; then
      echo "missing installed ${definition_id} manifest: ${manifest}" >&2
      exit 1
    fi
  done

  local -a runtime_binaries=("${pkg_root}${executable}")
  local shlibs_depends
  if [[ "${package}" == "${PX4_PACKAGE}" || "${package}" == "${SCOUT_PACKAGE}" || "${package}" == "${MECANUM_PACKAGE}" ]]; then
    shlibs_depends="$(binary_dependencies "${runtime_binaries[@]}")"
    if grep -Eq '(^|, )libxgc2-adapter-runtime-client[0-9]+( |[(])' <<<"${shlibs_depends}"; then
      echo "robot server unexpectedly links the legacy Adapter Runtime client" >&2; exit 1
    fi
  fi

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package}
Version: ${package_version}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${shlibs_depends}, ${extra_depends}
Description: ${summary}
 ${detail}
EOF
  append_source_digest "${pkg_root}/DEBIAN/control" "${package_source_digest}"
  cp "${REPO_ROOT}/README.md" "${pkg_root}/usr/share/doc/${package}/README.md"
  cp "${REPO_ROOT}/LICENSE" "${pkg_root}/usr/share/doc/${package}/copyright"

  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${pkg_root}/DEBIAN"
  chmod 0755 "${pkg_root}${executable}"

  fakeroot dpkg-deb --build "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${package_version}_${ARCH}.deb" >/dev/null
}


package_adapter \
  "${PX4_PACKAGE}" \
  "${PX4_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-mavros-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs" \
  "XGC2 PX4 multirotor ROS1 semantic adapter" \
  "Provides PX4 multirotor telemetry, diagnostics, and native command capabilities." \
  "px4-multirotor-physical-vrpn.yaml" \
  "xgc2-px4-multirotor-ros1-adapter" \
  "robot-adapter-profile-v4.schema.json"

bridge_package="ros-${ROS_DISTRO}-xgc2-ros1-native-bridge"
bridge_root="${BUILD_DIR}/${bridge_package}"
mkdir -p "${bridge_root}/DEBIAN"
copy_path "${INSTALL_ROOT}${NATIVE_BRIDGE}" "${bridge_root}"
for path in "${NATIVE_UTILITY_PATHS[@]}"; do
  copy_path "${INSTALL_ROOT}${path}" "${bridge_root}"
done
cat > "${bridge_root}/DEBIAN/control" <<EOF
Package: ${bridge_package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Description: XGC2 owning native ROS1 sensor and physical FCU edge
EOF
bridge_depends="$(BRIDGE_PRIVATE_ROOT="${bridge_root}" binary_dependencies \
  "${bridge_root}${NATIVE_BRIDGE}" "${bridge_root}/usr/lib/libxgc_ros_edge.so")"
printf '%s\n' "Depends: ${bridge_depends}, ros-${ROS_DISTRO}-roscpp, libxgc2-runtime-sdk-dev (>= 0.1.0-2~focal), libxgc2-robotics-interfaces-dev (>= 0.1.0-1~focal), libxgc2-hover-thrust-dev, ros-${ROS_DISTRO}-xgc2-estimator-rigid-state, ros-${ROS_DISTRO}-xgc2-multirotor-controller" >>"${bridge_root}/DEBIAN/control"
append_source_digest "${bridge_root}/DEBIAN/control" "${XGC2_SOURCE_DIGEST}"
fakeroot dpkg-deb --build "${bridge_root}" \
  "${OUTPUT_DIR}/${bridge_package}_${VERSION}_${ARCH}.deb" >/dev/null

package_adapter \
  "${SCOUT_PACKAGE}" \
  "${SCOUT_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-scout-msgs, ros-${ROS_DISTRO}-sensor-msgs" \
  "XGC2 Scout Mini ROS1 semantic adapter" \
  "Provides Scout Mini VRPN acceleration telemetry, discrete motion control, and channel-diagnostic capabilities." \
  "scout-mini-physical-vrpn.yaml" \
  "xgc2-scout-mini-ros1-adapter" \
  "robot-adapter-profile-v4.schema.json"

package_adapter \
  "${MECANUM_PACKAGE}" \
  "${MECANUM_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp" \
  "XGC2 Mecanum UGV ROS1 semantic adapter" \
  "Provides Mecanum UGV VRPN acceleration telemetry, discrete motion control, and channel-diagnostic capabilities." \
  "mecanum-ugv-physical-vrpn.yaml" \
  "xgc2-mecanum-ugv-ros1-adapter" \
  "robot-adapter-profile-v4.schema.json"


TOOLS_PACKAGE="ros-${ROS_DISTRO}-xgc2-ros1-tools-adapter"
tools_root="${BUILD_DIR}/${TOOLS_PACKAGE}"
test -f "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/xgc2-ros1-tools-adapter.json"
copy_path "${PREFIX_ROOT}/share/xgc_ros1_tools_adapter" "${tools_root}"
copy_path "${PREFIX_ROOT}/lib/xgc_ros1_tools_adapter" "${tools_root}"
copy_path "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/xgc2-ros1-tools-adapter.json" "${tools_root}"
mkdir -p "${tools_root}/DEBIAN"
tools_depends="$(binary_dependencies "${tools_root}${PREFIX}/lib/xgc_ros1_tools_adapter/xgc_ros1_tools_adapter_node" "${tools_root}${PREFIX}/lib/xgc_ros1_tools_adapter/xgc_ros1_tools_adapter_service_helper" "${tools_root}${PREFIX}/lib/xgc_ros1_tools_adapter/xgc_ros1_tools_adapter_probe" "${tools_root}${PREFIX}/lib/xgc_ros1_tools_adapter/xgc_ros1_tools_adapter_clock_wait")"
cat > "${tools_root}/DEBIAN/control" <<EOF
Package: ${TOOLS_PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${tools_depends}, python3, ros-${ROS_DISTRO}-rosbag, ros-${ROS_DISTRO}-rospy, ros-${ROS_DISTRO}-ros-babel-fish, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-roslib, ros-${ROS_DISTRO}-rosgraph-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-std-srvs
Description: Typed ROS1 Tools XRPC service and native ROS helpers
EOF
append_source_digest "${tools_root}/DEBIAN/control" "${XGC2_SOURCE_DIGEST}"
find "${tools_root}" -type d -exec chmod 0755 {} +
find "${tools_root}" -type f -exec chmod 0644 {} +
for name in node service_helper probe clock_wait rosbag_recorder; do
  chmod 0755 "${tools_root}${PREFIX}/lib/xgc_ros1_tools_adapter/xgc_ros1_tools_adapter_${name}"
done
fakeroot dpkg-deb --build "${tools_root}" "${OUTPUT_DIR}/${TOOLS_PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null

find "${OUTPUT_DIR}" -maxdepth 1 -type f \
  \( -name "${PX4_PACKAGE}_*.deb" -o -name "${SCOUT_PACKAGE}_*.deb" \
    -o -name "ros-${ROS_DISTRO}-xgc2-ros1-native-bridge_*.deb" \
    -o -name "${MECANUM_PACKAGE}_*.deb" -o -name "${TOOLS_PACKAGE}_*.deb" \) \
  -print | sort
