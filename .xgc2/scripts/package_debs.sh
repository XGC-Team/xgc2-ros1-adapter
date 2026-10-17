#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

COMMON_PACKAGE="ros-${ROS_DISTRO}-xgc2-ros1-adapter-common"
COMMON_ROS_PACKAGE="xgc_ros1_adapter_common"
PX4_PACKAGE="ros-${ROS_DISTRO}-xgc2-px4-multirotor-adapter"
PX4_ROS_PACKAGE="xgc_px4_multirotor_ros1_adapter"
UGV_PACKAGE="ros-${ROS_DISTRO}-xgc2-ugv-adapter"
UGV_ROS_PACKAGE="xgc_ugv_ros1_adapter"
TOOLS_PACKAGE="ros-${ROS_DISTRO}-xgc2-ros1-tools-adapter"
TOOLS_ROS_PACKAGE="xgc_ros1_tools_adapter"
SCENE_PACKAGE="ros-${ROS_DISTRO}-xgc2-scene-adapter"
SCENE_ROS_PACKAGE="xgc_scene_ros1_adapter"
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
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f \
  "${OUTPUT_DIR}/${COMMON_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${PX4_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${UGV_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${TOOLS_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${SCENE_PACKAGE}_"*.deb

mkdir -p "${BUILD_DIR}/debian"
cat > "${BUILD_DIR}/debian/control" <<EOF
Source: xgc2-ros1-adapter
Section: misc
Priority: optional
Maintainer: XGC2 <apt@example.com>

Package: ${COMMON_PACKAGE}
Architecture: any

Package: ${PX4_PACKAGE}
Architecture: any

Package: ${UGV_PACKAGE}
Architecture: any

Package: ${TOOLS_PACKAGE}
Architecture: any

Package: ${SCENE_PACKAGE}
Architecture: any

EOF

binary_dependencies() {
  local -a binaries=("$@")
  local -a options=()
  local binary
  local output
  local dependencies
  for binary in "${binaries[@]}"; do
    options+=("-e${binary}")
  done
  # The libraries of the common package are not installed yet: look into its
  # build tree, which carries the shlibs file that names the package.
  if [[ -d "${BUILD_DIR}/${COMMON_PACKAGE}/DEBIAN" ]]; then
    options+=("-S${BUILD_DIR}/${COMMON_PACKAGE}")
  fi
  # A package of libraries does not depend on itself.
  if [[ -n "${EXCLUDE_PACKAGE:-}" ]]; then
    options+=("-x${EXCLUDE_PACKAGE}")
  fi
  output="$(cd "${BUILD_DIR}" && dpkg-shlibdeps -O "${options[@]}")"
  dependencies="${output#shlibs:Depends=}"
  if [[ "${dependencies}" == "${output}" || -z "${dependencies}" ]]; then
    echo "dpkg-shlibdeps did not produce executable dependencies" >&2
    exit 1
  fi
  if grep -Eq '(^|, )(libxgc2-xrpc-dev|libxgc2-xrpc-grpc-dev|xgc2-protobuf-dev)( |[(,]|$)' \
      <<<"${dependencies}"; then
    echo "shlibs dependencies leaked a build-only XGC2 package" >&2
    exit 1
  fi
  printf '%s\n' "${dependencies}"
}

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" || -L "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local summary="$4"
  local detail="$5"
  mkdir -p "${pkg_root}/DEBIAN"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
Description: ${summary}
 ${detail}
EOF
  append_source_digest "${pkg_root}/DEBIAN/control" "${XGC2_SOURCE_DIGEST}"
}

build_deb() {
  local pkg_root="$1"
  local package="$2"
  local doc_dir="${pkg_root}/usr/share/doc/${package}"
  mkdir -p "${doc_dir}"
  cp "${REPO_ROOT}/README.md" "${doc_dir}/README.md"
  cp -r "${REPO_ROOT}/docs" "${doc_dir}/docs"
  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${pkg_root}/DEBIAN"
}

# The shared libraries of every type server. The executables are linked against
# them, so this package is built first and names its libraries in a shlibs file:
# the type server packages depend on exactly this version, through shlibs.
package_common() {
  local pkg_root="${BUILD_DIR}/${COMMON_PACKAGE}"
  local library
  local path
  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/${COMMON_ROS_PACKAGE}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/${COMMON_ROS_PACKAGE}" "${pkg_root}"
  for library in xgc2_robot_server xgc2_robot_server_ros1; do
    # lib<name>.so is a development symlink and stays out of the package
    for path in "${PREFIX_ROOT}/lib/lib${library}.so."*; do
      copy_path "${path}" "${pkg_root}"
    done
    if [[ ! -e "${pkg_root}${PREFIX}/lib/lib${library}.so.0" ]]; then
      echo "missing installed lib${library}.so.0" >&2
      exit 1
    fi
  done
  test -f "${pkg_root}${PREFIX}/share/${COMMON_ROS_PACKAGE}/package.xml"
  mkdir -p "${pkg_root}/DEBIAN"
  {
    printf 'libxgc2_robot_server 0 %s (= %s)\n' "${COMMON_PACKAGE}" "${VERSION}"
    printf 'libxgc2_robot_server_ros1 0 %s (= %s)\n' "${COMMON_PACKAGE}" "${VERSION}"
  } > "${pkg_root}/DEBIAN/shlibs"
  local shlibs_depends
  shlibs_depends="$(EXCLUDE_PACKAGE="${COMMON_PACKAGE}" binary_dependencies \
    "${pkg_root}${PREFIX}/lib/libxgc2_robot_server.so.0" \
    "${pkg_root}${PREFIX}/lib/libxgc2_robot_server_ros1.so.0")"
  write_control "${pkg_root}" "${COMMON_PACKAGE}" \
    "${shlibs_depends}, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp, python3" \
    "XGC2 ROS1 robot type server libraries" \
    "Shared server core and ROS1 binding of the XGC2 robot type servers."
  build_deb "${pkg_root}" "${COMMON_PACKAGE}"
  chmod 0755 "${pkg_root}${PREFIX}/lib/${COMMON_ROS_PACKAGE}/xgc2-project-robot-assets"
  fakeroot dpkg-deb --build "${pkg_root}" \
    "${OUTPUT_DIR}/${COMMON_PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
}

# One robot type server package: its executable, its profiles and catalogs, and
# the process definitions of every robot type it serves.
package_type_server() {
  local package="$1"
  local ros_package="$2"
  local extra_depends="$3"
  local summary="$4"
  local detail="$5"
  shift 5
  local pkg_root="${BUILD_DIR}/${package}"
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"
  local definition_id
  local manifest

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_package}" "${pkg_root}"
  if [[ ! -x "${pkg_root}${executable}" ]]; then
    echo "missing installed ${ros_package}_node executable" >&2
    exit 1
  fi
  if [[ ! -f "${pkg_root}${PREFIX}/share/${ros_package}/profiles/schema/robot-adapter-profile-v4.schema.json" ]]; then
    echo "missing installed ${ros_package} profile schema" >&2
    exit 1
  fi
  for definition_id in "$@"; do
    copy_path "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/${definition_id}.json" "${pkg_root}"
    copy_path "${INSTALL_ROOT}/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json" "${pkg_root}"
    for manifest in \
      "/usr/share/xgc2/process-definitions/${definition_id}.json" \
      "/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json"; do
      if [[ ! -f "${pkg_root}${manifest}" ]]; then
        echo "missing installed ${definition_id} file: ${manifest}" >&2
        exit 1
      fi
    done
  done

  local shlibs_depends
  shlibs_depends="$(binary_dependencies "${pkg_root}${executable}")"
  write_control "${pkg_root}" "${package}" \
    "${shlibs_depends}, ${extra_depends}" \
    "${summary}" "${detail}"
  build_deb "${pkg_root}" "${package}"
  chmod 0755 "${pkg_root}${executable}"
  fakeroot dpkg-deb --build "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

package_tools() {
  local tools_root="${BUILD_DIR}/${TOOLS_PACKAGE}"
  local library_root="${PREFIX}/lib/${TOOLS_ROS_PACKAGE}"
  local name
  test -f "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/xgc2-ros1-tools-adapter.json"
  copy_path "${PREFIX_ROOT}/share/${TOOLS_ROS_PACKAGE}" "${tools_root}"
  copy_path "${PREFIX_ROOT}/lib/${TOOLS_ROS_PACKAGE}" "${tools_root}"
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/xgc2-ros1-tools-adapter.json" "${tools_root}"
  local tools_depends
  tools_depends="$(binary_dependencies \
    "${tools_root}${library_root}/${TOOLS_ROS_PACKAGE}_node" \
    "${tools_root}${library_root}/${TOOLS_ROS_PACKAGE}_service_helper" \
    "${tools_root}${library_root}/${TOOLS_ROS_PACKAGE}_probe" \
    "${tools_root}${library_root}/${TOOLS_ROS_PACKAGE}_clock_wait")"
  write_control "${tools_root}" "${TOOLS_PACKAGE}" \
    "${tools_depends}, python3, ros-${ROS_DISTRO}-rosbag, ros-${ROS_DISTRO}-rospy, ros-${ROS_DISTRO}-ros-babel-fish, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-roslib, ros-${ROS_DISTRO}-rosgraph-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-std-srvs" \
    "Typed ROS1 Tools XRPC service and native ROS helpers" \
    "Run-owned ROS1 Tools process: typed topic publishing and service calls over XRPC http.v1."
  build_deb "${tools_root}" "${TOOLS_PACKAGE}"
  for name in node service_helper probe clock_wait rosbag_recorder; do
    chmod 0755 "${tools_root}${library_root}/${TOOLS_ROS_PACKAGE}_${name}"
  done
  fakeroot dpkg-deb --build "${tools_root}" \
    "${OUTPUT_DIR}/${TOOLS_PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
}

package_scene() {
  local scene_root="${BUILD_DIR}/${SCENE_PACKAGE}"
  local library_root="${PREFIX}/lib/${SCENE_ROS_PACKAGE}"
  local definition="/usr/share/xgc2/process-definitions/xgc2-ros1-scene-adapter.json"
  local name
  local -a binaries=()
  test -f "${PREFIX_ROOT}/share/${SCENE_ROS_PACKAGE}/package.xml"
  test -f "${INSTALL_ROOT}${definition}"
  copy_path "${PREFIX_ROOT}/share/${SCENE_ROS_PACKAGE}" "${scene_root}"
  copy_path "${PREFIX_ROOT}/lib/${SCENE_ROS_PACKAGE}" "${scene_root}"
  copy_path "${INSTALL_ROOT}${definition}" "${scene_root}"
  # The installed ROS-free Core target is consumed by native replay tools.
  test -f "${PREFIX_ROOT}/lib/libxgc_scene_core.a"
  test -f "${PREFIX_ROOT}/lib/cmake/XgcSceneCore/XgcSceneCoreConfig.cmake"
  test -f "${PREFIX_ROOT}/lib/cmake/XgcSceneCore/XgcSceneCoreTargets.cmake"
  test -f "${PREFIX_ROOT}/include/xgc_scene/document.hpp"
  copy_path "${PREFIX_ROOT}/lib/libxgc_scene_core.a" "${scene_root}"
  copy_path "${PREFIX_ROOT}/lib/cmake/XgcSceneCore" "${scene_root}"
  local header
  for header in "${PREFIX_ROOT}/include/xgc_scene/"*.hpp; do
    copy_path "${header}" "${scene_root}"
  done
  for name in xgc2_scene_adapter xgc2-scene-assemble xgc2-scene-prepare; do
    test -x "${scene_root}${library_root}/${name}"
    binaries+=("${scene_root}${library_root}/${name}")
  done
  local scene_depends
  scene_depends="$(binary_dependencies "${binaries[@]}")"
  write_control "${scene_root}" "${SCENE_PACKAGE}" \
    "${scene_depends}, libxgc2-xrpc2 (>= 0.2.0-1~focal), libxgc2-math-dev (>= 0.6.0-6~focal), nlohmann-json3-dev, libyaml-cpp-dev, libssl-dev, libassimp-dev, libpugixml-dev, libsdformat9-dev, pkg-config, python3, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-tf2-ros, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-visualization-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-xgc2-geometry-msgs (>= 1.5.0-4)" \
    "XGC2 ROS1 scene authoring service and finite scene tools" \
    "Run-owned scene authoring, visualization and simulator application over XRPC http.v1."
  build_deb "${scene_root}" "${SCENE_PACKAGE}"
  for name in xgc2_scene_adapter xgc2-scene-assemble xgc2-scene-prepare; do
    chmod 0755 "${scene_root}${library_root}/${name}"
  done
  fakeroot dpkg-deb --build "${scene_root}" \
    "${OUTPUT_DIR}/${SCENE_PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
}

package_common

package_type_server \
  "${PX4_PACKAGE}" \
  "${PX4_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-mavros-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs" \
  "XGC2 PX4 multirotor ROS1 type server" \
  "Run-owned type server for PX4 multirotors: telemetry, diagnostics, and native command capabilities." \
  "xgc2-px4-multirotor-ros1-adapter"

package_type_server \
  "${UGV_PACKAGE}" \
  "${UGV_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs" \
  "XGC2 Scout Mini and Mecanum UGV ROS1 type servers" \
  "Run-owned type servers for Scout Mini and Mecanum UGVs: VRPN acceleration telemetry, discrete motion control, and channel-diagnostic capabilities." \
  "xgc2-scout-mini-ros1-adapter" \
  "xgc2-mecanum-ugv-ros1-adapter"

package_tools
package_scene

find "${OUTPUT_DIR}" -maxdepth 1 -type f \
  \( -name "${COMMON_PACKAGE}_*.deb" -o -name "${PX4_PACKAGE}_*.deb" \
    -o -name "${UGV_PACKAGE}_*.deb" -o -name "${TOOLS_PACKAGE}_*.deb" \
    -o -name "${SCENE_PACKAGE}_*.deb" \) \
  -print | sort
