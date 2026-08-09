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
B2_PACKAGE="ros-${ROS_DISTRO}-xgc2-unitree-b2-adapter"
B2_ROS_PACKAGE="xgc_unitree_b2_ros1_adapter"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' \
    "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"
ADAPTER_RUNTIME_ABI_PACKAGE="libxgc2-adapter-runtime-client2"

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
  "${OUTPUT_DIR}/${PX4_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${SCOUT_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${MECANUM_PACKAGE}_"*.deb \
  "${OUTPUT_DIR}/${B2_PACKAGE}_"*.deb

mkdir -p "${BUILD_DIR}/debian"
cat > "${BUILD_DIR}/debian/control" <<EOF
Source: xgc2-ros1-adapter
Section: misc
Priority: optional
Maintainer: XGC2 <apt@example.com>

Package: ${PX4_PACKAGE}
Architecture: any

Package: ${SCOUT_PACKAGE}
Architecture: any

Package: ${MECANUM_PACKAGE}
Architecture: any

Package: ${B2_PACKAGE}
Architecture: any
EOF

shlibs_dependencies() {
  local -a binaries=("$@")
  local -a options=()
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
  if ! grep -Eq "(^|, )${ADAPTER_RUNTIME_ABI_PACKAGE}( |[(])" \
      <<<"${dependencies}"; then
    echo "shlibs dependencies do not include ${ADAPTER_RUNTIME_ABI_PACKAGE}" >&2
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
  local helper_name="${9:-}"
  local pkg_root="${BUILD_DIR}/${package}"
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"
  local helper_executable=""
  if [[ -n "${helper_name}" ]]; then
    helper_executable="${PREFIX}/lib/${ros_package}/${helper_name}"
  fi

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_package}" "${pkg_root}"
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/adapter-definitions/${definition_id}.json" "${pkg_root}"
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/${definition_id}.json" "${pkg_root}"
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/robot-adapter-profiles/${definition_id}.json" "${pkg_root}"

  if [[ ! -x "${pkg_root}${executable}" ]]; then
    echo "missing installed ${ros_package}_node executable" >&2
    exit 1
  fi
  if [[ -n "${helper_executable}" && ! -x "${pkg_root}${helper_executable}" ]]; then
    echo "missing installed ${ros_package} native service helper" >&2
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
  if [[ -n "${helper_executable}" ]]; then
    runtime_binaries+=("${pkg_root}${helper_executable}")
  fi
  local shlibs_depends
  shlibs_depends="$(shlibs_dependencies "${runtime_binaries[@]}")"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${shlibs_depends}, ${extra_depends}
Description: ${summary}
 ${detail}
EOF
  cp "${REPO_ROOT}/README.md" "${pkg_root}/usr/share/doc/${package}/README.md"
  cp "${REPO_ROOT}/LICENSE" "${pkg_root}/usr/share/doc/${package}/copyright"

  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${pkg_root}/DEBIAN"
  chmod 0755 "${pkg_root}${executable}"
  if [[ -n "${helper_executable}" ]]; then
    chmod 0755 "${pkg_root}${helper_executable}"
  fi

  fakeroot dpkg-deb --build "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

package_adapter \
  "${PX4_PACKAGE}" \
  "${PX4_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-mavros-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs" \
  "XGC2 PX4 multirotor ROS1 semantic adapter" \
  "Provides PX4 multirotor telemetry, diagnostics, and native command capabilities." \
  "px4-multirotor-ros1-v7.yaml" \
  "xgc2-px4-multirotor-ros1-adapter" \
  "robot-adapter-profile-v4.schema.json" \
  "xgc_px4_multirotor_ros1_adapter_service_helper"

package_adapter \
  "${SCOUT_PACKAGE}" \
  "${SCOUT_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-scout-msgs, ros-${ROS_DISTRO}-sensor-msgs" \
  "XGC2 Scout Mini ROS1 semantic adapter" \
  "Provides Scout Mini telemetry, discrete motion control, and channel-diagnostic capabilities." \
  "scout-mini-ros1-v6.yaml" \
  "xgc2-scout-mini-ros1-adapter" \
  "robot-adapter-profile-v4.schema.json"

package_adapter \
  "${MECANUM_PACKAGE}" \
  "${MECANUM_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp" \
  "XGC2 Mecanum UGV ROS1 semantic adapter" \
  "Provides Mecanum UGV VRPN telemetry, discrete motion control, and channel-diagnostic capabilities." \
  "mecanum-ugv-ros1-v3.yaml" \
  "xgc2-mecanum-ugv-ros1-adapter" \
  "robot-adapter-profile-v4.schema.json"

package_adapter \
  "${B2_PACKAGE}" \
  "${B2_ROS_PACKAGE}" \
  "ros-${ROS_DISTRO}-diagnostic-msgs, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-nav-msgs, ros-${ROS_DISTRO}-robot-state-publisher, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-tf2-ros, ros-${ROS_DISTRO}-xgc2-b2arx-description" \
  "XGC2 Unitree B2 ROS1 read-only semantic adapter" \
  "Provides bounded B2 wire decode, semantic projection, freshness, and ROS1/TF recovery without motion commands." \
  "unitree-b2-v1.yaml" \
  "xgc2-unitree-b2-ros1-adapter" \
  "robot-adapter-profile-v4.schema.json"

find "${OUTPUT_DIR}" -maxdepth 1 -type f \
  \( -name "${PX4_PACKAGE}_*.deb" -o -name "${SCOUT_PACKAGE}_*.deb" \
    -o -name "${MECANUM_PACKAGE}_*.deb" -o -name "${B2_PACKAGE}_*.deb" \) \
  -print | sort
