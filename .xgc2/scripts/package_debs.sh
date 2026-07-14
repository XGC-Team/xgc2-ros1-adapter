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

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' \
    "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"
ADAPTER_LINK_CLIENT_DEB_VERSION="${ADAPTER_LINK_CLIENT_DEB_VERSION:-0.1.0-1~focal}"

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
  "${OUTPUT_DIR}/${SCOUT_PACKAGE}_"*.deb

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
  local depends="$3"
  local summary="$4"
  local detail="$5"
  local pkg_root="${BUILD_DIR}/${package}"
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_package}" "${pkg_root}"

  if [[ ! -x "${pkg_root}${executable}" ]]; then
    echo "missing installed ${ros_package}_node executable" >&2
    exit 1
  fi

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
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
  cp "${REPO_ROOT}/README.md" "${pkg_root}/usr/share/doc/${package}/README.md"
  cp "${REPO_ROOT}/LICENSE" "${pkg_root}/usr/share/doc/${package}/copyright"

  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${pkg_root}/DEBIAN"
  chmod 0755 "${pkg_root}${executable}"

  fakeroot dpkg-deb --build "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

CLIENT_DEPENDENCY="libxgc2-adapter-link-client-dev (>= ${ADAPTER_LINK_CLIENT_DEB_VERSION})"

package_adapter \
  "${PX4_PACKAGE}" \
  "${PX4_ROS_PACKAGE}" \
  "${CLIENT_DEPENDENCY}, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-mavros-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs" \
  "XGC2 PX4 multirotor ROS1 semantic adapter" \
  "Maps all PX4 multirotors in one immutable AdapterPlan through MAVROS without Scout dependencies."

package_adapter \
  "${SCOUT_PACKAGE}" \
  "${SCOUT_ROS_PACKAGE}" \
  "${CLIENT_DEPENDENCY}, ros-${ROS_DISTRO}-nav-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-scout-msgs, ros-${ROS_DISTRO}-sensor-msgs" \
  "XGC2 Scout Mini ROS1 semantic adapter" \
  "Maps all Scout Mini robots in one immutable AdapterPlan without MAVROS dependencies."

find "${OUTPUT_DIR}" -maxdepth 1 -type f \
  \( -name "${PX4_PACKAGE}_*.deb" -o -name "${SCOUT_PACKAGE}_*.deb" \) \
  -print | sort
