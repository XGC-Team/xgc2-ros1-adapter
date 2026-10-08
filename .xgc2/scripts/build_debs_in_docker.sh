#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.3}"
DOCKER_PLATFORM="${DOCKER_PLATFORM:-}"
DOCKER_NETWORK="${DOCKER_NETWORK:-}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
COPY_OUTPUT="${COPY_OUTPUT:-true}"
BUILD_JOBS="${BUILD_JOBS:-}"
EXPECTED_RUNTIME_CLIENT_PRODUCT_VERSION="0.7.0-1"
EXPECTED_RUNTIME_CLIENT_DEB_VERSION="${EXPECTED_RUNTIME_CLIENT_PRODUCT_VERSION}~focal"
PINNED_RUNTIME_CLIENT_SHA="697d99b11ff90948e20872947706546c6205c9a9"
ADAPTER_RUNTIME_CLIENT_DEB_VERSION="${ADAPTER_RUNTIME_CLIENT_DEB_VERSION:-${EXPECTED_RUNTIME_CLIENT_DEB_VERSION}}"
XGC2_ADAPTER_RUNTIME_CLIENT_GIT_REF="${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_REF:-${PINNED_RUNTIME_CLIENT_SHA}}"
EXPECTED_PROTOBUF_PRODUCT_VERSION="0.6.0-1"
EXPECTED_PROTOBUF_DEB_VERSION="${EXPECTED_PROTOBUF_PRODUCT_VERSION}~focal"
PINNED_PROTOBUF_SHA="9ceeb01cc2de0369ed0956a010fad2d85424bc20"
XGC2_PROTOBUF_DEB_VERSION="${XGC2_PROTOBUF_DEB_VERSION:-${EXPECTED_PROTOBUF_DEB_VERSION}}"
XGC2_PROTOBUF_GIT_REF="${XGC2_PROTOBUF_GIT_REF:-${PINNED_PROTOBUF_SHA}}"
XGC2_BOOTSTRAP_COMMON_FROM_GIT="${XGC2_BOOTSTRAP_COMMON_FROM_GIT:-}"
XGC2_PROTOBUF_SOURCE_ROOT="${XGC2_PROTOBUF_SOURCE_ROOT:-}"
XGC2_DEPENDENCY_SET_DIGEST="${XGC2_DEPENDENCY_SET_DIGEST:-}"
PACKAGE_VERSION="${PACKAGE_VERSION:-}"
XGC2_SOURCE_DIGEST="${XGC2_SOURCE_DIGEST:-}"
MOCAP_ADAPTER_PACKAGE_VERSION="${MOCAP_ADAPTER_PACKAGE_VERSION:-}"
MOCAP_ADAPTER_SOURCE_DIGEST="${MOCAP_ADAPTER_SOURCE_DIGEST:-}"
EMPTY_DEPENDENCY_SET_DIGEST="4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --platform)
      DOCKER_PLATFORM="$2"
      shift 2
      ;;
    --network)
      DOCKER_NETWORK="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --jobs)
      BUILD_JOBS="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    --skip-output-copy)
      COPY_OUTPUT=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -n "${BUILD_JOBS}" && ! "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--jobs must be a positive integer" >&2
  exit 1
fi
if [[ -n "${XGC2_DEPENDENCY_SET_DIGEST}" &&
      ! "${XGC2_DEPENDENCY_SET_DIGEST}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "XGC2_DEPENDENCY_SET_DIGEST must be empty or 64 lowercase hex characters" >&2
  exit 1
fi
if [[ -n "${XGC2_SOURCE_DIGEST}" && ! "${XGC2_SOURCE_DIGEST}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "XGC2_SOURCE_DIGEST must be empty or 64 lowercase hex characters" >&2
  exit 1
fi
if [[ -n "${MOCAP_ADAPTER_SOURCE_DIGEST}" && ! "${MOCAP_ADAPTER_SOURCE_DIGEST}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "MOCAP_ADAPTER_SOURCE_DIGEST must be empty or 64 lowercase hex characters" >&2
  exit 1
fi
if [[ -n "${MOCAP_ADAPTER_PACKAGE_VERSION}" && -z "${MOCAP_ADAPTER_SOURCE_DIGEST}" ]] ||
   [[ -z "${MOCAP_ADAPTER_PACKAGE_VERSION}" && -n "${MOCAP_ADAPTER_SOURCE_DIGEST}" ]]; then
  echo "MOCAP_ADAPTER_PACKAGE_VERSION and MOCAP_ADAPTER_SOURCE_DIGEST must be set together" >&2
  exit 1
fi
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" && -z "${XGC2_DEPENDENCY_SET_DIGEST}" ]]; then
  echo "XGC2_APT_OVERLAY_URL requires XGC2_DEPENDENCY_SET_DIGEST" >&2
  exit 1
fi
if [[ "${XGC2_PROTOBUF_DEB_VERSION}" != "${EXPECTED_PROTOBUF_DEB_VERSION}" ||
      "${XGC2_PROTOBUF_GIT_REF}" != "${PINNED_PROTOBUF_SHA}" ]]; then
  echo "Adapter requires ${EXPECTED_PROTOBUF_DEB_VERSION} from protobuf ${PINNED_PROTOBUF_SHA}" >&2
  exit 1
fi
if [[ "${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}" != "${EXPECTED_RUNTIME_CLIENT_DEB_VERSION}" ||
      "${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_REF}" != "${PINNED_RUNTIME_CLIENT_SHA}" ]]; then
  echo "Adapter requires client ${EXPECTED_RUNTIME_CLIENT_DEB_VERSION} from ${PINNED_RUNTIME_CLIENT_SHA}" >&2
  exit 1
fi
if [[ -n "${XGC2_PROTOBUF_SOURCE_ROOT}" ]]; then
  [[ "$(git -C "${XGC2_PROTOBUF_SOURCE_ROOT}" rev-parse HEAD)" == "${PINNED_PROTOBUF_SHA}" &&
     -z "$(git -C "${XGC2_PROTOBUF_SOURCE_ROOT}" status --porcelain)" ]] || {
    echo "XGC2_PROTOBUF_SOURCE_ROOT must be the clean pinned protobuf source" >&2
    exit 1
  }
fi

if [[ -z "${XGC2_BOOTSTRAP_COMMON_FROM_GIT}" ]]; then
  if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
    XGC2_BOOTSTRAP_COMMON_FROM_GIT=false
  else
    XGC2_BOOTSTRAP_COMMON_FROM_GIT=true
  fi
fi
case "${XGC2_BOOTSTRAP_COMMON_FROM_GIT}" in
  true|false) ;;
  *)
    echo "XGC2_BOOTSTRAP_COMMON_FROM_GIT must be true or false" >&2
    exit 1
    ;;
esac
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" && "${XGC2_BOOTSTRAP_COMMON_FROM_GIT}" == "true" ]]; then
  echo "Release-train overlay must use the staged protobuf and client packages" >&2
  exit 1
fi
expected_deb_arch=""
case "${DOCKER_PLATFORM}" in
  "")
    ;;
  linux/amd64)
    expected_deb_arch="amd64"
    ;;
  linux/arm64|linux/arm64/v8)
    expected_deb_arch="arm64"
    ;;
  *)
    echo "unsupported Docker platform: ${DOCKER_PLATFORM}" >&2
    exit 1
    ;;
esac

if [[ "${COPY_OUTPUT}" == "true" ]]; then
  mkdir -p "${OUTPUT_DIR}"
  rm -f \
    "${OUTPUT_DIR}/ros-noetic-xgc2-px4-multirotor-adapter_"*.deb \
    "${OUTPUT_DIR}/ros-noetic-xgc2-scout-mini-adapter_"*.deb \
    "${OUTPUT_DIR}/ros-noetic-xgc2-mecanum-ugv-adapter_"*.deb \
    "${OUTPUT_DIR}/ros-noetic-xgc2-unitree-b2-adapter_"*.deb \
    "${OUTPUT_DIR}/ros-noetic-xgc2-mocap-rotor-adapter_"*.deb \
    "${OUTPUT_DIR}/ros-noetic-xgc2-mocap-rotor-forwarder_"*.deb
fi

docker_network_args=()
if [[ -n "${DOCKER_NETWORK}" ]]; then
  docker_network_args=(--network "${DOCKER_NETWORK}")
fi

docker_platform_args=()
if [[ -n "${DOCKER_PLATFORM}" ]]; then
  docker_platform_args=(--platform "${DOCKER_PLATFORM}")
fi

docker_env_args=(
  -e "PACKAGE_VERSION=${PACKAGE_VERSION}"
  -e "XGC2_SOURCE_DIGEST=${XGC2_SOURCE_DIGEST}"
  -e "MOCAP_ADAPTER_PACKAGE_VERSION=${MOCAP_ADAPTER_PACKAGE_VERSION}"
  -e "MOCAP_ADAPTER_SOURCE_DIGEST=${MOCAP_ADAPTER_SOURCE_DIGEST}"
  -e "XGC2_APT_OVERLAY_URL=${XGC2_APT_OVERLAY_URL:-}"
  -e "XGC2_DEPENDENCY_SET_DIGEST=${XGC2_DEPENDENCY_SET_DIGEST}"
  -e "EMPTY_DEPENDENCY_SET_DIGEST=${EMPTY_DEPENDENCY_SET_DIGEST}"
  -e "BUILD_JOBS=${BUILD_JOBS}"
  -e "DEBIAN_FRONTEND=noninteractive"
  -e "EXPECTED_DEB_ARCH=${expected_deb_arch}"
  -e "INSTALL_CHECK=${INSTALL_CHECK}"
  -e "ADAPTER_RUNTIME_CLIENT_DEB_VERSION=${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}"
  -e "EXPECTED_RUNTIME_CLIENT_PRODUCT_VERSION=${EXPECTED_RUNTIME_CLIENT_PRODUCT_VERSION}"
  -e "XGC2_PROTOBUF_DEB_VERSION=${XGC2_PROTOBUF_DEB_VERSION}"
  -e "EXPECTED_PROTOBUF_PRODUCT_VERSION=${EXPECTED_PROTOBUF_PRODUCT_VERSION}"
  -e "XGC2_BOOTSTRAP_COMMON_FROM_GIT=${XGC2_BOOTSTRAP_COMMON_FROM_GIT}"
  -e "XGC2_PROTOBUF_GIT_URL=${XGC2_PROTOBUF_GIT_URL:-https://github.com/XGC-Team/xgc2-protobuf.git}"
  -e "XGC2_PROTOBUF_GIT_REF=${XGC2_PROTOBUF_GIT_REF}"
  -e "XGC2_ADAPTER_RUNTIME_CLIENT_GIT_URL=${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_URL:-https://github.com/XGC-Team/xgc2-adapter-runtime-client-cpp.git}"
  -e "XGC2_ADAPTER_RUNTIME_CLIENT_GIT_REF=${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_REF}"
)

for proxy_var in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n "${!proxy_var:-}" ]]; then
    docker_env_args+=(-e "${proxy_var}=${!proxy_var}")
  fi
done

container_name="xgc2-ros1-adapters-build-$(date +%s)-$$"
container_created=false
cleanup() {
  if [[ "${container_created}" == "true" ]]; then
    docker rm -f "${container_name}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

docker_pull_lock_file="${DOCKER_PULL_LOCK_FILE:-/tmp/xgc2-ros1-adapters-docker-pull.lock}"
exec {docker_pull_lock_fd}>"${docker_pull_lock_file}"
flock "${docker_pull_lock_fd}"

docker pull "${docker_platform_args[@]}" "${DOCKER_IMAGE}"
docker create --name "${container_name}" \
  "${docker_platform_args[@]}" \
  "${docker_network_args[@]}" \
  "${docker_env_args[@]}" \
  "${DOCKER_IMAGE}" sleep infinity >/dev/null
container_created=true
flock -u "${docker_pull_lock_fd}"

docker start "${container_name}" >/dev/null
docker exec "${container_name}" mkdir -p /tmp/ros1-adapter /tmp/work /tmp/out
docker cp "${REPO_ROOT}/." "${container_name}:/tmp/ros1-adapter/"
if [[ -n "${XGC2_PROTOBUF_SOURCE_ROOT}" ]]; then
  [[ -f "${XGC2_PROTOBUF_SOURCE_ROOT}/.xgc2/scripts/build_deb.sh" ]] || {
    echo "XGC2_PROTOBUF_SOURCE_ROOT is not an xgc2-protobuf source tree" >&2
    exit 1
  }
  docker exec "${container_name}" mkdir -p /tmp/xgc2-protobuf-source
  docker cp "${XGC2_PROTOBUF_SOURCE_ROOT}/." "${container_name}:/tmp/xgc2-protobuf-source/"
fi
# The quoted payload is parsed by the inner Bash process; its continuations are
# intentionally literal to this outer shell.
# shellcheck disable=SC1004
docker exec "${container_name}" bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    sed -i \
      -e "s#http://archive.ubuntu.com/ubuntu#https://archive.ubuntu.com/ubuntu#g" \
      -e "s#http://security.ubuntu.com/ubuntu#https://archive.ubuntu.com/ubuntu#g" \
      -e "s#http://ports.ubuntu.com/ubuntu-ports#https://ports.ubuntu.com/ubuntu-ports#g" \
      /etc/apt/sources.list
    printf "%s\n" "Acquire::Retries \"5\";" \
      >/etc/apt/apt.conf.d/99-xgc2-retries
    apt_update() {
      local attempt
      for attempt in 1 2 3; do
        if apt-get update; then
          return 0
        fi
        [[ "${attempt}" -lt 3 ]] || return 1
        sleep "$((attempt * 5))"
      done
    }
    actual_deb_arch="$(dpkg --print-architecture)"
    if [[ -n "${EXPECTED_DEB_ARCH}" && "${actual_deb_arch}" != "${EXPECTED_DEB_ARCH}" ]]; then
      echo "container Debian architecture ${actual_deb_arch} does not match expected ${EXPECTED_DEB_ARCH}" >&2
      exit 1
    fi
    echo "Building Debian package for ${actual_deb_arch}"

    missing_image_packages=()
    for package in nlohmann-json3-dev patch libgrpc++-dev libc-ares-dev \
        protobuf-compiler-grpc python3-grpcio python3-protobuf; do
      dpkg-query -W -f="\${Status}" "${package}" 2>/dev/null \
        | grep -Fxq "install ok installed" \
        || missing_image_packages+=("${package}")
    done
    if [[ "${#missing_image_packages[@]}" -ne 0 ]]; then
      printf "XGC2 build image is missing required package: %s\n" \
        "${missing_image_packages[@]}" >&2
      exit 1
    fi

    install -d -m 0755 /etc/apt/keyrings
    curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
      -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" \
      > /etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL:-}" &&
          "${XGC2_DEPENDENCY_SET_DIGEST}" != "${EMPTY_DEPENDENCY_SET_DIGEST}" ]]; then
      echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] ${XGC2_APT_OVERLAY_URL%/} focal main" \
        > /etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt_update
    apt-get install -y --no-install-recommends ros-noetic-scout-msgs \
      libxgc2-runtime-sdk-dev libxgc2-robotics-interfaces-dev \
      libxgc2-hover-thrust-dev ros-noetic-xgc2-estimator-rigid-state \
      ros-noetic-xgc2-multirotor-controller
    dpkg --compare-versions "$(dpkg-query -W -f="\${Version}" libxgc2-runtime-sdk-dev)" ge 0.1.0-2~focal
    dpkg --compare-versions "$(dpkg-query -W -f="\${Version}" libxgc2-robotics-interfaces-dev)" ge 0.1.0-1~focal
    # Existing owning packages only; absent DTO payloads remain a hard failure.
    require_owned_header() {
      local package="$1" path="$2"
      test -f "${path}"
      dpkg-query -S "${path}" | grep -Fxq "${package}: ${path}"
    }
    for header in xgc_rt.h xgc_clock_source.h flat_config.hpp; do
      require_owned_header libxgc2-runtime-sdk-dev "/usr/include/xgc-runtime/${header}"
    done
    for header in robotics_interfaces_v1.h control_records_v1.h paired_state_v1.h; do
      require_owned_header libxgc2-robotics-interfaces-dev "/usr/include/xgc-robotics-interfaces/${header}"
    done
    require_owned_header libxgc2-hover-thrust-dev /opt/ros/noetic/include/hover_thrust_estimator/native/hover_thrust_wire.h
    require_owned_header ros-noetic-xgc2-estimator-rigid-state /opt/ros/noetic/include/estimator_vrpn_px4_rotor_state/native/rigid_state_wire_v1.h
    for header in reference_wire_v1.h reference_wire.hpp; do
      require_owned_header ros-noetic-xgc2-multirotor-controller "/opt/ros/noetic/include/multirotor_reference_trajectory/${header}"
    done
    test -f /usr/include/xgc-runtime/xgc_rt.h
    test -f /usr/share/cmake/XgcRuntimeSDK/XgcRuntimeSDKConfig.cmake
    dpkg-query -S /usr/include/xgc-runtime/xgc_rt.h
    if [[ "${XGC2_BOOTSTRAP_COMMON_FROM_GIT}" == "false" ]]; then
      client_protobuf_version="$(
        apt-cache show \
          "libxgc2-adapter-runtime-client-dev=${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}" |
          sed -nE \
            "s/^Depends:.*xgc2-protobuf-dev \\(= ([^)]+)\\).*/\\1/p" |
          head -n 1
      )"
      if [[ -z "${client_protobuf_version}" ]]; then
        echo "Adapter Runtime client does not declare an exact xgc2-protobuf-dev dependency" >&2
        exit 1
      fi
      if [[ "${client_protobuf_version}" != "${XGC2_PROTOBUF_DEB_VERSION}" ]]; then
        echo "Adapter Runtime client ${ADAPTER_RUNTIME_CLIENT_DEB_VERSION} requires protobuf ${client_protobuf_version}; Adapter requires ${XGC2_PROTOBUF_DEB_VERSION}" >&2
        exit 1
      fi
    fi

    if [[ "${XGC2_BOOTSTRAP_COMMON_FROM_GIT}" == "true" ]]; then
      rm -rf /tmp/xgc2-common-bootstrap
      mkdir -p /tmp/xgc2-common-bootstrap/debs

      if [[ -d /tmp/xgc2-protobuf-source ]]; then
        cp -a /tmp/xgc2-protobuf-source /tmp/xgc2-common-bootstrap/protobuf
      else
        git init -q /tmp/xgc2-common-bootstrap/protobuf
        git -C /tmp/xgc2-common-bootstrap/protobuf remote add origin "${XGC2_PROTOBUF_GIT_URL}"
        git -C /tmp/xgc2-common-bootstrap/protobuf fetch --depth 1 origin "${XGC2_PROTOBUF_GIT_REF}"
        git -C /tmp/xgc2-common-bootstrap/protobuf checkout -q --detach FETCH_HEAD
        test "$(git -C /tmp/xgc2-common-bootstrap/protobuf rev-parse HEAD)" = \
          "${XGC2_PROTOBUF_GIT_REF}"
      fi
      test "$(awk -F": *" "/^version:/ {print \$2; exit}" /tmp/xgc2-common-bootstrap/protobuf/.xgc2/product.yml)" = \
        "${EXPECTED_PROTOBUF_PRODUCT_VERSION}"
      PACKAGE_DISTRIBUTION=focal \
      PACKAGE_VERSION="${XGC2_PROTOBUF_DEB_VERSION}" \
      XGC2_PROTOBUF_DEB_OUTPUT_DIR=/tmp/xgc2-common-bootstrap/debs/protobuf \
        /tmp/xgc2-common-bootstrap/protobuf/.xgc2/scripts/build_deb.sh
      apt-get install -y \
        /tmp/xgc2-common-bootstrap/debs/protobuf/xgc2-protobuf-dev_*.deb

      git init -q /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp
      git -C /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp remote add origin \
        "${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_URL}"
      git -C /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp fetch --depth 1 origin \
        "${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_REF}"
      git -C /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp checkout -q --detach FETCH_HEAD
      test "$(git -C /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp rev-parse HEAD)" = \
        "${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_REF}"
      test "$(awk -F": *" "/^version:/ {print \$2; exit}" /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp/.xgc2/product.yml)" = \
        "${EXPECTED_RUNTIME_CLIENT_PRODUCT_VERSION}"
      PACKAGE_DISTRIBUTION=focal \
      PACKAGE_VERSION="${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}" \
      XGC2_ADAPTER_RUNTIME_DEB_OUTPUT_DIR=/tmp/xgc2-common-bootstrap/debs/client \
        /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp/.xgc2/scripts/build_deb.sh
      apt-get install -y \
        /tmp/xgc2-common-bootstrap/debs/client/libxgc2-adapter-runtime-client3_*.deb \
        /tmp/xgc2-common-bootstrap/debs/client/libxgc2-adapter-runtime-client-dev_*.deb
    else
      apt-get install -y \
        "xgc2-protobuf-dev=${XGC2_PROTOBUF_DEB_VERSION}" \
        "libxgc2-adapter-runtime-client-dev=${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}"
    fi

    installed_client_version="$(dpkg-query -W -f="\${Version}" libxgc2-adapter-runtime-client-dev)"
    if [[ "${installed_client_version}" != "${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}" ]]; then
      echo "Adapter Runtime client version mismatch: expected ${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}, got ${installed_client_version}" >&2
      exit 1
    fi
    installed_runtime_version="$(dpkg-query -W -f="\${Version}" libxgc2-adapter-runtime-client3)"
    if [[ "${installed_runtime_version}" != "${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}" ]]; then
      echo "Adapter Runtime ABI version mismatch: expected ${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}, got ${installed_runtime_version}" >&2
      exit 1
    fi
    installed_protobuf_version="$(dpkg-query -W -f="\${Version}" xgc2-protobuf-dev)"
    if [[ "${installed_protobuf_version}" != "${XGC2_PROTOBUF_DEB_VERSION}" ]]; then
      echo "XGC2 protobuf version mismatch: expected ${XGC2_PROTOBUF_DEB_VERSION}, got ${installed_protobuf_version}" >&2
      exit 1
    fi

    rm -rf /tmp/work /tmp/out
    mkdir -p /tmp/work /tmp/out
    rsync -a --delete /tmp/ros1-adapter/ /tmp/work/
    rm -rf /tmp/work/build /tmp/work/devel /tmp/work/install /tmp/work/logs /tmp/work/debs

    cd /tmp/work
    python3 -m unittest discover -v -s test -p "test_*.py"
    zenoh_build_args=(
      --prefix /tmp/xgc2-zenohc-prefix
      --work-root /tmp/xgc2-zenohc-build
    )
    if [[ -n "${BUILD_JOBS:-}" ]]; then
      zenoh_build_args+=(--jobs "${BUILD_JOBS}")
    fi
    /tmp/work/.xgc2/scripts/prepare_zenohc_focal.sh "${zenoh_build_args[@]}"
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    parallel_jobs="${BUILD_JOBS:-$(nproc)}"
    DESTDIR=/tmp/work/install-root catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_PREFIX_PATH="/tmp/xgc2-zenohc-prefix;/opt/ros/noetic" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    # Build the single owning Edge implementation and install its Helpers export.
    cmake -S /tmp/work/native/ros_io/helpers -B /tmp/work/build-ros-io-helpers \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr \
      -DCMAKE_INSTALL_DATADIR=share -DCMAKE_PREFIX_PATH="/usr;/opt/ros/noetic"
    cmake --build /tmp/work/build-ros-io-helpers --parallel "${parallel_jobs}"
    cmake --install /tmp/work/build-ros-io-helpers
    DESTDIR=/tmp/work/install-root cmake --install /tmp/work/build-ros-io-helpers
    ldconfig
    mkdir -p /tmp/work/install-root/opt/ros/noetic/lib
    XGC_RUNTIME_SDK_INCLUDE=/usr/include/xgc-runtime \
    XGC_ROBOTICS_INTERFACES_PREFIX=/usr \
    XGC_HOVER_THRUST_WIRE_PREFIX=/opt/ros/noetic \
    XGC_RIGID_STATE_WIRE_PREFIX=/opt/ros/noetic \
    XGC_REFERENCE_WIRE_PREFIX=/opt/ros/noetic \
    XGC_ROS_IO_HELPERS_PREFIX=/usr \
      /tmp/work/native/ros_io/build.sh /tmp/work/install-root/opt/ros/noetic/lib/libros_io.so

    # Original generic/physical transport regressions use this exact built DSO.
    /tmp/work/native/ros_io/run-slice-test.sh
    /tmp/work/native/ros_io/run-attitude-target-test.sh
    ROS_IO_LIB=/tmp/work/install-root/opt/ros/noetic/lib/libros_io.so \
      /tmp/work/native/ros_io/run-clock-test.sh

    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" run_tests
    catkin_test_results --verbose build/test_results

    ZENOHC_LICENSE_DIR=/tmp/xgc2-zenohc-prefix/share/licenses/zenoh-c \
      /tmp/ros1-adapter/.xgc2/scripts/package_debs.sh \
      --install-root /tmp/work/install-root \
      --output-dir /tmp/out

    /tmp/ros1-adapter/.xgc2/scripts/check_native_package_payload.sh \
      /tmp/work/install-root /tmp/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      # The official ROS Docker image excludes /usr/share/doc to reduce image
      # size. Restore normal dpkg extraction for our disposable install gate so
      # the packaged third-party licenses are verified as installed artifacts.
      if [[ -f /etc/dpkg/dpkg.cfg.d/excludes ]]; then
        mv /etc/dpkg/dpkg.cfg.d/excludes /tmp/xgc2-docker-dpkg-excludes
      fi
      apt-get install -y \
        /tmp/out/ros-noetic-xgc2-px4-multirotor-adapter_*.deb \
        /tmp/out/ros-noetic-xgc2-ros1-native-bridge_*.deb \
        /tmp/out/ros-noetic-xgc2-scout-mini-adapter_*.deb \
        /tmp/out/ros-noetic-xgc2-mecanum-ugv-adapter_*.deb \
        /tmp/out/ros-noetic-xgc2-unitree-b2-adapter_*.deb \
        /tmp/out/ros-noetic-xgc2-mocap-rotor-adapter_*.deb \
        /tmp/out/ros-noetic-xgc2-mocap-rotor-forwarder_*.deb
      if ! /tmp/ros1-adapter/.xgc2/scripts/check_installed_packages.sh; then
        echo "Installed-package gate failed; replaying it with command tracing" >&2
        bash -x /tmp/ros1-adapter/.xgc2/scripts/check_installed_packages.sh
      fi
      ROS_MASTER_URI=http://127.0.0.1:11331 ROS_IP=127.0.0.1 \
      ROBOT_SERVER_PRIVATE_TEST=1 \
        /tmp/ros1-adapter/.xgc2/scripts/test_robot_servers.sh \
        /tmp/work/build /tmp/out/robot-server-tests
    fi
  '

if [[ "${COPY_OUTPUT}" == "true" ]]; then
  docker cp "${container_name}:/tmp/out/." "${OUTPUT_DIR}/"

  echo "Debian package output:"
  find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
else
  echo "Debian package output copy skipped."
fi
