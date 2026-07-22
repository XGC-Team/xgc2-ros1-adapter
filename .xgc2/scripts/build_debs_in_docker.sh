#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
DOCKER_PLATFORM="${DOCKER_PLATFORM:-}"
DOCKER_NETWORK="${DOCKER_NETWORK:-}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
COPY_OUTPUT="${COPY_OUTPUT:-true}"
BUILD_JOBS="${BUILD_JOBS:-}"
ADAPTER_RUNTIME_CLIENT_DEB_VERSION="${ADAPTER_RUNTIME_CLIENT_DEB_VERSION:-}"
XGC2_PROTOBUF_DEB_VERSION="${XGC2_PROTOBUF_DEB_VERSION:-}"
XGC2_BOOTSTRAP_COMMON_FROM_GIT="${XGC2_BOOTSTRAP_COMMON_FROM_GIT:-}"

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
if [[ "${XGC2_BOOTSTRAP_COMMON_FROM_GIT}" == "true" ]]; then
  ADAPTER_RUNTIME_CLIENT_DEB_VERSION="${ADAPTER_RUNTIME_CLIENT_DEB_VERSION:-0.5.0-2~focal}"
  XGC2_PROTOBUF_DEB_VERSION="${XGC2_PROTOBUF_DEB_VERSION:-0.5.0-1~focal}"
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
    "${OUTPUT_DIR}/ros-noetic-xgc2-mecanum-ugv-adapter_"*.deb
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
  -e "XGC2_APT_OVERLAY_URL=${XGC2_APT_OVERLAY_URL:-}"
  -e "BUILD_JOBS=${BUILD_JOBS}"
  -e "DEBIAN_FRONTEND=noninteractive"
  -e "EXPECTED_DEB_ARCH=${expected_deb_arch}"
  -e "INSTALL_CHECK=${INSTALL_CHECK}"
  -e "ADAPTER_RUNTIME_CLIENT_DEB_VERSION=${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}"
  -e "XGC2_PROTOBUF_DEB_VERSION=${XGC2_PROTOBUF_DEB_VERSION}"
  -e "XGC2_BOOTSTRAP_COMMON_FROM_GIT=${XGC2_BOOTSTRAP_COMMON_FROM_GIT}"
  -e "XGC2_PROTOBUF_GIT_URL=${XGC2_PROTOBUF_GIT_URL:-https://github.com/lxk36/xgc2-protobuf.git}"
  -e "XGC2_PROTOBUF_GIT_TAG=${XGC2_PROTOBUF_GIT_TAG:-v0.5.0-1}"
  -e "XGC2_ADAPTER_RUNTIME_CLIENT_GIT_URL=${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_URL:-https://github.com/lxk36/xgc2-adapter-runtime-client-cpp.git}"
  -e "XGC2_ADAPTER_RUNTIME_CLIENT_GIT_TAG=${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_TAG:-v0.5.0-2}"
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
# The quoted payload is parsed by the inner Bash process; its continuations are
# intentionally literal to this outer shell.
# shellcheck disable=SC1004
docker exec "${container_name}" bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    actual_deb_arch="$(dpkg --print-architecture)"
    if [[ -n "${EXPECTED_DEB_ARCH}" && "${actual_deb_arch}" != "${EXPECTED_DEB_ARCH}" ]]; then
      echo "container Debian architecture ${actual_deb_arch} does not match expected ${EXPECTED_DEB_ARCH}" >&2
      exit 1
    fi
    echo "Building Debian package for ${actual_deb_arch}"

    apt-get update
    apt-get install -y --no-install-recommends ca-certificates curl gnupg
    install -d -m 0755 /etc/apt/keyrings
    curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
      -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" \
      > /etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
      echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] ${XGC2_APT_OVERLAY_URL%/} focal main" \
        > /etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      git \
      libgrpc++-dev \
      libprotobuf-dev \
      libre2-dev \
      pkg-config \
      protobuf-compiler \
      protobuf-compiler-grpc \
      python3-jsonschema \
      python3-protobuf \
      python3-yaml \
      rsync \
      ros-noetic-geometry-msgs \
      ros-noetic-mavros-msgs \
      ros-noetic-roscpp \
      ros-noetic-roslaunch \
      ros-noetic-rosmsg \
      ros-noetic-rospack \
      ros-noetic-scout-msgs \
      ros-noetic-sensor-msgs

    apt_candidate_version() {
      local package="$1"
      local candidate
      candidate="$(apt-cache policy "${package}" | awk "/Candidate:/ {print \$2; exit}")"
      if [[ -z "${candidate}" || "${candidate}" == "(none)" ]]; then
        echo "APT has no candidate for ${package}" >&2
        exit 1
      fi
      printf "%s\n" "${candidate}"
    }
    if [[ -z "${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}" ]]; then
      ADAPTER_RUNTIME_CLIENT_DEB_VERSION="$(
        apt_candidate_version libxgc2-adapter-runtime-client-dev
      )"
    fi
    if [[ -z "${XGC2_PROTOBUF_DEB_VERSION}" ]]; then
      XGC2_PROTOBUF_DEB_VERSION="$(apt_candidate_version xgc2-protobuf-dev)"
    fi

    if [[ "${XGC2_BOOTSTRAP_COMMON_FROM_GIT}" == "true" ]]; then
      rm -rf /tmp/xgc2-common-bootstrap
      mkdir -p /tmp/xgc2-common-bootstrap/debs

      git clone --depth 1 --branch "${XGC2_PROTOBUF_GIT_TAG}" \
        "${XGC2_PROTOBUF_GIT_URL}" \
        /tmp/xgc2-common-bootstrap/protobuf
      test "$(git -C /tmp/xgc2-common-bootstrap/protobuf describe --tags --exact-match)" = \
        "${XGC2_PROTOBUF_GIT_TAG}"
      PACKAGE_DISTRIBUTION=focal \
      PACKAGE_VERSION="${XGC2_PROTOBUF_DEB_VERSION}" \
      XGC2_PROTOBUF_DEB_OUTPUT_DIR=/tmp/xgc2-common-bootstrap/debs/protobuf \
        /tmp/xgc2-common-bootstrap/protobuf/.xgc2/scripts/build_deb.sh
      apt-get install -y \
        /tmp/xgc2-common-bootstrap/debs/protobuf/xgc2-protobuf-dev_*.deb

      git clone --depth 1 --branch "${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_TAG}" \
        "${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_URL}" \
        /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp
      test "$(git -C /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp describe --tags --exact-match)" = \
        "${XGC2_ADAPTER_RUNTIME_CLIENT_GIT_TAG}"
      PACKAGE_DISTRIBUTION=focal \
      PACKAGE_VERSION="${ADAPTER_RUNTIME_CLIENT_DEB_VERSION}" \
      XGC2_ADAPTER_RUNTIME_DEB_OUTPUT_DIR=/tmp/xgc2-common-bootstrap/debs/client \
        /tmp/xgc2-common-bootstrap/adapter-runtime-client-cpp/.xgc2/scripts/build_deb.sh
      apt-get install -y \
        /tmp/xgc2-common-bootstrap/debs/client/libxgc2-adapter-runtime-client1_*.deb \
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
    installed_runtime_version="$(dpkg-query -W -f="\${Version}" libxgc2-adapter-runtime-client1)"
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
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    parallel_jobs="${BUILD_JOBS:-$(nproc)}"
    DESTDIR=/tmp/work/install-root catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" run_tests
    catkin_test_results --verbose build/test_results

    /tmp/ros1-adapter/.xgc2/scripts/package_debs.sh \
      --install-root /tmp/work/install-root \
      --output-dir /tmp/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y \
        /tmp/out/ros-noetic-xgc2-px4-multirotor-adapter_*.deb \
        /tmp/out/ros-noetic-xgc2-scout-mini-adapter_*.deb \
        /tmp/out/ros-noetic-xgc2-mecanum-ugv-adapter_*.deb
      /tmp/ros1-adapter/.xgc2/scripts/check_installed_packages.sh
    fi
  '

if [[ "${COPY_OUTPUT}" == "true" ]]; then
  docker cp "${container_name}:/tmp/out/." "${OUTPUT_DIR}/"

  echo "Debian package output:"
  find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
else
  echo "Debian package output copy skipped."
fi
