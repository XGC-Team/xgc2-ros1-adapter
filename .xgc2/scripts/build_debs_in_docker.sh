#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
DOCKER_NETWORK="${DOCKER_NETWORK:-}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
COPY_OUTPUT="${COPY_OUTPUT:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
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

if [[ "${COPY_OUTPUT}" == "true" ]]; then
  mkdir -p "${OUTPUT_DIR}"
  rm -f "${OUTPUT_DIR}/ros-noetic-xgc2-ros1-adapter_"*.deb
fi

docker_network_args=()
if [[ -n "${DOCKER_NETWORK}" ]]; then
  docker_network_args=(--network "${DOCKER_NETWORK}")
fi

docker_env_args=(
  -e DEBIAN_FRONTEND=noninteractive
  -e INSTALL_CHECK="${INSTALL_CHECK}"
)

for proxy_var in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n "${!proxy_var:-}" ]]; then
    docker_env_args+=(-e "${proxy_var}=${!proxy_var}")
  fi
done

container_name="xgc2-ros1-adapter-build-$(date +%s)-$$"
container_created=false
cleanup() {
  if [[ "${container_created}" == "true" ]]; then
    docker rm -f "${container_name}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

docker pull "${DOCKER_IMAGE}"
docker create --name "${container_name}" \
  "${docker_network_args[@]}" \
  "${docker_env_args[@]}" \
  "${DOCKER_IMAGE}" sleep infinity >/dev/null
container_created=true

docker start "${container_name}" >/dev/null
docker exec "${container_name}" mkdir -p /tmp/ros1-adapter /tmp/work /tmp/out
docker cp "${REPO_ROOT}/." "${container_name}:/tmp/ros1-adapter/"
docker exec "${container_name}" bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
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
      rsync \
      ros-noetic-geometry-msgs \
      ros-noetic-mavros-msgs \
      ros-noetic-roscpp \
      ros-noetic-roslaunch \
      ros-noetic-rosmsg \
      ros-noetic-rospack \
      ros-noetic-sensor-msgs \
      ros-noetic-std-msgs \
      ros-noetic-tf2

    rm -rf /tmp/work /tmp/out
    mkdir -p /tmp/work /tmp/out
    rsync -a --delete /tmp/ros1-adapter/ /tmp/work/

    cd /tmp/work
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    export XGC2_CONTRACTS_DIR=/tmp/work/contracts
    parallel_jobs="$(nproc)"
    DESTDIR=/tmp/work/install-root catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /tmp/ros1-adapter/.xgc2/scripts/package_debs.sh \
      --install-root /tmp/work/install-root \
      --output-dir /tmp/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /tmp/out/ros-noetic-xgc2-ros1-adapter_*.deb
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
