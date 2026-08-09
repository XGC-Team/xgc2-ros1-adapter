#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE_NAME="ros-noetic-xgc2-unitree-b2-adapter"
PACKAGE_VERSION="${PACKAGE_VERSION:-$(
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' \
    "${REPO_ROOT}/.xgc2/product.yml"
)}"
PROTOBUF_PACKAGE_VERSION="${PROTOBUF_PACKAGE_VERSION:-0.5.0-11~focal}"
APT_BASE_URL="${XGC2_APT_BASE_URL:-https://xgc2.apt.xiaokang.ink}"
DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"

if [[ "${PACKAGE_VERSION}" != "0.5.0-16" ]]; then
  echo "public B2 APT gate is frozen to 0.5.0-16, got ${PACKAGE_VERSION}" >&2
  exit 1
fi

container_name="xgc2-b2-public-apt-check-$(date +%s)-$$"
cleanup() {
  docker rm -f "${container_name}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker create --name "${container_name}" "${DOCKER_IMAGE}" sleep infinity >/dev/null
docker start "${container_name}" >/dev/null
docker exec "${container_name}" mkdir -p /tmp/ros1-adapter
docker cp "${REPO_ROOT}/." "${container_name}:/tmp/ros1-adapter/"

docker exec \
  -e "APT_BASE_URL=${APT_BASE_URL}" \
  -e "PACKAGE_NAME=${PACKAGE_NAME}" \
  -e "PACKAGE_VERSION=${PACKAGE_VERSION}" \
  -e "PROTOBUF_PACKAGE_VERSION=${PROTOBUF_PACKAGE_VERSION}" \
  "${container_name}" bash -lc '
    set -euo pipefail
    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends ca-certificates curl gnupg
    install -d -m 0755 /etc/apt/keyrings
    curl -fsSL "${APT_BASE_URL%/}/xgc2-archive-keyring.gpg" \
      -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    printf "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] %s focal main\n" \
      "${APT_BASE_URL%/}" >/etc/apt/sources.list.d/xgc2.list
    apt-get update
    candidate="$(apt-cache policy "${PACKAGE_NAME}" | awk "/Candidate:/ {print \$2; exit}")"
    if ! apt-cache madison "${PACKAGE_NAME}" | awk "{print \$3}" | grep -Fxq "${PACKAGE_VERSION}"; then
      echo "public Focal APT does not contain ${PACKAGE_NAME}=${PACKAGE_VERSION}; candidate=${candidate:-none}" >&2
      exit 1
    fi
    apt-get install -y --no-install-recommends \
      python3-jsonschema python3-yaml \
      "xgc2-protobuf-dev=${PROTOBUF_PACKAGE_VERSION}" \
      "${PACKAGE_NAME}=${PACKAGE_VERSION}"
    EXPECTED_PRODUCT_VERSION="${PACKAGE_VERSION}" \
      /tmp/ros1-adapter/.xgc2/scripts/check_installed_b2_package.sh
  '

echo "Public Focal APT install gate passed: ${PACKAGE_NAME}=${PACKAGE_VERSION}"
