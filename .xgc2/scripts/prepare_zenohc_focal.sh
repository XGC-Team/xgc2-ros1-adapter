#!/usr/bin/env bash
set -euo pipefail

PREFIX=""
WORK_ROOT=""
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
METADATA_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

ZENOHC_VERSION="1.9.0"
ZENOHC_COMMIT="499de93af63e6a7d3497313f544e666fea1d33fd"
ZENOHC_ARCHIVE_SHA256="768a87fc3e6752965e98e15d4669143254c38a41174dc18193ff95702f5b238d"
ZENOHC_ARCHIVE_URL="https://codeload.github.com/eclipse-zenoh/zenoh-c/tar.gz/${ZENOHC_COMMIT}"
ZENOH_CORE_COMMIT="81c6c933b6e41d72a05f04c4442ef57717ddc72b"
RUST_VERSION="1.93.0"
RUSTUP_VERSION="1.28.2"
LOCK_SHA256="152fa7f09f683690b78dadd14e3065f2bae3ce84243b5f09e7edd646d0fde44d"
REVISION_PATCH_SHA256="9408ff849a7ae7a52cb848e3e1a5ba43a482a1f07c3127673e5c60164eb376f4"
LOCK_FILE="$METADATA_ROOT/vendor/zenoh-c-${ZENOHC_VERSION}.Cargo.lock"
REVISION_PATCH="$METADATA_ROOT/patches/zenoh-c-${ZENOHC_VERSION}-pin-revisions.patch"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      PREFIX="$2"
      shift 2
      ;;
    --work-root)
      WORK_ROOT="$2"
      shift 2
      ;;
    --jobs)
      BUILD_JOBS="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "$PREFIX" || -z "$WORK_ROOT" ]]; then
  echo "--prefix and --work-root are required" >&2
  exit 1
fi
if [[ "$PREFIX" != /* || "$WORK_ROOT" != /* ]]; then
  echo "Zenoh C prefix and work root must be absolute paths" >&2
  exit 1
fi
if [[ ! "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo "--jobs must be a positive integer" >&2
  exit 1
fi

for command in cmake curl dpkg patch rg sha256sum tar; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "$command is required to build the pinned Zenoh C dependency" >&2
    exit 1
  fi
done
for input_file in "$LOCK_FILE" "$REVISION_PATCH"; do
  if [[ ! -f "$input_file" ]]; then
    echo "missing pinned Zenoh C build input: $input_file" >&2
    exit 1
  fi
done
printf '%s  %s\n' "$LOCK_SHA256" "$LOCK_FILE" | sha256sum --check --strict
printf '%s  %s\n' "$REVISION_PATCH_SHA256" "$REVISION_PATCH" | \
  sha256sum --check --strict

STAMP_TEXT="zenoh-c=${ZENOHC_VERSION} commit=${ZENOHC_COMMIT} core_commit=${ZENOH_CORE_COMMIT} source_sha256=${ZENOHC_ARCHIVE_SHA256} revision_patch_sha256=${REVISION_PATCH_SHA256} cargo_lock_sha256=${LOCK_SHA256} rust=${RUST_VERSION} shared_memory=off static=on"
STAMP_FILE="$PREFIX/.xgc2-zenohc-focal-build"
if [[ -f "$STAMP_FILE" ]] &&
   [[ "$(<"$STAMP_FILE")" == "$STAMP_TEXT" ]] &&
   [[ -f "$PREFIX/lib/libzenohc.a" ]] &&
   [[ -f "$PREFIX/lib/cmake/zenohc/zenohcConfig.cmake" ]] &&
   [[ -f "$PREFIX/include/zenoh.h" ]]; then
  printf '%s\n' "$PREFIX"
  exit 0
fi
if [[ -e "$STAMP_FILE" ]]; then
  echo "Zenoh C prefix has a different build identity: $PREFIX" >&2
  exit 1
fi

ARCHIVE="$WORK_ROOT/zenoh-c-${ZENOHC_COMMIT}.tar.gz"
SOURCE_ROOT="$WORK_ROOT/source"
BUILD_ROOT="$WORK_ROOT/build"
RUST_CARGO_HOME="$WORK_ROOT/cargo-home"
mkdir -p "$PREFIX" "$WORK_ROOT" "$SOURCE_ROOT" "$BUILD_ROOT" "$RUST_CARGO_HOME"
export CARGO_HOME="$RUST_CARGO_HOME"

if [[ ! -f "$ARCHIVE" ]]; then
  curl --fail --location --show-error --silent "$ZENOHC_ARCHIVE_URL" \
    --output "$ARCHIVE"
fi
printf '%s  %s\n' "$ZENOHC_ARCHIVE_SHA256" "$ARCHIVE" | sha256sum --check --strict
if [[ ! -f "$SOURCE_ROOT/CMakeLists.txt" ]]; then
  tar --extract --gzip --file "$ARCHIVE" --directory "$SOURCE_ROOT" \
    --strip-components 1
fi

source_patch_stamp="$SOURCE_ROOT/.xgc2-zenoh-revision-patch"
if [[ ! -f "$source_patch_stamp" ]]; then
  patch --directory "$SOURCE_ROOT" --strip=1 --forward --fuzz=0 <"$REVISION_PATCH"
  printf '%s\n' "$REVISION_PATCH_SHA256" >"$source_patch_stamp"
elif [[ "$(<"$source_patch_stamp")" != "$REVISION_PATCH_SHA256" ]]; then
  echo "Zenoh C source tree has a different revision patch" >&2
  exit 1
fi

case "$(dpkg --print-architecture)" in
  amd64)
    rust_arch="x86_64"
    rustup_sha256="20a06e644b0d9bd2fbdbfd52d42540bdde820ea7df86e92e533c073da0cdd43c"
    ;;
  arm64)
    rust_arch="aarch64"
    rustup_sha256="e3853c5a252fca15252d07cb23a1bdd9377a8c6f3efa01531109281ae47f841c"
    ;;
  *)
    echo "Zenoh C Focal build supports only amd64 and arm64" >&2
    exit 1
    ;;
esac
rustup_init="$WORK_ROOT/rustup-init-${RUSTUP_VERSION}-${rust_arch}"
if [[ ! -f "$rustup_init" ]]; then
  curl --fail --location --show-error --silent \
    "https://static.rust-lang.org/rustup/archive/${RUSTUP_VERSION}/${rust_arch}-unknown-linux-gnu/rustup-init" \
    --output "$rustup_init"
fi
printf '%s  %s\n' "$rustup_sha256" "$rustup_init" | sha256sum --check --strict
chmod 0755 "$rustup_init"
export RUSTUP_HOME="$WORK_ROOT/rustup-home"
"$rustup_init" -y --no-modify-path --profile minimal \
  --default-toolchain "$RUST_VERSION"
export PATH="$RUST_CARGO_HOME/bin:$PATH"
if [[ "$(cargo --version | awk '{print $2}')" != "$RUST_VERSION" ]]; then
  echo "pinned Cargo $RUST_VERSION is unavailable" >&2
  exit 1
fi
export CARGO_NET_GIT_FETCH_WITH_CLI=true

cmake -S "$SOURCE_ROOT" -B "$BUILD_ROOT" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=OFF \
  -DZENOHC_BUILD_WITH_SHARED_MEMORY=OFF \
  -DZENOHC_BUILD_WITH_UNSTABLE_API=OFF \
  -DZENOHC_CARGO_CHANNEL="+${RUST_VERSION}" \
  -DZENOHC_COPY_SOURCE_CARGO_LOCK=FALSE \
  -DZENOHC_CARGO_FLAGS=--locked
install -m 0644 "$LOCK_FILE" "$BUILD_ROOT/debug/Cargo.lock"
install -m 0644 "$LOCK_FILE" "$BUILD_ROOT/release/Cargo.lock"
cmake --build "$BUILD_ROOT" --target install --parallel "$BUILD_JOBS"

test -f "$PREFIX/lib/libzenohc.a"
test -f "$PREFIX/lib/cmake/zenohc/zenohcConfig.cmake"
test -f "$PREFIX/include/zenoh.h"
if ! rg -q 'zenohc::static' "$PREFIX/lib/cmake/zenohc/zenohcConfig.cmake"; then
  echo "installed Zenoh C package does not expose its static target" >&2
  exit 1
fi
printf '%s  %s\n' "$LOCK_SHA256" "$BUILD_ROOT/release/Cargo.lock" | \
  sha256sum --check --strict
mkdir -p "$PREFIX/share/licenses/zenoh-c"
install -m 0644 "$SOURCE_ROOT/LICENSE" "$PREFIX/share/licenses/zenoh-c/LICENSE"
install -m 0644 "$SOURCE_ROOT/NOTICE.md" "$PREFIX/share/licenses/zenoh-c/NOTICE.md"
printf '%s\n' "$STAMP_TEXT" >"$STAMP_FILE"
printf '%s\n' "$PREFIX"
