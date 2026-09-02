#!/usr/bin/env bash

set -Eeuo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly TARGET="${1:-s100}"
readonly TOOLCHAIN_IMAGE="${TOOLCHAIN_IMAGE:-registry.d-robotics.cc/deliver/ai_toolchain_ubuntu_22_s100_s600_gpu:v3.7.0}"
readonly BUILD_IMAGE="${BUILD_IMAGE:-hobot-stereonet-toolchain:v3.7.0}"

if ! docker image inspect "${TOOLCHAIN_IMAGE}" >/dev/null 2>&1; then
  echo "Toolchain Docker image is not loaded: ${TOOLCHAIN_IMAGE}" >&2
  echo "Pull it from the registry or load the ai_toolchain_*.tar Docker archive first." >&2
  exit 1
fi

echo "Preparing build environment from ${TOOLCHAIN_IMAGE}"
docker build \
  --build-arg "BASE_IMAGE=${TOOLCHAIN_IMAGE}" \
  --file "${PROJECT_ROOT}/ci/Dockerfile.toolchain" \
  --tag "${BUILD_IMAGE}" \
  "${PROJECT_ROOT}"

echo "Building ${TARGET} inside ${BUILD_IMAGE}"
docker run --rm \
  --entrypoint /bin/bash \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${BUILD_IMAGE}" \
  -lc '
    set -Eeuo pipefail
    echo "Container compiler: ${LINARO_GCC_ROOT}/bin/aarch64-none-linux-gnu-g++"
    "${LINARO_GCC_ROOT}/bin/aarch64-none-linux-gnu-g++" --version | head -n 1
    exec bash ci/build_standalone.sh "$1"
  ' -- "${TARGET}"
