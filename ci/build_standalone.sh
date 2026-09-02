#!/usr/bin/env bash

set -Eeuo pipefail

readonly TARGET="${1:-s100}"
readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly SOURCE_DIR="${PROJECT_ROOT}/standalone"
readonly OUTPUT_DIR="${PROJECT_ROOT}/dist"

case "${TARGET}" in
  s100)
    readonly PLATFORM_OPTION="PLATFORM_S100"
    # This path is resolved inside the D-Robotics toolchain container.
    readonly TOOLCHAIN_ROOT="${LINARO_GCC_ROOT:-/arm-gnu-toolchain-12.2.rel1-x86_64-aarch64-none-linux-gnu}"
    readonly CC="${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-gcc"
    readonly CXX="${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-g++"
    readonly MODEL_GLOB="dstereo_s100*.hbm"
    readonly PACKAGE_NAME="StereoInfer_S100"
    ;;
  x5)
    readonly PLATFORM_OPTION="PLATFORM_X5"
    readonly TOOLCHAIN_ROOT="/opt/arm-gnu-toolchain-11.3.rel1-x86_64-aarch64-none-linux-gnu"
    readonly CC="${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-gcc"
    readonly CXX="${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-g++"
    readonly MODEL_GLOB="DStereoV*.bin"
    readonly PACKAGE_NAME="StereoInfer_X5"
    ;;
  *)
    echo "Unsupported target '${TARGET}'. Expected: s100 or x5." >&2
    exit 2
    ;;
esac

for command_path in "${CC}" "${CXX}"; do
  if [[ ! -x "${command_path}" ]]; then
    echo "Required compiler not found: ${command_path}" >&2
    exit 1
  fi
done

readonly BUILD_DIR="$(mktemp -d)"
readonly STAGE_DIR="$(mktemp -d)"
trap 'rm -rf "${BUILD_DIR}" "${STAGE_DIR}"' EXIT

cmake \
  -S "${SOURCE_DIR}" \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -D"${PLATFORM_OPTION}"=ON \
  -DCMAKE_C_COMPILER="${CC}" \
  -DCMAKE_CXX_COMPILER="${CXX}"
cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

readonly PACKAGE_ROOT="${STAGE_DIR}/${PACKAGE_NAME}"
mkdir -p \
  "${PACKAGE_ROOT}/bin" \
  "${PACKAGE_ROOT}/lib" \
  "${PACKAGE_ROOT}/config" \
  "${PACKAGE_ROOT}/samples" \
  "${PACKAGE_ROOT}/scripts" \
  "${PACKAGE_ROOT}/result"
install -m 0755 "${BUILD_DIR}/infer" "${BUILD_DIR}/test_perf" "${PACKAGE_ROOT}/bin/"
install -m 0755 "${PROJECT_ROOT}/ci/package/setup_env.sh" "${PACKAGE_ROOT}/"
install -m 0755 \
  "${PROJECT_ROOT}/ci/package/run_infer.sh" \
  "${PROJECT_ROOT}/ci/package/run_test_perf.sh" \
  "${PACKAGE_ROOT}/scripts/"
install -m 0644 "${PROJECT_ROOT}/ci/package/README.md" "${PACKAGE_ROOT}/"
cp -a "${SOURCE_DIR}/img" "${PACKAGE_ROOT}/samples/"

# Runtime libraries only: no headers, CMake metadata or host tools.
cp -a "${SOURCE_DIR}/3rdparty/ucp_3.13.6/lib/." "${PACKAGE_ROOT}/lib/"
cp -a "${SOURCE_DIR}"/3rdparty/ucp_3.13.6/lib*.so* "${PACKAGE_ROOT}/lib/"
cp -a "${SOURCE_DIR}/3rdparty/lib_opencv4.5.4/lib/." "${PACKAGE_ROOT}/lib/"

# The repository stores OpenCV link placeholders as text files. Convert them
# into real symlinks in the deployment package.
for real_library in "${PACKAGE_ROOT}"/lib/libopencv_*.so.4.5.4; do
  library_stem="${real_library%.4.5.4}"
  ln -sfn "$(basename "${real_library}")" "${library_stem}.4.5"
  ln -sfn "$(basename "${library_stem}.4.5")" "${library_stem}"
done

shopt -s nullglob
models=("${PROJECT_ROOT}"/config/${MODEL_GLOB})
if (( ${#models[@]} == 0 )); then
  echo "No model matched config/${MODEL_GLOB}" >&2
  exit 1
fi
cp -a "${models[@]}" "${PACKAGE_ROOT}/config/"

mkdir -p "${OUTPUT_DIR}"
readonly ARCHIVE="${OUTPUT_DIR}/${PACKAGE_NAME}.tar.gz"
tar -C "${STAGE_DIR}" -czf "${ARCHIVE}" "${PACKAGE_NAME}"

file "${PACKAGE_ROOT}/bin/infer" "${PACKAGE_ROOT}/bin/test_perf" | tee "${OUTPUT_DIR}/${PACKAGE_NAME}.elf.txt"
if ! file "${PACKAGE_ROOT}/bin/infer" "${PACKAGE_ROOT}/bin/test_perf" | grep -q 'ARM aarch64'; then
  echo "Build output is not ARM64." >&2
  exit 1
fi
if ! "${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-readelf" -d \
  "${PACKAGE_ROOT}/bin/infer" | grep -Fq '$ORIGIN/../lib'; then
  echo "Packaged executable does not contain the relocatable library RUNPATH." >&2
  exit 1
fi
for required_path in bin/infer bin/test_perf lib config scripts setup_env.sh README.md; do
  if [[ ! -e "${PACKAGE_ROOT}/${required_path}" ]]; then
    echo "Incomplete package: missing ${required_path}" >&2
    exit 1
  fi
done
tar -tzf "${ARCHIVE}" >/dev/null

echo "Package ready: ${ARCHIVE}"
