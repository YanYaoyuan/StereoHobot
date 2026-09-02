#!/usr/bin/env bash

# Reproducible ROS2 Humble cross-build for RDK S100. This script is shared by
# local builds and GitHub Actions and always compiles inside the official TROS
# cross-compilation image.

set -Eeuo pipefail

report_error() {
  local exit_code=$?
  printf '[stereo-s100] ERROR: line %s: %s (exit %s)\n' \
    "$1" "$2" "${exit_code}" >&2
  exit "${exit_code}"
}
trap 'report_error "${LINENO}" "${BASH_COMMAND}"' ERR

SOURCE_ROOT=${S100_SOURCE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
WORK_ROOT=${S100_WORK_ROOT:-${RUNNER_TEMP:-/tmp}/stereohobot-s100-cross}
CACHE_ROOT=${S100_CACHE_ROOT:-${WORK_ROOT}/cache}
S100_IMAGE=${S100_IMAGE:-pc_tros_ubuntu22.04:v1.0.0}
ROBOT_DEV_CONFIG_REF=${ROBOT_DEV_CONFIG_REF:-f44b1ad2575b34c189fd470de1db9c5ad48b6886}
S100_SYSROOT_REF=${S100_SYSROOT_REF:-de9fa286f71a72d24c349477cd59f41cc2cc3d8f}
ROBOT_DEV_CONFIG_URL=${ROBOT_DEV_CONFIG_URL:-https://github.com/D-Robotics/robot_dev_config.git}
S100_SYSROOT_URL=${S100_SYSROOT_URL:-https://github.com/D-Robotics/sysroot_docker.git}
S100_CLEAN=${S100_CLEAN:-1}

log() { printf '[stereo-s100] %s\n' "$*"; }
die() { printf '[stereo-s100] ERROR: %s\n' "$*" >&2; exit 1; }

for command_name in docker git rsync realpath; do
  command -v "${command_name}" >/dev/null 2>&1 || die "missing command: ${command_name}"
done

SOURCE_ROOT=$(realpath -m "${SOURCE_ROOT}")
WORK_ROOT=$(realpath -m "${WORK_ROOT}")
CACHE_ROOT=$(realpath -m "${CACHE_ROOT}")

[[ -f "${SOURCE_ROOT}/ros2_h265_stereonet/package.xml" ]] || die "invalid source root: ${SOURCE_ROOT}"
[[ -f "${SOURCE_ROOT}/config/dstereo_s100_320_640_352_v2.4.hbm" ]] || die "S100 HBM model is missing"
case "${WORK_ROOT}" in
  /|/data|/home|"${SOURCE_ROOT}") die "unsafe S100_WORK_ROOT: ${WORK_ROOT}" ;;
esac

docker image inspect "${S100_IMAGE}" >/dev/null 2>&1 || \
  die "Docker image is not loaded: ${S100_IMAGE}"

readonly TROS_ROOT="${WORK_ROOT}/cc_ws/tros_ws"
readonly SYSROOT_ROOT="${WORK_ROOT}/cc_ws/sysroot_docker"
readonly ROBOT_DEV_CONFIG_DIR="${TROS_ROOT}/robot_dev_config"
readonly SYSROOT_CACHE="${CACHE_ROOT}/sysroot_docker-${S100_SYSROOT_REF}"

if [[ "${S100_CLEAN}" == 1 ]]; then
  log "cleaning generated workspace: ${WORK_ROOT}/cc_ws"
  if [[ -d "${WORK_ROOT}/cc_ws" ]]; then
    # A failed container build can leave root-owned files in the mounted tree.
    docker run --rm -v "${WORK_ROOT}:/mnt/work" "${S100_IMAGE}" \
      rm -rf /mnt/work/cc_ws
  fi
fi
mkdir -p "${TROS_ROOT}/src/stereohobot_parent" "${SYSROOT_ROOT}" "${CACHE_ROOT}"

if [[ -n "${S100_ROBOT_DEV_CONFIG_SOURCE:-}" ]]; then
  config_source=$(realpath "${S100_ROBOT_DEV_CONFIG_SOURCE}")
  [[ -f "${config_source}/aarch64_toolchainfile.cmake" ]] || die "invalid robot_dev_config source"
  log "using local robot_dev_config: ${config_source}"
  rsync -a --delete --exclude='/.git/' "${config_source}/" "${ROBOT_DEV_CONFIG_DIR}/"
else
  log "checking out robot_dev_config ${ROBOT_DEV_CONFIG_REF}"
  git clone --quiet --filter=blob:none "${ROBOT_DEV_CONFIG_URL}" "${ROBOT_DEV_CONFIG_DIR}"
  if ! git -C "${ROBOT_DEV_CONFIG_DIR}" cat-file -e "${ROBOT_DEV_CONFIG_REF}^{commit}" 2>/dev/null; then
    git -C "${ROBOT_DEV_CONFIG_DIR}" fetch --quiet --no-tags origin "${ROBOT_DEV_CONFIG_REF}"
  fi
  git -C "${ROBOT_DEV_CONFIG_DIR}" checkout --quiet --detach "${ROBOT_DEV_CONFIG_REF}"
fi

prepare_sysroot_cache() {
  local attempt
  if [[ -f "${SYSROOT_CACHE}/.usr_s100-complete" && -d "${SYSROOT_CACHE}/usr_s100" ]]; then
    log "reusing complete S100 sysroot cache"
    return
  fi

  log "fetching pinned S100 sysroot ${S100_SYSROOT_REF}"
  mkdir -p "${SYSROOT_CACHE}"
  if [[ ! -d "${SYSROOT_CACHE}/.git" ]]; then
    git -C "${SYSROOT_CACHE}" init --quiet
    git -C "${SYSROOT_CACHE}" remote add origin "${S100_SYSROOT_URL}"
  else
    git -C "${SYSROOT_CACHE}" remote set-url origin "${S100_SYSROOT_URL}"
  fi
  git -C "${SYSROOT_CACHE}" config core.sparseCheckout true
  git -C "${SYSROOT_CACHE}" config extensions.partialClone origin
  git -C "${SYSROOT_CACHE}" config remote.origin.promisor true
  git -C "${SYSROOT_CACHE}" config remote.origin.partialclonefilter blob:none
  printf '/usr_s100/\n' > "${SYSROOT_CACHE}/.git/info/sparse-checkout"

  for attempt in 1 2 3; do
    if git -C "${SYSROOT_CACHE}" \
      -c http.version=HTTP/1.1 \
      -c http.lowSpeedLimit=1024 \
      -c http.lowSpeedTime=120 \
      fetch --progress --no-tags --depth=1 --filter=blob:none origin "${S100_SYSROOT_REF}"; then
      break
    fi
    (( attempt < 3 )) || die "failed to fetch S100 sysroot after ${attempt} attempts"
  done
  git -C "${SYSROOT_CACHE}" checkout --force --detach FETCH_HEAD
  [[ -d "${SYSROOT_CACHE}/usr_s100" ]] || die "sysroot checkout did not produce usr_s100"
  printf '%s\n' "${S100_SYSROOT_REF}" > "${SYSROOT_CACHE}/.usr_s100-complete"
}

prepare_sysroot_cache
log "exporting target sysroot"
rsync -a --delete "${SYSROOT_CACHE}/usr_s100/" "${SYSROOT_ROOT}/usr_s100/"
printf '%s\n' "${S100_SYSROOT_REF}" > "${SYSROOT_ROOT}/.s100-sysroot-ref"

log "staging StereoHobot source"
rsync -a --delete \
  --exclude='/.git/' \
  --exclude='/build/' \
  --exclude='/install/' \
  --exclude='/log/' \
  --exclude='/dist/' \
  --exclude='/ros2_h265_stereonet/build/' \
  --exclude='/ros2_h265_stereonet/install/' \
  --exclude='/ros2_h265_stereonet/log/' \
  --exclude='/ros2_h265_stereonet/result/' \
  "${SOURCE_ROOT}/" "${TROS_ROOT}/src/stereohobot_parent/"
touch "${TROS_ROOT}/src/stereohobot_parent/COLCON_IGNORE"
mkdir -p "${TROS_ROOT}/src/ros2_h265_stereonet"
rsync -a --delete \
  --exclude='/build/' --exclude='/install/' --exclude='/log/' --exclude='/result/' \
  "${SOURCE_ROOT}/ros2_h265_stereonet/" "${TROS_ROOT}/src/ros2_h265_stereonet/"
mkdir -p "${TROS_ROOT}/src/foxglove_msgs"
rsync -a --delete \
  "${SOURCE_ROOT}/third_party/foxglove_msgs/" "${TROS_ROOT}/src/foxglove_msgs/"

log "starting official TROS toolchain container"
docker run --rm -i \
  --network host \
  -e "HOST_UID=$(id -u)" \
  -e "HOST_GID=$(id -g)" \
  -v "${WORK_ROOT}:/mnt/stereohobot-s100" \
  -w /mnt/stereohobot-s100/cc_ws/tros_ws \
  "${S100_IMAGE}" bash -s <<'CONTAINER'
set -Eeuo pipefail

restore_host_ownership() {
  chown -R "${HOST_UID}:${HOST_GID}" build install log runtime 2>/dev/null || true
}
trap restore_host_ownership EXIT

for command_name in aarch64-linux-gnu-gcc aarch64-linux-gnu-g++ cmake colcon file; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "Official TROS image is missing ${command_name}" >&2
    exit 1
  }
done
test -f /opt/ros/humble/setup.bash
test -f robot_dev_config/aarch64_toolchainfile.cmake
test -d ../sysroot_docker/usr_s100
ln -sfn usr_s100 ../sysroot_docker/usr

set +u
source /opt/ros/humble/setup.bash
set -u

export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/aarch64-linux-gnu-
export PKG_CONFIG_SYSROOT_DIR="$(realpath ../sysroot_docker)"
export PKG_CONFIG_PATH="${PKG_CONFIG_SYSROOT_DIR}/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_SYSROOT_DIR}/usr/lib/pkgconfig"

echo '=== Cross-build ros2_h265_stereonet for S100 ==='
bash robot_dev_config/build.sh \
  -p S100 \
  -c "-DPLATFORM_S100=ON -DSTEREONET_ROOT=${PWD}/src/stereohobot_parent -DCMAKE_BUILD_TYPE=Release" \
  -r '--packages-up-to ros2_h265_stereonet'

# robot_dev_config/build.sh does not reliably propagate colcon failures.
test -x install/lib/ros2_h265_stereonet/stereo_h265_node
test -f install/share/ros2_h265_stereonet/model/dstereo_s100_320_640_352_v2.4.hbm
test -f install/share/foxglove_msgs/msg/CompressedVideo.msg
file install/lib/ros2_h265_stereonet/stereo_h265_node | grep -Eq 'ARM aarch64|ARM64'

echo '=== Assemble directly deployable ROS2 package ==='
bash src/stereohobot_parent/ci/package_ros2_s100.sh \
  install \
  src/stereohobot_parent \
  runtime/StereoH265_ROS2_S100

test -x runtime/StereoH265_ROS2_S100/bin/stereo_h265_node
test -x runtime/StereoH265_ROS2_S100/run_stereo_h265.sh
tar -C runtime -czf runtime/StereoH265_ROS2_S100.tar.gz StereoH265_ROS2_S100
CONTAINER

mkdir -p "${SOURCE_ROOT}/dist"
install -m 0644 \
  "${TROS_ROOT}/runtime/StereoH265_ROS2_S100.tar.gz" \
  "${SOURCE_ROOT}/dist/StereoH265_ROS2_S100.tar.gz"

log "artifact: ${SOURCE_ROOT}/dist/StereoH265_ROS2_S100.tar.gz"
ls -lh "${SOURCE_ROOT}/dist/StereoH265_ROS2_S100.tar.gz"
