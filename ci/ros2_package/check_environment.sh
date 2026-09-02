#!/usr/bin/env bash

set -Eeuo pipefail

readonly PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PACKAGE_ROOT}/setup_env.sh"

echo "Architecture: $(uname -m)"
echo "ROS distro  : ${ROS_DISTRO:-unknown}"
echo "Package     : $(ros2 pkg prefix ros2_h265_stereonet)"
echo "Executable  : ${PACKAGE_ROOT}/bin/stereo_h265_node"

file "${PACKAGE_ROOT}/bin/stereo_h265_node"
test "$(uname -m)" = aarch64 || {
  echo "WARNING: this deployment package is intended for an aarch64 S100 board." >&2
}

for topic in "${LEFT_TOPIC:-/image_left_raw/h265}" "${RIGHT_TOPIC:-/image_right_raw/h265}"; do
  topic_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
  printf 'Topic %-32s %s\n' "${topic}" "${topic_type:-NOT FOUND}"
done
