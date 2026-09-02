#!/usr/bin/env bash

set -e

STEREO_H265_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export STEREO_H265_HOME

if [[ -f /opt/tros/humble/setup.bash ]]; then
  STEREO_H265_ROS_BASE=/opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  STEREO_H265_ROS_BASE=/opt/ros/humble/setup.bash
else
  echo "StereoH265: ROS2 Humble was not found under /opt/tros or /opt/ros." >&2
  return 1 2>/dev/null || exit 1
fi

set +u
source "${STEREO_H265_ROS_BASE}"
# The base environment is sourced explicitly above. Using local_setup keeps
# this overlay relocatable and avoids the build-time /opt/ros prefix chain.
source "${STEREO_H265_HOME}/install/local_setup.bash"
set -u

export LD_LIBRARY_PATH="${STEREO_H265_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
