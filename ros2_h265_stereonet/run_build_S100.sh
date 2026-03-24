#!/bin/bash
# Cross-compile ros2_h265_stereonet for S100 (aarch64)
# Usage: bash run_build_S100.sh
#
# Prerequisites:
#   - aarch64-linux-gnu-g++ available at /usr/bin/ (or adjust below)
#   - ROS2 sysroot with foxglove_msgs, sensor_msgs, rclcpp, PCL, FFmpeg
#   - Parent hobot_stereonet source tree at ../

set -e
clear
cd "$(dirname "$0")"
echo "=> curr dir: $(pwd)"
echo "=> ================="

rm -rf build
mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release .. \
  -DPLATFORM_S100=ON \
  -DCMAKE_C_COMPILER=/usr/bin/aarch64-linux-gnu-gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++

make -j$(nproc)

echo "=> ================="
echo "=> Build complete: stereo_h265_node"
ls -lh stereo_h265_node
md5sum stereo_h265_node
echo "=> ================="
