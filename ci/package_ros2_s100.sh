#!/usr/bin/env bash

set -Eeuo pipefail

if (( $# != 3 )); then
  echo "Usage: $0 <colcon-install> <source-root> <output-root>" >&2
  exit 2
fi

readonly INSTALL_ROOT="$(realpath "$1")"
readonly SOURCE_ROOT="$(realpath "$2")"
readonly OUTPUT_ROOT="$(realpath -m "$3")"
readonly PACKAGE_NAME="ros2_h265_stereonet"

if [[ ! -x "${INSTALL_ROOT}/lib/${PACKAGE_NAME}/stereo_h265_node" ]]; then
  echo "Missing ROS2 executable in ${INSTALL_ROOT}" >&2
  exit 1
fi
case "${OUTPUT_ROOT}" in
  /|"${INSTALL_ROOT}"|"${SOURCE_ROOT}")
    echo "Unsafe output path: ${OUTPUT_ROOT}" >&2
    exit 1
    ;;
esac

rm -rf "${OUTPUT_ROOT}"
mkdir -p \
  "${OUTPUT_ROOT}/bin" \
  "${OUTPUT_ROOT}/lib" \
  "${OUTPUT_ROOT}/config" \
  "${OUTPUT_ROOT}/launch" \
  "${OUTPUT_ROOT}/model" \
  "${OUTPUT_ROOT}/result"

cp -a "${INSTALL_ROOT}" "${OUTPUT_ROOT}/install"
install -m 0755 \
  "${INSTALL_ROOT}/lib/${PACKAGE_NAME}/stereo_h265_node" \
  "${OUTPUT_ROOT}/bin/"
cp -a "${SOURCE_ROOT}/ros2_h265_stereonet/config/." "${OUTPUT_ROOT}/config/"
cp -a "${SOURCE_ROOT}/ros2_h265_stereonet/launch/." "${OUTPUT_ROOT}/launch/"
install -m 0644 \
  "${SOURCE_ROOT}/config/dstereo_s100_320_640_352_v2.4.hbm" \
  "${OUTPUT_ROOT}/model/"

install -m 0755 \
  "${SOURCE_ROOT}/ci/ros2_package/setup_env.sh" \
  "${SOURCE_ROOT}/ci/ros2_package/run_stereo_h265.sh" \
  "${SOURCE_ROOT}/ci/ros2_package/check_environment.sh" \
  "${OUTPUT_ROOT}/"
install -m 0644 "${SOURCE_ROOT}/ci/ros2_package/README.md" "${OUTPUT_ROOT}/"

# Bundle non-system accelerator and image-processing libraries. ROS2 itself is
# supplied by the board's matching TROS Humble installation.
cp -a "${SOURCE_ROOT}/standalone/3rdparty/ucp_3.13.6/lib/." "${OUTPUT_ROOT}/lib/"
cp -a "${SOURCE_ROOT}"/standalone/3rdparty/ucp_3.13.6/lib*.so* "${OUTPUT_ROOT}/lib/"
cp -a "${SOURCE_ROOT}/standalone/3rdparty/lib_opencv4.5.4/lib/." "${OUTPUT_ROOT}/lib/"

shopt -s nullglob
foxglove_libraries=(/opt/ros/humble/lib/libfoxglove_msgs*.so)
if (( ${#foxglove_libraries[@]} > 0 )); then
  cp -a "${foxglove_libraries[@]}" "${OUTPUT_ROOT}/lib/"
fi
installed_foxglove_libraries=("${INSTALL_ROOT}"/lib/libfoxglove_msgs*.so*)
if (( ${#installed_foxglove_libraries[@]} > 0 )); then
  cp -a "${installed_foxglove_libraries[@]}" "${OUTPUT_ROOT}/lib/"
fi

for real_library in "${OUTPUT_ROOT}"/lib/libopencv_*.so.4.5.4; do
  library_stem="${real_library%.4.5.4}"
  ln -sfn "$(basename "${real_library}")" "${library_stem}.4.5"
  ln -sfn "$(basename "${library_stem}.4.5")" "${library_stem}"
done

cat > "${OUTPUT_ROOT}/lib/DEPENDENCIES.txt" <<'EOF'
Bundled: D-Robotics UCP 3.13.6, OpenCV 4.5.4, foxglove_msgs type support.
Board base: RDK S100 RDK OS / TROS Humble, glibc, ROS2 core, FFmpeg and PCL.
EOF

file "${OUTPUT_ROOT}/bin/stereo_h265_node" | tee "${OUTPUT_ROOT}/BUILD_INFO.txt"
grep -Eq 'ARM aarch64|ARM64' "${OUTPUT_ROOT}/BUILD_INFO.txt"

for required_path in \
  bin/stereo_h265_node \
  config/stereo_h265_params.yaml \
  config/vita_calib.yaml \
  launch/stereo_h265.launch.py \
  model/dstereo_s100_320_640_352_v2.4.hbm \
  install/setup.bash \
  run_stereo_h265.sh; do
  test -e "${OUTPUT_ROOT}/${required_path}"
done

echo "ROS2 S100 runtime: ${OUTPUT_ROOT}"
du -sh "${OUTPUT_ROOT}"
