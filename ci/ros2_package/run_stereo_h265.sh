#!/usr/bin/env bash

set -Eeuo pipefail

readonly PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PACKAGE_ROOT}/setup_env.sh"

readonly LEFT_TOPIC="${LEFT_TOPIC:-/image_left_raw/h265}"
readonly RIGHT_TOPIC="${RIGHT_TOPIC:-/image_right_raw/h265}"
readonly PARAMS_FILE="${PARAMS_FILE:-${PACKAGE_ROOT}/config/stereo_h265_params.yaml}"
readonly MODEL_PATH="${MODEL_PATH:-${PACKAGE_ROOT}/model/dstereo_s100_320_640_352_v2.4.hbm}"
readonly CALIB_YAML_FILE="${CALIB_YAML_FILE:-${PACKAGE_ROOT}/config/vita_calib.yaml}"

for required_file in "${PARAMS_FILE}" "${MODEL_PATH}" "${CALIB_YAML_FILE}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "StereoH265: required file not found: ${required_file}" >&2
    exit 1
  fi
done

echo "StereoH265 left topic : ${LEFT_TOPIC}"
echo "StereoH265 right topic: ${RIGHT_TOPIC}"
echo "StereoH265 model      : ${MODEL_PATH}"

cd "${PACKAGE_ROOT}"
exec ros2 launch ros2_h265_stereonet stereo_h265.launch.py \
  params_file:="${PARAMS_FILE}" \
  left_topic:="${LEFT_TOPIC}" \
  right_topic:="${RIGHT_TOPIC}" \
  model_path:="${MODEL_PATH}" \
  calib_yaml_file:="${CALIB_YAML_FILE}" \
  "$@"
