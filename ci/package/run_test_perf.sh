#!/usr/bin/env bash

set -Eeuo pipefail
readonly PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PACKAGE_ROOT}/setup_env.sh"

if (( $# == 0 )); then
  echo "Usage: $0 <model.hbm> [threads] [frames] [uncertainty-threshold]" >&2
  echo "Example: $0 config/dstereo_s100_320_640_352_v2.4.hbm 1 30 0.10" >&2
  exit 2
fi

exec "${PACKAGE_ROOT}/bin/test_perf" "$@"
