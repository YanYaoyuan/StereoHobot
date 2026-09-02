#!/usr/bin/env bash

STEREOINFER_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export STEREOINFER_HOME
export LD_LIBRARY_PATH="${STEREOINFER_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
