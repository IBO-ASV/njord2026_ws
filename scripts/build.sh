#!/usr/bin/env bash
# build.sh
#
# Thin wrapper: sources njord_env.sh (platform detection) and runs
# colcon build with a Release build type. Build artifacts are isolated by
# ROS distro and hardware profile so CMake never reuses Humble/Jazzy (or
# miniPC/Jetson) configuration results. Extra arguments are forwarded to
# colcon build, e.g.:
#   scripts/build.sh --packages-select njord_platform
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${WS_DIR}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/njord_env.sh"

BUILD_VARIANT="${ROS_DISTRO}/${NJORD_PROFILE}"
colcon --log-base "log/${BUILD_VARIANT}" build \
  --build-base "build/${BUILD_VARIANT}" \
  --install-base "install/${BUILD_VARIANT}" \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"

echo "[build] source install/${BUILD_VARIANT}/setup.bash before launching"
