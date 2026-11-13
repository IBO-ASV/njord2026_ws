#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
SRC_DIR="${WS_DIR}/src"
IMAGE_COMMON_DIR="${SRC_DIR}/image_common"
IMAGE_COMMON_REF="${IMAGE_COMMON_REF:-3.0.0}"

if [[ "${ROS_DISTRO:-}" != "foxy" ]]; then
  echo "[ERROR] This helper is intended for ROS2 Foxy only. Current ROS_DISTRO='${ROS_DISTRO:-unset}'." >&2
  exit 1
fi

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "[ERROR] src directory not found at ${SRC_DIR}" >&2
  exit 1
fi

if [[ ! -d "${IMAGE_COMMON_DIR}/.git" ]]; then
  git clone --branch "${IMAGE_COMMON_REF}" --single-branch \
    https://github.com/ros-perception/image_common.git "${IMAGE_COMMON_DIR}"
else
  git -C "${IMAGE_COMMON_DIR}" fetch --tags origin
  git -C "${IMAGE_COMMON_DIR}" checkout "${IMAGE_COMMON_REF}"
fi

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  export AMENT_TRACE_SETUP_FILES=0
  # shellcheck disable=SC1091
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

cd "${WS_DIR}"
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

echo "[INFO] image_transport helper complete. Re-open the shell or source install/setup.bash."
