#!/usr/bin/env bash
# Start one of the persistent, machine-local ROS 2 bringups for systemd.
# This script is intentionally executed as the normal robot user, not root.
set -euo pipefail

usage() {
  echo "Usage: $0 <jetson|minipc> <ros-distro>" >&2
  exit 64
}

[[ $# -eq 2 ]] || usage
ROLE="$1"
ROS_DISTRO_NAME="$2"

case "${ROLE}" in
  jetson|minipc) ;;
  *) usage ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -r "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: workspace is not built: ${WORKSPACE_SETUP}" >&2
  exit 1
fi

# Keep the service's role deterministic even if CUDA/ZED detection changes.
export NJORD_PROFILE="${ROLE}"
export NJORD_ROLE="${ROLE}"
export ROS_DISTRO="${ROS_DISTRO_NAME}"
export ROS_LOCALHOST_ONLY=0

# Keep Humble on the miniPC isolated from the Jazzy machines.  Do not inherit
# an interactive shell's ROS_DOMAIN_ID: zenoh-bridge-ros2dds gives that value
# precedence over its JSON5 `plugins/ros2dds/domain` setting.
case "${ROLE}" in
  minipc) export ROS_DOMAIN_ID=5 ;;
  jetson) export ROS_DOMAIN_ID=6 ;;
esac

# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/njord_env.sh"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"

cd "${WORKSPACE_DIR}"
if [[ "${ROLE}" == "jetson" ]]; then
  exec ros2 launch robot jetson_bringup.launch.py
fi

# Task-specific Nav2 and waypoint nodes are deliberately not persistent.
# They must be controlled by the miniPC task-management layer.
exec ros2 launch robot minipc_bringup.launch.py enable_nav2:=false
