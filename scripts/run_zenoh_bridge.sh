#!/usr/bin/env bash
# Start zenoh-bridge-ros2dds with the ROS domain assigned to its machine role.
# ROS_DOMAIN_ID must be set before launch: it overrides the JSON5 plugin value.
set -euo pipefail

usage() {
  echo "Usage: $0 <groundpc|jetson|minipc> [ros-distro]" >&2
  exit 64
}

[[ $# -ge 1 && $# -le 2 ]] || usage
ROLE="$1"
ROS_DISTRO_NAME="${2:-${ROS_DISTRO:-}}"

case "${ROLE}" in
  groundpc) ROS_DOMAIN=6 ;;
  jetson) ROS_DOMAIN=6 ;;
  minipc) ROS_DOMAIN=5 ;;
  *) usage ;;
esac

if [[ -z "${ROS_DISTRO_NAME}" || ! -r "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  echo "Pass a ROS distribution installed under /opt/ros (for example: humble or jazzy)." >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${WORKSPACE_DIR}/config/zenoh/bridge_${ROLE}.json5"
[[ -r "${CONFIG_FILE}" ]] || { echo "Bridge config not found: ${CONFIG_FILE}" >&2; exit 1; }

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN}"

echo "Starting ${ROLE} Zenoh bridge with ROS_DOMAIN_ID=${ROS_DOMAIN}"
exec zenoh-bridge-ros2dds -c "${CONFIG_FILE}"
