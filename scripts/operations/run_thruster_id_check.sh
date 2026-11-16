#!/usr/bin/env bash
# Start the individual-thruster identification check on the vessel Jetson.
#
# The defaults match the normal Jetson bringup so the shore-PC joystick and
# vessel remain on the same ROS 2 network.  Set NJORD_RMW or ROS_DOMAIN_ID
# before invoking this script when using a different deployment profile.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -r "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: workspace is not built: ${WORKSPACE_SETUP}" >&2
  exit 1
fi

# Keep the defaults aligned with run_bringup_service.sh jetson jazzy while
# allowing an operator to select another DDS domain explicitly.
export NJORD_PROFILE="${NJORD_PROFILE:-jetson}"
export NJORD_ROLE="${NJORD_ROLE:-jetson}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-6}"
export ROS_LOCALHOST_ONLY=0

# ROS Jazzy's setup script reads AMENT_TRACE_SETUP_FILES before it initializes
# it, so it is not compatible with nounset.  Keep nounset for this wrapper
# while temporarily disabling it around the ROS-provided setup scripts.
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../setup/njord_env.sh"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"
set -u

cd "${WORKSPACE_DIR}"
exec ros2 launch thruster_tools thruster_id_test.launch.py "$@"
