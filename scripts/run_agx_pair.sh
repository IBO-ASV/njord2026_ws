#!/usr/bin/env bash
# Start the AGX vessel or Ground PC with matching DDS/service transport.
set -eo pipefail
case "${1:-}" in
  vessel) launch_file=agx_bringup.launch.py; domain=5 ;;
  ground) launch_file=ground_pc.launch.py; domain=6 ;;
  *) echo 'Usage: run_agx_pair.sh <vessel|ground> [launch arguments...]' >&2; exit 64 ;;
esac
shift
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
export PATH="${HOME}/.local/bin:${PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${workspace}/src/robot/config/cyclonedds_agx.xml"
export ROS_DOMAIN_ID="${domain}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
cd "${workspace}"
exec ros2 launch robot "${launch_file}" "$@"
