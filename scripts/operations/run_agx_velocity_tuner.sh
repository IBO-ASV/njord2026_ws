#!/usr/bin/env bash
# Start the idle velocity tuner in the same ROS domain as the AGX vessel.
set -eo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
export PATH="${HOME}/.local/bin:${PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${workspace}/src/bringup/robot/config/cyclonedds_agx.xml"
export ROS_DOMAIN_ID=126
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
exec ros2 launch velocity_autotuner velocity_autotuner.launch.py "$@"
