#!/usr/bin/env bash
# Start the AGX vessel or ground Jetson with matching DDS/service transport.
set -eo pipefail
case "${1:-}" in
  vessel) launch_file=agx_bringup.launch.py; role=vessel ;;
  ground) launch_file=ground_jetson.launch.py; role=ground ;;
  *) echo 'Usage: run_agx_pair.sh <vessel|ground> [launch arguments...]' >&2; exit 64 ;;
esac
shift
backend="${NJORD_AGX_TRANSPORT:-fastrtps}"
if [[ "${role}" == vessel ]]; then
  local_ip=100.73.137.72
  peer_ip=100.115.34.103
else
  local_ip=100.115.34.103
  peer_ip=100.73.137.72
fi
for launch_arg in "$@"; do
  case "${launch_arg}" in
    communication_backend:=*) backend="${launch_arg#communication_backend:=}" ;;
    fastdds_local_ip:=*) local_ip="${launch_arg#fastdds_local_ip:=}" ;;
    fastdds_peer_ip:=*) peer_ip="${launch_arg#fastdds_peer_ip:=}" ;;
  esac
done
export NJORD_AGX_TRANSPORT="${backend}"
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
export PATH="${HOME}/.local/bin:${PATH}"
case "${backend}" in
  fastrtps)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE="${workspace}/src/robot/config/dds/fastdds_agx_pair.xml"
    export ROS_DOMAIN_ID=126
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export NJORD_FASTDDS_LOCAL_IP="${local_ip}"
    export NJORD_FASTDDS_PEER_IP="${peer_ip}"
    ;;
  zenoh)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="${workspace}/src/robot/config/cyclonedds_agx.xml"
    export ROS_DOMAIN_ID=5
    if [[ "${role}" == ground ]]; then
      export ROS_DOMAIN_ID=6
    fi
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ;;
  *) echo "Unknown communication backend: ${backend} (expected fastrtps or zenoh)" >&2; exit 64 ;;
esac
unset ROS_LOCALHOST_ONLY
cd "${workspace}"
exec ros2 launch robot "${launch_file}" "$@"
