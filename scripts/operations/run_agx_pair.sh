#!/usr/bin/env bash
# Start the AGX vessel or Ground PC with matching DDS/service transport.
set -eo pipefail
case "${1:-}" in
  vessel) launch_file=agx_bringup.launch.py; domain=126 ;;
  ground) launch_file=ground_pc.launch.py; domain=126 ;;
  *) echo 'Usage: run_agx_pair.sh <vessel|ground> [launch arguments...]' >&2; exit 64 ;;
esac
shift
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
export PATH="${HOME}/.local/bin:${PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ "$launch_file" == ground_pc.launch.py ]]; then
  if [[ -n "${NJORD_GROUND_CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI="$NJORD_GROUND_CYCLONEDDS_URI"
  else
    agx_address="${NJORD_AGX_ADDRESS:-192.168.0.71}"
    ground_address=$(ip -4 route get "$agx_address" | awk '{for (i=1;i<NF;i++) if ($i=="src") {print $(i+1); exit}}')
    if [[ ! "$ground_address" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
      echo "Cannot select DDS interface for $agx_address; set NJORD_GROUND_CYCLONEDDS_URI explicitly." >&2
      exit 1
    fi
    export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface address=\"${ground_address}\" /></Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
    echo "Ground DDS interface address: $ground_address (route to $agx_address)" >&2
  fi
else
  export CYCLONEDDS_URI="${workspace}/src/bringup/robot/config/cyclonedds_agx.xml"
fi
export ROS_DOMAIN_ID="${domain}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
cd "${workspace}"
exec ros2 launch robot "${launch_file}" enable_critical_link:=false "$@"
