#!/usr/bin/env bash
# Targeted probe: launch a sim and run arbitrary introspection commands against
# it, instead of the fixed evidence set that sim_smoke_test.sh collects.
#
# Usage:
#   scripts/sim_probe.sh <outdir> <settle_s> <probe_script> <pkg> <launch> [args...]
#
# <probe_script> is a bash file run after the sim settles, with ROS sourced.

set -u

OUTDIR="${1:?outdir required}"
SETTLE="${2:?settle seconds required}"
PROBE="${3:?probe script required}"
PKG="${4:?package required}"
LAUNCH="${5:?launch file required}"
shift 5

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$OUTDIR"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

kill_all_ros() {
    pkill -9 -f "ros2 launch"   2>/dev/null
    pkill -9 -f -- "--ros-args" 2>/dev/null
    sleep 2
    return 0
}

cleanup() {
    [[ -n "${LAUNCH_PID:-}" ]] && kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 2
    kill_all_ros
    [[ -n "${ROUTER_PID:-}" ]] && kill -9 "$ROUTER_PID" 2>/dev/null
    return 0
}
trap cleanup EXIT

kill_all_ros

if ! pgrep -f rmw_zenohd >/dev/null; then
    ros2 run rmw_zenoh_cpp rmw_zenohd > "$OUTDIR/zenohd.log" 2>&1 &
    ROUTER_PID=$!
    sleep 3
fi

ros2 launch "$PKG" "$LAUNCH" "$@" > "$OUTDIR/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep "$SETTLE"

OUTDIR="$OUTDIR" bash "$PROBE" 2>&1 | tee "$OUTDIR/probe.txt"
