#!/usr/bin/env bash
# Sim smoke test harness.
#
# Launches a simulation launch file, lets it settle, then records the
# evidence that matters for "does it actually run": node/topic inventory,
# publication rates on the control+localization path, and the TF tree.
#
# Usage:
#   scripts/sim_smoke_test.sh <outdir> <duration_s> <pkg> <launch_file> [launch args...]
#
# Example:
#   scripts/sim_smoke_test.sh /tmp/t1 60 task1_sim task1_sim.launch.py
#   scripts/sim_smoke_test.sh /tmp/t3 90 task3_sim task3_sim.launch.py task_type:=task3_1

set -u

OUTDIR="${1:?outdir required}"
DURATION="${2:?duration required}"
PKG="${3:?package required}"
LAUNCH="${4:?launch file required}"
shift 4
LAUNCH_ARGS=("$@")

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$OUTDIR"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# ROS setup scripts reference unset variables; -u must be off while sourcing.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

# Stale nodes from an earlier run silently corrupt every measurement: duplicate
# node names, topics that look alive but are fed by the wrong process. Wipe all
# ROS processes before and after, not just the ones this script started.
kill_all_ros() {
    pkill -9 -f "ros2 launch"  2>/dev/null
    pkill -9 -f -- "--ros-args" 2>/dev/null
    sleep 2
    return 0
}

cleanup() {
    [[ -n "${LAUNCH_PID:-}" ]] && kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 3
    kill_all_ros
    [[ -n "${ROUTER_PID:-}" ]] && kill -9 "$ROUTER_PID" 2>/dev/null
    wait 2>/dev/null
    return 0
}
trap cleanup EXIT

kill_all_ros

# ── Zenoh router ─────────────────────────────────────────────────────────────
# rmw_zenoh_cpp needs a router for peers to discover each other. Without it
# every node comes up isolated and the sim silently does nothing.
if ! pgrep -f rmw_zenohd >/dev/null; then
    ros2 run rmw_zenoh_cpp rmw_zenohd > "$OUTDIR/zenohd.log" 2>&1 &
    ROUTER_PID=$!
    sleep 3
    echo "started rmw_zenohd pid=$ROUTER_PID"
else
    echo "rmw_zenohd already running"
fi

# ── Launch ───────────────────────────────────────────────────────────────────
ros2 launch "$PKG" "$LAUNCH" "${LAUNCH_ARGS[@]}" > "$OUTDIR/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "launched $PKG $LAUNCH ${LAUNCH_ARGS[*]} pid=$LAUNCH_PID"

sleep "$DURATION"

# ── Evidence collection ──────────────────────────────────────────────────────
echo "collecting evidence into $OUTDIR"
ros2 node list           > "$OUTDIR/nodes.txt"        2>&1
ros2 topic list          > "$OUTDIR/topics.txt"       2>&1
ros2 action list         > "$OUTDIR/actions.txt"      2>&1

# TF tree: raw /tf_static + /tf frame pairs, and connectivity checks.
timeout 10 ros2 topic echo /tf_static --once > "$OUTDIR/tf_static.txt" 2>&1
{
    for pair in "map odom" "odom base_link" "map base_link" "base_link livox_frame" "base_link imu_link"; do
        set -- $pair
        echo "--- $1 -> $2 ---"
        timeout 6 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | head -12
    done
} > "$OUTDIR/tf_echo.txt" 2>&1

# Publication rates on the paths that must be alive for the sim to move.
{
    for t in /odom /cmd_vel /cmd_vel_auto /thruster_command \
             /odometry/filtered/local /odometry/filtered/global \
             /odometry/gps /odometry/gps/um982 \
             /gps/fix /sensor/vehicle_gnss/fix/raw /wit/imu /livox/imu \
             /buoy_detections_3d /virtual_obstacles /sim_obstacles; do
        echo "=== $t ==="
        # `ros2 topic hz` intermittently fails with "wait set ... context is not
        # valid" under rmw_zenoh, so fall back to the publisher count, which
        # distinguishes "nobody publishes this" from a flaky measurement.
        out=$(timeout 8 ros2 topic hz "$t" 2>&1 | head -3)
        if echo "$out" | grep -q "average rate"; then
            echo "$out"
        else
            echo "$out"
            timeout 5 ros2 topic info "$t" 2>&1 | head -3
        fi
    done
} > "$OUTDIR/rates.txt" 2>&1

grep -iE "\[ERROR\]|\[FATAL\]|Traceback|died|process has died|Exception" \
    "$OUTDIR/launch.log" | grep -v rmw_zenoh > "$OUTDIR/errors.txt" 2>&1

echo "=== SUMMARY ==="
echo "nodes:  $(wc -l < "$OUTDIR/nodes.txt")"
echo "topics: $(wc -l < "$OUTDIR/topics.txt")"
echo "errors: $(wc -l < "$OUTDIR/errors.txt")"
