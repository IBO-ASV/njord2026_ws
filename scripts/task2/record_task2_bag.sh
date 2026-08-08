#!/usr/bin/env bash
# Record a Task 2 rosbag with all topics relevant to perception, planning and
# actuation debugging.
#
# Usage:
#   ./record_task2_bag.sh              # record the Task 2 topic list
#   ./record_task2_bag.sh --all       # record ALL topics instead
#   ./record_task2_bag.sh -o mybag    # custom output name
#
# Any extra arguments are forwarded to `ros2 bag record`.
set -euo pipefail

TASK2_TOPICS=(
    /livox/lidar
    /odom
    /odometry/filtered/local
    /odometry/filtered/global
    /task2/points_filtered
    /pcl/cluster_centroids
    /tracked_objects
    /other_ship/twist
    /pcl/nonground
    /planned_path
    /planned_path_pruned
    /cmd_vel_thruster
    /thruster_command
    /yolo/detections
    /buoy_roi
    /tf
    /tf_static
    /diagnostics
)

OUTPUT_NAME="task2_$(date +%Y%m%d_%H%M%S)"
RECORD_ALL=false
EXTRA_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --all)
            RECORD_ALL=true
            shift
            ;;
        -o|--output)
            OUTPUT_NAME="$2"
            shift 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

# ------------------------------------------------------------------
# Disk space warning (best-effort; never aborts the recording)
# ------------------------------------------------------------------
DISK_AVAIL_KB="$(df -k . 2>/dev/null | awk 'NR==2 {print $4}' || echo "")"
if [ -n "${DISK_AVAIL_KB}" ]; then
    DISK_AVAIL_GB=$((DISK_AVAIL_KB / 1024 / 1024))
    echo "Free disk space here: ${DISK_AVAIL_GB} GiB"
    if [ "${DISK_AVAIL_KB}" -lt 20971520 ]; then  # < 20 GiB
        echo "WARNING: less than 20 GiB free. Lidar-heavy Task 2 bags grow"
        echo "         quickly (order of GiB/min); consider freeing space or"
        echo "         recording to an external drive."
    fi
else
    echo "WARNING: could not determine free disk space (df failed)." || true
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not found. Source your ROS 2 workspace first."
    exit 1
fi

echo "Output bag: ${OUTPUT_NAME}"

if [ "${RECORD_ALL}" = true ]; then
    echo "Recording ALL topics (--all)"
    exec ros2 bag record --all -o "${OUTPUT_NAME}" "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"
else
    echo "Recording ${#TASK2_TOPICS[@]} Task 2 topics"
    exec ros2 bag record -o "${OUTPUT_NAME}" "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}" "${TASK2_TOPICS[@]}"
fi
