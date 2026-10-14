# GLIM/local-odom connection and stationary-geometry check

Date: 2026-10-14 JST. Host: ground Jetson 100.115.34.103.
Branch: fix/agx-startup. The live ground station was kept running; the experiment
used isolated ROS domain 74, loopback DDS, and sensor-only rosbag playback.

## Correct processing order

Current ship_tracking source transforms cluster detections from base_link to
fixed odom coordinates with timestamp-aligned ego pose BEFORE EKF tracking.
It exports relative, object-body-frame twist for the existing downstream API;
opponent_selector restores ego translation and omega x r before map rotation.
Previous README/no-op descriptions were outdated and have been corrected.
Extended-wall centroids can move as visibility changes even for a stationary
wall. A stable corner or compact fixed landmark is a better reference.

## Connection changes (offline test launch only)

New robot/glim_static_bag_check.launch.py:

- Copies current headless GLIM config into the result directory, sets distinct
  map and odom names, and leaves original production configs unchanged.
- GLIM dynamic TF stays private; base_link -> livox_frame static TF is shared.
- GLIM IMU pose/world velocity -> glim_base_odom_adapter -> local EKF ->
  /odometry/filtered/local and the sole odom -> base_link TF.
- Offline map -> odom is identity, NOT a GNSS georeferenced transform.
- Starts cloud filter and the existing PCL/ship tracker with simulated time.
- Records odometry, tracks, filtered clouds, TF and GNSS. No controls/planner.

The adapter uses the workspace's existing mounting extrinsic (0.5,0,0.8 m,
roll pi). It converts sensor pose to base center, rotates world velocity into
base axes, and corrects the rotating lever arm. Yaw rate is derived from
stamped orientation; installed GLIM's angular twist is zero. Its assumed
covariance values and planar approximation are for this offline test and
require calibration before production use. Do not use this adapter if the
GLIM version changes its velocity-frame contract without rechecking it.

The observed GLIM child_frame_id is livox_frame. Upstream source confirms
sensor-pose odometry with world velocity, despite child-frame labels:
https://github.com/koide3/glim_ros2/blob/master/src/glim_ros/rviz_viewer.cpp

## Data and results

1. July 21 bag rosbag2_2026_07_21-16_56_46, first 120 s:
   inferred own-position bounding span 6.77 m, max ego speed 0.70 m/s.
   1110 local odom messages, 407 track arrays, 1433 confirmed observed-track
   samples. No samples failed retrospective odom time matching (0.15 s limit).
   No track passed the position-stability gate. A 75–105 s moving window also
   produced no such candidate.
2. July 24 bag rosbag2_2026_07_24-14_26_20, first 60 s, rebuilt current tracker:
   inferred own-position bounding span 8.92 m, max ego speed 0.91 m/s.
   543 local odom messages, 201 track arrays, 429 confirmed observed-track
   samples. No track passed the position-stability gate. No TF_SELF_TRANSFORM
   or missing static-extrinsic warning in final trial. Some tracking updates
   were skipped while matching ego odom was not yet available (13 throttled
   warning log entries; this is not a count of skipped frames).

Track candidates were selected by >=8 s duration, fixed-position 95% radius
<=0.75 m, and >=10 observations while ego speed>=0.2 m/s. No target was selected
on a low velocity criterion. Thus automatic track-velocity validation did NOT
pass; no semantic or speed accuracy claim for other vessels is made.

The July 24 scene plot shows wall-like straight structures aligning after
transformation. A corner was manually selected geometrically, before speed
evaluation, using two fixed ROIs (x 5.5–8.5 m, y -8–22 m for the vertical line;
y -13.5–-10.5 m, x -14–6.5 m for the horizontal line). Median line offsets
provide a repeatable corner proxy.

Fixed-geometry result:

- 226 clouds; corner visible with enough support in 64 observations.
- 14 valid speed samples while ego speed>=0.2 m/s.
- 4 s least-squares velocity window, >=8 supporting observations and >=2 s span.
- Horizontal ground-speed median 0.038 m/s, 95th percentile 0.061 m/s.
- Corner-position 95% radius 0.350 m around the median.

This is a limited positive check of fixed geometric alignment, not proof that
the object tracker reports zero speed. The wall-like corner has no independent
semantic label or surveyed ground truth. Coverage is sparse; missing intervals
are not drawn as continuous measurements in the speed graph. GNSS was replayed
but not fused or used as ground truth.

## Artifacts and validation

/home/ibo/rosbags/glim_static_check_20261014_v3/: July 21 output bag, tracks, JSON.
/home/ibo/rosbags/glim_static_check_20261014_v4/: July 24 output bag (~12 MB),
track_samples.csv, static_track_summary.json, corner_summary.json,
corner_speed.csv, corner_check.png, scene_alignment.png, and run.log.

New adapter geometry tests plus existing tracking-glue tests: 23 passed.
Python syntax checks, relevant builds and git diff whitespace checks passed.
All trial processes were stopped. Existing user changes remain unstaged.
No push or production launch change was made.

Run in a new result directory with ROS_DOMAIN_ID=74 and the same loopback
CycloneDDS environment as Docs/glim_rosbag_check_20261014.md:

```bash
ros2 launch robot glim_static_bag_check.launch.py \
  bag_path:=/home/ibo/njord2026_ws/rosbag2_2026_07_24-14_26_20 \
  output_dir:=/home/ibo/rosbags/NEW_RESULT_DIRECTORY duration:=60
# After playback and clean recorder shutdown:
PYTHONNOUSERSITE=1 python3 scripts/evaluate_glim_static_tracks.py RESULT_DIRECTORY
# This corner evaluator applies only to the documented July 24 geometry:
PYTHONNOUSERSITE=1 python3 scripts/evaluate_fixed_corner.py RESULT_DIRECTORY
```

Remaining work for other-ship recognition: label a compact stationary feature,
check association stability and centroid changes, queue/delay detection updates
until stamped ego odom is available, and test continuous moving/turning intervals.
GNSS fusion requires valid fixes and an independently validated map datum.
