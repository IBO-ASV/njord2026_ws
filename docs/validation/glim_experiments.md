# GLIM 検証条件と結果

2026-10-14 の複数実験をまとめた検証資料です。各入力bagと評価区間を区別してください。実測の速度・船種・絶対位置精度を保証する資料ではありません。

過去の専用スクリプトは整理前 commit `74fe7ec6` から取得できます。削除済みスクリプトを使う以下の手順は当時の再現条件であり、現行コマンドではありません。

---

# August 10 bag follow-up

Host: ground Jetson. Branch: fix/agx-startup. Run date: 2026-10-14.

Previous experiments used rosbag2_2026_07_21-16_56_46 (first 120 s) and
rosbag2_2026_07_24-14_26_20 (first 60 s). The previous 0.038/0.061 m/s
corner result applies ONLY to July 24.

At the user's request, reran the unchanged isolated GLIM/adapter/EKF/tracker
launch on /home/ibo/njord2026_ws/rosbag2_2026_08_10-03_52_00, first 120 s.
The full original bag is 405.429 s, 2.4 GiB, with 3328 LiDAR, 76670 IMU,
and 8068 raw GNSS messages. All GNSS status values are 0 (fix present;
not evidence of RTK accuracy). First 1000 IMU acceleration norms have
median 0.99936, consistent with the existing g-to-SI conversion.
GNSS was recorded but not fused or used as ground truth.

Results: /home/ibo/rosbags/glim_static_aug10_20261014/
- output_bag, run.log, static_track_summary.json, track_samples.csv,
  static_tracks.png.
- 1121 local odom messages, 436 track arrays, 1092 matched confirmed samples.
- 26 samples excluded because nearest ego timestamp exceeded 0.15 s.
- Estimated position bounding span 8.81 m; max ego speed 0.761 m/s.
- Unwrapped yaw span 421 degrees: not independently validated as actual turn.

Same position-based candidate rule as July trials, with no low-speed selection:
duration >=8 s, position radius95 <=0.75 m, >=10 observations at ego speed >=0.2 m/s.

| Track ID | Moving samples | Ground median m/s | Ground p95 m/s |
|---|---:|---:|---:|
| 22 | 20 | 0.112 | 0.291 |
| 28 | 15 | 0.314 | 0.795 |
| 43 | 21 | 0.106 | 0.175 |

Combined 56 samples: relative-speed median 1.161 m/s; compensated median
0.128 m/s and p95 0.430 m/s. IDs 22 and 43 occupy nearly the same position;
they may be the same feature with fragmented IDs, not distinct landmarks.
The duration rule measures first-to-last span, not uninterrupted observation.
Two candidates meet the provisional median<=0.2/p95<=0.5 thresholds; ID 28 fails.
Candidates have no verified shore/semantic labels or independent ground truth.
The July-specific corner ROI evaluator was not applied to this different scene.

The log reports points/IMU timestamp difference 1.340560 s around source
94 s, plus skipped tracker updates waiting for aligned ego odom. This limits
confidence; source timestamps and replay delivery need separate investigation.
Trial processes were stopped cleanly; original ground station remained running.
No production configuration changes and no push.


---

# August 10 moving interval, GNSS fusion and replay-order improvement

2026-10-14, ground Jetson, fix/agx-startup.
Source: /home/ibo/njord2026_ws/rosbag2_2026_08_10-03_52_00 (405.43 s).
Both GLIM runs cover the entire bag. Evaluation: 180-270 s, where GNSS shows
14-21 m displacement per 30 s. No production localization/control changes.

## Decision

Keep continuous GLIM local odometry for tracking. Neither tested GNSS fusion
variant improves local scene consistency, so do not replace tracker odometry.
GNSS agreement is not independent proof of accuracy. Comparison nodes publish
no TF. Global GNSS anchoring remains a separate future evaluation.

## Replay improvement

Source recording delays reach 13.815 s for IMU and 3.086 s for LiDAR.
Acquisition gaps reach 2.590 s and 4.050 s respectively. Sensor headers are
monotonic, but arrival-order playback can deliver old IMU after newer LiDAR.

reorder_aug10_sensors.py creates a separate acquisition-ordered bag using a
20 s heap. Serialized payloads and header/point timestamps are unchanged;
only playback times/order change. LiDAR is released at last-point time +1 ms.
Counts remain 76670 IMU, 3328 LiDAR, 8068 GNSS and 1259 compass messages.
This is an offline remedy; missing measurements and live latency are not fixed.

| Measurement | Original order | Acquisition order |
|---|---:|---:|
| Local odom messages | 2574 | 3358 |
| Retrospectively matched confirmed track samples | 2465 | 3149 |
| Samples excluded by nearest-odom 0.15 s gate | 308 | 0 |
| Large point/IMU time-difference log entries | 22225 | 551 |

Warnings are log entries, not lost-frame counts. Callback-time tracker skips
remain; zero retrospective exclusions does not establish live synchronization.

## GNSS comparison

Native robot_localization EKF fuses GLIM body velocity/yaw rate, ENU-aligned
GLIM yaw, and GNSS XY position. GLIM absolute XY is not also fused. GNSS position
sigma floors of 0.5 m and 2 m are experimental assumptions, not calibrated errors.
Raw GNSS covariance is always 0.0004 m^2 and status 0; this does not prove RTK
accuracy. Compass messages cover only 2.08-66.09 s, not the evaluation window.

WGS84 ECEF-to-ENU projects GNSS. Only 0-150 s is used to fit a rigid GLIM/GNSS
alignment, without scaling. Evaluation is later, at 180-270 s. Recorded TF
antenna lever arm (-0.5,+0.4) m is removed using GLIM yaw. Alignment residual
median/p95 is 0.987/2.481 m: agreement with input GNSS, not independent accuracy.
Planar geometry, mounting and GLIM covariance remain experimental assumptions.

155 common scan pairs about 5 s apart are evaluated. Metric: median across pairs
of symmetric nearest-point XY distance medians. Visibility changes and moving
objects are included. This is NOT surveyed position accuracy or wall velocity.

| Acquisition-order configuration | Scene distance median | Median pair p80 |
|---|---:|---:|
| GLIM | 0.135 m | 0.491 m |
| GNSS + GLIM, sigma floor 0.5 m | 0.788 m | 1.375 m |
| GNSS + GLIM, sigma floor 2 m | 0.689 m | 1.206 m |

Original-order GLIM/fused distances were 0.208/0.885 m over 140 pairs. Coverage
differs, so comparisons across replay modes are not strictly paired accuracy tests.

## Moving objects

The original-order run had moving candidates around 189-194 and 249-253 s.
After reordering, no segment in 180-270 s passes the main rule: >=4 s duration,
observation gaps <=0.75 s, >=10 samples, net displacement >=2 m, dimensions
p90 <=8 m, median compensated speed >=0.3 m/s. This does not prove no ship exists.

A separate exploratory inspection of 2-4 s segments finds:
- Track 142, 241.0-244.8 s: 10.34 m displacement, dimensions approximately
  2.7 x 0.8 m, tracker speed median approximately 2.55 m/s.
- Track 55, 188.9-192.7 s: 8.48 m displacement, dimensions approximately
  2.9 x 1.9 m, tracker speed median approximately 1.83 m/s.

These are unverified short candidates, not identified ships or validated speeds.
Changing centroids, shape, association switches and gaps remain possible. The
main continuity threshold was not relaxed to declare success. Snapshots show
tracker centers and nearby returns, without semantic labeling.

## Artifacts/reproduction

Under /home/ibo/rosbags:
- glim_aug10_motion_fusion: source.npz, timing.json, source_info.txt, reorder.log.
- aug10_acquisition_ordered: separate input bag and sibling JSON message counts.
- glim_aug10_full_20261014: original-order results and fusion outputs.
- glim_aug10_ordered_20261014: corrected-order results, fusion_input_bag,
  fusion_output_bag, fusion_comparison.json/png, scene_consistency.csv,
  track_samples.csv, short_moving_candidates.json/png, moving_scene.gif, logs.
- glim_aug10_ordered_20261014/conservative: weaker-GNSS comparison artifacts.

Use loopback CycloneDDS, domain 74 for GLIM and domain 75 for fusion.
Source extraction/timing: scripts/inspect_aug10_source.py and check_aug10_timing.py
(these intentionally target this specific bag).
Reorder: scripts/reorder_aug10_sensors.py SOURCE_BAG NEW_BAG.
Run robot/glim_static_bag_check.launch.py with reordered input and duration:=406.
Stop trial/recorder cleanly. Run prepare_aug10_fusion.py RESULT_DIR SOURCE_NPZ;
launch src/robot/launch/glim_gnss_comparison.launch.py result_dir:=RESULT_DIR
by absolute path, then cleanly stop after replay. Analysis uses PYTHONNOUSERSITE=1:
evaluate_glim_static_tracks.py RESULT_DIR; evaluate_aug10_fusion.py RESULT_DIR
SOURCE_NPZ; plot_aug10_short_tracks.py RESULT_DIR. Conservative analysis passes
/comparison/fused_conservative as third argument and uses a separate directory
with input symlinks, as in the saved artifacts. Do not replay source controls.

Validation: trials stopped; output odometry finite; ENU origin/east/north checks,
Python compilation and whitespace checks pass; selected sensor counts preserved.
Existing user changes are excluded. No push. A live solution needs bounded sensor
latency and stamped ego-odom buffering before tracking updates.

Reference for local/global separation:
https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/integrating_gps.rst

## Follow-up: approximately one-meter vessel candidates

Searching the complete acquisition-order result for clusters with median longest
extent 0.5-1.8 m identifies a stronger compact moving candidate outside 180-270 s:
track 220 at 337.02-340.72 s (5:37-5:41). There are 13 confirmed observations,
median dimensions 0.943 x 0.569 m, median 22 points, net displacement 6.99 m,
and estimated ground-speed median 1.73 m/s. Dimension p10-p90 ranges are
0.893-1.063 m long and 0.532-0.667 m wide. In three point-cloud snapshots the
compact isolated returns follow a nearly straight trajectory. It is the preferred
one-meter vessel candidate among those inspected, not a verified ship identity.
Review 330-355 s to include its intermittently observed surroundings.

Track 0 at 20.67-32.97 s has median dimensions 1.370 x 0.649 m and 37 observations,
but its dimensions and track path vary substantially (length p10-p90 0.886-3.265 m).
It is a weaker candidate with possible segmentation/association changes.

These are observed cluster extents, not measured hull dimensions. Speed and vessel
identity remain unvalidated. Artifact directory glim_aug10_ordered_20261014 contains
one_meter_candidates.png/json and one_meter_plot.py; the upper plot row is track 220.


---

# Jetson rosbag inventory and GLIM check — 2026-10-14 JST

Host: ibo@100.115.34.103 (ground Jetson).
Workspace: /home/ibo/njord2026_ws; branch: fix/agx-startup.
The existing ground station was left running. No live sensor/actuator graph was
started or modified. The trial used an isolated ROS domain 73 with loopback DDS.

## Bags containing LiDAR, IMU and GNSS messages

Ten bags with nonzero counts for all three canonical topics were found among
the workspace metadata and /home/ibo/rosbag2_2026_08_13-03_12_01.
Examples below use the prefix /home/ibo/njord2026_ws/ unless indicated.

| Bag | Duration s | /livox/lidar | /livox/imu | /sensor/vehicle_gnss/fix/raw |
|---|---:|---:|---:|---:|
| rosbag2_2026_07_21-16_22_30 | 586.6 | 5809 | 117152 | 11715 |
| rosbag2_2026_07_24-14_26_20 | 518.8 | 4862 | 101269 | 10376 |
| rosbag2_2026_08_10-03_52_00 | 405.4 | 3328 | 76670 | 8068 |
| /home/ibo/rosbag2_2026_08_13-03_12_01 | 542.5 | 3422 | 44702 | 5435 |

All are MCAP rosbag2 data. A topic listed with zero messages was not counted
as recorded data. The complete inspected inventory is saved in
/home/ibo/rosbags/glim_check_20261014/bag_inventory.csv.

## GLIM trial

The first 120 s of rosbag2_2026_07_21-16_22_30 was replayed once at real-time
rate, selecting only the three sensor topics and publishing /clock.
Current workspace config: src/robot/config/glim_config_headless.
GLIM executable is installed in /opt/ros/jazzy, package version 1.2.2;
there is no GLIM implementation source/build overlay in this workspace.
The GPU odometry module and headless ROS viewer extension loaded successfully.
Local and global mapping are disabled in this current configuration, so this
trial evaluates odometry, not a completed globally optimized map.
GNSS is recorded as reference only; this configuration does not fuse GNSS.

The inspected bag IMU acceleration norm was about 0.975 g, matched by the
config acc_scale=9.80665. /livox/imu was remapped into the configured
/livox/imu_raw input without modifying message timestamps. PointCloud2 fields
include x/y/z/intensity/tag/line/timestamp, with absolute nanosecond point times,
matched by perpoint_time_scale=1e-9. The live-workspace static Livox extrinsic
was published inside the isolated domain.

Results:

- Received 1190 LiDAR, 23854 IMU and 2383 GNSS messages.
- GLIM emitted 1181 odometry messages over 118.10 s, approximately 10 Hz.
- No nonfinite odometry or GLIM crash observed.
- Start position approximately (0.00096, 0.00438, 0.00096) m;
  end approximately (-0.00178, 0.01303, -0.00200) m.
- Summed frame-to-frame motion was 3.31 m, including estimation jitter; this
  is not a verified vessel travel distance or an accuracy metric.
- All 2383 GNSS messages had status=-2. No valid GNSS ground truth was established.
- At startup GLIM reported one 0.216 s IMU gap.
- Current map_frame_id and odom_frame_id are both odom. TF_SELF_TRANSFORM
  warnings persisted. Odometry was produced, but this TF configuration needs
  review before using the full TF graph. No config change was made in this test.

## Saved results

/home/ibo/rosbags/glim_check_20261014/

- output_bag/: finalized MCAP, 7108 messages, odometry/pose/TF/GNSS.
- trajectory.csv: timestamp and estimated position/quaternion.
- summary.json: counts, finite checks and trajectory summary.
- config_used/: copy of the tested workspace config.
- glim.log, play.log, odom-hz.txt, bag_inventory.csv.

All test processes were stopped after saving. Ground station remained running.
No bag file was modified, no hardware command topic was replayed, and no push
was performed.

## Reproduction

Set the environment before starting every test process:

```bash
cd /home/ibo/njord2026_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=$PWD/src/robot/config/cyclonedds_agx.xml
```

Start a static transform and GLIM in separate terminals with that environment:

```bash
ros2 run tf2_ros static_transform_publisher --x 0.5 --y 0 --z 0.8 \
  --roll 3.141592653589793 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id livox_frame
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=$PWD/src/robot/config/glim_config_headless \
  -p use_sim_time:=true -r /livox/imu_raw:=/livox/imu
```

Replay the selected inputs (one pass, no timestamp rewriting):

```bash
ros2 bag play $PWD/rosbag2_2026_07_21-16_22_30 \
  --topics /livox/lidar /livox/imu /sensor/vehicle_gnss/fix/raw \
  --clock 100 --playback-duration 120 --disable-keyboard-controls
```


---

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
CycloneDDS environment as docs/validation/glim_experiments.md:

```bash
ros2 launch robot glim_static_bag_check.launch.py \
  bag_path:=/home/ibo/njord2026_ws/rosbag2_2026_07_24-14_26_20 \
  output_dir:=/home/ibo/rosbags/NEW_RESULT_DIRECTORY duration:=60
# After playback and clean recorder shutdown:
PYTHONNOUSERSITE=1 python3 scripts/simulation/evaluate_glim_static_tracks.py RESULT_DIRECTORY
# This corner evaluator applies only to the documented July 24 geometry:
PYTHONNOUSERSITE=1 python3 scripts/simulation/evaluate_fixed_corner.py RESULT_DIRECTORY
```

Remaining work for other-ship recognition: label a compact stationary feature,
check association stability and centroid changes, queue/delay detection updates
until stamped ego odom is available, and test continuous moving/turning intervals.
GNSS fusion requires valid fixes and an independently validated map datum.
