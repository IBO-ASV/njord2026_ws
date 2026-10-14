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
