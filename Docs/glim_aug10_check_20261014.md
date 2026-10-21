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
