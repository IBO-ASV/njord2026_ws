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
