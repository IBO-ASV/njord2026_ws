# Task 2 GLIM integration

## Data flow
LiDAR + IMU -> GLIM `/task2/glim/raw_odom` -> base lever-arm/world-to-body
velocity adapter `/task2/glim/base_odom` -> local planar EKF
`/task2/odometry/local` -> ship tracker AND opponent selector -> `/other_ship/twist`.
GLIM TF stays private. Local EKF is the sole odom->base_link authority.
Raw GLIM twist has world-frame velocity and zero yaw rate in the installed
version; feeding it directly to Task 2 is incorrect. The adapter converts the
velocity and estimates yaw rate from consecutive orientations.

## Perception-only entry point
On Jetson, after sourcing ROS and this workspace:

```bash
ros2 launch robot task2_glim_perception.launch.py
```
This starts GLIM, adapter, EKF, point filtering, clustering, tracking and
opponent selection, but no sensor drivers, planner or control commands.
Defaults: /livox/imu in SI (acc_scale=1), /livox/lidar; base_link->livox_frame
must already be supplied by robot state publisher. Local odom is the perception
world frame and geographic corridor gating is disabled for this evaluation entry.
Do not run beside another Task 2 stack, another GLIM consuming the same sensors,
or an existing odom->base_link EKF in the same ROS domain. This is an explicit
opt-in integration, not a change to existing ground-station startup defaults.
Existing robot live nodes were not restarted or reconfigured.

The canonical `task2_jetson_autonomy.launch.py` now supports
`ego_odom_source:=glim`; both consumers and the optional planner select
`/task2/odometry/local`. Its default remains external for compatibility.
`start_planner:=false` suppresses the planner. Full navigation needs a correct
map->odom GNSS/global alignment and corridor waypoints; do not substitute an
identity alignment for real geographic navigation.

Node-scoped task2 YAML is merged with the selected odom/frame/corridor overrides
into a temporary file. This fixes a tested precedence problem where wildcard
launch overrides left opponent_selector on /odometry/filtered/local while the
tracker had switched to GLIM. Shared configs remain unchanged.

## Bag verification
Use an isolated domain, preserving measurement timestamps (never preview-restamped
points for GLIM). For the historical Aug 10 bag, IMU acceleration is in g:

```bash
ROS_DOMAIN_ID=78 ros2 launch robot task2_glim_perception.launch.py \
  use_sim_time:=true glim_acc_scale:=9.80665 publish_livox_mount_tf:=true
ROS_DOMAIN_ID=78 ros2 bag play /home/ibo/rosbags/aug10_acquisition_ordered \
  --clock 100 --disable-keyboard-controls --topics /livox/lidar /livox/imu
```
Start GLIM at the beginning for initialization. Do not loop raw input without
restarting GLIM and the filters. The optional mount TF matches this recorded
vessel configuration: (0.5,0,0.8), roll pi.

2026-10-14, 25-second subscription check during a 60-second replay:
- GLIM raw odom: 250; converted base odom: 249; local EKF odom: 249.
- Tracked-object arrays: 85; maximum 11 objects in a frame.
- /other_ship/twist messages: 224. This count alone does not establish a valid
  selected vessel or accurate velocity; zero/no-selection outputs are possible.
- Both ship_tracker_node and opponent_selector subscribed to the same GLIM local odom.
- /cmd_vel publishers: zero.
- Build and Python compilation passed. Integration test processes stopped afterward.
- Logs: /home/ibo/rosbags/task2_glim_integration_v2.log and player_v2.log.

## Remaining limits
The adapter is planar and uses assumed fixed covariances from the prior bag
check, not calibrated uncertainty. GNSS absolute drift correction, roll/pitch/heave
tracking compensation and physical sensor calibration need separate validation.
Classical LiDAR clusters are not semantic boat classification. Real vessel
recognition accuracy, ground-speed accuracy and live AGX/Jetson sensor/network
performance have not been validated by this launch test. Existing Foxglove
preview continues to replay the earlier processed GLIM results in domain 76.
