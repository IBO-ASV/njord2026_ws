# Hardware-free mission simulation

ROS 2 Jazzy CPU simulation through the production Mission Manager, Control
Manager, command arbiter and Nav2. No hardware sensor drivers, serial/CAN
transport or real actuators are started. Each run starts MANUAL / IDLE.
Interactive launches start Foxglove by default at `0.0.0.0:8767`; the automated
runner explicitly disables it so parallel runs do not compete for a GUI port.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --paths src/sim/njord_sim
source install/setup.bash
python3 scripts/run_mission_sim.py --task task1 --output /tmp/njord/task1
python3 scripts/run_mission_sim.py --task task1_2 --model mmg --output /tmp/njord/task1-mmg
python3 scripts/run_mission_sim.py --task task3_1 --noise --seed 2026 --output /tmp/njord/dock
python3 scripts/run_sim_suite.py --tasks task1,task1_2 --truth --workers 2 --output /tmp/njord/task1-suite
python3 scripts/run_sim_suite.py --tasks task2 --encounters head_on,port,starboard,none --truth --workers 1 --output /tmp/njord/task2-suite
python3 scripts/run_sim_suite.py --tasks task3_1,task3_2,task4 --truth --workers 1 --output /tmp/njord/docking-suite
```

Tasks: `task1`, `task1_2`, `task2`, `task3_1`, `task3_2`, `task4`.
Noise seeds default to 2026/2027/2028 in the suite. Add `--disturbance` for
an additional model disturbance. `--domain` defaults to 89; suite workers
use separate domains starting at 100. Use domains not in use by another run.
The runner cleans only its own process groups, including orphaned children.
A run has a bounded timeout and returns nonzero on failed acceptance.

Task2 additionally needs CPU PyTorch (`python3-torch` or a compatible Python
installation), the existing `asv_trajectory_planner`, `task2_perception`,
`pcl_preprocessing`, `pcl_segmentation`, `ship_tracking`, and
`ship_perception_bringup` installed. `--encounter` selects `head_on`, `port`,
`starboard`, or `none`. It uses synthetic 3D hull/water returns through the
production filter, ground removal, clustering, tracker and opponent selector;
MPPI consumes their output, not the other vessel's truth pose. This launch
also enables an optional observed-object path guard: it rejects predicted
hull/gate conflicts, tries smooth GPS-relative lane alternatives, and cancels
FollowPath when no candidate is safe. The production default is disabled;
`path_guard_enabled`, COG-heading inference and starboard cost require separate
real-vessel assessment. The synthetic port-side NPC never yields, so this
fixture includes emergency avoidance rather than assuming stand-on priority
alone prevents collision.

For interactive use, install the bridge (`sudo apt install
ros-jazzy-foxglove-bridge`). Without sudo, this workspace provides a local
runtime install: `python3 scripts/install_foxglove_bridge_local.py` (a no-op when
the system bridge is already installed), then source
`install/setup.bash` again. This install persists across reboots. CPU PyTorch can be installed with
`sudo apt install python3-torch`. Build the dependency packages named above
before running Task2.


```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch njord_sim mission_sim.launch.py task:=task1 noise:=true
# In a terminal with the same ROS_DOMAIN_ID / RMW:
ros2 run mission_manager njord-task start task1 --auto
```

Connect Foxglove directly to `ws://<AGX-IP>:8767` from the viewing PC and import
`foxglove_setting.json`. This is a WebSocket endpoint, not an HTTP-hosted GUI.
Use `foxglove_address:=127.0.0.1` for loopback plus SSH forwarding, or
`foxglove:=false` to disable the bridge. `foxglove_port:=8768` selects another
port for a second interactive run. The Mission Control panel lists the
run's task and shows progress, permission/inhibition, effective command source,
health and the retained last result. Start requests AUTO; stop uses the active
execution ID. State older than 3 s disables start. The map publishes the private
sim route and noisy estimate; `/sim/truth/odom` stays available for inspection.
Battery telemetry is an explicitly synthetic display value.

The default model is an explicitly labelled kinematic model with a 0.3 s
velocity response. `--model mmg` uses the existing C++ MMG / T200 force model
and the production thruster allocator/controller. Neither is a calibrated
model of the real vessel.

## Coordinates and sensors

Task1 competition simulation uses the surveyed ENU geometry already stored
in `task1_sim/config/task1_params.yaml`, including GPS1, intermediate points,
GPS3, cardinal buoys and GPS4. The current live waypoint YAML holds a different,
short field-test route; it is deliberately not overwritten. The private sim
registry is generated with inverse-ECEF-projected geographic waypoints. Other
tasks use their current surveyed route. Environment, /fromLL and evaluator
share the same scenario datum. The generated YAML is saved per run.

`/sim/truth/odom` is for independent scoring. Navigation sees
`/odometry/filtered/local`, `/odometry/filtered/global`, `/odom` and their TF,
with optional Gaussian pose noise and random-walk bias. GNSS, dual-antenna
heading, IMU, range/FOV-limited buoy detections and point clouds have real
message/frame contracts. This tests downstream interfaces; it does not run
GLIM, optical recognition or qualify their real-world accuracy. TF is owned
only by the sensor stand-in, with identity map -> odom for this local ENU run.

The footprint is the existing URDF's 1.5 m x 1.0 m rectangle. Task3 berth
geometry is open at the approach side, 2 m x 2 m normal / 4 m x 2 m parallel.
Normal berth orientation follows the surveyed approach-to-berth vector in
the private sim route because the dock structure itself is invented. Parallel
berth orientation follows the requested yaw, with the mouth on the approach
side. Neither adjustment changes real-vessel waypoint YAML.
Task4 transit buoys are an explicit synthetic fixture, not surveyed obstacles.
Task2 gate pairs are synthetic 5 m gaps at 15% and 85% of the GPS5 -> GPS6 route.

## Acceptance and artifacts

Each run writes `run.json`, `config/`, `launch.log`, `observer.log`,
`trajectory.csv` and `evaluation.json`. Success requires the Mission action
result, sequential waypoint visits, no hull contact, correct cardinal side
at closest approach within 10 m, and at least 2 s of final stationarity.
Task1/1.2 additionally require a nonempty `/virtual_obstacles` cloud while the
canonical Task1 wall gate is enabled; the summary records its maximum point count.
The action result is authoritative because an immediately followed IDLE
status can overwrite a terminal transient-local sample.

Dock dwell starts after 2 s stationary settling, then counts 10 s normal / 5 s
parallel. Position must remain within 0.8 m, heading within 10 degrees, speed
within 0.10 m/s, and yaw rate within 0.05 rad/s. Loss resets the continuous
interval; missing odometry fails closed. The wait is bounded by 120 s plus dwell.
The independent evaluator also requires the entire truth footprint inside
the berth and checks dock segment contact. Nav2's docking goal is stricter:
0.15 m XY / 5 degrees yaw; the broader stationary window does not relax the
whole-hull criterion. Mission parameters prefixed `docking_stationary_`,
`docking_goal_yaw_tolerance_rad`, and `docking_settle_time_sec` expose these defaults.
ThetaStar must preserve requested goal orientation (`use_final_approach_orientation:
false`), and the docking smoother preserves endpoint quaternion.

Task2 additionally requires both gates, a selected moving opponent (unless
`none`), >= 0.5 m hull separation, a starboard deviation for head-on encounters,
and passing astern for the starboard crossing fixture. These are scenario
acceptance checks, not a complete COLREG certification.

Add `--plot` to create `trajectory.png` after saving JSON. To plot an existing
run without ROS or rerunning navigation (requires matplotlib):

```bash
python3 -m njord_sim.plotting /tmp/njord/dock
```

This uses saved `config/scenario.json`, never a newly generated fixture. JSON
is saved before optional plotting so a missing/slow plot cannot hide a result.
Use at most two concurrent runs on the tested 8-core host; one is preferable
for full docking suites. Cancellation and sensor dropout can be checked with
`--cancel-after 25` and `--dropout-after 25` respectively. Both require typed
cancellation and zero final command, rather than a successful mission.

See `Docs/simulation_tuning_log.md` and `Docs/simulation_acceptance.md` for
observations, changes, measured results and remaining limitations.

## Task-specific ROS endpoints and course bounds

The common Foxglove layout is shared by all tasks. Task2 processors were already
launched only for `task:=task2`; its sensor stand-in now also registers
`/planned_path_pruned`, `/task2/points_filtered`, `/mission/task2/enabled`,
`/sim/truth/opponent` and the PointStamped `/buoy_detections` only in Task2.
Other tasks retain the BuoyDetectionArray used by cardinal-wall perception.
Mission Manager creates Task2 enable/completion endpoints when Task2 starts,
removes its completion subscription when stopped, and retains the disabled
publisher after first use so late Task2 clients receive `false`. Completion
uses volatile durability to avoid replaying a prior run's retained success.
The Mission Control panel subscribes to `/mission/task2/path_safe` only while
Task2 is active; use the rebuilt `mission-control-0.1.0.foxe` with this change.
A private registry with no staged docking tasks also omits dock correction
subscriptions and nominal berth publishers.

This does not impose a blanket publisher-present rule. Manual/SBUS inputs,
failsafe triggers, Nav2 optional inputs and runtime status can have publishers
that join later. Their subscriptions remain available. Waiting for a publisher
on every topic can also break producers that start publishing only after a
subscriber appears. The running launch must be restarted to apply endpoint
construction changes; an already running process is not changed by editing files.

The legacy `task1_sim` rectangular boundary became visualization-only in
commit `55c5982e` (2026-08-02 JST). Its old lethal obstacle perimeter at y=15
crossed waypoints 9 to 10 and prevented navigation; that commit removed
`_append_course_boundary(points)` from `build_sim_obstacle_points`. The legacy
sim still publishes `/sim/boundary_markers`; its current configured bounds are
`[-30, 30, -55, 15]` metres. This new `njord_sim` does not currently reproduce
that display or enforce a rectangular geofence. Its cardinal-wall
`course_bounds=[-150,150,-150,150]` only bounds generated virtual walls (also
capped at 13 m); it is not a boat operating-area constraint. Restoring an
operating-area constraint requires an explicit polygon and checking that the
surveyed route and avoidance space lie within it; the former synthetic obstacle
perimeter has not been reinstated as part of endpoint cleanup.

Validation: ROS endpoint construction for Task1/Task2, Task2 stop/restart and
rejection of retained completion from a previous run are covered by
`test_task_endpoints.py`. GUI tests cover switching Task1 -> Task2 -> Task1,
Task2 restart and completion using the same panel.

Waypoint display update: the selected sim route is now previewed from launch,
including while IDLE and after completion. `/ground_waypoint_markers` carries
WGS84 markers for GNSS Map Telemetry; `/sim/waypoint_markers` carries odom-frame
points, numbered labels and a route line for the standard 3D display. Both are
published at 1 Hz with transient-local durability for late-joining viewers. The
common layout enables the latter and `/mission/waypoint_markers`. The previous
sim behavior deleted geographic markers outside active mission states.
