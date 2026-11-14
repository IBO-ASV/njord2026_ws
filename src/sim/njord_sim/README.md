# Hardware-free mission simulation

ROS 2 Jazzy CPU simulation through the production Mission Manager, Control
Manager, command arbiter and Nav2. No hardware sensor drivers, serial/CAN
transport or real actuators are started. Each run starts MANUAL / IDLE.
Interactive launches start Foxglove by default at `0.0.0.0:8767`; the automated
runner explicitly disables it so parallel runs do not compete for a GUI port.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select thruster_driver dutyed_tf_pub_with_disturbance njord_sim
source install/setup.bash
python3 scripts/simulation/run_mission_sim.py --task task1 --output /tmp/njord/task1
python3 scripts/simulation/run_mission_sim.py --task task1_2 --model mmg --output /tmp/njord/task1-mmg
python3 scripts/simulation/run_mission_sim.py --task task3_1 --noise --seed 2026 --output /tmp/njord/dock
python3 scripts/simulation/run_sim_suite.py --tasks task1,task1_2 --truth --workers 2 --output /tmp/njord/task1-suite
python3 scripts/simulation/run_sim_suite.py --tasks task2 --encounters head_on,port,starboard,none --truth --workers 1 --output /tmp/njord/task2-suite
python3 scripts/simulation/run_sim_suite.py --tasks task3_1,task3_2,task4 --truth --workers 1 --output /tmp/njord/docking-suite
```

Tasks: `task1`, `task1_2`, `task2`, `task3_1`, `task3_2`, `task4`.
Noise seeds default to 2026/2027/2028 in the suite. Add `--disturbance` for
an additional model disturbance. `--domain` defaults to 89; suite workers
use separate domains starting at 100. Use domains not in use by another run.
The runner cleans only its own process groups, including orphaned children.
A run has a bounded timeout and returns nonzero on failed acceptance.

Task2 additionally needs CPU PyTorch in the same Python interpreter as ROS
(`python3-torch`, where available, or a compatible persistent Python installation),
the existing `asv_trajectory_planner`, `task2_perception`,
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

Task2 costmaps additionally use `/sim/costmap_clearing_scan`, a clear-only
360-degree scan bounded by current physical objects. It supplements the existing
100 m costmap stand-in; the recognition LiDAR remains limited to 40 m. Costmap
display streams publish at approximately 5 Hz. Acceptance runs save a separate
`costmap_audit.json`; see [the correction and trial record](../../../docs/validation/task2_costmap.md).

For interactive use, install the bridge (`sudo apt install
ros-jazzy-foxglove-bridge`). Without sudo, this workspace provides a local
runtime install: `python3 scripts/setup/install_foxglove_bridge_local.py` (a no-op when
the system bridge is already installed), then source
`install/setup.bash` again. This install persists across reboots.
For Task2, verify `python3 -c 'import torch; print(torch.__version__)'`.
When CPU PyTorch is missing, run `python3 scripts/setup/install_task2_dependencies.py`
with pip available for that interpreter. Some AGX apt sources do not provide
`python3-torch`, so do not assume rosdep or a Nav2 build installed it. The
installer uses the official CPU wheel index and the persistent Python user site.
An already unpacked CPU environment can be migrated without another download:
`python3 scripts/setup/install_task2_dependencies.py --from-directory PATH`.
Its source can be temporary; the resulting installation is copied to
`~/.local/lib/python3.12/site-packages` on this Python 3.12 AGX. Runtime does not
require that temporary source, a PYTHONPATH export or a separate shell setup.
Build the dependency packages named above before running Task2.


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

The default model is MMG with the production velocity controller and thruster
allocator. Rebuild `thruster_driver`, `dutyed_tf_pub_with_disturbance` and `njord_sim`
before using this default. `--model kinematic` explicitly selects the lightweight
0.3 s velocity-response stand-in and bypasses thruster control. Neither model
is calibrated against the real vessel.

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

See `docs/validation/simulation_tuning_log.md` and `docs/validation/simulation_acceptance.md` for
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


## Editing a simulated course on the GNSS map

The launch includes `mission_manager/course_editor_node` with the selected
`sim_task`. GNSS Map Telemetry offers waypoint/buoy dragging, buoy type selection,
addition/removal and explicit YAML save while MANUAL/IDLE. Edits persist per task
in `config/course_overrides.yaml`; restart the sim after saving to apply the same
geometry to environment, generated mission route, costmaps and scoring. This is
separate from the real waypoint YAML editor. See the
[map editor instructions](../../gui/foxglove_extensions/gnss_map_telemetry/README.md).


### Task2 appears stuck after opponent-selector warnings

`No valid opponent track or bounded straight-line coast` describes unavailable
opponent input, not a Nav2 lifecycle failure. With this sim's
`require_other_ship=false`, MPPI can plan without an opponent; selection starts
when a valid moving track enters range. If `/planned_path_pruned` has no
publisher, first check whether `planner_node` exited (in particular because
`import torch` failed). Nav2 servers can already be active while the MPPI path
producer is absent, leaving `/autonomy/ready=false` and Task2 unable to start.
Task2 launch now checks a real CPU PyTorch operation before bringing up nodes,
and reports the installer command if that check fails. Task1/3/4 do not run this
check. Restart the failed launch after installing dependencies; an exited planner
process is not restored by changing Python packages beneath the remaining nodes.


## Software readiness follow-up (2026-11-13)

The suite accepts `--model mmg`, `--disturbance`, `--initial-x-offset`,
`--initial-y-offset` (metres in odom), and `--initial-yaw-offset` (radians).
Offsets change the initial vessel pose, not the route or scoring geometry.
Use separate output directories for every matrix; failed results remain failures.
For example, a truth-only full MMG matrix and perturbed Task2 matrix:

```bash
python3 scripts/simulation/run_sim_suite.py --model mmg --truth --seeds '' \
  --workers 2 --domain-base 150 --output artifacts/mmg-matrix
python3 scripts/simulation/run_sim_suite.py --tasks task2 --seeds 2026 \
  --encounters head_on,port,starboard,none --opponent-variable-speed \
  --initial-x-offset 0.5 --initial-y-offset 0.3 --initial-yaw-offset 0.15 \
  --detection-loss-period 15 --detection-loss-duration 2 \
  --workers 2 --domain-base 160 --output artifacts/task2-robustness
```

`--opponent-variable-speed` adds a continuous 30 s sinusoidal speed variation
of ±0.4 m/s around the existing 1.2861 m/s NPC speed. Truth velocity is the
analytic derivative of truth position. Periodic detection loss begins after
one period and removes only NPC hull returns from the recognition LiDAR.
Physical obstacle clouds, clearing occlusion and evaluator truth retain the
NPC. This isolates tracker/opponent-selection resilience; it does not simulate
loss of every collision sensor, general visibility or measured vessel statistics.
A period/duration of zero disables loss. Existing `--dropout-after` instead
simulates a persistent health/sensor failure and requires a safe canceled stop.

`/sim/estimated_path` now uses retained depth-one QoS. It publishes at most
2 Hz and adds a pose only after 0.10 m displacement or 0.05 rad rotation from
the last recorded sample, suppressing stationary jitter. Above 600 poses,
the history is decimated while retaining its beginning and latest endpoint.
Late subscribers receive the latest complete path without periodic idle sends.
This display sampling never changes the full-rate truth trajectory used for
scoring. The common Foxglove layout hides individual obstacle-layer grids and
`/received_global_plan`; combined local/global costmaps remain visible. Reimport
the layout and restart the sim to apply these changes to an existing session.

## Velocity qualification and vessel identification

Mission completion alone does not qualify velocity tracking. The kinematic
model prescribes a 0.3 s velocity response and bypasses thruster control.
The MMG path runs the production `thruster_driver` velocity controller and
allocator. Its former P-only settings leave drag-dependent steady errors.
The simulated MMG launch now adds `config/velocity_control.yaml`: P/I gains,
a reference filter, per-axis wrench limits and allocation anti-windup.
Qualification uses DOB disabled. Hardware defaults remain P-only (I=0, filter=0). Integral gains/filter are
startup parameters; command/feedback watchdogs reset both stored states.

Targets are body-frame surge `u`, sway `v` [m/s], and yaw rate `r` [rad/s].
For world-frame desired derivatives use
`u = cos(yaw)*xdot + sin(yaw)*ydot`,
`v = -sin(yaw)*xdot + cos(yaw)*ydot`, `r = yawdot`.
A zero yaw-rate target is not an absolute heading-hold controller; use the
outer navigation/heading controller for that. Transient cross-axis response
and thrust imbalance must be measured separately.

Build the actual controller/model first, then qualify isolated velocity
tracking before navigation (all commands below run from the workspace root):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select thruster_driver dutyed_tf_pub_with_disturbance njord_sim
source install/setup.bash
python3 scripts/calibration/calibrate_velocity.py --plot --output artifacts/new-calibration
# After identifying a real model: add --mmg-config artifacts/real-mmg/candidate.yaml
python3 scripts/calibration/check_velocity_ros.py --output artifacts/new-ros-loop
python3 scripts/simulation/run_sim_suite.py --model mmg --truth --seeds '' \
  --encounters head_on,port,starboard,none --workers 2 --output artifacts/new-mmg-missions
```

The accelerated C++ benchmark shares the production controller header,
allocator and MMG equations. It publishes no topics and never starts serial
hardware. Positive/negative axis steps, stops/reversals and a coupled waveform
score steady bias/RMSE (0.05 m/s, 0.05 m/s, 0.03 rad/s), settling <=6 s,
active-axis overshoot <=20%, dynamic RMSE (0.15,0.10,0.10) and final residual
velocity (0.03,0.03,0.02). Gains are selected on three plants; nine independent
holdouts vary mass, added mass/inertia, damping, actuator scale and delay,
including a 24 kg loaded model. The real-time ROS check additionally exercises
parameter loading, driver subscriptions, 50 Hz feedback and the actual node.
It uses only isolated `/calibration/*` topics, no hardware transport.

Each MMG mission copies its selected plant and velocity YAML into `config/`
and writes `velocity_tracking.csv` plus body-axis RMSE/p95 in evaluation JSON.
These include acceleration and turns and are observational mission metrics;
independent step/sweep criteria qualify the low-level controller. Change the
plant/controller explicitly with `--mmg-config PATH` and
`--velocity-control-config PATH` on either mission or suite runners.
Controller tuning does not validate perception-loss recovery or path safety.

### Pair thrust balance before model identification

Use physical forces, not electrical channel signs. Channel order is
**RR, RL, FR, FL**. With the current geometry:

| Pair | Positive physical thrust | Relative adjustment |
|---|---|---|
| FR + FL | forward | FR anchor, vary FL |
| FL + RL | sideways toward -y | keep FL reference, vary RL |
| RL + RR | backward | keep RL reference, vary RR |

Making the last pair go forward requires negative physical thrust on both
motors and tests their reverse response. Three fixed equal-pulse pairs do not
identify all eight forward/reverse gains. Sweep the adjustable motor through
several levels, repeat all three pairs in both directions from near rest,
include zero-force coasts, and reserve independent levels/repeats for holdout.
The generated `pair_plan` provides this design. Pair tests must bypass the
normal wrench allocator, which otherwise redistributes the requested wrench
to four motors; the offline benchmark has an explicit per-thruster mode.

The fit uses short near-rest yaw acceleration, not GPS track slope. It solves
relative forward/reverse response with a nuisance yaw-acceleration bias and
checks rank/signs and independent holdout error. Corrections reduce stronger
channels to the weakest and map physical directions back through each channel's
wiring reversal to the driver's electrical `forward_gain` / `reverse_gain`.
Known synthetic mismatches are validated by re-running all six pair directions
and checking cross-force fraction <1% and yaw-moment/force residual <0.005 m.
This is a scalar-gain model: mounting angle errors, different spool-up dynamics,
current, heading bias and geometry changes require separate checks. Repeat
under the intended voltage/loading and check both yaw and transverse motion.

### Reuse measured data

The recorder is passive by default; no thruster transport or controller is
started. It accepts only `base_link` twist, logs timestamps, command/force ages,
yaw, body velocities and inferred physical wrench. The default electrical-to-
physical wiring signs are `-1,1,1,-1` in RR/RL/FR/FL order; pass explicit signs
for other verified wiring. Passive pair recording does not require `/cmd_vel`.

```bash
ros2 run njord_sim velocity_experiment --output real_train.csv --duration 180
# Explicit optional velocity excitation via the existing MANUAL control path:
ros2 run njord_sim velocity_experiment --output real_speed.csv \
  --reference artifacts/new-calibration/velocity_reference.csv \
  --publish-topic /cmd_vel_manual --command-topic /cmd_vel
```

Publishing requires an explicit topic and velocity reference. It zeros output
and latches an abort on feedback loss; clean termination sends zero commands.
The full generated reference includes +/-1.029 m/s surge and 0.4 m/s sway;
choose the actual field-test amplitudes/area before activating it. Pair plans
are not automatically published to real thrusters by this recorder; use a
controlled individual-actuator trial and record it passively. The existing
single-motor `thruster_id_test` is not a paired balance experiment.

```bash
python3 scripts/calibration/identify_vessel.py --balance \
  --training real_pair_train.csv --holdout real_pair_holdout.csv \
  --output artifacts/real-balance
python3 scripts/calibration/identify_vessel.py \
  --training real_train.csv --holdout real_holdout.csv \
  --mass "$VESSEL_MEASURED_MASS_KG" --iz "$VESSEL_MEASURED_IZ" \
  --output artifacts/real-mmg
```

Balance fitting emits a complete `candidate.yaml` preserving the supplied
`--driver-config` channel order, geometry, safety and base settings. MMG fitting
emits a plant candidate only if excitation rank and independent force-prediction
error pass. These tools write candidates; they never apply them to hardware.
Do not replace a complete driver config with a sparse simulation overlay.

The MMG fitter uses integrated force/motion balances, avoiding pointwise noisy
velocity differentiation. It currently requires x_G=0, known Lpp/draft,
calibrated applied thrust and water-relative body velocity in steady/sheltered
water. Dry mass and dry yaw inertia must be measured anchors: logs identify
`mass+m_x`, `mass+m_y`, `I_zG+J_z`, not their components separately. Relative
thrust balance does not identify absolute Newtons. For identified real thrust,
pass `--balance-report artifacts/real-balance/report.json` and
`--force-anchor-scale` (measured FR-positive actual N / command N) to the recorder.
Its `command_rr/rl/fr/fl` columns retain the signed physical commands, while
`tau_x/y/n` then use the calibrated directional responses and absolute anchor.
Without these options the wrench is nominal command Newtons. With only nominal command
Newtons the result is an effective command-response model, not independently
measured physical hydrodynamic coefficients. Unknown current, force scale and
CG offset cannot be silently folded into a claim of physical identification.


Normal mission acceptance retains two seconds of current continuous stationary
truth at the final goal. After action completion the observer waits at least
four seconds and, if needed, up to ten seconds for that condition. Motion resets
the duration; a prior stationary interval cannot substitute for the final stop.
Cancellation/sensor-loss checks retain their separate body-stop and zero-output
criteria rather than requiring arrival at the route's final goal.
