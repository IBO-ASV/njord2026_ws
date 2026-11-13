# Task3.2 continuous docking reference

Task3.2 keeps GPS11 as a separate stopping goal. Only GPS11 → berth2 and
berth2 → GPS11 use `DockingSE2`, a Nav2 Controller plugin. Task3.1 and outside
docking stages retain DWB. The waypoint YAML's `se2_docking_stages` selects
these stages; the executor sends them through FollowPath rather than through
a planner/smoother that could overwrite bow orientation.

GPS11 yaw faces the surveyed GPS11 → berth2 direction (approximately −115°).
Berth2 retains its approximately 167° bow heading. Entry turns clockwise by
approximately 78°, while exit reverses the heading change. Coordinates are
unchanged. This is a geometric reference, not a schedule of absolute arrival
times.

## Reference and controller

Each docking action samples a straight position reference from fresh current
odometry to the requested target, with bow heading interpolated along the
shortest angular displacement using `3s² − 2s³`. A corrected berth target is
applied before reference generation. A retry starts from the new actual pose.

The controller advances along the reference with bounded speed and slows its
progress when position or heading lag increases. World-frame reference velocity
plus proportional position feedback is rotated into the current vessel body
frame. Yaw-rate feedforward plus wrapped yaw-error feedback runs concurrently.
At the end of the reference feedforward becomes zero and feedback converges to
the terminal pose. The existing thruster driver's velocity feedback and allocation
remain the inner loop.

`DockingSE2` parameters in the Task3 Nav2 YAML are read at configuration time:

| Parameters | Defaults | Meaning |
|---|---|---|
| `reference_speed`, `max_speed` | 0.20, 0.25 m/s | Progress and command speed |
| `max_yaw_rate` | 0.35 rad/s | Command yaw-rate limit |
| `max_acceleration`, `max_yaw_acceleration` | 0.25 m/s², 0.5 rad/s² | Command slew limits |
| `position_gain`, `yaw_gain` | 0.8 /s, 1.2 /s | Pose feedback gains |
| `position_lag`, `yaw_lag` | 0.35 m, 0.25 rad | Progress slows above these errors and pauses at twice these errors |
| `collision_horizon` | 1.5 s | Constant-command footprint prediction |

The plugin transforms reference and measured poses into consistent frames.
It checks the padded costmap footprint along predicted translation and rotation,
rejecting lethal, unknown, or out-of-map footprints. Rejection follows Nav2's
normal stop/failure handling. The command still passes through the existing
velocity smoother, collision monitor, arbiter and safety gates. This prediction
is a kinematic check; it is not a certified braking model of the real vessel.

Docking uses a separate, non-stateful goal checker (0.15 m and 5°). Existing
stationary berth dwell checks remain in force. Normal BT FollowPath nodes name
`general_goal_checker` explicitly so adding a second checker does not make
normal navigation ambiguous. Cancellation before action acceptance is also
forwarded when the action handle arrives.

## Inspection and validation

- `/task/plan` shows the reference supplied to FollowPath.
- `/controller_server/docking_reference` publishes the current reference pose.
- `/controller_server/docking_tracking_error` publishes position error magnitude
  in `twist.linear.x` (m) and wrapped yaw error in `twist.angular.z` (rad).
- The common simulation evaluator saves `docking_tracking.csv` for Task3.2:
  reference/actual pose, final command, actual speed/yaw rate and the minimum
  hull-corner-to-wall distance. Wall contact is independently checked with the
  entire oriented hull. Acceptance also requires simultaneous translation and
  yaw samples on both entry and exit.

Run truth cases concurrently in isolated ROS domains:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/simulation/run_sim_suite.py \
  --tasks task3_1,task3_2 --truth --seeds '' --workers 2 \
  --output /tmp/njord/task3-se2-truth
python3 scripts/simulation/run_mission_sim.py \
  --task task3_2 --noise --seed 2026 \
  --output /tmp/njord/task3-se2-noise
```

The default simulation is the kinematic model. Passing these runs does not
qualify MMG tuning or real-vessel tracking.

## Validation record — 2026-11-13

| Run | Result | Dock contacts | Simultaneous motion samples (entry / exit) |
|---|---|---|---|
| Task3.1 truth, parallel domain 135 | PASS | 0 | Existing DWB behavior |
| Task3.2 truth, parallel domain 136 | PASS | 0 | 276 / 272 |
| Task3.2 noise, seed 2026, domain 137 | PASS | 0 | 276 / 268 |
| Task3.2 cancellation at 50 s, domain 138 | PASS | 0 | Canceled during entry |

Evidence is saved under `/tmp/njord/se2-docking-validation`,
`/tmp/njord/se2-docking-noise2026`, and `/tmp/njord/se2-docking-cancel`.
Maximum reference-to-truth position error during entry/exit was 0.138 m in
truth and 0.140 m with noise; maximum wrapped yaw error was 1.64° and 2.83°,
respectively. These include asynchronous reference/measurement sampling.
Berth stationary dwell was 5.76 s / 5.86 s in Task3.2 truth/noise, and
10.50 s in the Task3.1 regression. Canceling during entry left all three final
command components at zero.

The controller and tests build with ROS 2 Jazzy. CTest passes all five robot
package test targets, including three new control-law cases. At the time of this test, the standard
colcon build of `robot` was blocked locally by the existing `ntripcaster`
dependency's missing `wsocket` / `ulog` submodules. The software-readiness
follow-up on 2026-11-13 initialized them, added bundled-libev fallback and
verified the normal colcon build; see [acceptance follow-up](simulation_acceptance.md). The robot package was
configured, built and installed directly with CMake; Mission Manager,
waypoint_publisher, task1_sim and njord_sim were built with colcon.

Python regression across `robot`, `mission_manager` and `njord_sim` produced
183 passing tests and one existing Nav2 costmap discovery timeout. The isolated
retry with matching `ROS_LOCALHOST_ONLY=1` / `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
on both sides passed (1 test). The 14 executor/action-adapter tests also passed
after the final stale-action-response handling adjustment.
