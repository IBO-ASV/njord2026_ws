# Control and Mission Refactor Baseline

This document records the ownership and behavior observed before replacing the
legacy task-launch path.  It is deliberately descriptive: it is the regression
reference for the incremental migration in
`control_mission_refactor_implementation_plan.md`.

## Runtime ownership

| Launch | Persistent owners / notable task owners |
| --- | --- |
| `minipc_bringup.launch.py` | localization, UM982/Spatial, thrusters, BMS, back camera, `joy_converter`, `command_arbiter`, lamp, diagnostics, networking; Nav2 is optional and disabled by default. |
| `jetson_bringup.launch.py` | LiDAR, ZED/perception, GLIM and Jetson networking. |
| `ground_pc.launch.py` | joystick driver, ground-station heartbeat, video receivers, route display and networking. |
| `task1.launch.py` | miniPC or standalone role bringup, delayed Nav2, waypoint publisher and cardinal-wall publisher. |
| `task2.launch.py` | miniPC or standalone role bringup, delayed Task-2 Nav2 and waypoint publisher. |
| `task3.launch.py` | miniPC or standalone role bringup, delayed Task-3 Nav2, waypoint publisher and cardinal-wall publisher. |

The task launches therefore duplicate role ownership when used after a role
bringup.  The replacement owner for task selection is `mission_manager`; role
bringup remains the owner of hardware, control and the resident Nav2 graph.

## Legacy command truth table

`simple_manual/command_arbiter_node` publishes zero unless one of these cases
applies.  Emergency stop has priority in every case.

| E-stop | mode | other condition | output |
| --- | --- | --- | --- |
| asserted | any | any | zero |
| clear | manual | fresh `/cmd_vel_manual` | manual command |
| clear | auto | `/autonomy/ready` and fresh `/cmd_vel_nav` | Nav2 command |
| clear | any | otherwise | zero |

Phase 5 preserves this external safety behavior while making permission and the
reason for zero output typed and observable.

## Waypoint baseline

The legacy `waypoint_publisher` maps `task1`, `task1_skip_1_1`, `task2`,
`task3_1`, and `task3_2` to the YAML files in
`navigation/path_generator/waypoint_publisher/config`.  Tasks 1 and 2 submit a
single `NavigateThroughPoses` goal; Task 3 progresses through gate, dock,
berth-wait, next-berth and final-exit stages.  The mission executors must retain
these routes until their equivalence tests pass.

## Nav2 profile inventory

The Humble files are intentionally not treated as safely runtime-switchable yet:

| Profile | Frame / odometry | controller topology | costmap topology |
| --- | --- | --- | --- |
| base (`nav2_params_humble.yaml`) | `odom`, local filtered odometry | regulated pure pursuit | Livox and virtual-wall sources |
| Task 2 | `map`, global filtered odometry | regulated pure pursuit with different limits | PointCloud2 obstacle source |
| Task 3 | `map`, global filtered odometry | DWB holonomic controller | static field boundary plus PointCloud2 |

Frame, odometry, controller plugin and costmap-plugin differences require lifecycle
reconfiguration and cannot be applied by an arbitrary parameter update.  Until a
profile transaction with rollback exists, Mission Manager must reject a task whose
profile differs from the active profile.

## Task availability at baseline

| Task | Availability | Reason |
| --- | --- | --- |
| `task1` / `task1_skip_1_1` | experimental | Existing route and Nav2 launch path. |
| `task1_2` | not implemented | Marker-driven route generation is absent. |
| `task2` | experimental | Existing route and dedicated Nav2 profile. |
| `task3_1` / `task3_2` | experimental | Existing staged route behavior. |
