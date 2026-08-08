# Nav2 profile inventory and safe application boundary

This inventory records the Phase 7 migration state. The checked-in profile
catalog resolves a common base, one typed task overlay, and the ROS-distribution
overlay. Its resolved output is tested byte-semantically (as a YAML mapping)
against every legacy full parameter file.

## Distribution differences

Humble and Jazzy have the same parameter values and topology in this repository.
Their only current differences are plugin identifier syntax:

- Humble uses `nav2_navfn_planner/NavfnPlanner`,
  `nav2_theta_star_planner/ThetaStarPlanner`, and `nav2_behaviors/<Behavior>`.
- Jazzy uses the corresponding `package::Class` identifiers.

These differences live in `config/nav2_profiles/distros/jazzy`; the task overlays
use the Humble spelling as their compatibility baseline.

## Task topology differences

| Area | Task 1 | Task 2 | Task 3 | Classification |
| --- | --- | --- | --- | --- |
| Controller | Regulated Pure Pursuit | Regulated Pure Pursuit | DWB, holonomic sampling | Plugin topology / lifecycle reconfiguration |
| Planner | NavFn | Theta* | Theta* | Plugin topology / lifecycle reconfiguration |
| Global costmap layers | obstacle, inflation | obstacle, inflation | static field boundary, obstacle, inflation | Plugin topology / lifecycle reconfiguration |
| Obstacle inputs | Livox plus virtual wall | `/pointcloud` | `/pointcloud` | Lifecycle reconfiguration |
| Collision monitor | configured and lifecycle-managed | configured and lifecycle-managed on `/pointcloud` | configured and lifecycle-managed | Plugin topology; must be verified, not inferred |
| Frames/odometry | primarily `odom`, local odometry | `map`, global odometry | mixed `map`/`odom`, global odometry | Lifecycle reconfiguration |
| Frequencies and limits | baseline ASV tuning | Task 2 tuning | higher-rate docking tuning | Potentially dynamic, but not yet certified |

The task overlay is therefore not treated as a bag of dynamically settable
parameters. The catalog validates controller, planner, costmap plugins, lifecycle
membership, and collision-monitor presence before returning a resolved startup
configuration.

## Current application contract

`Nav2ProfileApplicationManager` is intentionally `startup_only`:

- `minipc_bringup.launch.py` passes one `active_nav2_profile` value to both the
  resident Nav2 graph and Mission Manager;
- `nav2.launch.py` resolves that profile from the installed catalog for the
  running `ROS_DISTRO`, validates its typed topology, and materializes the full
  parameter mapping before any Nav2 process is launched;
- a task matching the resident profile is accepted without changing Nav2;
- a different profile returns `CONFIGURATION_FAILED` and leaves the active profile
  unchanged;
- active and requested profiles are published in `MissionStatus`;
- the common base and overlays perform no live ROS parameter or lifecycle calls.

Select Task 2 or Task 3 only when starting the persistent role bringup:

```bash
ros2 launch robot minipc_bringup.launch.py active_nav2_profile:=task2
```

Changing the launch argument later is not a runtime switch. Stop the task, return
to MANUAL, and restart the role bringup with the desired profile. A Mission request
for any other profile remains fail-closed. The `params_file` argument on
`nav2.launch.py` is retained only for legacy comparison launches; persistent role
bringup does not expose or use that override.

Live switching remains a material design decision. A safe implementation must own
the lifecycle sequence for controller, planner, both costmaps, BT navigator,
velocity smoother, and collision monitor; prove effective command output is ZERO;
atomically apply or restore parameters; reactivate nodes; and verify action servers,
TF, costmap outputs, `/cmd_vel_nav`, and the collision-monitor state topic. The
current explicit process graph and every resident profile include collision monitor.
That makes Task 2 startup topology match the canonical velocity pipeline, but does
not make live profile switching certified.

Regenerate the factored files after intentionally changing a legacy full profile:

```bash
python3 scripts/generate_nav2_profile_overlays.py
```
