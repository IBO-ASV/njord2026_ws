# Task2 sim costmap: update versus obstacle removal (2026-11-13)

Read-only ROS observation of the user's running
`ros2 launch njord_sim mission_sim.launch.py task:=task2 noise:=true`, domain 89.
No mission was started/stopped and no parameters, costmaps or launch files were
changed. Mission was already IDLE with `Task 2 FollowPath goal reached`.

## Timing and transport

Measured after 8 s graph discovery, over 12 s with publisher-matched QoS:

| Stream | Configured rate | Observed rate | Mean ROS receive age |
|---|---:|---:|---:|
| `/sim_obstacles` | 10 Hz | 10.02 Hz | 15.9 ms |
| global costmap update loop | 5 Hz | not directly measured | — |
| `/global_costmap/costmap_updates` | publish 2 Hz | 1.74 Hz | 30.8 ms |
| `/global_costmap/obstacle_layer_updates` | publish 2 Hz | 1.75 Hz | 33.0 ms |
| `/local_costmap/costmap_updates` | publish 2 Hz | 1.70 Hz | 14.2 ms |
| `/planned_path_pruned` | 2 Hz | 2.00 Hz | 6.9 ms |
| `/mppi/crm_costmap` | 2 Hz | 2.00 Hz | 5.7 ms |

Rates/ages were measured at an additional ROS subscriber on the AGX, not at the
Windows renderer. No end-to-end Windows screen latency was measured. The
configured 5 Hz update / 2 Hz publish loop can quantize publication to around
0.6 s: Jazzy publishes from the map update loop once `last_publish + period < now`.
The observed difference from exactly 2 Hz does not establish CPU overload.

`always_send_full_costmap=false`. The full global OccupancyGrid was a retained
snapshot about 212 s old, while its incremental updates were current. That is
expected; the old full-grid timestamp is not a 212 s display delay. The existing
Foxglove bridge was already subscribed to global/local `_updates` topics as well
as the base grids and obstacle-layer grids. The current Foxglove 3D panel
explicitly supports this topic pairing. Thus no missing-update subscription was
found in this session. Full global grid size: 490 x 600 at 0.1 m resolution;
local grid: 160 x 160 at 0.1 m.

## Residual obstacles were confirmed

A second observation reconstructed the current obstacle layer from its retained
base grid plus incremental updates. It compared lethal-cell centres against
current `/sim_obstacles` returns with z >= 0.2 m. A 0.22 m nearest-return threshold
exceeds cell-centre quantization at the 0.1 m map resolution.

- Global: 262 lethal cells, of which **214** were more than 0.22 m from every
  current physical return. Current fixture buoys account for the remaining 48.
- The opponent was already far outside the map, while old cells remained along
  previously traversed areas. This is actual retained obstacle-layer state,
  not just a retained old display snapshot.
- A separate local-map observation found 24 lethal cells and **0** residual
  cells by the same metric. Do not infer an active local-map fault or a current
  controller failure solely from the global visualization.

These counts describe this post-completion snapshot, not the worst case over an
entire moving encounter. Samples of global and local maps were collected at
different times. Only the sim's physical layer is compared; this method is not
an evaluation of perception accuracy on hardware.

## Cause and impact

The sim uses Nav2 ObstacleLayer with `marking=true`, `clearing=true`,
`observation_persistence=0`, sensor frame `base_link`, and physical
`min_obstacle_height=0.2`. The point cloud contains buoy/hull surfaces plus water
returns at z=0. The observation buffer filters the water returns before storing
the cloud, so they do not supply clearing rays for this source. Empty space
without endpoints also supplies no rays. Some old moving-hull cells are therefore
never traversed by a subsequent clearing ray.

ObstacleLayer does not temporally decay those cells. Its zero persistence means
retain the latest observation message, not erase map cells after zero seconds.
Inflation radius 1.2 m / cost scaling factor 3.0 define spatial falloff around
remaining lethal cells; that scaling factor is not a temporal decay rate.

Task2's custom MPPI planner consumes tracked-opponent and detected-buoy inputs;
it does not read the Nav2 global costmap. It sends its result to Nav2 FollowPath,
whose controller uses the local costmap. `/mppi/crm_costmap` is a separate risk
visualization, recomputed at 2 Hz rather than accumulated in this ObstacleLayer.

## Recommended order of correction

1. Correct sim clearing observations for recently observed empty space, or use
   a dedicated dynamic-obstacle layer with explicit expiry. Verify stale moving
   hulls disappear while real current obstacles stay marked. Restrict clearing
   to genuinely observed space; do not periodically wipe all costmaps.
2. Re-run Task2 encounter/clearance checks and compare global/local residuals.
3. Then raise display publication toward 5 Hz if smoother visualization is
   desired. Raising frequency or forcing full-grid messages alone cannot remove
   retained lethal cells. Unconditional full-grid publication also adds traffic.

This audit records the problem; it does not implement those corrections.

Evidence: [timing/parameters](topic_audit_20261113/task2_costmap_live.json),
[global residuals](topic_audit_20261113/task2_global_costmap_residual.json),
[local residuals](topic_audit_20261113/task2_local_costmap_residual.json).

Primary references:
- [Jazzy update/publish loop](https://api.nav2.org/nav2-jazzy/html/costmap__2d__ros_8cpp_source.html)
- [Jazzy obstacle layer and ray tracing](https://api.nav2.org/nav2-jazzy/html/obstacle__layer_8cpp_source.html)
- [Observation-buffer height filtering](https://api.nav2.org/nav2-jazzy/html/observation__buffer_8cpp_source.html)
- [Foxglove incremental-grid pairing](https://docs.foxglove.dev/docs/visualization/panels/3d#incremental-occupancy-grid-updates)
