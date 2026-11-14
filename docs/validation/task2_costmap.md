# Task 2 costmap 検証

2026-11-13 の監査と修正の検証条件・結果を統合しています。失敗・中断した試行と最終成功行列は別の結果です。詳細生成物は Git 履歴 `74fe7ec6` に保存されており、新規の実験出力は `artifacts/` に置きます。

---

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

Evidence: [timing/parameters](recorded_results.md),
[global residuals](recorded_results.md),
[local residuals](recorded_results.md).

Primary references:
- [Jazzy update/publish loop](https://api.nav2.org/nav2-jazzy/html/costmap__2d__ros_8cpp_source.html)
- [Jazzy obstacle layer and ray tracing](https://api.nav2.org/nav2-jazzy/html/obstacle__layer_8cpp_source.html)
- [Observation-buffer height filtering](https://api.nav2.org/nav2-jazzy/html/observation__buffer_8cpp_source.html)
- [Foxglove incremental-grid pairing](https://docs.foxglove.dev/docs/visualization/panels/3d#incremental-occupancy-grid-updates)


---

# Task2 sim costmap correction (2026-11-13)

The preceding [audit](task2_costmap.md) found two separate issues:
obstacle endpoints alone did not clear previously occupied open water, and
2 Hz publication inside the 5 Hz Nav2 update loop delivered about 1.7 Hz.
This change addresses Task2 simulation only. Restart the sim launch to load it;
the user's existing domain-89 session was left running unchanged.

## Implementation

- `/sim/costmap_clearing_scan` supplies 8,192 clear-only rays at nominal 10 Hz,
  covering 360 degrees. Each ray stops 3 cm before the nearest buoy or NPC hull.
  No hit is a finite range just below 100 m; origins inside an obstacle produce
  invalid rays, rather than falsely declaring that obstacle free.
- This is a supplement to the existing **100 m physical costmap stand-in**.
  It does not claim the recognition LiDAR sees 100 m: `/livox/lidar` retains its
  40 m range and the production cloud filtering/tracking pipeline. The planner
  does not consume the clearing scan or NPC truth.
- The scan uses the published noisy TF pose to align with the existing map-frame
  physical cloud. Existing marking and inflation remain enabled. Occluded cells
  remain unknown/occupied until reobserved; no blanket clearing service, timeout
  expiry or cost reduction is used.
- Task2 global/local costmaps still update at 5 Hz. Publication is requested at
  10 Hz so Jazzy's strict deadline check inside that loop publishes each update,
  giving approximately 5 Hz. Incremental publication remains enabled; rolling
  local maps can naturally publish full snapshots when their origin changes.
- The acceptance observer saves `costmap_audit.json` alongside `evaluation.json`.
  It reconstructs full grids plus patches, rejects stale patches after a newer
  full grid, and reports both streams' combined rate. It does not alter mission,
  collision, gate or stopping acceptance criteria.

## Validation and tuning order

1. Add free-space clearing while leaving publication at 2 Hz. All four
   encounters, seed 2026, passed. Each final global/local obstacle-layer sample
   had zero residual cells under the audit's 0.22 m surface-distance metric.
2. Enable approximately 5 Hz publication and run 12 noisy cases, seeds
   2026/2027/2028 and head-on/port/starboard/none. **11 passed, one failed**:
   head-on 2027 completed its mission but contacted the NPC (true hull gap 0 m).
   The guard repeatedly canceled propulsion because no candidate met predicted
   hull clearance, and the non-yielding NPC reached the stopped vessel.
   This failure is retained; mission success alone was not counted as a pass.
3. Inspection found the sim selected `motion_filter_mode: standard`, whereas
   `task2_jetson_autonomy.launch.py` selects `straight_line`. The former resets
   selection/smoothing on a gap; the latter retains a confirmed constant-velocity
   observation for at most the configured 3 s. Match the real Task2 launch in the
   sim. No MPPI weights, trajectory margins, controller gains or pass thresholds
   were changed at this step. The diagnostic single repeat passed (2.154 m hull
   gap), but head-on 2027 collided again in the two-worker matrix, so this setting
   alignment alone did **not** resolve the failure. Head-on 2028 also fell below
   the required gap (0.446 m, without contact). That matrix was interrupted
   after retaining its evaluations, including the explicitly interrupted case.
   Scheduling and noisy tracker
   inputs vary between runs; clearing is not established as the unique cause.
4. Replaying recorded guard inputs showed both 5 m and 3.5 m lanes rejected,
   while a 7 m lane on the same committed side met the exact same checks.
   Add that wider fallback before requesting a hold. This does not weaken the
   2.85 m predicted centre separation, buoy margins or observed gate bounds,
   and does not permit switching to the opposite side. Add a regression test
   using the recorded delayed-approach observation, rotated to GPS-course axes.
   The guard remains opt-in (default disabled in the vehicle planner).
5. Wider candidates alone still failed head-on 2028 (0.007 m hull gap, then
   no progress for 45 s). The vessel approached a buoy too closely at the first
   gate. Inspection also found that the lane function multiplied the measured
   lateral position by gate tapers after forcibly inserting `(0, 0)`, creating
   a discontinuity at the path head on replanning. Use a fixed course-relative
   lane profile with a straight entry/exit through each gate, avoid the initial
   pre-gate excursion, and blend the measured lateral correction continuously
   into that profile. Add tests for straight gate approaches and head continuity.
   Retain the unchanged geometric safety checks and the wider same-side fallback.

The new profile stays on the GPS line until 1 m after the entry gate, transitions
to the encounter lane over 6 m, and reverses that envelope before a straight
approach from 1 m before the exit gate. A measured lateral correction decays
over at most 7 m (or the next gate/goal, whichever is closer). These are synthetic
fixture choices to avoid lookahead corner-cutting and replan jumps, not measured
real-vessel tracking gains. No RPP/MPPI gain or target-speed adjustment was made.

Raw evidence is retained under
[task2_clearing](recorded_results.md), including the failed trial,
its relevant guard log, run manifests/source hashes, and both publication configs.
The diagnostic head-on repeat additionally logged guard inputs without changing
its decisions. The final matrix and repeat results are recorded below.

## What these measurements establish

The real Nav2 integration test checks moving hull removal, preservation of a
current hull and stationary buoys, retention of a stale cell behind an opaque
hull, and subsequent removal after the occluder moves. This isolates clearing
behavior from navigation outcomes.

The residual count compares the last received grid with the latest received
physical cloud, without synchronizing their timestamps. A moving hull can leave
a small transient discrepancy during one update interval; it is not an exact
persistent-ghost count. The 12-case pre-alignment matrix had local residual 0 in
all cases and global residual 0 except six cells in head-on 2026. The original
stationary-session audit had 214 global residual cells. Do not interpret the six
cells as proof of either complete removal or persistent ghosts; use the bounded
Nav2 fixture for the persistence/occlusion assertion.

Transport rates are measured at a subscriber on AGX, not at the Windows display.
This work does not measure end-to-end Foxglove rendering latency, qualify the
real sensor's free-space coverage, or certify collision avoidance on water.

## Regression checks

142 tests passed, including the real Nav2 clearing/occlusion fixture, mission and
control checks, opponent selection/coasting checks, and eight guard geometry
checks. Ruff F/I checks for the changed helpers/tests and `git diff --check`
also passed. The CPU ROS Python environment was used; the YOLO GPU venv was
not activated for these runs.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=120 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 -m pytest -q \
  scripts/yolo/test/test_cusparselt_metadata.py \
  src/control/motion_supervisor/test src/mission/mission_manager/test \
  src/sim/njord_sim/test \
  src/navigation/path_generator/mppi/test/test_path_guard.py \
  src/detection/task2_perception/test
/usr/bin/python3 scripts/simulation/run_sim_suite.py --tasks task2 \
  --seeds 2026,2027,2028 --encounters head_on,port,starboard,none \
  --workers 2 --domain-base 139 --timeout 300 \
  --output /run/user/1000/njord-task2-clearing-continuous-final-suite
```

Choose unused ROS domains and a new output directory when repeating the suite.
The Nav2 fixture reserves domain 128 and must not run alongside another instance
of itself. Run manifests retain per-file source hashes for the uncommitted trial
configurations; failed/interrupted intermediate runs are kept separately from
the final matrix.

## Final navigation matrix

All **12/12 noisy cases passed** after the wider same-side candidate and continuous
gate profile changes. Both gates, opponent selection where applicable, stopping,
head-on starboard avoidance and starboard-crossing astern passage retain their
original acceptance checks.

Minimum true hull gap to the other vessel by seed (metres):

| Encounter | 2026 | 2027 | 2028 |
|---|---:|---:|---:|
| head_on | 1.259 | 2.432 | 3.984 |
| port | 3.072 | 4.189 | 4.023 |
| starboard | 4.305 | 2.626 | 5.606 |
| none | — | — | — |

Overall minimum buoy hull gap: **0.642 m**. Combined full/incremental
costmap delivery: **4.89–5.00 Hz** at the AGX subscriber.

The original matrix saved 11/12 synchronized audit snapshots: port 2028 delivered
its newest scan before the matching odometry callback at save time. Cache scans
and select the newest exactly matched scan/pose pair to fix this observer-only
race; log the selected timestamp and lag. A separate port-2028 repeat validates
this change. Navigation/sensor/clearing code did not change after the 12-case
matrix; the run source hashes were checked for those modules.

Of the 11 available original snapshots, all local residual counts were zero;
global counts were zero except seven cells in head-on 2027. These are the
unsynchronized grid-versus-cloud counts described above, not proof of seven
persistent ghosts. Full final evidence is in `task2_clearing/final_suite`.

The observer-fix repeat (port, seed 2028) also **passed**, with 3.216 m
NPC hull gap and a ready audit. The selected scan/pose pair lag was
0.0 ms; local/global residual counts were 0/0. See `task2_clearing/audit_repeat_port2028`.
