# Simulation changes and tuning log

All results below are simulation results. No sensor drivers or real actuators
were started. Raw runs are under `/tmp/njord-night`; final retained summaries
will be stored beside this document. Changing a scoring predicate requires a
new run and is not recorded as controller improvement.

| Trial / observation | Change | Result / rationale |
|---|---|---|
| Existing short live Task1 route ran, but competition buoys were far outside it | Generate a private sim registry from the existing surveyed Task1 ENU fixture; keep live WP YAML untouched | Short-route success is only an integration smoke check, not competition completion |
| Per-goal Nav2 feedback always reports at most one pose remaining | Enable Task1 walls only on the successful competition GPS3 waypoint callback | Removes the mismatch with the sequential Mission executor |
| GNSS heading stand-in originally used Quaternion, while the wall node expects PoseWithCovarianceStamped | Match the actual UM982 message and ENU yaw contract | True-north lock now occurs; pre-fix runs do not qualify wall behavior |
| Nav2 action server existed during configure but rejected a goal as inactive | Query actual lifecycle ACTIVE state for controller, navigator and collision monitor before starting | MMG rerun was admitted and completed |
| Legacy Task1 sim inverted allocation sway/yaw for an old thruster order | New MMG launch keeps [surge,sway,yaw] signs [1,1,1], matching the current RR/RL/FR/FL geometry consistency test | Task1.2 MMG reached every goal with correct nearest-approach cardinal sides; minimum buoy hull clearance 0.216 m |
| Task3 accepted Nav2 completion before proving stationary dwell | Require 2 s stationary settling, then continuous 10 s / 5 s dwell; reset on motion/stale odometry | Normal and parallel docking sequences complete; a drift/reset/cancel unit test protects the behavior |
| Existing periodic berth timer was left alive after elapsed callback | Destroy/cancel the timer on expiry; ignore queued callbacks from the previous stage | Prevents a previous dwell timer advancing later stages |
| Task3 controller failed to configure with empty strict-goal coordinate arrays | Fill sim strict-goal arrays with projected approach/berth coordinates before Nav2 starts | Normal and parallel docking launch and activate |
| Oblique GPS3->GPS4 plane classified an E/W passage before the actual closest approach | Score the geographic side at closest approach within 10 m; keep separation/contact checks independent | This is a scoring correction, not a tuning improvement; subsequent runs use the corrected predicate |
| Task2 head-on: actual recognition and both gates succeeded, but closest hull gap was only 0.00093 m, below the 0.5 m target | CPU MPPI samples 256 -> 512; horizon 100 -> 150 at dt 0.15 s; path cost 150 -> 50; collision cost 10/50 -> 30/150; own LOA 1.2 -> 1.5 m; avoid radius/offset 2/3 -> 4/5 m | Under retest; this run remains a failure despite Mission success. Overrides are sim-only |

The first MMG run was rejected before motion because the navigator was not
ACTIVE. Do not treat it as evidence against the allocation or MMG tuning.
Reported CPU settings are simulation experiments; compare their metrics and
compute time before copying them to real-vessel settings.

| Later trial / observation | Change | Result / rationale |
|---|---|---|
| Normal docking seed 2028 touched a dock segment for one truth sample | Sim footprint padding -> 0.10 m, strict docking XY tolerance 0.80 -> 0.35 m (DWB and goal checker together) | Same seed rerun: no contact, >10 s stationary dwell, final stop; passed |
| Task4 seed 2027 aborted on `smooth_path` / `follow_path` acknowledgement timeouts under concurrent CPU load | Sim BT server acknowledgement timeout 20 -> 1000 ms; service wait 2000 ms | Retesting; this changes timing tolerance, not completion or collision criteria |
| Higher Task2 collision cost alone still collided | Rejected 30/150 collision scaling and path weight 50; returned to existing 10/50 and 150 | Not adopted as an improvement |
| Task2 selector output is absolute odom velocity but planner default assumes relative velocity | Explicit `other_twist_is_relative:=false` for sim | Removes addition of own velocity to already absolute target velocity |
| Moving cluster box yaw differed from target course; CRM only predicts scalar forward speed along yaw | Add opt-in `other_heading_from_velocity` (default false), select it for straight-line sim targets; add rotated-vector regression test | After rebuilding the previously copied planner install: predicted target points toward ownship; head-on hull gap improved to 2.40 m, but passing side was wrong, so run still failed |
| Task2 showed safe port-side avoidance in a head-on fixture | Add opt-in route-relative starboard cost for head-on / starboard crossing, default weight 0; sim experiment weight 300 | Under retest. Production defaults remain unchanged |
| StopTask service canceled a running action without an action cancel request; ROS rejected EXECUTING -> CANCELED | Use ABORTED transport state with typed CANCELED result for service stop; actual action cancellation still uses CANCELED | Cancellation and sensor-loss/zero-output integration checks passed; prevents an exception returning a default success code |

Task2 `other_heading_from_velocity` and `mppi.starboard_bias_weight` are optional
new planner inputs. The latter defaults to zero; it is a scenario tuning
experiment, not a claim of general COLREG compliance. `mppi.random_seed` defaults
to -1 (unchanged random behavior) and is fixed to the run seed in simulation.

| Final implementation / observed behavior | Change | Result / rationale |
|---|---|---|
| Docking could report success at a heading different from the requested berth heading | Refresh projected goal arrays and tolerances inside `SelectiveHeadingGoalChecker`; add a real plugin gtest | Previously initialization-only arrays ignored later Mission updates. C++ regression test passes, including unequal-array rejection |
| `/task/plan` requested 0.4381 rad but ThetaStar produced a different final quaternion | Set sim `use_final_approach_orientation: false`; use the existing natural cubic spline docking smoother | Probe confirms task plan / planner / smoother preserve 0.4381 rad. This fixes orientation handling rather than relaxing scoring |
| Normal dock structure had no surveyed orientation; invented walls conflicted with the waypoint yaw | In the private sim fixture derive normal axis from approach -> berth, align approach/berth/exit yaw; parallel dock opens toward approach on the nearest side | Live waypoint YAML remains intact. Whole-hull containment is now independently required in addition to zero contact |
| Strict 0.35 m goals still permit a 1.5 x 1.0 m hull too near a 2 x 2 m berth edge | Tighten sim docking XY to 0.15 m and goal yaw to 5 degrees; keep stationary window 10 degrees and continuous dwell | Normal and parallel truth + seeds 2026/2027/2028: all eight passed with complete hull inside berth |
| Task2 turning changed which side was classified as give-way when classification used the bow frame | Classify relative position and observed velocity against fixed GPS course; keep soft starboard margin 3 m | Rotated-course regression test passes. Production starboard weight still defaults to zero |
| Combining real buoy inputs, moving target and noisy tracking still produced a 0.019 m hull gap in a port crossing despite Mission success | Keep that failure; enable optional CV observed-object path guard only in the new sim launch. Check buoy/gate bounds and 2.85 m predicted centre separation over plausible own speeds; try GPS-relative lanes, otherwise cancel FollowPath | First noisy port retest: both gates, minimum true hull gap 1.684 m, final stop; full encounter/seed matrix recorded separately |
| RPP inherited Task1 rotate-in-place behavior for Task2, delaying progress while target continued | Use Task2 profile's `use_rotate_to_heading: false`, 1.5 s lookahead / max 8 m | Helps follow the moving avoidance path without repeated stationary rotations; sim-only override |
| Path permission could close while a FollowPath acceptance was in flight | Cancel late accepted handles from older generations, as well as active goals; clear rejected path | Two behavioral regression tests pass; guard has no dependency on simulator truth |
| Running many Nav2/PCL/MPPI processes together on this 8-core host caused action/TF timeouts and delayed plotting | Final suites use one berth worker + one Task2 worker maximum; JSON saved before opt-in plotting; runner grants 90 s startup/artifact watchdog margin | Overload failures remain under `/tmp/njord-night/{berths-release,crossing-release}` and are not treated as completed acceptance cases |
| Broad robot static checks assert old networking/video configuration literals | Do not modify current platform settings to satisfy unrelated stale assertions | 162 passed / 7 failed in broad robot check; all seven concern baseline configuration. Focused mission/control/geometry/Task2 tests and new plugin gtest pass |

The berth wait durations remain explicit provisional defaults: normal 10 s,
parallel 5 s, both after 2 s settling. Sensor noise is synthetic and does not
constitute a measured sensor error model. CPU MMG qualification currently
covers Task1.2 only; the full task/seed matrix uses the kinematic model.
The original MPPI-only failures are retained as evidence against deploying
these simulation tuning overrides unchanged on the real vessel.
