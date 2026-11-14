# NJORD Mission Control

Build with `bash scripts/package.sh`, drag `mission-control-0.1.0.foxe` into
Foxglove, then load the repository's `foxglove_setting.json`.
The connected bridge must enable services and expose the mission/control topics.
Task discovery uses `/mission/list_tasks`; start/stop use the existing typed
services. Stop includes the current execution ID, preventing a delayed stop
from canceling a newer mission.

The panel separates requested mode, AUTO permission, effective output source,
mission stage/progress, inhibit reasons, health and lamp commands. Combined lamp bit flags are shown as named colors,
with a blink period, rather than raw numeric codes. During Task2, a failed or
stale `/mission/task2/path_safe` check is shown with the stopping reason. Inputs older
than 3 seconds display STALE and disable starting. Stop remains available for
a known active execution even if control telemetry becomes stale. The retained
`/mission/last_result` shows success/failure even after Mission returns to IDLE.
`/mission/plan` contains the complete ideal route for a Foxglove 3D/Plot panel.

Validation: `node --test test/panel.test.cjs`. The simulated-DOM test verifies
service payloads, execution IDs, disabled starts and stale-state rendering.
A physical display/operator usability check remains separate.

A real loopback bridge handshake and Mission service/channel advertisement
were checked separately from the DOM test; `/mission/list_tasks` was also
called over the bridge using typed CDR request/response. SDK-based bridges negotiate
`foxglove.sdk.v1`; legacy bridges negotiate `foxglove.websocket.v1`.
Custom test clients should offer both (see the [official SDK handshake source](https://docs.rs/foxglove/latest/src/foxglove/websocket/handshake.rs.html)).
This does not replace checking the installed panel in an operator's Foxglove UI.

The health display lists local odometry, GNSS reception and driver heartbeat
individually, including signal state, age, rate and monitor message. The active
policy and ON/OFF gates come from retained `/control/requirements`, not a
hardcoded GUI assumption. Missing driver-heartbeat signals remain visible.
GNSS frequency health is explicitly labelled separately from RTK FIX quality.
The serial writer's independent command-watchdog reason comes from
`/micon/thruster_command_status`.

The panel uses actual HTML tables for mode request, AUTO output permission,
effective output source, health signals, receive age/frequency and enabled
requirements. `AUTO要求中` is operator intent; only `実際の出力源: 自律航行`
means AUTO was selected by the arbiter. Before a task is configured, permission
is displayed as `準備前` rather than an unexplained `不可`.

`準備してAUTO開始` requests task/profile preparation and AUTO intent. The
SafetySupervisor publishes `preparation_permitted` and
`preparation_inhibit_reasons` in the existing `/control/requirements` JSON.
These check the common emergency/health conditions even while no task policy is
active. Task and Nav2 readiness are deferred to mission preparation, where the
existing AUTO safety decision still applies. Missing/stale telemetry, emergency
stop, failed preparation checks and current physical/configuration faults disable
the button. Click handlers recheck the same conditions. This is an operator UI
check; the command arbiter remains the authority for actual actuator output.
Task-specific conditions are checked after the selected task is configured.
`停止・MANUALへ` also cancels an orphaned AUTO request when no mission is active.

After updating, restart the ROS launch (including SafetySupervisor), reinstall
`mission-control-0.1.0.foxe`, and reimport the common `foxglove_setting.json`.
An older SafetySupervisor lacks the preparation fields, so this panel keeps the
start button disabled with an explicit update message. No sensor interlock
configuration is changed by this display update.

Waypoint display inputs:

- `/ground_waypoint_markers`: WGS84 longitude/latitude for **GNSS Map Telemetry**;
  it is not an ENU marker array for the standard 3D panel.
- `/mission/waypoint_markers`: validated route markers published by Mission
  Manager when navigation or dry-run preview starts, now enabled in the common
  3D layout.
- `/sim/waypoint_markers`: ENU points, labels and route preview for the selected
  sim task. Published at 1 Hz with transient-local durability, starting before
  mission execution. The sim's geographic preview also remains visible while
  IDLE and after completion. Hardware uses the existing ground waypoint source.

A headless Firefox visual check with simulated telemetry is saved at
[Mission Control preview](../../../../docs/reference/mission_control_preview_20261113.png).

## Velocity tuning controls

The panel includes a **速度ゲイン調整** section when connected to the AGX bridge. Enter target speed, test speed ceiling, and per-thruster thrust ceiling, then click **自動調整を開始**. The panel applies these values to `/velocity_autotuner/set_parameters_atomically` and starts `/velocity_tuning/start` only if the latest control state is AUTO-permitted and not in emergency stop.

**一時停止して手動へ** pauses the tuner and requests MANUAL. After repositioning, **AUTOで再開** requests AUTO, waits for an updated `/control/state` showing AUTO permission, then resumes the interrupted candidate. **終了して設定を復元** requests MANUAL, waits for the mode update, then calls the tuner stop service. The panel displays tuner phase, measured forward speed, and the FR/FL/RR/RL force feedback.

The AGX bridge must expose the tuner services and `/velocity_tuning/status`, `/odometry/filtered/local`, and `/debug/current_force` topics. The tuner must be started from the same ROS workspace as the updated motion supervisor and thruster controller. Values are locked while tuning is active. Starting a trial can move the vessel; the UI is only an operator control surface and the ROS safety supervisor/arbiter remains authoritative.
