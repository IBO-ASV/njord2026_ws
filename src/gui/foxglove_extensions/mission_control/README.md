# NJORD Mission Control

Build with `bash scripts/package.sh`, drag `mission-control-0.1.0.foxe` into
Foxglove, then load the repository's `foxglove_setting.json`.
The connected bridge must enable services and expose the mission/control topics.
Task discovery uses `/mission/list_tasks`; start/stop use the existing typed
services. Stop includes the current execution ID, preventing a delayed stop
from canceling a newer mission.

The panel separates requested mode, AUTO permission, effective output source,
mission stage/progress, inhibit reasons, health and lamp commands. Combined lamp bit flags are shown as named colors,
with a blink period, rather than raw numeric codes. Inputs older
than 3 seconds display STALE and disable starting. Stop remains available for
a known active execution even if control telemetry becomes stale. The retained
`/mission/last_result` shows success/failure even after Mission returns to IDLE.
`/mission/plan` contains the complete ideal route for a Foxglove 3D/Plot panel.

Validation: `node --test test/panel.test.cjs`. The simulated-DOM test verifies
service payloads, execution IDs, disabled starts and stale-state rendering.
A physical display/operator usability check remains separate.

A real loopback bridge handshake and Mission service/channel advertisement
were checked separately from the DOM test. SDK-based bridges negotiate
`foxglove.sdk.v1`; legacy bridges negotiate `foxglove.websocket.v1`.
Custom test clients should offer both (see the [official SDK handshake source](https://docs.rs/foxglove/latest/src/foxglove/websocket/handshake.rs.html)).
This does not replace checking the installed panel in an operator's Foxglove UI.
