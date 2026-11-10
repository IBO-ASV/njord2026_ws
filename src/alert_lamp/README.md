# alert_lamp

`alert_lamp` centralizes vessel status evaluation and controls the alert lamp through
`/alert_lamp/command`. The manager never drives hardware directly. The driver turns
the command into on/off outputs and enters red blink when manager commands time out.

## Nodes

- `alert_lamp_manager_node`: consumes canonical typed `ControlState`,
  `MissionStatus`, and `HealthState` topics. It publishes
  `alert_lamp/msg/AlertLampCommand` on `/alert_lamp/command` and manager diagnostics.
- `alert_lamp_driver_node`: performs blinking and publishes `std_msgs/msg/Bool` to
  `/red`, `/yellow`, and `/green`. Those are the existing inputs of
  `micon_driver_fd/serial_writer`, which owns the USB Serial device.

Heartbeat monitors publish named `HealthSignal` entries and the health
aggregator publishes `HealthState`. The alert lamp displays that canonical
summary; it does not independently apply GNSS, heartbeat, localization, or
global diagnostic severity policy. The manager input contract is:

| Input | Type | Default topic |
| --- | --- | --- |
| Control state | `njord_interfaces/msg/ControlState` | `/control/state` |
| Mission status | `njord_interfaces/msg/MissionStatus` | `/mission/status` |
| Health summary | `njord_interfaces/msg/HealthState` | `/health/state` |

The manager waits for all three canonical messages at startup and shows a
critical/initializing state until they are available.

## Lamp patterns

| State | Output |
| --- | --- |
| `INITIALIZING` / `CRITICAL_FAULT` | red blink, 0.05 s |
| `MANUAL_NORMAL` | yellow solid |
| `AUTO_NORMAL` | green solid |
| `AUTONOMY_NOT_READY` | green and yellow blink together, 0.1 s |
| `GROUND_COMMUNICATION_LOST` in AUTO | green and red solid |
| `GROUND_COMMUNICATION_LOST` in MANUAL | yellow and red solid |

Critical faults have priority. The driver also red-blinks at a 0.05 s period if no manager
command arrives within one second.
`AlertLampCommand.color` is a bit flag, so compound states may intentionally
turn on multiple lamps. State selection itself has priority in the evaluator:
critical faults first, then ground-station loss, then autonomy-not-ready, then
normal operating states.

## Configuration and diagnostics

Canonical inputs, the topic-condition-to-pattern table, named display patterns,
and driver output topics are in `config/alert_lamp.yaml`.  The `topic_rules`
section makes the relationship explicit as `topic condition -> alert state ->
pattern`; its priority matches the safety evaluator.  The `patterns` section is
read by the manager at runtime, so colors and timing are configuration values.
Both nodes publish standard ROS 2
diagnostics on `/diagnostics` named `Alert Lamp Manager` and `Alert Lamp Driver`.

The initial `output_type: topic` is deliberate: GPIO/CAN/Serial ownership is not
implemented in this package because the existing Micon serial writer already owns the
physical serial connection. A dedicated hardware backend can be added later without
changing manager behavior.

## Run and test

```bash
ros2 launch alert_lamp alert_lamp.launch.py
ros2 topic echo /alert_lamp/command

colcon test --packages-select alert_lamp
colcon test-result --verbose
```
