# micon_driver_fd

Transport package for communication with the Micon controller. The current
implementation provides a serial file-descriptor transport; a UDP transport can
be added alongside it without changing the manual-control package.

## `thruster_serial` (thruster serial)

`thruster_serial` is the named executable used by the miniPC bringup.
`serial_writer` remains as a compatibility alias for existing standalone launch files.

- subscribes `command_topic` (`std_msgs/msg/Float32MultiArray`), first 4 thrust commands in N
- subscribes `/soft_emg`, `/red`, `/yellow`, `/green` (`std_msgs/msg/Bool`)
- publishes `/micon/relay_active` (`std_msgs/msg/Bool`) from the firmware's relay-state byte
  and `/safety/emergency_stop` (`std_msgs/msg/UInt8`): `RUNNING=0`, `SOFT_EMG=1`,
  `HARD_EMG=2`
- parameters: `serial_port` (default `/dev/ttyUSB0`), `baud` (default `115200`),
  `command_topic` (default `/thruster_command` from `thruster_driver`). Set
  `ground_station_heartbeat_timeout_sec` to a
  positive value to require `std_msgs/msg/Empty` messages on
  `ground_station_heartbeat_topic` (default `/heartbeat/ground_station`); a
  timeout forces `SOFT_EMG`.

Every 50 ms, `serial_writer` sends a `THRUSTER_COMMAND` frame compatible with
`Docs/PROTOCOL.md`:

```text
COBS(raw_frame) || 0x00
```

The raw frame is 24 bytes: 5 byte header, 17 byte payload, and little-endian
CRC-16/CCITT-FALSE. The payload contains four little-endian `float32` thrust
values followed by `control_flags`: `bit3=emg`, `bit2=green`, `bit1=yellow`,
`bit0=red`.

The relay-state byte is telemetry only: GPIO15 follows the relay and can become
active during a software stop, so it is not an independent physical-E-stop signal.
`SOFT_EMG` is emitted whenever `/soft_emg` is true or the enabled ground-station
heartbeat watchdog has timed out; otherwise an active relay is reported as
`HARD_EMG` for operational status.

## `bms_serial` (BMS serial)

`Docs/njord_BMS_v0/master` sends one CSV row per second over its dedicated
serial port. `bms_serial` extracts `cell1_V` through `cell4_V` and publishes them
as a four-element `std_msgs/msg/Float32MultiArray` on `bms_topic`. CSV headers
and malformed rows are ignored. A valid `STALE` row is still published with
`nan` values intact, so heartbeat monitoring reflects serial-message arrival
while consumers can detect unavailable cell measurements.

`bms_serial` opens its serial device read-only. It never sends thruster-command
frames and does not publish relay or emergency-stop topics.
