# Spatial v8.0 Handoff

## Current branch

- Branch: `40-advanced-navigation-spatial-ins` (PR #41)
- Base integration: `90ff400 feat: integrate advanced navigation spatial ins`

## Hardware connection verified

The Advanced Navigation Spatial v8.0 was detected on:

- Device path: `/dev/ttyUSB0`
- Driver parameter: `com_port: ttyUSB0`
- Baud rate: `115200`
- Device ID: `1`
- Software: `6510`
- Hardware: `8000`
- Serial: `29003c5343501620383736`

Do not write `/dev/ttyUSB0` in `adnav_spatial.yaml`. The upstream driver builds
the full device path internally as `/dev/<com_port>`.

## ROS configuration

Use:

```bash
ros2 launch robot localization.launch.py
```

The launch starts:

- `adnav_driver`
- GLIM
- local/global EKF
- normal `navsat_transform_node`
- `spatial_navsat_transform_node`

Current Spatial config:

```yaml
baud_rate: 115200
com_port: ttyUSB0
packet_request: [20, 50, 28, 50]
packet_timer_period: 1000
```

Packet 20 is the filtered INS/GNSS system state. Packet 28 is raw sensor data
for observation and logging. At 115200 baud, requesting these faster caused
`SERIAL PORT DATA OVERFLOW`; 20 Hz cleared the overflow during testing.

## Test result on 2026-07-06

Working:

- `/adnav_driver/imu`
- `/adnav_driver/nav_sat_fix`
- `/adnav_driver/filter_status`
- `/adnav_driver/system_status`
- `/adnav_driver/pose`
- `/adnav_driver/twist`
- `/adnav_driver/imu_raw`

Observed status:

- `Dual Antenna Heading Active`
- `Velocity Heading Enabled`
- `Orientation Filter Initialised`
- `Heading Initialised`
- `UTC Time Initialised`
- No GNSS antenna fault was reported

Not yet healthy:

- `NavSatFix.status` was `-1` (`STATUS_NO_FIX`)
- `Navigation Filter NOT Initialised`
- `Internal GNSS NOT Enabled`
- `External Position/Velocity/Heading NOT Active`

Conclusion: ROS can communicate with Spatial v8.0 and receive packet 20/28
topics. GNSS tight-coupled navigation is not confirmed yet because GNSS fix and
navigation filter initialization were not healthy during this indoor/bench test.

## Test result on 2026-07-07

Re-ran `adnav_driver` standalone (indoors, no sky view) to reconfirm ROS
communication ahead of PR #41. Result matches 2026-07-06: hardware/driver/topic
plumbing is confirmed working, GNSS fix is not (as expected indoors).

Topics publishing at ~20 Hz (matches `packet_timer_period`/`packet_request`):

- `/adnav_driver/imu` (orientation + angular velocity; `linear_acceleration` is
  zero here because packet 20 does not carry raw accel)
- `/adnav_driver/imu_raw` (full IMU incl. `linear_acceleration`, e.g.
  `z: -9.37 m/s^2`)
- `/adnav_driver/nav_sat_fix`, `/adnav_driver/filter_status`,
  `/adnav_driver/system_status`

`nav_sat_fix.status.status` is `-1` (`STATUS_NO_FIX`); lat/lon are the
Advanced Navigation default placeholder coordinates, not a real fix.
`filter_status` still reports `Navigation Filter NOT Initialised` and
`Internal GNSS NOT Enabled`, with `Orientation Filter Initialised` and
`Dual Antenna Heading Active` already up. `system_status` reports no faults
(no `GNSS FAILURE`, no antenna fault, no serial overflow).

Conclusion unchanged: GNSS/INS data is retrievable over ROS 2 today without
RTK; a real position fix still requires an outdoor test with sky view per
the checklist below.

## Next checks

Run outdoors with sky view and the GNSS antenna connected:

```bash
source install/setup.bash
ros2 launch robot localization.launch.py
```

Then check:

```bash
ros2 topic echo /adnav_driver/system_status --once --full-length
ros2 topic echo /adnav_driver/filter_status --once --full-length
ros2 topic echo /adnav_driver/nav_sat_fix --once --full-length
ros2 topic hz /adnav_driver/nav_sat_fix
ros2 topic hz /adnav_driver/imu
```

Pass criteria:

- `/adnav_driver/system_status` has no `GNSS FAILURE`, no `GNSS ANTENNA FAULT`,
  and no `SERIAL PORT DATA OVERFLOW`
- `/adnav_driver/filter_status` reports `Navigation Filter Initialised` and
  `Internal GNSS Enabled`
- `/adnav_driver/nav_sat_fix.status.status` is not `-1`
- `/odometry/gps/spatial` is published by `spatial_navsat_transform_node`
- `/odometry/filtered/global` remains stable without jumps
