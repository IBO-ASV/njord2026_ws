# simple_manual

Real-vessel manual bringup and velocity-command input for the maintained X4
thruster control pipeline. Navigation is not started.

The launch starts:

- MID360S, UM982, Advanced Navigation Spatial, and wired Drogger drivers
- the complete localization launch
- the LiDAR preprocessing, segmentation/clustering, and tracking pipeline
- `/joy` from the shore PC -> `joy_converter` -> `thruster_driver` ->
  `micon_driver_fd`

Camera/YOLO detection is not started because it is not part of this sensor
configuration. The LiDAR detection pipeline can be disabled for troubleshooting
with `enable_detection:=false`.

## Interfaces

- `joy` (`sensor_msgs/msg/Joy`) is read by both manual nodes.
- `joy_converter_node` publishes `/cmd_vel_manual`, `/soft_emg`, `/red`, `/yellow`, and `/green`,
  and requests `manual` / `auto` on `/system/operating_mode`.
- `command_arbiter_node` is the sole publisher of `/cmd_vel` and `/system/control_status`.
  It selects manual commands only in `manual` mode, selects `/cmd_vel_nav` only in `auto` mode
  when `/autonomy/ready` is true (unless its `require_autonomy_ready` parameter is set to
  `false`), and publishes zero velocity during an emergency stop or when
  the selected command is stale (0.5 s).
- `/soft_emg` uses positive logic: button 0 sends `true` (emergency stop), while its
  normal released state sends `false` (emergency stop released).
- `manual_control.launch.py` uses UM982-only feedback by default, without the
  EKF.  The UM982 driver natively publishes `odometry/feedback`
  (`nav_msgs/msg/Odometry`), and this launch remaps it to the existing
  `/odometry/filtered/local` interface.  Its surge/sway velocity is a filtered
  GNSS position difference rotated into the boat frame by the dual-antenna
  heading, and its yaw rate is the heading difference.  This requires a stable
  outdoor GNSS fix.  Pass
  `enable_um982_velocity_feedback:=false` to restore the prior EKF path.
  The default `um982_feedback_mode:=ekf` uses the dedicated UM982-only EKF;
  `um982_feedback_mode:=window` selects the time-window regression alternative.
- `micon_driver_fd/serial_writer` automatically requests ARM. Joystick input cannot
  request DISARM; after an emergency stop is released or the ESP32 resets, the
  driver re-requests ARM once the required zero-thrust command is acknowledged.
  The ESP32's physical and communication emergency-stop interlocks remain active.

The command flags are `bit3=emg`, `bit2=green`, `bit1=yellow`, and `bit0=red`.
The ESP32 enforces the emergency stop and other safety interlocks. Validate that
behavior on restrained hardware before operation.

The shipped joystick mapping makes the green (circle) button both request AUTO
and publish the green indicator request. The miniPC competition bringup sets
`require_autonomy_ready:=false` so mode selection is not blocked while Nav2 is
starting; emergency-stop and command-timeout protection remain active.

```bash
ros2 launch simple_manual manual_control.launch.py serial_port:=/dev/ttyUSB0
```

`simple_manual.launch.py` is also provided as a compatibility alias:

```bash
ros2 launch simple_manual simple_manual.launch.py serial_port:=/dev/ttyUSB0
```

The `axis.*`, `button.*`, and `scale.*` parameters belong to the
`/joy_converter` node and can be changed with ROS 2 parameter commands.

## Running the joystick on the shore PC

`manual_control.launch.py` does not start `joy_node`. Start it on the shore PC
in the same ROS domain so its `/joy` topic reaches the vessel PC.

## Pairing a DualShock4 controller over Bluetooth

The shore PC's `joy` node reads from `/dev/input/js0`, which requires the
DualShock4 to be paired and connected over Bluetooth first.

MAC address on record: `28:C1:3C:3F:D4:80`

1. Put the controller into pairing mode: hold **PS + SHARE** simultaneously
   for ~5 seconds until the light bar flashes white rapidly.
2. On the host (Jetson), run `bluetoothctl` and execute:
   ```
   agent on
   default-agent
   pairable on
   scan on
   ```
   Wait until `Device 28:C1:3C:3F:D4:80 Wireless Controller` appears, then
   `scan off`.
3. Pair, trust, and connect:
   ```
   pair 28:C1:3C:3F:D4:80
   trust 28:C1:3C:3F:D4:80
   connect 28:C1:3C:3F:D4:80
   ```
   `pair` only needs to succeed once; on subsequent uses (after re-enabling
   pairing mode on the controller) `connect` alone is enough. If `connect`
   fails with `br-connection-create-socket`, the controller is not currently
   in pairing mode — repeat step 1.
4. Verify the joystick device node exists:
   ```bash
   ls /dev/input/js0
   ```
5. Start `joy_node` on the shore PC:
   ```bash
   ros2 run joy joy_node
   ```

To disconnect: `bluetoothctl disconnect 28:C1:3C:3F:D4:80`. The pairing
(trust) persists across reboots, so a bare `connect` (after waking the
controller with the PS button) is usually sufficient after the first setup.
