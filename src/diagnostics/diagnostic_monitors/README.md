# diagnostic_monitors

Generic diagnostic monitor components for topics that do not yet publish their
own diagnostics.

## HeartbeatAggregator

`HeartbeatAggregator` converts real topic freshness into a heartbeat. It emits
`std_msgs/msg/Empty` only while every configured input has a publisher and a
fresh message. Its output can feed another aggregator, forming the vessel tree:

```text
/heartbeat/driver/camera/front + back -> /heartbeat/driver/camera
camera + lidar + gnss + micon         -> /heartbeat/driver
local + global odometry               -> /heartbeat/localization
```

The Jetson owns the front-camera and LiDAR leaves. The miniPC owns the rear
camera, GNSS, Micon, localization leaves, and aggregate heartbeats. Therefore a
process that is alive but no longer receiving hardware data cannot keep its
branch healthy by publishing an unconditional timer heartbeat.

## TopicHeartbeatMonitor

`TopicHeartbeatMonitor` subscribes with `rclcpp::GenericSubscription`, so it
does not inspect or deserialize message contents. It only checks graph state,
last receive time, and measured frequency, then publishes `/diagnostics` through
`diagnostic_updater`.

Required parameters:

- `monitor_name`
- `topic`
- `topic_type`
- `mode`: `required_frequency`, `heartbeat_only`, or `optional`
- `expected_frequency`
- `minimum_frequency`
- `timeout`

Modes:

| Mode | Intended use | No publisher | Slow frequency |
|---|---|---|---|
| `required_frequency` | sensors, odometry, periodic state | ERROR | WARN |
| `heartbeat_only` | event-like command topics | WARN | ignored |
| `optional` | useful when present | OK until stale data appears | ignored |

Example:

```bash
ros2 launch diagnostic_monitors topic_heartbeat_monitor.launch.py \
  monitor_name:=sim_odom \
  topic:=/odom \
  topic_type:=nav_msgs/msg/Odometry \
  mode:=required_frequency \
  expected_frequency:=50.0 \
  minimum_frequency:=20.0 \
  timeout:=0.5
```
