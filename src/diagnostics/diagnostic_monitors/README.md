# diagnostic_monitors

Generic diagnostic monitor components for topics that do not yet publish their
own diagnostics.

## HeartbeatAggregator

`HeartbeatAggregator` converts real topic freshness into a heartbeat. It emits
`std_msgs/msg/Empty` only while every configured input has a publisher and a
fresh message. The component remains available for aggregates that provide
distinct downstream value; the active miniPC health inventory monitors GNSS,
Micon, localization, and link signals individually and does not publish the
redundant `/heartbeat/driver` aggregate.

The component does not publish unconditional timer heartbeats: every aggregate
must be configured with the source topics whose freshness it represents.

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
