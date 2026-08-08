# Mission runtime readiness

`runtime_readiness_adapter_node` is launched with the persistent Mission
Manager.  It is a read-only, fail-closed adapter: it never starts a task, sends
a Nav2 goal, publishes a velocity command, or changes another node's state.

It publishes the three inputs consumed by Mission Manager:

| Output | Concrete evidence | Becomes false when |
| --- | --- | --- |
| `/mission/readiness/collision_monitor` | a valid `nav2_msgs/CollisionMonitorState` on `/collision_monitor_state` | no state arrives within 1 s |
| `/mission/readiness/buoy_perception` | a stamped, framed `njord_interfaces/BuoyDetectionArray` on `/buoy_detections_3d` | no detection cycle arrives within 1 s |
| `/mission/readiness/dynamic_gate_tf` | fresh TFs from `map` to all four Task 3 gate frames | any TF is absent, zero-stamped, or older than 1 s |

An empty buoy-detection array is valid evidence that perception ran and found no
buoy; a missing array is not.  The Task 3 adapter requires both normal and
parallel gate pairs (`b31_*_gate`, `b32_*_gate`) so either Task 3 variant is
safe to arm.  Override the topics, target frame, frame list, or timeouts only
for a deployment whose producers provide equivalent concrete evidence.

These interlocks are deliberately separate from Nav2 command freshness.  AUTO
may become armed after its preflight evidence is available, while the Command
Arbiter still outputs zero until it receives a fresh Nav2 command.
