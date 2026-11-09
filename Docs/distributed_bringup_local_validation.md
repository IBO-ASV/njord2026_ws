# Distributed bringup: local validation record

Date: 2026-11-09 (JST)
Branches: `integrate/platform/distributed-bringup`, then
`integrate/platform/agx-runtime-recording`

## Checks that passed

- `colcon build --packages-select robot` with ROS 2 Jazzy completed on the
  distributed-bringup branch and again after adding the AGX runtime and rosbag
  components. The latter also installed `runtime_parameter_sync.py` and
  `agx_rosbag_record_manager`.
- `python3 -m py_compile` passed for the networking, AGX, ground Jetson,
  ground PC, Jetson, and miniPC launch files.
- `bash -n` passed for `scripts/run_agx_pair.sh` and
  `scripts/run_zenoh_bridge.sh`.
- Zenoh bridge processes using the ground Jetson, ground PC, Jetson, and miniPC
  configurations started under a short timeout, confirming those configs were
  parsed and accepted by the installed bridge.

## Checks blocked or incomplete

- A process-level network namespace test could not be set up. Running
  `unshare --user --map-root-user --net true` failed because writing
  `/proc/self/uid_map` is not permitted. `sudo -n ip netns add ...` also could
  not run because sudo requires a password.
- The AGX Zenoh configuration could not start on this host: it tried to bind
  `tcp/10.42.0.196:7447`, but that address is not assigned to a local interface
  here, so the OS returned `Cannot assign requested address`. This is an
  expected limitation of using a real-device address on this host; it does not
  verify that the configured AGX interface is correct on the vessel.
- No cross-domain ROS topic or service was sent through Zenoh. The bridge
  startup checks do not establish end-to-end routing, namespace isolation,
  multicast behavior, or critical-link operation.
- No physical AGX/ground station startup or rosbag recording operation was
  tested. Those features only received the package build check.

## Reproduction commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot --event-handlers console_direct+

python3 -m py_compile \
  src/robot/launch/networking.launch.py \
  src/robot/launch/agx_bringup.launch.py \
  src/robot/launch/ground_jetson.launch.py \
  src/robot/launch/ground_pc.launch.py \
  src/robot/launch/jetson_bringup.launch.py \
  src/robot/launch/minipc_bringup.launch.py
bash -n scripts/run_agx_pair.sh scripts/run_zenoh_bridge.sh
```

To complete the blocked checks, run the namespace test on a host that permits
network namespaces, assign each simulated platform its intended IP address,
then verify a ROS topic and service across the Zenoh bridge. Repeat AGX startup
and rosbag start/stop checks on the vessel and ground station.
