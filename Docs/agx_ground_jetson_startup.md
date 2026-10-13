# AGX vessel + ground Jetson startup

Verified on 2026-10-13 (JST). Both workspaces: `/home/ibo/njord2026_ws`.

- Vessel AGX: `ibo@100.73.137.72`, ROS domain 5.
- Ground Jetson: `ibo@100.115.34.103`, ROS domain 6.
- Branch on both: `fix/agx-startup`. Their original `test07089` commit IDs differ.

## Start

Start the receiver first, once per host:

```bash
# Ground Jetson
./scripts/run_agx_pair.sh ground
# Vessel AGX
./scripts/run_agx_pair.sh vessel
```

This script must set CycloneDDS before invoking `ros2 launch`: the launch
process itself creates component-loading service clients. Setting RMW only
inside a launch description does not configure those clients.
DDS uses loopback/unicast discovery with 120 participant slots. Zenoh connects
between the Tailscale addresses, and raw camera/depth/LiDAR clouds remain local.
Ground Foxglove listens on TCP 8765; front JPEG-RTP uses UDP 5600.

The AGX launch combines the CPU vessel/control graph and SDK-backed ZED driver.
Thruster output, alert lamp output, BMS, back camera, Nav2, and MID360 are OFF
by default. UM982 and ZED are enabled only if their known by-id paths exist.
No mission is automatically started. These are startup-check defaults, not
an assertion that the vessel is ready to navigate.

## Installed dependencies and build

AGX required `ros-jazzy-xacro`, `ros-jazzy-robot-localization`, and
`gstreamer1.0-libav`; ground required `gstreamer1.0-libav`.
Both have `rmw_cyclonedds_cpp`. The AGX was missing Zenoh; the existing
ARM64 v1.10.1 executable from the ground Jetson was installed into
`~/.local/bin/zenoh-bridge-ros2dds` on AGX.

The AGX CUDA path must be visible before configuring GPU packages:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PATH=/usr/local/cuda/bin:$PATH
export CUDACXX=/usr/local/cuda/bin/nvcc
export NJORD_PROFILE=jetson
colcon build --symlink-install --packages-select njord_platform zed2i_driver \
  foxglove_image_preview robot --cmake-clean-cache --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --packages-select mission_manager robot
```

The authenticated critical link uses UDP 45101 over Tailscale. A newly generated
shared key was provisioned with user approval at `/etc/njord/critical_link.key`
on both machines, mode 0600. The key is not stored in Git or this document.

## Verification

- ZED 2i serial 34432991: SDK opened, HD720 at 15 fps; images measured
  approximately 14.6 Hz and depth approximately 14.2 Hz.
- Hardware JPEG encoding started; ground received approximately 2.7 fps.
- Zenoh TCP connection established over Tailscale.
- Ground `/mission/get_status` returned IDLE, MANUAL requested, AUTO prohibited.
- Ground received transient-local `/mission/status`.
- AGX received `/heartbeat/ground_station` through critical link at 1.0 Hz.
- Hardware thruster writer was not started.
- Final running logs: AGX `/tmp/njord-startup-check/agx-active.log`,
  ground `/tmp/njord-startup-check/ground-complete.log`; no launch process
  crash was found in these final logs. Absent SBUS device warns repeatedly.
- Installed-resource regression tests: 3 passed. Existing registry/geodesy
  tests: 13 passed, 1 failed on the unchanged Task 2 runtime-readiness
  expectation (`buoy_perception` versus an empty set); not changed here.

## Hardware limits

MID360 addresses in the configurations (`192.168.1.114`, `192.168.1.151`)
did not respond to ping or ARP on the connected AGX Ethernet segment. A
192.168.1.5/24 alias was temporarily added for this check and then removed.
MID360 data startup is therefore unverified. The checked-in Livox SDK/driver
submodules already had staged deletions; these existing changes were preserved.

No UM982 by-id device, BMS, SBUS, or back camera was confirmed. The CP2102
serial device was present, but motor actuation was not tested. Autonomous
navigation and vehicle motion were not tested. No boot-time service was enabled.

For a connected MID360, first establish its dedicated network/IP and build the
Livox driver without overwriting existing submodule changes; then explicitly
start `./scripts/run_agx_pair.sh vessel enable_mid360:=true`.

## Fixes

Mission Manager and return-home monitor now locate configuration under the
installed ament package share rather than relative to Python `site-packages`.
The ground wrapper owns Foxglove's port explicitly, avoiding generic `port`
arguments from video receiver includes leaking into the WebSocket server.

DDS/Zenoh deployment reference:
https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds
