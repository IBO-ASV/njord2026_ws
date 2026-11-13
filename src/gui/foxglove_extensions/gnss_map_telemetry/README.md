# GNSS Map Telemetry

Foxglove custom Map panel with a fixed top-right legend for latitude, longitude,
base_link heading, and speed over ground. It consumes
`/sensor/vehicle_gnss/fix/raw`, `/gui/ground_speed_mps`, `/tf`, and `/tf_static`.
The panel renders OpenStreetMap tiles with its bundled Leaflet 1.9.4 runtime.
After centering on the first valid fix (or on the waypoint set when no fix is
available), the map remains under manual pan and zoom control while the vessel
arrow moves and rotates. It also consumes the
transient-local `/ground_waypoint_markers` `visualization_msgs/msg/MarkerArray`
from the Ground PC's local `waypoint_publisher` configuration and overlays each
configured WP.  This marker list is generated from the installed waypoint YAML;
it does not depend on waypoint-marker traffic from the vessel.  The ground PC
selects the YAML from the lightweight `/mission/status.task_id` value published
after the operator starts a task; it shows no stale WPs while idle.

Waypoint labels show their route order and competition label, a dark blue dashed line
connects the route, and the legend identifies the selected task. Version 0.4.0
also adds **LiDAR Mission Splat**, a WebGL2 Gaussian/surfel view of the bounded
`/gui/livox/splat_map` cloud with waypoint, actual-route, and vessel overlays.
Use its ⚙ Topics control to persistently change input topic names. Install
[gnss-map-telemetry-0.4.0.foxe](gnss-map-telemetry-0.4.0.foxe) by dragging it
into Foxglove, then import `foxglove_setting.json`.

The catamaran's bright cyan bow and darker stern point along the `base_link` +X
axis. Its orientation is
resolved on the Ground PC from the `odom -> base_link` TF chain, assuming the fixed
`odom` frame is ENU (+X east, +Y north). If the chain is unavailable, the panel shows a
position dot and `HDG --`.

Leaflet is distributed under the BSD 2-Clause License. Its license text is included
at `dist/vendor/LEAFLET-LICENSE.txt`.

Rebuild the self-contained extension and `.foxe` archive without Node.js or npm:

```bash
./scripts/package.sh
```


## Geographic course editor

The map now shows a **dark blue, opaque, 4 px dashed waypoint connection**.
This is the configured course, not Nav2's changing global plan.

1. Start the updated `njord_sim mission_sim.launch.py`, or the real AGX bringup
   which includes `mission_manager/mission.launch.py`.
2. Reinstall `gnss-map-telemetry-0.4.0.foxe`. The editor controls appear along
   the bottom of GNSS Map Telemetry. The selected task, **SIM / 実機用** and exact
   YAML source path are shown there.
3. With Mission **IDLE** and requested mode **MANUAL**, click **編集を開始**.
   Drag a waypoint or buoy. Click a buoy to select red, green, north, east,
   south or west cardinal type, or delete it. **ブイを追加** places a new red
   buoy at the next map click; then change its type if required.
4. Click **YAMLへ保存**, or **変更を破棄・再読込** to abandon the draft.
   Unconfirmed edits stay in the panel and do not affect navigation. Editing
   locks when control/status is stale or a mission/AUTO request is active.

Simulation saves to `njord_sim/config/course_overrides.yaml` (in this symlink
workspace: `src/sim/njord_sim/config/course_overrides.yaml`), per task.
**Restart the sim after saving.** On restart, mission route generation, sensor
stand-ins, costmap extents and independent scoring use the same edited
waypoints/buoys. Synthetic docks translate with edited berth targets; their
orientation and the stored waypoint yaw are preserved. A Task1.2 override is
separate from Task1. The empty default `tasks: {}` retains all surveyed defaults.

The real editor task selector reads registry-owned waypoint YAML files.
Saving patches only latitude/longitude scalar spans and a top-level
`map_buoys.<route_key>` block. Comments, YAML anchors/aliases, yaw, stages,
constraints and other waypoint metadata are preserved. Shared aliases continue
following their anchored coordinates. The next Mission loads those edited
waypoints; **real buoy entries are map annotations, not fabricated sensor
measurements or navigation obstacles**. Active real missions cannot be edited.

The server is `mission_manager/course_editor_node`, enabled by default in the
mission and sim launches. `/course_editor/state` is a transient-local JSON
`std_msgs/msg/String`. `/course_editor/apply` uses
`rcl_interfaces/srv/SetParametersAtomically` with one string parameter named
`edit`. It accepts `load` and `save`, registry task IDs, coordinates, known buoy
types and a source revision. Client-supplied paths are not accepted. Saves
validate IDs/order/coordinates/types, reject stale revisions and atomically
replace the resolved source file while preserving install symlinks. If a file
is read-only, the service reports the failure and retains the draft.

Only the editor state is new map telemetry; no task-specific layout is needed.
The old geographic MarkerArray display remains available when the editor server
is absent. Course fitting now permits zoom 19 so nearby markers remain usable.
[A browser preview with synthetic telemetry and an offline background](../../../../docs/reference/course_editor_preview_20261113.png).

Validation covers repeated telemetry during a drag, buoy type/add/delete,
save/reload/conflict handling, stale/control locks, YAML comments/aliases,
revision rejection, sim geometry after reload and real YAML edits over ROS.
