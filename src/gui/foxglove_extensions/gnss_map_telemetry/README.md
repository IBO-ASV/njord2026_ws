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

Waypoint labels show their route order and competition label, a dashed cyan line
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
