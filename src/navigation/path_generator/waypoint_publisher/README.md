# waypoint_publisher Package

Publishes NJORD task-specific waypoint sequences through Nav2's
`NavigateThroughPoses` action. It also publishes the same intended route on
`/plan` as `nav_msgs/Path` so simulation validators and GUI overlays can compare
the ASV track against the planned route.

## Supported Tasks

- `task1`: NJORD 2026 Task 1 maneuvering/path-finding route
- `task2`: collision-avoidance route
- `task3_1`: normal docking sequence
- `task3_2`: parallel docking sequence

## Interfaces

- Action client: `/navigate_through_poses`
- Publisher: `/plan` (`nav_msgs/Path`)

## Parameters

- `task_type` (`task1`, `task2`, `task3_1`, or `task3_2`; default `task1`)
- `frame_id` (default `map`)
- `publish_rate_hz` (default `2.0`)

## Usage

```bash
ros2 launch waypoint_publisher waypoint_publisher.launch.py task_type:=task1
```

For Task1 simulation, prefer launching through `task1_sim` so dynamics,
Nav2, waypoint publishing, and validation use the same configuration:

```bash
ros2 launch task1_sim task1_sim.launch.py
```

## Task1 Notes

The Task1 config uses a simulation-local Cartesian approximation of the NJORD
2026 course:

- GPS point 1
- waypoint 1.1 through 1.10
- GPS point 2
- GPS point 3
- waypoint 3.1 through 3.3
- GPS point 4

`task1_sim` owns the cardinal-mark stub, virtual obstacles on
`/virtual_obstacles`, route/avoidance status, and `/sim/goal_reached`.

### Real-vessel GPS waypoint mode

Every waypoint must provide `latitude` and `longitude`; the publisher always
projects them through `navsat_transform_node`'s `/fromLL` service. Any number
of waypoint records (including sub-waypoints such as `1.1`) may be added or
removed; their YAML order is the route order. `x`/`y`, `origin`, and
coordinate-mode launch options are unsupported, so every projected goal uses
the same datum and map axes as GNSS localization and the virtual buoy walls.
