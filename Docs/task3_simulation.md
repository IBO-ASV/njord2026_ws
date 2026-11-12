# Task3 Navigation and Simulation

Mission Manager経由のセンサなし完遂試験は [新しい共通sim手順](../src/sim/njord_sim/README.md)
と [実行結果](simulation_acceptance.md) を参照してください。以下は既存のTask3単体起動の説明です。

## Run

Build and source the workspace, then launch either Task3 mode:

```bash
ros2 launch task3_sim task3_sim.launch.py task_type:=task3_1
ros2 launch task3_sim task3_sim.launch.py task_type:=task3_2
```

Startup delays can be overridden:

```bash
ros2 launch task3_sim task3_sim.launch.py \
  task_type:=task3_1 driver_delay:=0.0 nav2_delay:=5.0 goal_delay:=8.0
```

## Architecture

```mermaid
flowchart LR
    WP["waypoint_publisher"] -->|NavigateThroughPoses| Nav2
    Nav2["Theta* -> cubic spline -> RPP"] -->|cmd_vel| Kin["X4 kinematics"]
    Kin -->|thruster_command| Physics["MMG dynamics + disturbance"]
    Physics -->|odom + TF| Sensors["GNSS / IMU noise simulation"]
    Sensors --> EKF["local/global EKF + navsat"]
    Orchestrator["Task3 orchestrator<br/>buoys + dock + pointcloud"] --> BuoyGrid["buoy obstacle publisher"]
    BuoyGrid --> Nav2
    EKF --> Nav2
```

The launch uses three startup layers:

1. physics, sensors, Task3 environment, kinematics, EKF, and obstacle grids;
2. Nav2 after TF and filtered odometry become available;
3. waypoint publication after Nav2 action servers become active.

## TF ownership

The simulation dynamics node is the sole Task3 simulation authority for:

```text
world -> map -> odom -> base_link
```

Both EKF nodes use `publish_tf: false` and publish filtered odometry topics
only. On the real robot, GLIM owns the equivalent dynamic TF chain.

## Navigation

Task3 uses a dedicated Nav2 configuration:

- `ThetaStarPlanner` for holonomic planning;
- natural cubic spline smoothing;
- Regulated Pure Pursuit without rotate-to-heading;
- X4 kinematics with surge, sway, and yaw commands;
- dynamic buoy obstacles from TF frames.

## Validation boundary

The files and package metadata are statically checked in this Windows
environment. GitHub CI builds the affected ROS packages. End-to-end behavior,
timing, docking accuracy, and controller tuning still require ROS 2 simulation
execution.
