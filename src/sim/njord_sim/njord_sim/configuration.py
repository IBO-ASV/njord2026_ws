"""Generate sim-only overrides without touching deployed vehicle configuration."""

import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from .scenario import scenario


def configurations(task_id, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    s = scenario(task_id)
    docking = task_id in ("task3_1", "task3_2", "task4")
    base = (
        Path(share("robot")) / "config/nav2_params_task3_jazzy.yaml"
        if docking
        else Path(share("task1_sim")) / "config/task1_nav2_params_jazzy.yaml"
    )
    nav = yaml.safe_load(base.read_text())
    nav["bt_navigator"]["ros__parameters"].update(
        global_frame="odom",
        odom_topic="/odometry/filtered/local",
        default_server_timeout=1000,
        wait_for_service_timeout=2000,
    )
    controller = nav["controller_server"]["ros__parameters"]
    controller["enable_stamped_cmd_vel"] = False
    controller["controller_frequency"] = 15.0
    controller["progress_checker_plugins"] = ["progress_checker"]
    controller["progress_checker"] = {
        "plugin": "nav2_controller::SimpleProgressChecker",
        "required_movement_radius": 0.15,
        "movement_time_allowance": 45.0,
    }
    if docking:
        c = controller["FollowPath"]
        c.update(vx_samples=11, vy_samples=11, vtheta_samples=9, xy_goal_tolerance=0.35)
        controller["general_goal_checker"]["xy_goal_tolerance"] = 0.35
        controller["general_goal_checker"]["yaw_goal_tolerance"] = math.radians(10)
        strict = [
            w for w in s.route.waypoints if w.waypoint_type in ("dock", "dock_approach")
        ]
        controller["general_goal_checker"]["heading_required_goal_xs"] = [
            w.x for w in strict
        ]
        controller["general_goal_checker"]["heading_required_goal_ys"] = [
            w.y for w in strict
        ]
    else:
        controller["general_goal_checker"] = {
            "plugin": "nav2_controller::SimpleGoalChecker",
            "xy_goal_tolerance": 0.7,
            "yaw_goal_tolerance": 6.28,
            "stateful": True,
        }
        controller["FollowPath"].update(
            use_rotate_to_heading=True, rotate_to_heading_angular_vel=0.4
        )
        controller["min_y_velocity_threshold"] = 0.001
        if task_id == "task2":
            controller["FollowPath"]["desired_linear_vel"] = 1.0288888889
    xs = [w.x for w in s.route.waypoints] + [x for x, y, _ in s.buoys]
    ys = [w.y for w in s.route.waypoints] + [y for x, y, _ in s.buoys]
    for name in ("local_costmap", "global_costmap"):
        p = nav[name][name]["ros__parameters"]
        p.update(
            global_frame="odom",
            update_frequency=5.0,
            publish_frequency=2.0,
            resolution=0.1,
            footprint="[[0.75,0.5],[0.75,-0.5],[-0.75,-0.5],[-0.75,0.5]]",
            footprint_padding=0.1,
            plugins=["obstacle_layer", "inflation_layer"],
        )
        if name == "global_costmap":
            p.update(
                width=int(max(xs) - min(xs) + 30),
                height=int(max(ys) - min(ys) + 30),
                origin_x=float(min(xs) - 15),
                origin_y=float(min(ys) - 15),
            )
        else:
            p.update(width=16, height=16)
        p["obstacle_layer"] = {
            "plugin": "nav2_costmap_2d::ObstacleLayer",
            "observation_sources": "physical walls",
            "physical": {
                "topic": "/sim_obstacles",
                "data_type": "PointCloud2",
                "sensor_frame": "base_link",
                "marking": True,
                "clearing": True,
                "obstacle_max_range": 100.0,
                "raytrace_max_range": 100.0,
                "max_obstacle_height": 2.0,
                "min_obstacle_height": 0.2,
            },
            "walls": {
                "topic": "/virtual_obstacles",
                "data_type": "PointCloud2",
                "sensor_frame": "base_link",
                "marking": True,
                "clearing": True,
                "obstacle_max_range": 100.0,
                "raytrace_max_range": 100.0,
                "max_obstacle_height": 2.0,
                "min_obstacle_height": -0.1,
            },
        }
        p["inflation_layer"] = {
            "plugin": "nav2_costmap_2d::InflationLayer",
            "inflation_radius": 1.2,
            "cost_scaling_factor": 3.0,
        }
    # Route every command through canonical arbitration, using unstamped Twist.
    for n in (
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
    ):
        nav.setdefault(n, {}).setdefault("ros__parameters", {})[
            "enable_stamped_cmd_vel"
        ] = False
    nav["velocity_smoother"]["ros__parameters"].update(
        max_velocity=[1.1, 0.5, 0.65],
        min_velocity=[-0.5, -0.5, -0.65],
        max_accel=[0.5, 0.5, 0.7],
        max_decel=[-0.5, -0.5, -0.7],
    )
    collision = nav["collision_monitor"]["ros__parameters"]
    collision.update(
        observation_sources=["physical"],
        polygons=["FootprintApproach"],
        source_timeout=1.0,
    )
    collision["physical"] = {
        "type": "pointcloud",
        "topic": "/sim_obstacles",
        "min_height": -0.5,
        "max_height": 2.0,
        "enabled": True,
    }
    collision["FootprintApproach"] = {
        "type": "polygon",
        "action_type": "approach",
        "footprint_topic": "/local_costmap/published_footprint",
        "time_before_collision": 0.6,
        "simulation_time_step": 0.1,
        "min_points": 3,
        "max_points": 3,
        "visualize": False,
        "enabled": True,
    }
    nav.setdefault("smoother_server", {}).setdefault("ros__parameters", {}).update(
        smoother_plugins=["simple_smoother"],
        simple_smoother={
            "plugin": "nav2_smoother::SimpleSmoother",
            "tolerance": 1e-10,
            "max_its": 1000,
            "do_refinement": True,
        },
    )
    # The supplied BT expects this smoother id.
    nav["smoother_server"]["ros__parameters"]["smoother_plugins"] = [
        "natural_cubic_spline"
    ]
    nav["smoother_server"]["ros__parameters"]["natural_cubic_spline"] = nav[
        "smoother_server"
    ]["ros__parameters"].pop("simple_smoother")
    navfile = directory / "nav2.yaml"
    navfile.write_text(yaml.safe_dump(nav))
    policy = yaml.safe_load(
        (Path(share("control_manager")) / "config/control_policy.yaml").read_text()
    )
    common = policy["control_policy"]["common"]
    common.update(
        require_nav2_ready=True,
        require_task_ready=True,
        require_localization_heartbeat=True,
    )
    policyfile = directory / "control.yaml"
    policyfile.write_text(yaml.safe_dump(policy))
    registry = yaml.safe_load(
        (Path(share("mission_manager")) / "config/task_registry.yaml").read_text()
    )
    entry = registry["tasks"][task_id]
    if task_id == "task2":
        entry.setdefault("features", {})["buoy_perception"] = True
    # Store a fully geodetic sim route next to its private registry.
    entry.update(
        route_package="",
        route="route.yaml",
        route_key="sim_route",
        frame_id=s.route.frame_id,
    )
    raw = {
        "frame_id": s.route.frame_id,
        "waypoints": [
            {
                "id": w.waypoint_id,
                "competition_id": w.competition_id,
                "latitude": w.latitude,
                "longitude": w.longitude,
                "yaw": w.yaw,
                "type": w.waypoint_type,
                "name": w.name,
            }
            for w in s.route.waypoints
        ],
        "stages": dict(s.route.stages),
        "full_sequence_stages": dict(s.route.full_sequence_stages),
        "constraints": dict(s.route.constraints),
    }
    (directory / "route.yaml").write_text(yaml.safe_dump({"sim_route": raw}))
    # Only advertise this run's task; unrelated registry-local routes don't exist here.
    registry["tasks"] = {task_id: entry}
    registryfile = directory / "registry.yaml"
    registryfile.write_text(yaml.safe_dump(registry))
    return str(navfile), str(policyfile), str(registryfile)
