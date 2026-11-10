#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint Publisher Node
Publishes task-specific waypoint sequences using Nav2 NavigateThroughPoses action

Supports:
- Task 1: Maneuvering and Path Finding
- Task 2: Collision Avoidance
- Task 3: Docking (normal and parallel)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from geographic_msgs.msg import GeoPoint
from nav2_msgs.action import NavigateThroughPoses
from robot_localization.srv import FromLL
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
from pathlib import Path
from enum import Enum

class TaskType(Enum):
    """Enumeration for supported task types"""
    TASK1 = "task1"
    TASK1_SKIP_1_1 = "task1_skip_1_1"
    TASK1_FOLLOW = "task1_follow"
    TASK2 = "task2"
    TASK2_BAK = "task2_bak"
    TASK3_1 = "task3_1"
    TASK3_2 = "task3_2"

class DockingState(Enum):
    """State machine for task3 multi-stage docking."""
    IDLE = 0
    STAGE1_GATE = 1
    STAGE1_APPROACH = 2
    STAGE1_WAIT = 3
    STAGE2_APPROACH = 4
    STAGE2_WAIT = 5
    FINAL_EXIT = 6
    COMPLETE = 7
    FAILED = 8

class WaypointPublisher(Node):
    """
    Waypoint publisher node for NJORD competition tasks
    Publishes waypoints via Nav2's NavigateThroughPoses action
    """
    
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Declare parameters
        self.declare_parameter('task_type', 'task1')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('use_dynamic_gate_midpoints', True)
        self.declare_parameter('run_full_sequence', False)
        self.declare_parameter('max_goal_retries', 1)
        self.declare_parameter('waypoint_marker_topic', '/waypoint_markers')
        self.declare_parameter('nav2_goal_tolerance_m', 1.0)
        self.declare_parameter('show_waypoint_route_line', False)
        self.declare_parameter('waypoint_route_z', -0.05)
        self.declare_parameter('cardinal_wall_enable_topic', '/task1/cardinal_wall_enable')
        self.declare_parameter('cardinal_wall_enable_waypoint', '3')
        self.declare_parameter('start_competition_waypoint', '')
        self.declare_parameter('from_ll_service', '/fromLL')
        self.declare_parameter('cardinal_retirement_heading_topic', '/task1/gps3_to_gps4_heading')

        # Get parameters
        self.task_type_str = self.get_parameter('task_type').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.use_dynamic_gate_midpoints = self.get_parameter('use_dynamic_gate_midpoints').value
        self.run_full_sequence = self.get_parameter('run_full_sequence').value
        self.max_goal_retries = int(self.get_parameter('max_goal_retries').value)
        self.waypoint_marker_topic = self.get_parameter('waypoint_marker_topic').value
        self.nav2_goal_tolerance_m = float(self.get_parameter('nav2_goal_tolerance_m').value)
        self.show_waypoint_route_line = self.get_parameter('show_waypoint_route_line').value
        self.waypoint_route_z = float(self.get_parameter('waypoint_route_z').value)
        self.cardinal_wall_enable_topic = self.get_parameter('cardinal_wall_enable_topic').value
        self.cardinal_wall_enable_waypoint = str(
            self.get_parameter('cardinal_wall_enable_waypoint').value)
        self.start_competition_waypoint = str(
            self.get_parameter('start_competition_waypoint').value).strip()
        self.from_ll_service = self.get_parameter('from_ll_service').value
        self.cardinal_retirement_heading_topic = self.get_parameter(
            'cardinal_retirement_heading_topic').value
        
        # Validate task type
        try:
            self.task_type = TaskType(self.task_type_str)
        except ValueError:
            self.get_logger().error(f"Unknown task type: {self.task_type_str}")
            raise
        
        # Load configuration
        self.config = self._load_config()

        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, self.waypoint_marker_topic, marker_qos)
        self.cardinal_wall_enable_pub = self.create_publisher(
            Bool, self.cardinal_wall_enable_topic, marker_qos)
        self.cardinal_retirement_heading_pub = self.create_publisher(
            Float64, self.cardinal_retirement_heading_topic, marker_qos)
        self.cardinal_wall_enabled = False
        self.cardinal_wall_enable_goal_index = self._cardinal_wall_enable_goal_index()
        self._publish_cardinal_wall_enable()
        self._publish_waypoint_markers()

        # TF lookup is used to replace Task3 gate waypoints with the live midpoint
        # between the corresponding red and green buoys when those frames exist.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize action client for NavigateThroughPoses
        self.nav_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.from_ll_client = self.create_client(FromLL, self.from_ll_service)
        self.geodetic_futures = None
        self.geodetic_indices = []
        self.geodetic_positions = []
        
        # Task3 state machine
        self.docking_state = DockingState.IDLE
        self.dock_wait_timer = None
        self.goal_handle = None
        self.current_task3_stage_name = None
        self.current_task3_poses = []
        self.task3_goal_retry_count = 0
        self.warned_gate_fallback_ids = set()
        
        # Subscription to navigation feedback
        self.goal_reached_subscription = None
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self.first_publish = True
        
        self.get_logger().info(
            f"WaypointPublisher initialized for task: {self.task_type.value}, "
            f"run_full_sequence={self.run_full_sequence}"
        )
        
    def _load_config(self) -> dict:
        """
        Load waypoint configuration from YAML file
        Returns: dict with task configuration
        """
        # Installed ROS packages keep data files under share/<package>, not
        # beside the Python module under lib/python*/site-packages.
        package_share_dir = Path(get_package_share_directory('waypoint_publisher')) / 'config'
        
        config_mapping = {
            TaskType.TASK1: 'task1_waypoints.yaml',
            TaskType.TASK1_SKIP_1_1: 'task1_skip_1_1_waypoints.yaml',
            TaskType.TASK1_FOLLOW: 'task1_follow_waypoints.yaml',
            TaskType.TASK2: 'task2_waypoints.yaml',
            TaskType.TASK2_BAK: 'task2_bak_waypoints.yaml',
            TaskType.TASK3_1: 'task3_waypoints.yaml',
            TaskType.TASK3_2: 'task3_waypoints.yaml',
        }

        config_file = package_share_dir / config_mapping[self.task_type]

        if not config_file.exists():
            self.get_logger().error(f"Config file not found: {config_file}")
            raise FileNotFoundError(f"Config file not found: {config_file}")

        with open(config_file, 'r') as f:
            full_config = yaml.safe_load(f)

        # Extract relevant config based on task type
        if self.task_type == TaskType.TASK1:
            return full_config['task1_config']
        elif self.task_type == TaskType.TASK1_SKIP_1_1:
            return full_config['task1_skip_1_1_config']
        elif self.task_type == TaskType.TASK1_FOLLOW:
            return full_config['task1_follow_config']
        elif self.task_type == TaskType.TASK2:
            return full_config['task2_config']
        elif self.task_type == TaskType.TASK2_BAK:
            return full_config['task2_bak_config']
        elif self.task_type == TaskType.TASK3_1:
            return full_config['task3_1_config']
        elif self.task_type == TaskType.TASK3_2:
            return full_config['task3_2_config']

    def _cardinal_wall_enable_goal_index(self):
        """Return the Task1 NavigateThroughPoses index for competition WP3."""
        if self.task_type != TaskType.TASK1:
            return None
        for index, waypoint in enumerate(self._active_waypoints()):
            if str(waypoint.get('competition_id', waypoint.get('id'))) == self.cardinal_wall_enable_waypoint:
                return index
        self.get_logger().warn(
            f'competition waypoint {self.cardinal_wall_enable_waypoint} is absent; '
            'cardinal virtual walls remain disabled')
        return None

    def _publish_cardinal_wall_enable(self):
        msg = Bool()
        msg.data = self.cardinal_wall_enabled
        self.cardinal_wall_enable_pub.publish(msg)

    def _on_navigation_feedback(self, feedback_msg):
        """Enable Task1 cardinal walls only after Nav2 reports GPS3 passed."""
        if self.cardinal_wall_enabled or self.cardinal_wall_enable_goal_index is None:
            return
        # NavigateThroughPoses feedback in Humble does not expose the active
        # waypoint index.  It does expose the number of poses still pending;
        # it decreases only after the current pose is passed.  This works both
        # for the complete Task1 route and for the route sliced to start at
        # competition WP3.
        total_poses = len(self._active_waypoints())
        poses_remaining = int(feedback_msg.feedback.number_of_poses_remaining)
        if poses_remaining < total_poses - self.cardinal_wall_enable_goal_index:
            self.cardinal_wall_enabled = True
            self._publish_cardinal_wall_enable()
            self.get_logger().info('GPS3 reached: enabled cardinal virtual walls')
    
    def _timer_callback(self):
        """Timer callback to publish waypoints"""
        if self.geodetic_futures is not None:
            self._finish_geodetic_waypoint_conversion()
            return
        if not self.first_publish:
            return
        
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateThroughPoses action server not ready, waiting...")
            return
        
        self.first_publish = False
        self._start_geodetic_waypoint_conversion()
        return
        
        if self.task_type in [
            TaskType.TASK1,
            TaskType.TASK1_SKIP_1_1,
            TaskType.TASK1_FOLLOW,
            TaskType.TASK2,
        ]:
            self._publish_waypoints_single_stage()
        elif self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2]:
            self._publish_task3_first_stage()

    def _start_geodetic_waypoint_conversion(self):
        """Resolve every YAML latitude/longitude waypoint through /fromLL."""
        waypoints = self._active_waypoints()
        incomplete = [self._display_waypoint_id(wp, i) for i, wp in enumerate(waypoints)
                      if 'latitude' not in wp or 'longitude' not in wp]
        if incomplete:
            self.get_logger().error('Every waypoint needs latitude and longitude: ' + ', '.join(incomplete))
            self.first_publish = True
            return
        self.geodetic_indices = list(range(len(waypoints)))
        if self.geodetic_indices and not self.from_ll_client.service_is_ready():
            self.get_logger().warn(f'Waiting for map projection service {self.from_ll_service}')
            self.first_publish = True
            return
        self.geodetic_futures = []
        for index in self.geodetic_indices:
            waypoint = waypoints[index]
            request = FromLL.Request()
            request.ll_point = GeoPoint(latitude=float(waypoint['latitude']),
                                        longitude=float(waypoint['longitude']),
                                        altitude=float(waypoint.get('altitude', 0.0)))
            self.geodetic_futures.append(self.from_ll_client.call_async(request))

    def _finish_geodetic_waypoint_conversion(self):
        if not all(future.done() for future in self.geodetic_futures):
            return
        try:
            self.geodetic_positions = [
                (future.result().map_point.x, future.result().map_point.y)
                for future in self.geodetic_futures
            ]
        except Exception as error:
            self.get_logger().error(f'GPS waypoint map conversion failed: {error}')
            self.geodetic_futures = None
            self.geodetic_indices = []
            self.first_publish = True
            return
        self.geodetic_futures = None
        waypoints = self._active_waypoints()
        self._publish_waypoint_markers(self.geodetic_positions)
        poses = [self._pose_from_waypoint(wp, position)
                 for wp, position in zip(waypoints, self.geodetic_positions)]
        self._publish_cardinal_retirement_heading(waypoints, self.geodetic_positions)
        self._send_navigate_through_poses_goal(poses)
        self.get_logger().info(
            f'Published {len(poses)} latitude/longitude waypoints for {self.task_type.value}')
    
    def _publish_waypoints_single_stage(self):
        """Publish all waypoints for task1 and task2"""
        waypoints = self._build_poses_from_config()
        self._publish_cardinal_retirement_heading(
            self._active_waypoints(),
            [(pose.pose.position.x, pose.pose.position.y) for pose in waypoints])
        self._send_navigate_through_poses_goal(waypoints)
        
        task_name = self.task_type.value
        self.get_logger().info(f"Published {len(waypoints)} waypoints for {task_name}")

    def _publish_cardinal_retirement_heading(self, configured_waypoints, positions):
        """Publish the map-frame course heading calculated from GPS3 to GPS4."""
        by_competition_id = {
            str(waypoint.get('competition_id')): position
            for waypoint, position in zip(configured_waypoints, positions)
        }
        gps3, gps4 = by_competition_id.get('3'), by_competition_id.get('4')
        if gps3 is None or gps4 is None:
            return
        dx, dy = gps4[0] - gps3[0], gps4[1] - gps3[1]
        if dx * dx + dy * dy < 1.0e-8:
            self.get_logger().warn('GPS3 and GPS4 map positions are identical; cannot set wall retirement heading')
            return
        heading = Float64()
        heading.data = math.atan2(dy, dx)
        self.cardinal_retirement_heading_pub.publish(heading)

    def _active_waypoints(self) -> list:
        """Return the configured route, optionally sliced at a competition WP."""
        waypoints = self.config.get('waypoints', [])
        if not self.start_competition_waypoint:
            return waypoints
        for index, waypoint in enumerate(waypoints):
            if str(waypoint.get('competition_id', '')) == self.start_competition_waypoint:
                return waypoints[index:]
        self.get_logger().warn(
            f'competition waypoint {self.start_competition_waypoint} is absent; using full route')
        return waypoints
    
    def _publish_task3_first_stage(self):
        """Publish the gate approach stage for the selected Task3 mode."""
        gate_stage_ids = self._task3_stage_ids('stage_1_gate')
        if not gate_stage_ids:
            self._publish_task3_dock_stage()
            return

        stage1 = self._build_poses_for_ids(gate_stage_ids)
        if stage1:
            self.docking_state = DockingState.STAGE1_GATE
            self._send_navigate_through_poses_goal(stage1, stage_name='stage_1_gate')
            if self.task_type == TaskType.TASK3_2:
                self.get_logger().info(
                    "Task3.2: gate stage sent [gps9 -> corridor_gate -> gps9_gate]"
                )
            else:
                self.get_logger().info(
                    "Task3.1: gate stage sent [gps7 -> corridor_gate -> gps8]"
                )
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage_1_gate")

    def _publish_task3_dock_stage(self):
        """Publish the dock approach stage after the mandatory gate goal."""
        stage1 = self._build_poses_for_ids(self._task3_stage_ids('stage_1'))
        if stage1:
            self.docking_state = DockingState.STAGE1_APPROACH
            self._send_navigate_through_poses_goal(stage1, stage_name='stage_1')
            if self.task_type == TaskType.TASK3_2:
                self.get_logger().info(
                    "Task3.2: dock stage sent [berth2_approach -> berth2]"
                )
            else:
                self.get_logger().info(
                    "Task3.1: dock stage sent [berth1_approach -> berth1]"
                )
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 1")
    
    def _build_poses_from_config(self) -> list:
        """
        Build list of PoseStamped from configuration waypoints
        Returns: list of geometry_msgs.msg.PoseStamped
        """
        poses = []
        waypoints = self._active_waypoints()
        
        for wp in waypoints:
            poses.append(self._pose_from_waypoint(wp))
        
        return poses

    def _publish_waypoint_markers(self, projected_positions=None):
        """Publish numbered waypoint discs and their Nav2 reach radius."""
        waypoints = self._active_waypoints()
        if not waypoints:
            return
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        route = Marker()
        route.header.frame_id, route.header.stamp = self.frame_id, stamp
        route.ns, route.id, route.type, route.action = 'task_waypoint_route', 0, Marker.LINE_STRIP, Marker.ADD
        route.pose.orientation.w, route.scale.x = 1.0, 0.12
        route.color.r, route.color.g, route.color.b, route.color.a = 0.1, 0.9, 1.0, 0.9
        for index, waypoint in enumerate(waypoints):
            x, y = (projected_positions[index] if projected_positions is not None
                    else (float(waypoint.get('x', 0.0)), float(waypoint.get('y', 0.0))))
            route.points.append(Point(x=x, y=y, z=self.waypoint_route_z))
            disc = Marker()
            disc.header.frame_id, disc.header.stamp = self.frame_id, stamp
            disc.ns, disc.id, disc.type, disc.action = 'task_waypoint_reach_radius', index, Marker.CYLINDER, Marker.ADD
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.01
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 2.0 * self.nav2_goal_tolerance_m
            disc.scale.z = 0.02
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = 0.1, 0.8, 1.0, 0.22
            markers.markers.append(disc)
            dot = Marker()
            dot.header.frame_id, dot.header.stamp = self.frame_id, stamp
            dot.ns, dot.id, dot.type, dot.action = 'task_waypoint_point', index, Marker.SPHERE, Marker.ADD
            dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = x, y, 0.18
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.35
            dot.color.g, dot.color.b, dot.color.a = 0.95, 1.0, 1.0
            markers.markers.append(dot)
            label = Marker()
            label.header.frame_id, label.header.stamp = self.frame_id, stamp
            label.ns, label.id, label.type, label.action = 'task_waypoint_label', index, Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.65
            label.pose.orientation.w, label.scale.z = 1.0, 0.45
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = (
                f"WP {self._display_waypoint_id(waypoint, index)}\\n"
                f"reach {self.nav2_goal_tolerance_m:.1f} m"
            )
            markers.markers.append(label)
        if self.show_waypoint_route_line:
            markers.markers.insert(0, route)
        self.waypoint_marker_pub.publish(markers)

    def _display_waypoint_id(self, waypoint: dict, index: int) -> str:
        """Return Task1's grouped competition number for display only."""
        competition_id = waypoint.get('competition_id')
        if competition_id is not None:
            return str(competition_id)
        waypoint_id = int(waypoint.get('id', index + 1))
        if self.task_type == TaskType.TASK1:
            if waypoint_id == 1:
                return '1'
            if 2 <= waypoint_id <= 11:
                return f'1.{waypoint_id - 1}'
            if waypoint_id == 12:
                return '2'
            if waypoint_id == 13:
                return '3'
            if 14 <= waypoint_id <= 16:
                return f'3.{waypoint_id - 13}'
            if waypoint_id == 17:
                return '4'
        return str(waypoint_id)

    def _build_poses_for_ids(self, waypoint_ids: list) -> list:
        """Build poses for the requested waypoint ids, preserving the id order."""
        by_id = {str(wp.get('id')): wp for wp in self.config.get('waypoints', [])}
        poses = []
        missing = []
        for waypoint_id in waypoint_ids:
            wp = by_id.get(str(waypoint_id))
            if wp is None:
                missing.append(str(waypoint_id))
                continue
            poses.append(self._pose_from_waypoint(wp))
        if missing:
            self.get_logger().warn(f"Task3: missing waypoint ids: {', '.join(missing)}")
            return []
        return poses

    def _task3_stage_ids(self, stage_name: str) -> list:
        stages = self.config.get('stages', {})
        if self.run_full_sequence:
            stages = self.config.get('full_sequence_stages', stages)
        return stages.get(stage_name, [])

    def _pose_from_waypoint(self, wp: dict, position_override=None) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        x, y = position_override or (float(wp.get('x', 0.0)), float(wp.get('y', 0.0)))
        midpoint = self._gate_midpoint_from_tf(wp)
        if midpoint is not None:
            x, y = midpoint

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        yaw = float(wp.get('yaw', 0.0))
        pose.pose.orientation = self._yaw_to_quaternion(yaw)
        return pose

    def _gate_midpoint_from_tf(self, wp: dict):
        if not self.use_dynamic_gate_midpoints or wp.get('type') != 'gate':
            return None
        gate_pair = wp.get('gate_pair')
        if not gate_pair:
            return None
        try:
            red_tf = self.tf_buffer.lookup_transform(
                self.frame_id, gate_pair['red_frame'], rclpy.time.Time())
            green_tf = self.tf_buffer.lookup_transform(
                self.frame_id, gate_pair['green_frame'], rclpy.time.Time())
        except Exception as exc:
            waypoint_id = str(wp.get('id'))
            if waypoint_id not in self.warned_gate_fallback_ids:
                self.warned_gate_fallback_ids.add(waypoint_id)
                self.get_logger().warn(
                    f"Task3: using configured gate waypoint for {waypoint_id}; "
                    f"gate TF unavailable: {exc}"
                )
            return None

        rx = red_tf.transform.translation.x
        ry = red_tf.transform.translation.y
        gx = green_tf.transform.translation.x
        gy = green_tf.transform.translation.y
        self.get_logger().debug(
            f"Task3: using live gate midpoint for {wp.get('id')} from "
            f"{gate_pair['red_frame']} and {gate_pair['green_frame']}"
        )
        return (0.5 * (rx + gx), 0.5 * (ry + gy))
    
    def _yaw_to_quaternion(self, yaw: float):
        """Convert yaw angle to quaternion"""
        from geometry_msgs.msg import Quaternion
        import math
        
        half_yaw = yaw / 2.0
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(half_yaw)
        quat.w = math.cos(half_yaw)
        
        return quat
    
    def _send_navigate_through_poses_goal(self, poses: list, stage_name: str = None):
        """
        Send NavigateThroughPoses action goal
        
        Args:
            poses: list of PoseStamped messages
        """
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateThroughPoses action server not ready, retrying...")
            return

        if self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2] and stage_name:
            if stage_name != self.current_task3_stage_name:
                self.task3_goal_retry_count = 0
            self.current_task3_stage_name = stage_name
            self.current_task3_poses = poses
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        
        self.get_logger().debug(f"Sending goal with {len(poses)} poses")
        
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_navigation_feedback,
        ).add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """Callback for NavigateThroughPoses goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by NavigateThroughPoses action server")
            if self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2]:
                self._retry_or_fail_current_task3_stage("goal rejected")
            else:
                self.first_publish = True
            return
        
        self.get_logger().info("Goal accepted by NavigateThroughPoses action server")

        # For task3, wait for goal completion and then trigger stage 2
        if self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2]:
            self.goal_handle.get_result_async().add_done_callback(self._task3_goal_result_callback)
    
    def _task3_goal_result_callback(self, future):
        """Dispatch to correct handler based on current docking state"""
        wrapped_result = future.result()
        if wrapped_result is None or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped_result is None else wrapped_result.status
            self._retry_or_fail_current_task3_stage(f"goal status={status}")
            return

        if self.docking_state == DockingState.STAGE1_GATE:
            self.get_logger().info("Task3: mandatory gate reached; continuing to dock stage")
            self._publish_task3_dock_stage()

        elif self.docking_state == DockingState.STAGE1_APPROACH:
            self.docking_state = DockingState.STAGE1_WAIT
            wait_time = float(self.config.get('constraints', {}).get('wait_time_s', 10))
            berth_name = "Berth 2" if self.task_type == TaskType.TASK3_2 else "Berth 1"
            self.get_logger().info(f"Task3: {berth_name} reached, waiting {wait_time:.0f}s...")
            self.dock_wait_timer = self.create_timer(wait_time, self._stage1_wait_complete)

        elif self.docking_state == DockingState.STAGE2_APPROACH:
            # Arrived at berth2 -> start wait
            self.docking_state = DockingState.STAGE2_WAIT
            constraints = self.config.get('constraints', {})
            wait_time = float(
                constraints.get('second_wait_time_s', constraints.get('wait_time_s', 5))
            )
            self.get_logger().info(f"Task3: Berth 2 reached, waiting {wait_time:.0f}s...")
            self.dock_wait_timer = self.create_timer(wait_time, self._stage2_wait_complete)

        elif self.docking_state == DockingState.FINAL_EXIT:
            self.docking_state = DockingState.COMPLETE
            finish_name = "gps10" if self._task3_stage_ids('stage_3') else "gps8"
            self.get_logger().info(f"Task3: Complete! Reached {finish_name}.")

    def _retry_or_fail_current_task3_stage(self, reason: str):
        """Retry the active Task3 stage without falling back to stage 1."""
        if (
            not self.current_task3_stage_name or
            not self.current_task3_poses or
            self.docking_state in [DockingState.COMPLETE, DockingState.FAILED]
        ):
            self.docking_state = DockingState.FAILED
            self.get_logger().error(f"Task3: failed with no retryable current stage: {reason}")
            return

        if self.task3_goal_retry_count >= self.max_goal_retries:
            self.docking_state = DockingState.FAILED
            self.get_logger().error(
                f"Task3: {self.current_task3_stage_name} failed after "
                f"{self.task3_goal_retry_count} retries: {reason}"
            )
            return

        self.task3_goal_retry_count += 1
        self.get_logger().warn(
            f"Task3: retrying {self.current_task3_stage_name} "
            f"({self.task3_goal_retry_count}/{self.max_goal_retries}) after {reason}"
        )
        self._send_navigate_through_poses_goal(
            self.current_task3_poses,
            stage_name=self.current_task3_stage_name,
        )

    def _stage1_wait_complete(self):
        """Continue according to the selected Task3 flow."""
        if self.dock_wait_timer:
            self.destroy_timer(self.dock_wait_timer)
            self.dock_wait_timer = None
        if self.task_type == TaskType.TASK3_2:
            self.docking_state = DockingState.FINAL_EXIT
            self._publish_task3_third_stage()
        elif self.run_full_sequence:
            self.get_logger().info(
                "========== Task3-1 complete, transitioning to Task3-2 =========="
            )
            self.docking_state = DockingState.STAGE2_APPROACH
            self._publish_task3_second_stage()
        else:
            self.docking_state = DockingState.FINAL_EXIT
            self._publish_task3_second_stage()

    def _publish_task3_second_stage(self):
        """Stage 2: either return to GPS8 or continue to berth2."""
        stage2 = self._build_poses_for_ids(self._task3_stage_ids('stage_2'))
        if stage2:
            self._send_navigate_through_poses_goal(stage2, stage_name='stage_2')
            if self.run_full_sequence:
                self.get_logger().info(
                    "Task3: Stage 2 sent [berth1_exit -> corridor_gate -> "
                    "gps9 -> berth2_approach -> berth2]"
                )
            else:
                self.get_logger().info("Task3.1: Stage 2 sent [berth1_exit -> gps8 - finish]")
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 2")

    def _stage2_wait_complete(self):
        """After berth2 wait: exit to gps10"""
        if self.dock_wait_timer:
            self.destroy_timer(self.dock_wait_timer)
            self.dock_wait_timer = None
        self.docking_state = DockingState.FINAL_EXIT
        self._publish_task3_third_stage()

    def _publish_task3_third_stage(self):
        """Stage 3: gps10 (finish)"""
        stage3 = self._build_poses_for_ids(self._task3_stage_ids('stage_3'))
        if stage3:
            self._send_navigate_through_poses_goal(stage3, stage_name='stage_3')
            self.get_logger().info("Task3: Stage 3 sent [gps10 - finish]")
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 3")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
