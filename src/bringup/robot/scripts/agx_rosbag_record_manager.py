#!/usr/bin/env python3
"""Safe, GUI-controlled rosbag2 recording manager for the AGX."""
import json, os, shutil, signal, subprocess, time
from datetime import datetime
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

EXACT_TOPICS = {
    "/odom", "/odometry/filtered/global", "/odometry/filtered/local", "/tf", "/tf_static",
    "/livox/lidar", "/livox/imu", "/sensor/vehicle_gnss/fix/raw", "/sensor/vehicle_gnss/compass/raw",
    "/mission/status", "/diagnostics", "/foxglove_log",
    "/zed2i/left/preview/q40/compressed", "/yolo/debug_preview/q40/compressed",
    "/zed2i/yolo/debug_image", "/zed2i/debug/detections_image",
    "/heartbeat/driver/camera/front", "/heartbeat/driver/lidar",
    "/task1/safety_points", "/task1/safety_points_visual", "/task2/safety_points",
    "/task2/points_filtered_visual", "/task2/self_vessel_marker", "/task2/opponent_detected",
    "/task2/selected_opponent_marker", "/task2/selected_opponent_velocity_marker",
    "/task2/buoy_pose_correction", "/other_ship/twist", "/tracked_objects/markers",
    "/buoy_detections", "/buoy_detections_3d", "/virtual_obstacles", "/buoy_roi",
    "/actual_path_marker", "/mission/waypoint_markers",
}
SAFE_PREFIXES = ("/zed2i/", "/sensor/vehicle_gnss/", "/odometry/",
                 "/pcl/", "/tracked_objects/", "/other_ship/", "/heartbeat/driver/")
RECORD_ROOT = Path("/home/ibo/rosbags")
MIN_FREE_GB_TO_START, MIN_FREE_GB_TO_CONTINUE = 5.0, 1.0

class AgxRosbagRecordManager(Node):
    def __init__(self):
        super().__init__("agx_rosbag_record_manager")
        self.declare_parameter("record_root", str(RECORD_ROOT))
        self.record_root = Path(self.get_parameter("record_root").value)
        self.record_root.mkdir(parents=True, exist_ok=True)
        self.process = self.log_file = None
        self.output_path, self.started_at = "", 0.0
        self.state, self.message = "IDLE", "Ready"
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/agx/rosbag/recording_status", qos)
        self.service = self.create_service(SetBool, "/agx/rosbag/set_recording", self.set_recording)
        self.timer = self.create_timer(1.0, self.update)
        self.publish_status()

    def publish_status(self):
        payload = {"state": self.state, "path": self.output_path, "message": self.message,
                   "elapsed_sec": max(0, int(time.monotonic()-self.started_at)) if self.started_at else 0,
                   "free_gb": round(shutil.disk_usage(self.record_root).free/(1024**3), 2)}
        msg = String(); msg.data = json.dumps(payload, ensure_ascii=False); self.status_pub.publish(msg)

    def active_topics(self):
        active = []
        for name, _types in self.get_topic_names_and_types():
            if name not in EXACT_TOPICS and not name.startswith(SAFE_PREFIXES): continue
            if name.startswith(("/cmd_vel", "/thrust", "/joy", "/control/", "/mission/start", "/mission/stop")): continue
            try:
                if self.get_publishers_info_by_topic(name): active.append(name)
            except Exception as exc: self.get_logger().debug(f"Could not inspect {name}: {exc}")
        return sorted(set(active))

    def set_recording(self, request, response):
        if request.data:
            if self.process and self.process.poll() is None:
                response.success = False; response.message = f"Recording already active: {self.output_path}"; return response
            free_gb = shutil.disk_usage(self.record_root).free/(1024**3)
            if free_gb < MIN_FREE_GB_TO_START:
                response.success = False; response.message = f"Not enough free space ({free_gb:.1f} GiB; need {MIN_FREE_GB_TO_START:.0f} GiB)"
                self.state, self.message = "ERROR", response.message; self.publish_status(); return response
            topics = self.active_topics()
            if not topics:
                response.success = False; response.message = "No supported sensor or navigation topics are publishing yet"
                self.state, self.message = "ERROR", response.message; self.publish_status(); return response
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_path = str(self.record_root / f"agx_{stamp}")
            try:
                self.log_file = open(self.record_root / f"agx_{stamp}.log", "ab", buffering=0)
                self.process = subprocess.Popen(
                    ["ros2", "bag", "record", "--storage", "mcap", "--output", self.output_path, "--topics", *topics],
                    stdin=subprocess.DEVNULL, stdout=self.log_file, stderr=subprocess.STDOUT,
                    start_new_session=True, env=os.environ.copy())
                self.started_at = time.monotonic(); self.state, self.message = "STARTING", f"Starting with {len(topics)} topics"
                self.get_logger().info(f"Starting rosbag at {self.output_path}: {topics}")
                response.success = True; response.message = f"Recording requested: {self.output_path}"
            except Exception as exc:
                if self.log_file: self.log_file.close(); self.log_file = None
                self.process = None; self.state, self.message = "ERROR", str(exc)
                response.success = False; response.message = str(exc)
            self.publish_status(); return response
        if not self.process or self.process.poll() is not None:
            response.success = False; response.message = "No rosbag recording is active"; return response
        self.state, self.message = "STOPPING", "Finishing bag metadata"
        try:
            os.killpg(self.process.pid, signal.SIGINT)
            response.success = True; response.message = f"Stop requested: {self.output_path}"
        except ProcessLookupError:
            response.success = True; response.message = "Recorder has already stopped"
        self.publish_status(); return response

    def update(self):
        if self.process:
            code = self.process.poll()
            if code is not None:
                self.state = "IDLE" if code == 0 or self.state == "STOPPING" else "ERROR"
                self.message = "Recording saved" if self.state == "IDLE" else f"Recorder exited with code {code}; see adjacent .log"
                self.get_logger().info(self.message); self.process = None
                if self.log_file: self.log_file.close(); self.log_file = None
                self.started_at = 0.0
            elif self.state == "STARTING": self.state, self.message = "RECORDING", "Recording sensor and navigation data"
            elif shutil.disk_usage(self.record_root).free/(1024**3) < MIN_FREE_GB_TO_CONTINUE:
                self.state, self.message = "STOPPING", "Low disk space; stopping safely"
                self.get_logger().error(self.message)
                try: os.killpg(self.process.pid, signal.SIGINT)
                except ProcessLookupError: pass
        self.publish_status()

    def destroy_node(self):
        if self.process and self.process.poll() is None:
            try: os.killpg(self.process.pid, signal.SIGINT); self.process.wait(timeout=10)
            except Exception:
                try: os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError: pass
        if self.log_file: self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = AgxRosbagRecordManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
