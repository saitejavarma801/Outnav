#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus, CompressedImage
from std_msgs.msg import Float64
import math
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray
import json
import cv2
import os
import threading
import subprocess
import time
import base64
import platform
import sys
import re
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class OutdoorNavNode(Node):
    def __init__(self):
        super().__init__('outdoornav_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mission_task_pub = self.create_publisher(String, '/mission_tasks', 10)
        self.start_pub = self.create_publisher(Bool, '/start_mission', 10)
        self.stop_pub = self.create_publisher(Bool, '/stop_mission', 10)
        self._sync_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.route_pub = self.create_publisher(Float64MultiArray, '/route_points', self._sync_qos)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.mission_file_pub = self.create_publisher(String, '/mission_file', 10)
        self.mission_file_status_pub = self.create_publisher(String, '/mission_file_status', 10)
        self.heartbeat_pub = self.create_publisher(String, '/outnav/heartbeat', 10)
        self.mission_data_pub = self.create_publisher(String, '/mission_data', self._sync_qos)
        self.settings_pub = self.create_publisher(String, '/outnav/settings', self._sync_qos)
        self.status_pub = self.create_publisher(String, '/outnav/status', 10)
        self.route_ack_pub = self.create_publisher(String, '/outnav/route_ack', 10)
        self.camera_control_pub = self.create_publisher(String, '/outnav/camera_control', 10)

        # Subscriptions (for feedback / sensors)
        self.create_subscription(Bool, '/lidar_alert', self.lidar_callback, 10)
        self.create_subscription(String, '/mission_progress', self.progress_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.create_subscription(Float64, '/gps/heading', self.gps_heading_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/pose', self.localization_pose_callback, 10)
        self.create_subscription(String, '/outnav/settings', self.settings_callback, self._sync_qos)
        self.create_subscription(String, '/mission_file', self.mission_file_callback, 10)
        self.create_subscription(String, '/mission_data', self.mission_data_callback, self._sync_qos)
        self.create_subscription(Float64MultiArray, '/route_points', self.route_callback, self._sync_qos)
        self.create_subscription(String, '/mission_tasks', self.mission_tasks_callback, 10)
        self.create_subscription(Bool, '/start_mission', self.start_mission_callback, 10)
        self.create_subscription(Bool, '/stop_mission', self.stop_mission_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        self.create_subscription(String, '/outnav/camera_control', self.camera_control_callback, 10)
        
        self.get_logger().info('OutdoorNavNode initialized with /cmd_vel and /mission_tasks.')
        self.waypoint_buffer = []
        self.home_ll = None
        self.speed_mps = 0.5
        self.camera_running = False
        self.cap = None
        self._camera_device = None
        self._camera_stop_event = threading.Event()
        self._camera_thread = None
        self._camera_lock = threading.Lock()
        self.obstacle_detected = False
        self.camera2_running = False
        self.cap2 = None
        self._camera2_device = None
        self._camera2_stop_event = threading.Event()
        self._camera2_thread = None
        self._camera2_lock = threading.Lock()
        self.gps_port = "/dev/ttyTHS1"
        self.gps_enabled = True
        self.localization_pose_enabled = False
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.device_mode = "laptop"
        self.node_id = platform.node() or "outnav"
        self._is_jetson = self._detect_jetson()
        self.camera_fps = 10
        self._camera_frame_delay = 1.0 / max(self.camera_fps, 1)
        self.camera_width = 640
        self.camera_height = 480
        self.camera_backend = self._default_camera_backend()
        self.camera_use_gst = self.camera_backend == "gstreamer"
        self._camera_backend_user_override = False
        self.camera_use_mjpeg = True
        self.camera_watchdog_sec = float(os.getenv("OUTNAV_CAMERA_WATCHDOG_SEC", "8.0"))
        self.camera_jpeg_quality = 70
        self.camera_warmup_frames = 5
        self.camera_read_timeout_ms = int(os.getenv("OUTNAV_CAMERA_READ_TIMEOUT_MS", "2000"))
        self.camera_open_timeout_ms = int(os.getenv("OUTNAV_CAMERA_OPEN_TIMEOUT_MS", "2000"))
        self._camera_last_frame_wall = 0.0
        self._camera2_last_frame_wall = 0.0
        self._camera_last_restart_wall = 0.0
        self._camera2_last_restart_wall = 0.0
        self._camera_restart_cooldown_sec = 5.0
        self._camera_watchdog_stop = threading.Event()
        self._camera_watchdog_thread = None
        self.camera_use_worker = self._default_camera_worker()
        self._camera_proc = None
        self._camera2_proc = None
        self._camera_proc_lock = threading.Lock()
        self._camera_frame_sub = None
        self._camera2_frame_sub = None
        self._camera_worker_path = os.path.join(self.base_dir, "camera_worker.py")
        # Robot-side camera recording (no streaming)
        self._recording_lock = threading.Lock()
        self._recording_thread = None
        self._recording_stop = threading.Event()
        self._recording_path = None
        self._recording_device = None
        self._recording_fps = None

        # Mission state
        self.mission_running = False
        self.current_wp_idx = 0
        self.task_running = False
        self.mission_tasks = {}  # {waypoint_index: "program.py"}
        self.exec_mode = "at_waypoints"
        self.tasks_enabled = True
        self.emergency_stop_active = False
        self.last_position = None  # (lat, lng) from GPS
        self.last_heading_rad = 0.0  # radians; from GPS heading or odom yaw
        self.last_position_ts = 0.0
        self.last_position_source = "none"
        self.last_fix_wall = 0.0
        self.last_odom_wall = 0.0
        self.last_progress_wall = 0.0
        self.last_heartbeat_wall = 0.0
        self.nav_timer = self.create_timer(0.2, self._navigation_tick)
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self.status_timer = self.create_timer(1.0, self._publish_status)

    # === Command velocity publishing ===
    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        if self.device_mode != "robot":
            return
        if self.emergency_stop_active:
            return
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'/cmd_vel published → linear={linear_x:.2f}, angular={angular_z:.2f}')

    # === Mission task publishing ===
    def publish_mission_tasks(self, json_str: str):
        msg = String()
        msg.data = json_str
        self.mission_task_pub.publish(msg)
        self.get_logger().info(f'/mission_tasks published ({len(json_str)} bytes)')
        self._cache_mission_tasks(json_str)

    def _cache_mission_tasks(self, json_str: str):
        try:
            parsed = json.loads(json_str)
            tasks = parsed.get("tasks", [])
            self.exec_mode = parsed.get("execution_mode", "at_waypoints")
            self.tasks_enabled = bool(parsed.get("tasks_enabled", True))
            self.mission_tasks = {}
            for t in tasks:
                idx = t.get("waypoint")
                program = t.get("program")
                if program is not None:
                    self.mission_tasks[int(idx)] = program
            self.get_logger().info(f"[Mission] Cached {len(self.mission_tasks)} task(s), mode={self.exec_mode}")
        except Exception as e:
            self.get_logger().warn(f"[Mission] Failed to parse tasks JSON: {e}")

    # === Mission control ===
    def start_mission(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self.get_logger().info('Start mission signal sent.')
        self._start_mission_local()

    def stop_mission(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)
        self.get_logger().info('Stop mission signal sent.')
        self._stop_mission_local()

    def _start_mission_local(self):
        if self.emergency_stop_active:
            self.get_logger().warn("[Mission] start requested while emergency stop active.")
            return
        if self.waypoint_buffer:
            self.current_wp_idx = 0
            self.mission_running = True
            self.get_logger().info(f"[Mission] Navigation started with {len(self.waypoint_buffer)} waypoint(s).")
        else:
            self.get_logger().warn("[Mission] start_mission called with no waypoints.")

    def _stop_mission_local(self):
        self.mission_running = False
        self.publish_cmd_vel(0.0, 0.0)

    def resume_mission(self):
        """Resume a paused mission (re-uses start signal)."""
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self.get_logger().info('Resume mission signal sent.')
        if self.emergency_stop_active:
            self.get_logger().warn("[Mission] resume requested while emergency stop active.")
            return
        if self.waypoint_buffer:
            self.mission_running = True
            self.get_logger().info("[Mission] Navigation resumed.")

    # === Feedback from LiDAR ===
    def lidar_callback(self, msg: Bool):
        if msg.data:
            # Only react on new detections to avoid spamming stop signals
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.get_logger().warn('Obstacle detected! Stopping robot.')

                # Immediately halt motion
                self.publish_cmd_vel(0.0, 0.0)
                
                # Tell mission logic to stop and raise emergency stop flag
                self.stop_mission()
                self._publish_emergency_stop(True)
        else:
            if self.obstacle_detected:
                self.get_logger().info('LiDAR clear. Emergency stop released.')
                self._publish_emergency_stop(False)
            self.obstacle_detected = False

    # === Feedback from robot progress ===
    def progress_callback(self, msg: String):
        self.last_progress_wall = time.time()
        self.get_logger().info(f'Mission progress update: {msg.data}')

    def _publish_heartbeat(self):
        now = time.time()
        self.last_heartbeat_wall = now
        try:
            msg = String()
            msg.data = json.dumps({
                "ts": now,
                "node": self.node_id,
                "mode": self.device_mode,
            })
            self.heartbeat_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"[ROS] heartbeat publish failed: {e}")

    def _publish_status(self):
        try:
            msg = String()
            msg.data = json.dumps({
                "node": self.node_id,
                "mode": self.device_mode,
                "mission_running": bool(self.mission_running),
                "emergency_stop": bool(self.emergency_stop_active),
                "wp_count": int(len(self.waypoint_buffer) if self.waypoint_buffer else 0),
                "wp_index": int(self.current_wp_idx),
                "gps_enabled": bool(self.gps_enabled),
                "localization_pose_enabled": bool(self.localization_pose_enabled),
                "pose_source": self.last_position_source,
            })
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"[ROS] status publish failed: {e}")

    def publish_route(self, coords):
        """Publish full GPS route for the robot to follow."""
        from std_msgs.msg import Float64MultiArray
        msg = Float64MultiArray()
        flat = []
        for lat, lon in coords:
            flat.extend([lat, lon])
        msg.data = flat
        self.route_pub.publish(msg)
    def publish_route_json(self, coords_json: str):
        try:
            coords_raw = json.loads(coords_json)
            coords = []
            for item in coords_raw:
                if isinstance(item, dict) and 'lat' in item and 'lng' in item:
                    coords.append((item['lat'], item['lng']))
                elif isinstance(item, (list, tuple)) and len(item) >= 2:
                    coords.append((item[0], item[1]))
            if coords:
                self.publish_route(coords)
            else:
                self.get_logger().warn("[ROS] publish_route_json received empty/invalid coords")
        except Exception as e:
            self.get_logger().error(f"[ROS] publish_route_json failed: {e}")

    def publish_mission_file(self, payload: dict):
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self.mission_file_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[ROS] publish_mission_file failed: {e}")

    def publish_mission_file_status(self, payload: dict):
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self.mission_file_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[ROS] publish_mission_file_status failed: {e}")

    def publish_mission_data(self, payload: dict):
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self.mission_data_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[ROS] publish_mission_data failed: {e}")

    def publish_settings(self, payload: dict):
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self.settings_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[ROS] publish_settings failed: {e}")

    def go_road(self, coords_json: str):
        """Alias used by UI for OSRM routes."""
        self.publish_route_json(coords_json)

    def clear_waypoints(self):
        self.waypoint_buffer = []

    def add_waypoint(self, lat: float, lng: float):
        self.waypoint_buffer.append((lat, lng))
        # keep silent per-request; only log aggregated info elsewhere

    def commit_mission(self):
        """Publish buffered waypoints as a route."""
        if not self.waypoint_buffer:
            self.get_logger().warn("[ROS] commit_mission called with no waypoints")
            return
        self.publish_route(self.waypoint_buffer)

    def set_speed(self, speed: float):
        self.speed_mps = speed
    
    def set_gps_port(self, port: str):
        """Persist the chosen GPS communication port."""
        self.gps_port = port
        self.get_logger().info(f"[ROS] GPS port set to {port}")

    @staticmethod
    def _coerce_bool(value, default: bool = False) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("1", "true", "yes", "on"):
                return True
            if lowered in ("0", "false", "no", "off"):
                return False
        return default

    def _apply_sensor_mode_policy(self):
        # Enforce requested behavior:
        # GPS ON  -> localization pose OFF
        # GPS OFF -> localization pose ON
        self.localization_pose_enabled = not self.gps_enabled
        # Force fresh pose from newly selected source.
        self.last_position = None
        self.last_position_ts = 0.0
        self.last_position_source = "none"
        mode_label = "gps" if self.gps_enabled else "localization_pose"
        self.get_logger().info(
            f"[Localization] mode={mode_label} gps_enabled={self.gps_enabled} "
            f"localization_pose_enabled={self.localization_pose_enabled}"
        )

    def set_gps_enabled(self, enabled: bool):
        self.gps_enabled = self._coerce_bool(enabled, default=True)
        self._apply_sensor_mode_policy()

    def set_localization_pose_enabled(self, enabled: bool):
        local_enabled = self._coerce_bool(enabled, default=False)
        self.gps_enabled = not local_enabled
        self._apply_sensor_mode_policy()

    def set_device_mode(self, mode: str):
        cleaned = (mode or "").strip().lower()
        if cleaned in ("laptop", "robot"):
            self.device_mode = cleaned
            self.get_logger().info(f"[ROS] Device mode set to {cleaned}")

    def set_home(self, lat: float, lng: float):
        self.home_ll = (lat, lng)
        self.get_logger().info(f"[ROS] Home set to ({lat:.6f}, {lng:.6f})")

    def reset_mission(self):
        self.clear_waypoints()
        self.stop_mission()
        self.get_logger().info("[ROS] Mission reset")

    def deactivate_emergency_stop(self):
        if hasattr(self, 'emergency_stop_pub'):
            msg = Bool()
            msg.data = False
            self.emergency_stop_pub.publish(msg)
            self.get_logger().info("[ROS] Emergency stop cleared")

    def _publish_emergency_stop(self, active: bool):
        """Helper to toggle the emergency stop topic if available."""
        if hasattr(self, 'emergency_stop_pub'):
            msg = Bool()
            msg.data = active
            self.emergency_stop_pub.publish(msg)

    def odom_callback(self, msg):
        """Monitor odometry to compute speed and heading."""
        self.last_odom_wall = time.time()
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = (vx ** 2 + vy ** 2) ** 0.5

        # heading from quaternion
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        heading_deg = (math.degrees(yaw) + 360) % 360
        self.last_heading_rad = yaw

        # send to dashboard if Qt connected
        try:
            if hasattr(self, 'window') and hasattr(self.window, "view"):
                # Update speed and heading
                self.window.view.page().runJavaScript(
                    f"updateSpeedHUD({speed}); updateHeadingHUD({heading_deg});"
                )
        except Exception:
            pass

    # === GPS callbacks (preferred source for navigation) ===
    def gps_callback(self, msg: NavSatFix):
        if not self.gps_enabled:
            return
        # NavSatStatus holds the fix status; ignore invalid / no-fix samples
        self.last_fix_wall = time.time()
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if abs(msg.latitude) < 1e-6 and abs(msg.longitude) < 1e-6:
            return
        self.last_position = (msg.latitude, msg.longitude)
        self.last_position_ts = self.get_clock().now().nanoseconds / 1e9
        self.last_position_source = "gps"

    def gps_heading_callback(self, msg: Float64):
        if not self.gps_enabled:
            return
        # Heading provided in degrees; convert to radians
        self.last_heading_rad = math.radians(msg.data % 360)

    def localization_pose_callback(self, msg: PoseWithCovarianceStamped):
        if not self.localization_pose_enabled:
            return
        try:
            pose = msg.pose.pose
            self.last_position = (float(pose.position.x), float(pose.position.y))
            self.last_position_ts = self.get_clock().now().nanoseconds / 1e9
            q = pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.last_heading_rad = yaw
            self.last_position_source = "localization_pose"
        except Exception as e:
            self.get_logger().warn(f"[Localization] Failed to parse /localization/pose: {e}")

    def settings_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        sender_id = payload.get("sender_id")
        if sender_id and sender_id == self.node_id:
            return
        try:
            if "gpsUartEnabled" in payload:
                self.set_gps_enabled(payload.get("gpsUartEnabled"))
            elif "gpsEnabled" in payload:
                self.set_gps_enabled(payload.get("gpsEnabled"))
            elif "localizationPoseEnabled" in payload:
                self.set_localization_pose_enabled(payload.get("localizationPoseEnabled"))

            if "maxSpeed" in payload:
                self.set_speed(float(payload.get("maxSpeed")))
            gps_port = payload.get("gpsPort")
            if gps_port:
                self.set_gps_port(str(gps_port))
        except Exception as e:
            self.get_logger().warn(f"[ROS] settings_callback apply failed: {e}")

    # === Remote command callbacks ===
    def start_mission_callback(self, msg: Bool):
        if not msg.data:
            return
        if self.device_mode != "robot":
            return
        self.get_logger().info("[ROS] start_mission command received")
        self._start_mission_local()

    def stop_mission_callback(self, msg: Bool):
        if not msg.data:
            return
        if self.device_mode != "robot":
            return
        self.get_logger().info("[ROS] stop_mission command received")
        self._stop_mission_local()

    def emergency_stop_callback(self, msg: Bool):
        if self.device_mode != "robot":
            return
        self.emergency_stop_active = bool(msg.data)
        if self.emergency_stop_active:
            self.get_logger().warn("[ROS] Emergency stop command received")
            self._stop_mission_local()
        else:
            self.get_logger().info("[ROS] Emergency stop cleared command received")

    def route_callback(self, msg: Float64MultiArray):
        if self.device_mode != "robot":
            return
        data = list(msg.data or [])
        if len(data) < 2 or len(data) % 2 != 0:
            self.get_logger().warn("[ROS] route_points received invalid data")
            return
        coords = []
        for i in range(0, len(data), 2):
            coords.append((data[i], data[i + 1]))
        self.waypoint_buffer = coords
        self.current_wp_idx = 0
        self.mission_running = False
        self.get_logger().info(f"[Mission] Route updated with {len(coords)} waypoint(s).")
        try:
            ack = String()
            ack.data = json.dumps({
                "node": self.node_id,
                "count": int(len(coords)),
            })
            self.route_ack_pub.publish(ack)
        except Exception:
            pass

    def mission_tasks_callback(self, msg: String):
        if self.device_mode != "robot":
            return
        self._cache_mission_tasks(msg.data)

    def mission_file_callback(self, msg: String):
        if self.device_mode != "robot":
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        sender_id = payload.get("sender_id")
        if sender_id and sender_id == self.node_id:
            return
        name = payload.get("name")
        data_b64 = payload.get("data_b64")
        overwrite = bool(payload.get("overwrite", False))
        if not name or not data_b64:
            return
        try:
            script_path = os.path.join(self.base_dir, "missions", "programs", name)
            if os.path.exists(script_path) and not overwrite:
                self.publish_mission_file_status({
                    "status": "exists",
                    "name": name,
                    "sender_id": sender_id,
                })
                return
            raw = base64.b64decode(data_b64.encode("ascii"))
            os.makedirs(os.path.dirname(script_path), exist_ok=True)
            with open(script_path, "wb") as f:
                f.write(raw)
            self.publish_mission_file_status({
                "status": "saved",
                "name": name,
                "sender_id": sender_id,
                "size": len(raw),
            })
        except Exception as e:
            self.publish_mission_file_status({
                "status": "error",
                "name": name,
                "sender_id": sender_id,
                "error": str(e),
            })

    def mission_data_callback(self, msg: String):
        if self.device_mode != "robot":
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        sender_id = payload.get("sender_id")
        if sender_id and sender_id == self.node_id:
            return
        name = payload.get("name") or "mission_latest"
        data_b64 = payload.get("data_b64")
        overwrite = bool(payload.get("overwrite", False))
        if not data_b64:
            return
        try:
            safe = "".join(c for c in name if c.isalnum() or c in ("-", "_"))
            folder = os.path.join(self.base_dir, "missions", "saved_missions")
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f"{safe}.json")
            if os.path.exists(path) and not overwrite:
                self.publish_mission_file_status({
                    "status": "exists",
                    "name": safe,
                    "kind": "mission",
                    "sender_id": sender_id,
                })
                return
            raw = base64.b64decode(data_b64.encode("ascii"))
            with open(path, "wb") as f:
                f.write(raw)
            latest_path = os.path.join(folder, "mission_latest.json")
            try:
                with open(latest_path, "wb") as f:
                    f.write(raw)
            except Exception:
                pass
            try:
                decoded = raw.decode("utf-8")
                parsed = json.loads(decoded)
                waypoints = parsed.get("waypoints") or []
                coords = []
                for item in waypoints:
                    if isinstance(item, dict) and "lat" in item and "lng" in item:
                        coords.append((float(item["lat"]), float(item["lng"])))
                    elif isinstance(item, (list, tuple)) and len(item) >= 2:
                        coords.append((float(item[0]), float(item[1])))
                if coords:
                    # Publish so UI subscribers (including robot GUI) receive the route.
                    self.publish_route(coords)
                else:
                    self.get_logger().warn("[Mission] mission_data received with no waypoints")
            except Exception as e:
                self.get_logger().warn(f"[Mission] Failed to apply mission waypoints: {e}")
            self.publish_mission_file_status({
                "status": "saved",
                "name": safe,
                "kind": "mission",
                "sender_id": sender_id,
                "size": len(raw),
            })
        except Exception as e:
            self.publish_mission_file_status({
                "status": "error",
                "name": name,
                "kind": "mission",
                "sender_id": sender_id,
                "error": str(e),
            })

    # === Navigation controller ===
    def _navigation_tick(self):
        if self.device_mode != "robot":
            return
        if self.emergency_stop_active:
            return
        if not self.mission_running:
            return
        if self.task_running:
            return  # paused while executing a task
        if not self.waypoint_buffer or self.current_wp_idx >= len(self.waypoint_buffer):
            return
        if not self.last_position:
            return
        # Skip if GPS is stale (>3s old)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.last_position_ts and (now_sec - self.last_position_ts) > 3.0:
            return

        cur_lat, cur_lng = self.last_position
        tgt_lat, tgt_lng = self.waypoint_buffer[self.current_wp_idx]

        dist = self._haversine(cur_lat, cur_lng, tgt_lat, tgt_lng)
        bearing = math.atan2(tgt_lng - cur_lng, tgt_lat - cur_lat)  # rough; ok for small deltas
        heading_err = self._normalize_angle(bearing - self.last_heading_rad)

        # Simple proportional control
        ang_gain = 1.5
        lin_gain = 1.0
        angular_z = max(-1.0, min(1.0, ang_gain * heading_err))

        # Slow down if far off heading or close to target
        speed_scale = max(0.2, 1.0 - min(abs(heading_err) / math.pi, 0.8))
        linear_x = self.speed_mps * speed_scale * lin_gain
        if dist < 1.0:
            linear_x = min(linear_x, max(0.1, dist))

        # Reached waypoint?
        if dist < 0.4:
            self.publish_cmd_vel(0.0, 0.0)
            self._handle_waypoint_reached(self.current_wp_idx)
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoint_buffer):
                self.get_logger().info("[Mission] Final waypoint reached. Stopping.")
                self.mission_running = False
                self.publish_cmd_vel(0.0, 0.0)
            return

        self.publish_cmd_vel(linear_x, angular_z)

    def _handle_waypoint_reached(self, idx: int):
        if not self.tasks_enabled:
            return

        program = self.mission_tasks.get(idx)
        if not program:
            return
        self.task_running = True
        self.get_logger().info(f"[Mission] Executing task at waypoint {idx+1}: {program}")

        def _run():
            try:
                script_path = self._resolve_mission_program_path(program)
                if not os.path.isfile(script_path):
                    self.get_logger().warn(f"[Mission] Script not found: {script_path}")
                    return
                subprocess.run(["python3", script_path], check=True)
                self.get_logger().info(f"[Mission] Task {program} completed.")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"[Mission] Task failed ({program}): {e}")
            except Exception as e:
                self.get_logger().error(f"[Mission] Task error ({program}): {e}")
            finally:
                # small pause to avoid jerky restart
                time.sleep(0.2)
                self.task_running = False

        threading.Thread(target=_run, daemon=True).start()

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _detect_jetson(self) -> bool:
        try:
            if os.path.exists("/etc/nv_tegra_release"):
                return True
            release = platform.release().lower()
            if "tegra" in release or "jetson" in release:
                return True
            model_path = "/proc/device-tree/model"
            if os.path.exists(model_path):
                with open(model_path, "r", encoding="utf-8", errors="ignore") as f:
                    model = f.read().lower()
                if "nvidia" in model and "jetson" in model:
                    return True
        except Exception:
            pass
        return False

    def _default_camera_backend(self) -> str:
        env = (os.getenv("OUTNAV_CAMERA_BACKEND") or "").strip().lower()
        if env in ("gstreamer", "gst"):
            return "gstreamer"
        if env in ("v4l2", "opencv"):
            return "v4l2"
        return "gstreamer" if self._is_jetson else "v4l2"

    def _default_camera_worker(self) -> bool:
        env = (os.getenv("OUTNAV_CAMERA_WORKER") or "").strip().lower()
        if env in ("1", "true", "yes", "on"):
            return True
        if env in ("0", "false", "no", "off"):
            return False
        return bool(self._is_jetson)

    def _ensure_camera_liveness_subs(self):
        if self._camera_frame_sub is None:
            try:
                self._camera_frame_sub = self.create_subscription(
                    CompressedImage,
                    "/camera/image_raw/compressed",
                    self._on_camera_frame,
                    1,
                )
            except Exception:
                self._camera_frame_sub = None
        if self._camera2_frame_sub is None:
            try:
                self._camera2_frame_sub = self.create_subscription(
                    CompressedImage,
                    "/camera2/image_raw/compressed",
                    self._on_camera2_frame,
                    1,
                )
            except Exception:
                self._camera2_frame_sub = None

    def _on_camera_frame(self, _msg: CompressedImage):
        self._camera_last_frame_wall = time.time()

    def _on_camera2_frame(self, _msg: CompressedImage):
        self._camera2_last_frame_wall = time.time()

    def _auto_select_camera_format(self, device: str):
        if not device or not str(device).startswith("/dev/video"):
            return
        env = (os.getenv("OUTNAV_CAMERA_AUTOFMT") or "").strip().lower()
        if env in ("0", "false", "no", "off"):
            return
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "--list-formats-ext", "-d", device],
                stderr=subprocess.STDOUT,
                text=True,
                timeout=3,
            )
        except Exception as exc:
            self.get_logger().warn(f"[Camera] Format probe failed: {exc}")
            return

        formats = []
        current_fmt = None
        for line in out.splitlines():
            fmt_match = re.search(r"\s*\[\d+\]:\s*'([A-Z0-9]+)'", line)
            if fmt_match:
                current_fmt = fmt_match.group(1)
                continue
            size_match = re.search(r"Size:\s*Discrete\s*(\d+)x(\d+)", line)
            if size_match and current_fmt:
                w = int(size_match.group(1))
                h = int(size_match.group(2))
                formats.append({"fmt": current_fmt, "width": w, "height": h})

        if not formats:
            return

        desired_w = int(self.camera_width)
        desired_h = int(self.camera_height)

        def _match(fmt: str, w: int, h: int):
            return any(f["fmt"] == fmt and f["width"] == w and f["height"] == h for f in formats)

        if _match("MJPG", desired_w, desired_h):
            self.camera_use_mjpeg = True
            return
        if _match("YUYV", desired_w, desired_h):
            self.camera_use_mjpeg = False
            return

        preferred = None
        for candidate in ((640, 480, "MJPG"), (1280, 720, "MJPG"), (640, 480, "YUYV")):
            for f in formats:
                if f["width"] == candidate[0] and f["height"] == candidate[1] and f["fmt"] == candidate[2]:
                    preferred = f
                    break
            if preferred:
                break
        if not preferred:
            preferred = formats[0]

        if preferred:
            self.camera_width = int(preferred["width"])
            self.camera_height = int(preferred["height"])
            self.camera_use_mjpeg = preferred["fmt"] == "MJPG"
            self._camera_frame_delay = 1.0 / max(self.camera_fps, 1)
            self.get_logger().info(
                f"[Camera] Auto format: {self.camera_width}x{self.camera_height} {preferred['fmt']}"
            )

    def _apply_capture_timeouts(self, cap):
        if cap is None:
            return
        for prop_name, value in (
            ("CAP_PROP_OPEN_TIMEOUT_MSEC", self.camera_open_timeout_ms),
            ("CAP_PROP_READ_TIMEOUT_MSEC", self.camera_read_timeout_ms),
        ):
            prop = getattr(cv2, prop_name, None)
            if prop is None:
                continue
            try:
                cap.set(prop, float(value))
            except Exception:
                pass

    def _start_camera_worker(self, which: str, device: str):
        if not device:
            return
        if not os.path.isfile(self._camera_worker_path):
            self.get_logger().error("[Camera] Worker script not found.")
            return
        if which == "secondary":
            topic = "/camera2/image_raw/compressed"
            node_name = "outnav_camera_worker2"
        else:
            topic = "/camera/image_raw/compressed"
            node_name = "outnav_camera_worker"

        self._auto_select_camera_format(device)
        args = [
            sys.executable,
            self._camera_worker_path,
            "--device",
            device,
            "--topic",
            topic,
            "--node-name",
            node_name,
            "--width",
            str(int(self.camera_width)),
            "--height",
            str(int(self.camera_height)),
            "--fps",
            str(int(self.camera_fps)),
            "--jpeg-quality",
            str(int(self.camera_jpeg_quality)),
            "--warmup-frames",
            str(int(self.camera_warmup_frames)),
            "--backend",
            self.camera_backend,
            "--read-timeout-ms",
            str(int(self.camera_read_timeout_ms)),
            "--open-timeout-ms",
            str(int(self.camera_open_timeout_ms)),
        ]
        if self.camera_use_mjpeg:
            args.append("--mjpeg")
        env = os.environ.copy()

        with self._camera_proc_lock:
            proc = subprocess.Popen(args, env=env)
            if which == "secondary":
                self._camera2_proc = proc
            else:
                self._camera_proc = proc

        self._ensure_camera_liveness_subs()
        self._ensure_camera_watchdog()

    def _stop_camera_worker(self, which: str):
        with self._camera_proc_lock:
            proc = self._camera2_proc if which == "secondary" else self._camera_proc
            if proc is None:
                return
            if proc.poll() is None:
                try:
                    proc.terminate()
                except Exception:
                    pass
                try:
                    proc.wait(timeout=2.0)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            if which == "secondary":
                self._camera2_proc = None
            else:
                self._camera_proc = None

    def _ensure_camera_watchdog(self):
        if self._camera_watchdog_thread and self._camera_watchdog_thread.is_alive():
            return
        self._camera_watchdog_stop.clear()
        t = threading.Thread(target=self._camera_watchdog_loop, daemon=True)
        self._camera_watchdog_thread = t
        t.start()

    def _camera_watchdog_loop(self):
        while rclpy.ok() and not self._camera_watchdog_stop.is_set():
            time.sleep(0.5)
            now = time.time()
            if self.camera_running:
                last = self._camera_last_frame_wall
                if last > 0.0 and (now - last) > self.camera_watchdog_sec:
                    if (now - self._camera_last_restart_wall) > self._camera_restart_cooldown_sec:
                        self._camera_last_restart_wall = now
                        self.get_logger().warn("[Camera] Watchdog timeout. Restarting camera.")
                        self._restart_camera("primary")
            if self.camera2_running:
                last2 = self._camera2_last_frame_wall
                if last2 > 0.0 and (now - last2) > self.camera_watchdog_sec:
                    if (now - self._camera2_last_restart_wall) > self._camera_restart_cooldown_sec:
                        self._camera2_last_restart_wall = now
                        self.get_logger().warn("[Camera2] Watchdog timeout. Restarting camera.")
                        self._restart_camera("secondary")

    def _restart_camera(self, which: str):
        if which == "secondary":
            device = self._camera2_device
            if not device:
                return
            self.stop_camera2(join=False)
            time.sleep(0.2)
            self.set_camera_source2(device)
            return
        device = self._camera_device
        if not device:
            return
        self.stop_camera(join=False)
        time.sleep(0.2)
        self.set_camera_source(device)
    # === Camera handling ===
    def set_camera_source(self, device):
        """Open camera device and start ROS image publishing."""
        with self._camera_lock:
            self.stop_camera(join=True)
        try:
            self.camera_running = False
            self._camera_device = device
            if self.camera_use_worker:
                self._camera_last_frame_wall = time.time()
                self.camera_running = True
                self._start_camera_worker("primary", device)
                self.get_logger().info(f"[Camera] Worker started for {device}")
                return
            self._auto_select_camera_format(device)
            self.cap = self._open_camera(device)
            if self.cap.isOpened():
                try:
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                self.get_logger().info(f"Camera opened on {device}")
                self._camera_last_frame_wall = time.time()
                self.camera_running = True
                self.start_camera_publisher()
            else:
                self.get_logger().error(f"Failed to open camera {device}")
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def set_camera_source2(self, device):
        """Open second camera device and start ROS image publishing on /camera2."""
        with self._camera2_lock:
            self.stop_camera2(join=True)
        try:
            self.camera2_running = False
            self._camera2_device = device
            if self.camera_use_worker:
                self._camera2_last_frame_wall = time.time()
                self.camera2_running = True
                self._start_camera_worker("secondary", device)
                self.get_logger().info(f"[Camera2] Worker started for {device}")
                return
            self._auto_select_camera_format(device)
            self.cap2 = self._open_camera(device)
            if self.cap2.isOpened():
                try:
                    self.cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                self.get_logger().info(f"Second camera opened on {device}")
                self._camera2_last_frame_wall = time.time()
                self.camera2_running = True
                self.start_camera_publisher2()
            else:
                self.get_logger().error(f"Failed to open second camera {device}")
        except Exception as e:
            self.get_logger().error(f"Second camera error: {e}")

    def start_camera_publisher(self):
        """Continuously read from cv2.VideoCapture and publish as ROS2 Image."""
        if self.camera_use_worker:
            return
        from sensor_msgs.msg import CompressedImage
        import threading
        import time

        self.camera_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self._ensure_camera_watchdog()

        def _capture_loop():
            if not hasattr(self, 'cap') or not self.cap.isOpened():
                self.get_logger().error("Camera not opened. Call set_camera_source() first.")
                return
            self._camera_stop_event.clear()
            self._camera_last_frame_wall = time.time()
            self.get_logger().info(" Camera streaming started")
            failures = 0
            last_frame_wall = self._camera_last_frame_wall
            warmup_left = self.camera_warmup_frames
            while rclpy.ok() and self.camera_running and not self._camera_stop_event.is_set():
                ret, frame = self.cap.read()
                if not ret:
                    failures += 1
                    time.sleep(0.05)
                    if time.time() - last_frame_wall > self.camera_watchdog_sec:
                        self.get_logger().warn("[Camera] No frames received. Stopping camera.")
                        break
                    if failures >= 30 and self._camera_device is not None:
                        try:
                            self.cap.release()
                        except Exception:
                            pass
                        self.cap = self._open_camera(self._camera_device)
                        try:
                            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        except Exception:
                            pass
                        failures = 0
                    continue
                failures = 0
                now_wall = time.time()
                last_frame_wall = now_wall
                self._camera_last_frame_wall = now_wall
                if warmup_left > 0:
                    warmup_left -= 1
                    time.sleep(self._camera_frame_delay)
                    continue
                try:
                    ok, enc = cv2.imencode(
                        ".jpg",
                        frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(self.camera_jpeg_quality)],
                    )
                except Exception:
                    ok, enc = False, None
                if not ok or enc is None:
                    time.sleep(self._camera_frame_delay)
                    continue
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = enc.tobytes()
                try:
                    self.camera_pub.publish(msg)
                except Exception as exc:
                    self.get_logger().warn(f"Camera publish stopped: {exc}")
                    break
                time.sleep(self._camera_frame_delay)
            
            self.get_logger().info(" Camera streaming stopped")
            if hasattr(self, 'cap') and self.cap:
                self.cap.release()
                self.cap = None

        t = threading.Thread(target=_capture_loop, daemon=True)
        self._camera_thread = t
        t.start()

    def start_camera_publisher2(self):
        """Continuously read from second camera and publish to /camera2/image_raw."""
        if self.camera_use_worker:
            return
        from sensor_msgs.msg import CompressedImage
        import threading
        import time

        self.camera2_pub = self.create_publisher(CompressedImage, '/camera2/image_raw/compressed', 10)
        self._ensure_camera_watchdog()

        def _capture_loop():
            if not hasattr(self, 'cap2') or not self.cap2.isOpened():
                self.get_logger().error("Second camera not opened. Call set_camera_source2() first.")
                return

            self._camera2_stop_event.clear()
            self._camera2_last_frame_wall = time.time()
            self.get_logger().info(" Second camera streaming started")
            failures = 0
            last_frame_wall = self._camera2_last_frame_wall
            warmup_left = self.camera_warmup_frames
            while rclpy.ok() and self.camera2_running and not self._camera2_stop_event.is_set():
                ret, frame = self.cap2.read()
                if not ret:
                    failures += 1
                    time.sleep(0.05)
                    if time.time() - last_frame_wall > self.camera_watchdog_sec:
                        self.get_logger().warn("[Camera2] No frames received. Stopping camera.")
                        break
                    if failures >= 30 and self._camera2_device is not None:
                        try:
                            self.cap2.release()
                        except Exception:
                            pass
                        self.cap2 = self._open_camera(self._camera2_device)
                        try:
                            self.cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        except Exception:
                            pass
                        failures = 0
                    continue
                failures = 0
                now_wall = time.time()
                last_frame_wall = now_wall
                self._camera2_last_frame_wall = now_wall
                if warmup_left > 0:
                    warmup_left -= 1
                    time.sleep(self._camera_frame_delay)
                    continue
                try:
                    ok, enc = cv2.imencode(
                        ".jpg",
                        frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(self.camera_jpeg_quality)],
                    )
                except Exception:
                    ok, enc = False, None
                if not ok or enc is None:
                    time.sleep(self._camera_frame_delay)
                    continue
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = enc.tobytes()
                try:
                    self.camera2_pub.publish(msg)
                except Exception as exc:
                    self.get_logger().warn(f"Second camera publish stopped: {exc}")
                    break
                time.sleep(self._camera_frame_delay)

            self.get_logger().info(" Second camera streaming stopped")
            if hasattr(self, 'cap2') and self.cap2:
                self.cap2.release()
                self.cap2 = None

        t = threading.Thread(target=_capture_loop, daemon=True)
        self._camera2_thread = t
        t.start()

    def _join_thread(self, thread):
        if not thread or not thread.is_alive():
            return
        if threading.current_thread() is thread:
            return
        thread.join(timeout=1.5)

    def stop_camera(self, join=False):
        """Signal camera capture loop to stop and release device."""
        self._camera_stop_event.set()
        self.camera_running = False
        self._camera_last_frame_wall = 0.0
        if self.camera_use_worker:
            self._stop_camera_worker("primary")
            return
        if join:
            self._join_thread(self._camera_thread)
        if hasattr(self, 'cap') and self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def stop_camera2(self, join=False):
        """Signal second camera capture loop to stop and release device."""
        self._camera2_stop_event.set()
        self.camera2_running = False
        self._camera2_last_frame_wall = 0.0
        if self.camera_use_worker:
            self._stop_camera_worker("secondary")
            return
        if join:
            self._join_thread(self._camera2_thread)
        if hasattr(self, 'cap2') and self.cap2:
            try:
                self.cap2.release()
            except Exception:
                pass
            self.cap2 = None

    def _build_gstreamer_pipeline(self, device: str, use_mjpeg: bool = None) -> str:
        width = int(self.camera_width)
        height = int(self.camera_height)
        fps = int(self.camera_fps)
        if use_mjpeg is None:
            use_mjpeg = self.camera_use_mjpeg
        if use_mjpeg:
            return (
                f"v4l2src device={device} io-mode=2 do-timestamp=true ! "
                f"image/jpeg, width={width}, height={height}, framerate={fps}/1 ! "
                "jpegdec ! videoconvert ! video/x-raw, format=BGR ! "
                "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                "appsink drop=1 max-buffers=1 sync=false"
            )
        return (
            f"v4l2src device={device} io-mode=2 do-timestamp=true ! "
            f"video/x-raw, width={width}, height={height}, framerate={fps}/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            "appsink drop=1 max-buffers=1 sync=false"
        )

    def _open_camera(self, device):
        cap = None
        backend = getattr(self, "camera_backend", "v4l2")
        if backend == "gstreamer" and isinstance(device, str) and device.startswith("/dev/video"):
            pipeline = self._build_gstreamer_pipeline(device, use_mjpeg=self.camera_use_mjpeg)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap is not None and cap.isOpened():
                self._apply_capture_timeouts(cap)
                self.get_logger().info(f"[Camera] Opened via GStreamer: {device}")
                return cap
            if self.camera_use_mjpeg:
                if cap:
                    try:
                        cap.release()
                    except Exception:
                        pass
                pipeline = self._build_gstreamer_pipeline(device, use_mjpeg=False)
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if cap is not None and cap.isOpened():
                    self.camera_use_mjpeg = False
                    self._apply_capture_timeouts(cap)
                    self.get_logger().warn("[Camera] GStreamer MJPEG failed; using raw YUYV.")
                    self.get_logger().info(f"[Camera] Opened via GStreamer: {device}")
                    return cap
            if cap:
                try:
                    cap.release()
                except Exception:
                    pass
            self.camera_use_gst = False
            self.camera_backend = "v4l2"
            self.get_logger().warn("[Camera] GStreamer failed; falling back to V4L2/OpenCV.")
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(device)
        if cap is not None and cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if self.camera_use_mjpeg:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.camera_width))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.camera_height))
                cap.set(cv2.CAP_PROP_FPS, float(self.camera_fps))
                self._apply_capture_timeouts(cap)
            except Exception:
                pass
            self.get_logger().info(f"[Camera] Opened via V4L2/OpenCV: {device}")
        return cap

    def set_camera_profile(self, width: int, height: int, fps: int, use_mjpeg: bool = True):
        try:
            self.camera_width = int(width)
            self.camera_height = int(height)
            self.camera_fps = int(fps)
            self._camera_frame_delay = 1.0 / max(self.camera_fps, 1)
            self.camera_use_mjpeg = bool(use_mjpeg)
            self.get_logger().info(
                f"[Camera] Profile set: {self.camera_width}x{self.camera_height} @ {self.camera_fps} FPS, MJPEG={self.camera_use_mjpeg}"
            )
        except Exception as e:
            self.get_logger().warn(f"[Camera] Failed to set profile: {e}")

    def set_camera_backend(self, backend: str):
        clean = (backend or "").strip().lower()
        if clean in ("v4l2", "opencv"):
            self.camera_backend = "v4l2"
            self.camera_use_gst = False
            self._camera_backend_user_override = True
        elif clean in ("gst", "gstreamer"):
            self.camera_backend = "gstreamer"
            self.camera_use_gst = True
            self._camera_backend_user_override = True
        else:
            return
        self.get_logger().info(f"[Camera] Backend set: {self.camera_backend}")

    def _sanitize_filename(self, name: str) -> str:
        safe = "".join(c for c in (name or "") if c.isalnum() or c in ("-", "_"))
        return safe or "session"

    def _recording_path_for(self, mission: str) -> str:
        safe = self._sanitize_filename(mission or "session")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        folder = os.path.join(self.base_dir, "videos")
        os.makedirs(folder, exist_ok=True)
        return os.path.join(folder, f"{safe}_{stamp}.mp4")

    def _start_robot_recording(self, device: str, mission: str):
        with self._recording_lock:
            if self._recording_thread and self._recording_thread.is_alive():
                self.get_logger().warn("[Camera] Recording already active; restarting.")
        self._stop_robot_recording(join=True)
        # Ensure streaming is stopped before taking the device
        self.stop_camera(join=False)
        self._recording_stop.clear()
        self._recording_device = device or "/dev/video0"
        self._recording_path = self._recording_path_for(mission)
        self._recording_fps = int(self.camera_fps or 10)
        t = threading.Thread(target=self._recording_loop, daemon=True)
        self._recording_thread = t
        t.start()
        self.get_logger().info(f"[Camera] Recording started → {self._recording_path}")

    def _stop_robot_recording(self, join: bool = True):
        self._recording_stop.set()
        t = self._recording_thread
        if join and t and t.is_alive():
            t.join(timeout=3.0)
        self._recording_thread = None
        self._recording_path = None
        self._recording_device = None
        self._recording_fps = None

    def _recording_loop(self):
        path = self._recording_path
        device = self._recording_device or "/dev/video0"
        cap = self._open_camera(device)
        if cap is None or not cap.isOpened():
            self.get_logger().error(f"[Camera] Recording failed to open device {device}")
            return
        fps = float(self._recording_fps or self.camera_fps or 10)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or self.camera_width or 640)
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or self.camera_height or 480)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
        if writer is None or not writer.isOpened():
            self.get_logger().error(f"[Camera] Recording failed to open writer for {path}")
            try:
                cap.release()
            except Exception:
                pass
            return
        failures = 0
        try:
            while rclpy.ok() and not self._recording_stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    failures += 1
                    if failures >= 30:
                        self.get_logger().warn("[Camera] Recording stalled; stopping.")
                        break
                    time.sleep(0.05)
                    continue
                failures = 0
                writer.write(frame)
        finally:
            try:
                writer.release()
            except Exception:
                pass
            try:
                cap.release()
            except Exception:
                pass
            if path:
                self.get_logger().info(f"[Camera] Recording saved → {path}")

    def publish_camera_control(self, payload: dict):
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self.camera_control_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[Camera] publish_camera_control failed: {e}")

    def camera_control_callback(self, msg: String):
        if self.device_mode != "robot":
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        action = (payload.get("action") or "").strip().lower()
        backend = payload.get("backend")
        if backend:
            self.set_camera_backend(backend)
        width = payload.get("width")
        height = payload.get("height")
        fps = payload.get("fps")
        mjpeg = payload.get("mjpeg")
        if width or height or fps or mjpeg is not None:
            try:
                self.set_camera_profile(
                    width if width is not None else self.camera_width,
                    height if height is not None else self.camera_height,
                    fps if fps is not None else self.camera_fps,
                    self.camera_use_mjpeg if mjpeg is None else bool(mjpeg),
                )
            except Exception:
                pass
        if action == "config":
            device = payload.get("device")
            if device:
                self._camera_device = device
            return
        if action == "record_start":
            device = payload.get("device") or self._camera_device or "/dev/video0"
            mission = payload.get("mission") or "session"
            self._start_robot_recording(device, mission)
            return
        if action == "record_stop":
            self._stop_robot_recording(join=False)
            self.stop_camera(join=False)
            return
        if action == "start":
            device = payload.get("device") or self._camera_device or "/dev/video0"
            self.set_camera_source(device)
        elif action == "stop":
            self.stop_camera(join=False)
        elif action == "start2":
            device = payload.get("device") or self._camera2_device or "/dev/video0"
            self.set_camera_source2(device)
        elif action == "stop2":
            self.stop_camera2(join=False)


def start_ros_node():
    rclpy.init()
    node = OutdoorNavNode()
    return node


if __name__ == '__main__':
    rclpy.init()
    node = OutdoorNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
