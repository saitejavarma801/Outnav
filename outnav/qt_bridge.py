#!/usr/bin/env python3
import os
import sys
import json
import base64
import time
import shutil
import urllib.request
import subprocess
import threading
import logging
import re
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# --- Prevent OpenCV Qt plugin conflicts globally ---
os.environ.pop("QT_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QFileDialog, QDialog, QListWidget, QListWidgetItem, QPushButton, QLabel, QVBoxLayout, QHBoxLayout
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float64MultiArray


class QtBridge(QtCore.QObject):
    """
    Bridge between JavaScript (map_template.html) and Python (ROS + PyQt).
    Handles waypoints, missions, status, camera, and ROS2 task publishing.
    """

    sendToJS = QtCore.pyqtSignal(str, arguments=['message'])

    # camera signals
    cameraFrameReady = QtCore.pyqtSignal(str)   # base64 JPEG (no prefix)
    cameraFileReady  = QtCore.pyqtSignal(str)   # absolute file path
    camera2FrameReady = QtCore.pyqtSignal(str)  # base64 JPEG (second)
    camera2FileReady  = QtCore.pyqtSignal(str)  # absolute file path (second)

    def __init__(self, node, window=None):
        super().__init__()
        self.node = node
        self.window = window
        self._logger = logging.getLogger("qt_bridge")
        self.base_dir = os.path.dirname(__file__)
        self.missions_dir = os.path.join(self.base_dir, "missions")
        self.mission_programs_dir = os.path.join(self.missions_dir, "programs")
        self.current_mission_name = "session"
        self._frame_skip = 0
        self._tmp_jpg = "/tmp/outnav_cam.jpg"
        self._tmp_jpg2 = "/tmp/outnav_cam2.jpg"
        self._camera_fps = 10
        self._camera_backend = getattr(node, "camera_backend", None) or "v4l2"
        self._last_frame_ts1 = 0.0
        self._last_frame_ts2 = 0.0
        self._last_cam_wall = 0.0
        self._max_mission_file_bytes = 512 * 1024

        # Recording state
        self.recording_process = None
        self.recording_file = None
        self._recording1 = False
        self._recording2 = False
        self._recorder1 = None
        self._recorder2 = None
        self._recording_path1 = None
        self._recording_path2 = None
        self._recording_fps1 = None
        self._recording_fps2 = None
        self._last_jpg1 = None
        self._last_jpg2 = None
        # Remote camera configuration (robot-side device)
        self._remote_camera_device = "/dev/video0"
        self._remote_camera2_device = "/dev/video2"
        self._camera_width = getattr(node, "camera_width", None) or 640
        self._camera_height = getattr(node, "camera_height", None) or 480
        self._remote_stream_enabled = False

        # connect camera signals → JS
        self.cameraFrameReady.connect(self._send_b64_to_js)
        self.cameraFileReady.connect(self._send_path_to_js)
        self.camera2FrameReady.connect(self._send_b64_to_js2)
        self.camera2FileReady.connect(self._send_path_to_js2)

        # misc holders
        self.waypoints = []
        self.saved_places = {}
        self.recording = False
        self.gps_port = "/dev/ttyTHS1"
        self._ui_ready = False
        self._pending_mission_data = None
        self._pending_route_points = None

        # connection status timer
        self._conn_timer = QtCore.QTimer()
        self._conn_timer.setInterval(1000)
        self._conn_timer.timeout.connect(self._update_connection_status)
        self._conn_timer.start()

        # mission file status subscription
        try:
            self._mission_status_sub = self.node.create_subscription(
                String, '/mission_file_status', self._on_mission_file_status, 10
            )
        except Exception as e:
            print("[QtBridge] Mission status subscription failed:", e)

        # heartbeat subscription
        self._last_heartbeat_wall = 0.0
        self._last_remote_heartbeat_wall = 0.0
        try:
            self._heartbeat_sub = self.node.create_subscription(
                String, '/outnav/heartbeat', self._on_heartbeat, 10
            )
        except Exception as e:
            print("[QtBridge] Heartbeat subscription failed:", e)

        # settings + mission sync subscriptions
        self._sync_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        try:
            self._settings_sub = self.node.create_subscription(
                String, '/outnav/settings', self._on_settings, self._sync_qos
            )
        except Exception as e:
            print("[QtBridge] Settings subscription failed:", e)
        try:
            self._mission_data_sub = self.node.create_subscription(
                String, '/mission_data', self._on_mission_data, self._sync_qos
            )
        except Exception as e:
            print("[QtBridge] Mission data subscription failed:", e)
        try:
            self._status_sub = self.node.create_subscription(
                String, '/outnav/status', self._on_status, 10
            )
        except Exception as e:
            print("[QtBridge] Status subscription failed:", e)
        try:
            self._localization_status_sub = self.node.create_subscription(
                String, '/localization/status', self._on_localization_status, 10
            )
        except Exception as e:
            print("[QtBridge] Localization status subscription failed:", e)
        try:
            self._route_ack_sub = self.node.create_subscription(
                String, '/outnav/route_ack', self._on_route_ack, 10
            )
        except Exception as e:
            print("[QtBridge] Route ack subscription failed:", e)
        try:
            self._route_points_sub = self.node.create_subscription(
                Float64MultiArray, '/route_points', self._on_route_points, self._sync_qos
            )
        except Exception as e:
            print("[QtBridge] Route points subscription failed:", e)

    def _dispatch_mission_data(self, data: str):
        # Use the same injection path as local mission load, plus a debug ping.
        try:
            self._inject_js_load(data)
        except Exception:
            pass
        js_dbg = (
            "try{"
            "if(window.setPyDebug){setPyDebug('DEBUG: PY inject mission');}"
            "if(window.debugMissionToast){debugMissionToast('PY inject mission');}"
            "}catch(e){}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js_dbg))

    def _dispatch_route_points(self, coords):
        js = (
            "if(window.applyRoutePoints){"
            f"applyRoutePoints({json.dumps(coords)});"
            "}else if(window.queueRoutePoints){"
            f"queueRoutePoints({json.dumps(coords)});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))

    def _flush_pending_ui(self):
        if not self._ui_ready:
            return
        if self._pending_mission_data:
            data = self._pending_mission_data
            self._pending_mission_data = None
            self._dispatch_mission_data(data)
        if self._pending_route_points:
            coords = self._pending_route_points
            self._pending_route_points = None
            self._dispatch_route_points(coords)

    @QtCore.pyqtSlot()
    def uiReady(self):
        self._ui_ready = True
        print("[QtBridge] UI ready; flushing pending UI data")
        self._flush_pending_ui()

    # =====================================================================
    #                           CAMERA  BRIDGE
    # =====================================================================

    def _is_laptop_mode(self) -> bool:
        """Disable robot camera streaming to the laptop when in laptop mode."""
        return getattr(self.node, "device_mode", "laptop") == "laptop"

    def _streaming_enabled(self) -> bool:
        return self._is_laptop_mode() and bool(self._remote_stream_enabled)

    def _set_remote_stream_enabled(self, enabled: bool):
        prev = bool(self._remote_stream_enabled)
        self._remote_stream_enabled = bool(enabled)
        if not prev or self._remote_stream_enabled:
            return
        # Streaming toggled off: stop remote stream and drop subscriptions.
        try:
            if self.node and hasattr(self.node, "publish_camera_control"):
                for action in ("stop", "stop2"):
                    payload = {
                        "action": action,
                        "sender_id": getattr(self.node, "node_id", "outnav"),
                    }
                    self.node.publish_camera_control(payload)
        except Exception:
            pass
        if getattr(self, "_sub_img", None):
            try:
                self.node.destroy_subscription(self._sub_img)
            except Exception:
                pass
            self._sub_img = None
        if getattr(self, "_sub_img2", None):
            try:
                self.node.destroy_subscription(self._sub_img2)
            except Exception:
                pass
            self._sub_img2 = None
        if getattr(self, "_sub_img2", None):
            try:
                self.node.destroy_subscription(self._sub_img2)
            except Exception:
                pass
            self._sub_img2 = None

    @QtCore.pyqtSlot()
    def start_camera_sub(self):
        """Subscribe to /camera/image_raw and stream to the web UI."""
        if self._is_laptop_mode() and not self._remote_stream_enabled:
            print("[QtBridge] Laptop mode: camera streaming disabled.")
            return
        try:
            self._sub_img = self.node.create_subscription(
                CompressedImage, '/camera/image_raw/compressed', self._on_img, 10
            )
            print("[QtBridge] Camera subscription active on /camera/image_raw/compressed")
        except Exception as e:
            print("[QtBridge] Camera subscription failed:", e)

    def _on_img(self, msg: CompressedImage):
        """Convert ROS Image → JPEG and emit to JS (b64 + file fallback)."""
        if self._is_laptop_mode() and not self._remote_stream_enabled:
            return
        try:
            now = time.time()
            min_interval = 1.0 / float(self._camera_fps)
            if (now - self._last_frame_ts1) < min_interval:
                return
            self._last_frame_ts1 = now
            self._last_cam_wall = now

            if not msg.data:
                return
            # send as base64 (already JPEG)
            b64 = base64.b64encode(msg.data).decode('ascii')
            self._last_jpg1 = np.frombuffer(msg.data, dtype=np.uint8)
            self.cameraFrameReady.emit(b64)

            # also write to file and send file:// fallback
            try:
                with open(self._tmp_jpg, "wb") as f:
                    f.write(msg.data)
                self.cameraFileReady.emit(self._tmp_jpg)
            except Exception as e:
                pass

            # record if active
            if self._recording1:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    self._write_recording_frame(frame, second=False)

        except Exception as e:
            print("[QtBridge] Error converting image:", e)

    def _send_b64_to_js(self, b64_str: str):
        """Run on GUI thread → inject data: URI into JS."""
        if not self.window:
            return
        
        def inject():
            try:
                page = self.window.view.page()
                js = f"""
                if(window.updateCameraFrame) {{
                    try {{
                        window.updateCameraFrame('data:image/jpeg;base64,{b64_str}');
                    }} catch(e) {{
                        console.error('cam b64 err', e);
                    }}
                }}
                """
                page.runJavaScript(js)
            except Exception as e:
                print(f"[QtBridge] Failed to inject camera frame: {e}")
        
        QtCore.QTimer.singleShot(0, inject)

    def _send_path_to_js(self, path: str):
        """Run on GUI thread → inject file:// URL into JS (cache-busted)."""
        if not self.window:
            return
        
        def inject():
            try:
                page = self.window.view.page()
                url = f"file://{path}?t={int(time.time()*1000)}"
                js = f"""
                if(window.updateCameraFrameFile) {{
                    try {{
                        window.updateCameraFrameFile('{url}');
                    }} catch(e) {{
                        console.error('cam file err', e);
                    }}
                }}
                """
                page.runJavaScript(js)
            except Exception as e:
                print(f"[QtBridge] Failed to inject camera file path: {e}")
        
        QtCore.QTimer.singleShot(0, inject)

    # =====================================================================
    #                           CAMERA 2 BRIDGE
    # =====================================================================
    @QtCore.pyqtSlot()
    def start_camera2_sub(self):
        """Subscribe to /camera2/image_raw and stream to the second HUD."""
        if self._is_laptop_mode() and not self._remote_stream_enabled:
            print("[QtBridge] Laptop mode: camera streaming disabled for camera2.")
            return
        try:
            self._sub_img2 = self.node.create_subscription(
                CompressedImage, '/camera2/image_raw/compressed', self._on_img2, 10
            )
            print("[QtBridge] Camera 2 subscription active on /camera2/image_raw/compressed")
        except Exception as e:
            print("[QtBridge] Camera 2 subscription failed:", e)

    def _on_img2(self, msg: CompressedImage):
        """Convert ROS Image → JPEG for second camera."""
        if self._is_laptop_mode() and not self._remote_stream_enabled:
            return
        try:
            now = time.time()
            min_interval = 1.0 / float(self._camera_fps)
            if (now - self._last_frame_ts2) < min_interval:
                return
            self._last_frame_ts2 = now
            self._last_cam_wall = now

            if not msg.data:
                return
            self._last_jpg2 = np.frombuffer(msg.data, dtype=np.uint8)
            b64 = base64.b64encode(msg.data).decode('ascii')
            self.camera2FrameReady.emit(b64)

            try:
                with open(self._tmp_jpg2, "wb") as f:
                    f.write(msg.data)
                self.camera2FileReady.emit(self._tmp_jpg2)
            except Exception:
                pass
        except Exception as e:
            print("[QtBridge] Error converting camera2 image:", e)
            return

        # record if active
        if self._recording2:
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    self._write_recording_frame(frame, second=True)
            except Exception as e:
                print("[QtBridge] Camera2 recording error:", e)

    def _send_b64_to_js2(self, b64_str: str):
        """Run on GUI thread → inject data: URI into JS for second HUD."""
        if not self.window:
            return
        
        def inject():
            try:
                page = self.window.view.page()
                js = f"""
                if(window.updateSecondCameraFrame) {{
                    try {{
                        window.updateSecondCameraFrame('data:image/jpeg;base64,{b64_str}');
                    }} catch(e) {{
                        console.error('cam2 b64 err', e);
                    }}
                }}
                """
                page.runJavaScript(js)
            except Exception as e:
                print(f"[QtBridge] Failed to inject camera2 frame: {e}")
        
        QtCore.QTimer.singleShot(0, inject)

    def _send_path_to_js2(self, path: str):
        """Run on GUI thread → inject file:// URL into JS for second HUD."""
        if not self.window:
            return
        
        def inject():
            try:
                page = self.window.view.page()
                url = f"file://{path}?t={int(time.time()*1000)}"
                js = f"""
                if(window.updateSecondCameraFrameFile) {{
                    try {{
                        window.updateSecondCameraFrameFile('{url}');
                    }} catch(e) {{
                        console.error('cam2 file err', e);
                    }}
                }}
                """
                page.runJavaScript(js)
            except Exception as e:
                print(f"[QtBridge] Failed to inject camera2 file path: {e}")
        
        QtCore.QTimer.singleShot(0, inject)

    def _run_js(self, code: str):
        """Utility – execute JavaScript safely on GUI thread."""
        try:
            if hasattr(self.window, "view") and hasattr(self.window.view, "page"):
                self.window.view.page().runJavaScript(code)
            elif hasattr(self.window, "page"):
                self.window.page().runJavaScript(code)
        except Exception as e:
            print("[QtBridge] Failed to run JS:", e)

    def _on_mission_file_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        js = (
            "if(window.handleMissionFileStatus){"
            f"handleMissionFileStatus({json.dumps(payload)});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))

    def _on_settings(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if "cameraStreamEnabled" in payload:
            self._set_remote_stream_enabled(bool(payload.get("cameraStreamEnabled")))
        self._logger.debug("Settings received: keys=%s", list(payload.keys()))
        sender_id = payload.get("sender_id")
        if sender_id and self.node and hasattr(self.node, "node_id"):
            if sender_id == self.node.node_id:
                return
        try:
            if self.node:
                if "maxSpeed" in payload and hasattr(self.node, "set_speed"):
                    self.node.set_speed(float(payload.get("maxSpeed")))
                gps_port = payload.get("gpsPort")
                if gps_port and hasattr(self.node, "set_gps_port"):
                    self.node.set_gps_port(str(gps_port))
                if "gpsUartEnabled" in payload and hasattr(self.node, "set_gps_enabled"):
                    self.node.set_gps_enabled(payload.get("gpsUartEnabled"))
                if "gpsEnabled" in payload and hasattr(self.node, "set_gps_enabled"):
                    self.node.set_gps_enabled(payload.get("gpsEnabled"))
                if "localizationPoseEnabled" in payload and hasattr(self.node, "set_localization_pose_enabled"):
                    self.node.set_localization_pose_enabled(payload.get("localizationPoseEnabled"))
                if "localizationMotionEnabled" in payload and hasattr(self.node, "set_localization_motion_enabled"):
                    self.node.set_localization_motion_enabled(payload.get("localizationMotionEnabled"))
                if "localizationCmdVelEnabled" in payload and hasattr(self.node, "set_localization_motion_enabled"):
                    self.node.set_localization_motion_enabled(payload.get("localizationCmdVelEnabled"))
                backend = payload.get("cameraBackend")
                if backend and hasattr(self.node, "set_camera_backend"):
                    self.node.set_camera_backend(str(backend))
                if hasattr(self.node, "set_camera_profile"):
                    width = int(payload.get("cameraWidth", getattr(self.node, "camera_width", 640)))
                    height = int(payload.get("cameraHeight", getattr(self.node, "camera_height", 480)))
                    fps = int(payload.get("cameraFps", getattr(self.node, "camera_fps", 10)))
                    mjpeg = bool(payload.get("cameraMjpeg", getattr(self.node, "camera_use_mjpeg", True)))
                    self.node.set_camera_profile(width, height, fps, mjpeg)
        except Exception as exc:
            self._logger.debug("Node settings apply skipped: %s", exc)
        self._logger.info("Settings received from remote.")
        js = (
            "if(window.applyRemoteSettings){"
            f"applyRemoteSettings({json.dumps(payload)});"
            "}else if(window.queueRemoteSettings){"
            f"queueRemoteSettings({json.dumps(payload)});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))
        self._send_sync_audit("settings", "Received from robot")

    def _on_mission_data(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self._logger.debug("Mission data received: name=%s size=%s", payload.get("name"), payload.get("size", "n/a"))
        sender_id = payload.get("sender_id")
        if sender_id and self.node and hasattr(self.node, "node_id"):
            if sender_id == self.node.node_id:
                return
        data_b64 = payload.get("data_b64")
        if not data_b64:
            return
        try:
            data = base64.b64decode(data_b64.encode("ascii")).decode("utf-8")
        except Exception:
            return
        try:
            parsed = json.loads(data)
            wp_count = len(parsed.get("waypoints", []))
        except Exception:
            wp_count = "n/a"
        self._logger.info("Mission data received from remote: %s (%s waypoints)", payload.get("name"), wp_count)
        self._logger.debug("Mission UI ready=%s; dispatching=%s", self._ui_ready, bool(self._ui_ready))
        if not self._ui_ready:
            self._pending_mission_data = data
        else:
            self._dispatch_mission_data(data)
        self._send_sync_audit("mission", f"Received {payload.get('name')}")

    def _on_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        js = (
            "if(window.updateRobotStatus){"
            f"updateRobotStatus({json.dumps(payload)});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))

    def _on_localization_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        js = (
            "if(window.updateLocalizationStatus){"
            f"updateLocalizationStatus({json.dumps(payload)});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))

    def _on_route_ack(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        js = (
            "if(window.updateRouteAck){"
            f"updateRouteAck({json.dumps(payload)});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))
        count = payload.get("count")
        self._send_sync_audit("route", f"ACK {count} wp")

    def _on_route_points(self, msg: Float64MultiArray):
        if not self.node:
            return
        data = list(msg.data or [])
        if len(data) < 2 or len(data) % 2 != 0:
            return
        coords = []
        for i in range(0, len(data), 2):
            try:
                coords.append({"lat": float(data[i]), "lng": float(data[i + 1])})
            except Exception:
                continue
        if not coords:
            return
        if not self._ui_ready:
            self._pending_route_points = coords
        else:
            self._dispatch_route_points(coords)

    def _on_heartbeat(self, msg: String):
        now = time.time()
        self._last_heartbeat_wall = now
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        sender = payload.get("node")
        sender_mode = payload.get("mode")
        local_id = getattr(self.node, "node_id", None)
        local_mode = getattr(self.node, "device_mode", None)
        if sender and local_id and sender != local_id:
            if sender_mode and local_mode and sender_mode == local_mode:
                return
            self._last_remote_heartbeat_wall = now

    def _update_connection_status(self):
        if not self.node:
            return
        try:
            now = time.time()
            last_remote = getattr(self, "_last_remote_heartbeat_wall", 0.0)
            if last_remote > 0.0:
                age = now - last_remote
                connected = age < 2.5
            else:
                connected = False
                age = -1.0

            js = (
                "if(window.updateConnectionStatus){"
                f"updateConnectionStatus({str(connected).lower()}, {age:.2f});"
                "}"
            )
            QtCore.QTimer.singleShot(0, lambda: self._run_js(js))
        except Exception as e:
            print("[QtBridge] Connection status update failed:", e)

    @QtCore.pyqtSlot(int)
    def setCameraFps(self, fps: int):
        try:
            val = int(fps)
        except Exception:
            return
        val = max(1, min(60, val))
        self._camera_fps = val
        print(f"[QtBridge] Camera FPS set to {val}")

    @QtCore.pyqtSlot(str)
    def setCameraBackend(self, backend: str):
        clean = (backend or "").strip().lower()
        if clean in ("v4l2", "opencv"):
            self._camera_backend = "v4l2"
        elif clean in ("gst", "gstreamer"):
            self._camera_backend = "gstreamer"
        else:
            return
        try:
            if self.node and hasattr(self.node, "set_camera_backend"):
                self.node.set_camera_backend(self._camera_backend)
        except Exception:
            pass

    @QtCore.pyqtSlot(str, result=bool)
    def syncMissionScripts(self, json_str: str):
        try:
            payload = json.loads(json_str)
            files = payload.get("files", [])
        except Exception:
            return False
        if not files:
            return True
        for name in files:
            if not name:
                continue
            ok = self._send_mission_file(name, overwrite=False, new_name=None)
            if not ok:
                return False
        return True

    @QtCore.pyqtSlot(str, bool, str, result=bool)
    def resendMissionScript(self, name: str, overwrite: bool, new_name: str):
        return self._send_mission_file(name, overwrite=overwrite, new_name=new_name or None)

    def _sanitize_mission_program_name(self, name: str) -> str:
        raw = os.path.basename((name or "").strip())
        if not raw:
            return ""
        safe = re.sub(r"[^A-Za-z0-9._-]", "_", raw)
        if not safe:
            return ""
        if not safe.endswith(".py"):
            safe = f"{safe}.py"
        return safe

    def _list_mission_program_files(self):
        names = []
        seen = set()
        for folder in (self.mission_programs_dir, self.missions_dir):
            if not os.path.isdir(folder):
                continue
            try:
                entries = sorted(os.listdir(folder))
            except Exception:
                continue
            for entry in entries:
                if not entry.endswith(".py") or entry == "__init__.py":
                    continue
                if entry in seen:
                    continue
                full_path = os.path.join(folder, entry)
                if not os.path.isfile(full_path):
                    continue
                seen.add(entry)
                names.append(entry)
        return names

    def _resolve_local_mission_program_path(self, name: str):
        safe_name = self._sanitize_mission_program_name(name)
        if not safe_name:
            return "", ""
        for folder in (self.mission_programs_dir, self.missions_dir):
            full_path = os.path.join(folder, safe_name)
            if os.path.isfile(full_path):
                return safe_name, full_path
        return safe_name, os.path.join(self.mission_programs_dir, safe_name)

    def _next_available_mission_program_name(self, preferred_name: str) -> str:
        safe_name = self._sanitize_mission_program_name(preferred_name)
        if not safe_name:
            return ""
        stem, ext = os.path.splitext(safe_name)
        candidate = safe_name
        counter = 1
        while os.path.exists(os.path.join(self.mission_programs_dir, candidate)):
            candidate = f"{stem}_{counter}{ext}"
            counter += 1
        return candidate

    def _import_mission_program_file(self, source_path: str) -> str:
        if not source_path or not os.path.isfile(source_path):
            raise FileNotFoundError(source_path)
        os.makedirs(self.mission_programs_dir, exist_ok=True)
        preferred_name = self._sanitize_mission_program_name(os.path.basename(source_path))
        if not preferred_name:
            raise ValueError("invalid program name")
        dest_path = os.path.join(self.mission_programs_dir, preferred_name)
        source_abs = os.path.abspath(source_path)
        dest_abs = os.path.abspath(dest_path)
        if source_abs == dest_abs:
            return preferred_name
        final_name = preferred_name
        if os.path.exists(dest_path):
            final_name = self._next_available_mission_program_name(preferred_name)
            dest_path = os.path.join(self.mission_programs_dir, final_name)
        shutil.copy2(source_path, dest_path)
        return final_name

    @QtCore.pyqtSlot(result='QVariant')
    def importMissionPrograms(self):
        imported = []
        try:
            files, _ = QFileDialog.getOpenFileNames(
                self.window,
                "Add Mission Programs",
                os.path.expanduser("~"),
                "Python Files (*.py)",
            )
            for path in files or []:
                try:
                    imported.append(self._import_mission_program_file(path))
                except Exception as exc:
                    print(f"[QtBridge] Failed to import mission program {path}: {exc}")
            return imported
        except Exception as e:
            print("[QtBridge] importMissionPrograms failed:", e)
            return imported

    @QtCore.pyqtSlot(str)
    def setDeviceModeRuntime(self, mode: str):
        try:
            if self.node and hasattr(self.node, "set_device_mode"):
                self.node.set_device_mode(mode)
        except Exception:
            pass

    def _send_mission_file(self, name: str, overwrite: bool, new_name: str):
        try:
            file_name = self._sanitize_mission_program_name(new_name or name)
            if not file_name:
                self._send_toast_to_js("Invalid program name")
                return False
            _, script_path = self._resolve_local_mission_program_path(name)
            if not os.path.isfile(script_path):
                self._send_toast_to_js(f"Script not found: {name}")
                return False
            size = os.path.getsize(script_path)
            if size > self._max_mission_file_bytes:
                max_kb = int(self._max_mission_file_bytes / 1024)
                self._send_toast_to_js(
                    f"Script too large ({int(size/1024)} KB). Max {max_kb} KB. Upload manually."
                )
                return False
            with open(script_path, "rb") as f:
                raw = f.read()
            data_b64 = base64.b64encode(raw).decode("ascii")
            payload = {
                "name": file_name,
                "size": len(raw),
                "data_b64": data_b64,
                "overwrite": bool(overwrite),
                "sender_id": getattr(self.node, "node_id", "outnav"),
            }
            if self.node and hasattr(self.node, "publish_mission_file"):
                self.node.publish_mission_file(payload)
                return True
        except Exception as e:
            print("[QtBridge] Mission file send failed:", e)
        return False


    @QtCore.pyqtSlot()
    def restartApp(self):
        """Restart the current Python process (used after ROS domain changes)."""
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            run_sh = os.path.join(base_dir, "run.sh")
            if os.path.isfile(run_sh):
                print(f"[QtBridge] Restarting app via {run_sh}")
                os.execv("/bin/bash", ["/bin/bash", run_sh])
                return
            args = [sys.executable] + sys.argv
            print(f"[QtBridge] Restarting app: {args}")
            os.execv(sys.executable, args)
        except Exception as exc:
            print(f"[QtBridge] Restart failed: {exc}")

    @QtCore.pyqtSlot()
    def requestCameraPort(self):
        """Enumerate /dev/video* and let user select a USB camera."""
        try:
            devs = [f"/dev/{d}" for d in sorted(os.listdir("/dev")) if d.startswith("video")]
        except Exception:
            devs = []
        if not devs:
            print("[QtBridge] No USB cameras detected.")
            return
        dlg = QDialog()
        dlg.setWindowTitle("Select USB Camera")
        dlg.setStyleSheet("""
            QDialog { background: #0f172a; color: #e5edff; }
            QListWidget { background:#0b1220; color:#e5edff; border:1px solid #334155; }
            QListWidget::item { padding:8px; }
            QListWidget::item:selected { background:#1f2937; color:#10b981; }
            QPushButton { background:#1b2335; color:#e5edff; border:1px solid #334155; border-radius:6px; padding:8px 12px; }
            QPushButton:hover { background:#243247; }
            QLabel { color:#9fb4ff; font-weight:600; }
        """)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Available video devices:"))
        listw = QListWidget()
        for d in devs:
            QListWidgetItem(d, listw)
        listw.setCurrentRow(0)
        layout.addWidget(listw)
        btns = QHBoxLayout()
        ok_btn = QPushButton("Start")
        cancel_btn = QPushButton("Cancel")
        btns.addWidget(ok_btn)
        btns.addWidget(cancel_btn)
        layout.addLayout(btns)
        dlg.setLayout(layout)

        chosen = {"path": None}
        def accept_choice():
            item = listw.currentItem()
            if item:
                chosen["path"] = item.text()
                dlg.accept()
        ok_btn.clicked.connect(accept_choice)
        cancel_btn.clicked.connect(dlg.reject)

        if dlg.exec_() == QDialog.Accepted and chosen["path"] and hasattr(self.node, "set_camera_source"):
            choice = chosen["path"]
            print(f"[QtBridge] Camera starting: {choice}")
            if getattr(self, "_sub_img", None):
                try:
                    self.node.destroy_subscription(self._sub_img)
                except Exception:
                    pass
                self._sub_img = None
            try:
                if self.node and hasattr(self.node, "set_camera_backend"):
                    self.node.set_camera_backend(self._camera_backend)
            except Exception:
                pass
            self.node.set_camera_source(choice)
            self.start_camera_sub()

    @QtCore.pyqtSlot(str, int, int, int, bool, str)
    def startRemoteCamera(self, device: str, width: int, height: int, fps: int, use_mjpeg: bool, backend: str):
        try:
            if not self.node or not hasattr(self.node, "publish_camera_control"):
                return
            clean_backend = (backend or self._camera_backend or "v4l2").strip().lower()
            if clean_backend in ("gst", "gstreamer"):
                clean_backend = "gstreamer"
            else:
                clean_backend = "v4l2"
            if self._is_laptop_mode():
                self._remote_camera_device = device or self._remote_camera_device or "/dev/video0"
                if width:
                    try:
                        self._camera_width = int(width)
                    except Exception:
                        pass
                if height:
                    try:
                        self._camera_height = int(height)
                    except Exception:
                        pass
                if fps:
                    try:
                        self._camera_fps = int(fps)
                    except Exception:
                        pass
                if self._remote_stream_enabled:
                    payload = {
                        "action": "start",
                        "device": self._remote_camera_device,
                        "width": int(width) if width else self._camera_width,
                        "height": int(height) if height else self._camera_height,
                        "fps": int(fps) if fps else self._camera_fps,
                        "mjpeg": bool(use_mjpeg),
                        "backend": clean_backend,
                        "sender_id": getattr(self.node, "node_id", "outnav"),
                    }
                    self.node.publish_camera_control(payload)
                    if getattr(self, "_sub_img", None):
                        try:
                            self.node.destroy_subscription(self._sub_img)
                        except Exception:
                            pass
                        self._sub_img = None
                    self.start_camera_sub()
                    self._send_toast_to_js("Robot camera streaming started")
                    print("[QtBridge] Robot camera streaming started")
                    return
                payload = {
                    "action": "config",
                    "device": self._remote_camera_device,
                    "width": int(width) if width else self._camera_width,
                    "height": int(height) if height else self._camera_height,
                    "fps": int(fps) if fps else self._camera_fps,
                    "mjpeg": bool(use_mjpeg),
                    "backend": clean_backend,
                    "sender_id": getattr(self.node, "node_id", "outnav"),
                }
                self.node.publish_camera_control(payload)
                self._send_toast_to_js(f"Robot camera set to {self._remote_camera_device}")
                print(f"[QtBridge] Robot camera set: {self._remote_camera_device}")
                return
            payload = {
                "action": "start",
                "device": device or "/dev/video0",
                "width": int(width) if width else self.node.camera_width,
                "height": int(height) if height else self.node.camera_height,
                "fps": int(fps) if fps else self.node.camera_fps,
                "mjpeg": bool(use_mjpeg),
                "backend": clean_backend,
                "sender_id": getattr(self.node, "node_id", "outnav"),
            }
            self.node.publish_camera_control(payload)
            if getattr(self, "_sub_img", None):
                try:
                    self.node.destroy_subscription(self._sub_img)
                except Exception:
                    pass
                self._sub_img = None
            self.start_camera_sub()
        except Exception as e:
            print(f"[QtBridge] startRemoteCamera failed: {e}")

    @QtCore.pyqtSlot(str, int, int, int, bool, str)
    def startRemoteCamera2(self, device: str, width: int, height: int, fps: int, use_mjpeg: bool, backend: str):
        try:
            if not self.node or not hasattr(self.node, "publish_camera_control"):
                return
            clean_backend = (backend or self._camera_backend or "v4l2").strip().lower()
            if clean_backend in ("gst", "gstreamer"):
                clean_backend = "gstreamer"
            else:
                clean_backend = "v4l2"
            if self._is_laptop_mode():
                self._remote_camera2_device = device or self._remote_camera2_device or "/dev/video2"
                if width:
                    try:
                        self._camera_width = int(width)
                    except Exception:
                        pass
                if height:
                    try:
                        self._camera_height = int(height)
                    except Exception:
                        pass
                if fps:
                    try:
                        self._camera_fps = int(fps)
                    except Exception:
                        pass
                if not self._remote_stream_enabled:
                    self._send_toast_to_js("Enable robot camera streaming to start camera 2")
                    print("[QtBridge] Remote stream disabled; camera 2 not started")
                    return
                payload = {
                    "action": "start2",
                    "device": self._remote_camera2_device,
                    "width": int(width) if width else self._camera_width,
                    "height": int(height) if height else self._camera_height,
                    "fps": int(fps) if fps else self._camera_fps,
                    "mjpeg": bool(use_mjpeg),
                    "backend": clean_backend,
                    "sender_id": getattr(self.node, "node_id", "outnav"),
                }
                self.node.publish_camera_control(payload)
                if getattr(self, "_sub_img2", None):
                    try:
                        self.node.destroy_subscription(self._sub_img2)
                    except Exception:
                        pass
                    self._sub_img2 = None
                self.start_camera2_sub()
                self._send_toast_to_js("Robot camera 2 streaming started")
                print("[QtBridge] Robot camera 2 streaming started")
                return
            payload = {
                "action": "start2",
                "device": device or "/dev/video0",
                "width": int(width) if width else self.node.camera_width,
                "height": int(height) if height else self.node.camera_height,
                "fps": int(fps) if fps else self.node.camera_fps,
                "mjpeg": bool(use_mjpeg),
                "backend": clean_backend,
                "sender_id": getattr(self.node, "node_id", "outnav"),
            }
            self.node.publish_camera_control(payload)
            if getattr(self, "_sub_img2", None):
                try:
                    self.node.destroy_subscription(self._sub_img2)
                except Exception:
                    pass
                self._sub_img2 = None
            self.start_camera2_sub()
        except Exception as e:
            print(f"[QtBridge] startRemoteCamera2 failed: {e}")

    @QtCore.pyqtSlot()
    def stopRemoteCamera(self):
        try:
            if self._is_laptop_mode() and not self._remote_stream_enabled:
                return
            if self.node and hasattr(self.node, "publish_camera_control"):
                payload = {
                    "action": "stop",
                    "sender_id": getattr(self.node, "node_id", "outnav"),
                }
                self.node.publish_camera_control(payload)
            # Tear down subscription to incoming frames
            if hasattr(self, '_sub_img') and self._sub_img:
                try:
                    self.node.destroy_subscription(self._sub_img)
                except Exception:
                    pass
                self._sub_img = None
        except Exception as e:
            print(f"[QtBridge] stopRemoteCamera failed: {e}")

    @QtCore.pyqtSlot()
    def stopRemoteCamera2(self):
        try:
            if self._is_laptop_mode() and not self._remote_stream_enabled:
                return
            if self.node and hasattr(self.node, "publish_camera_control"):
                payload = {
                    "action": "stop2",
                    "sender_id": getattr(self.node, "node_id", "outnav"),
                }
                self.node.publish_camera_control(payload)
            if hasattr(self, "_sub_img2") and self._sub_img2:
                try:
                    self.node.destroy_subscription(self._sub_img2)
                except Exception:
                    pass
                self._sub_img2 = None
        except Exception as e:
            print(f"[QtBridge] stopRemoteCamera2 failed: {e}")
    @QtCore.pyqtSlot()
    def requestSecondCameraPort(self):
        """Enumerate /dev/video* and select USB camera for second HUD."""
        try:
            devs = [f"/dev/{d}" for d in sorted(os.listdir("/dev")) if d.startswith("video")]
        except Exception:
            devs = []
        if not devs:
            print("[QtBridge] No USB cameras detected for camera 2.")
            return
        dlg = QDialog()
        dlg.setWindowTitle("Select USB Camera (Second)")
        dlg.setStyleSheet("""
            QDialog { background: #0f172a; color: #e5edff; }
            QListWidget { background:#0b1220; color:#e5edff; border:1px solid #334155; }
            QListWidget::item { padding:8px; }
            QListWidget::item:selected { background:#1f2937; color:#10b981; }
            QPushButton { background:#1b2335; color:#e5edff; border:1px solid #334155; border-radius:6px; padding:8px 12px; }
            QPushButton:hover { background:#243247; }
            QLabel { color:#9fb4ff; font-weight:600; }
        """)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Available video devices:"))
        listw = QListWidget()
        for d in devs:
            QListWidgetItem(d, listw)
        listw.setCurrentRow(0)
        layout.addWidget(listw)
        btns = QHBoxLayout()
        ok_btn = QPushButton("Start Camera 2")
        cancel_btn = QPushButton("Cancel")
        btns.addWidget(ok_btn)
        btns.addWidget(cancel_btn)
        layout.addLayout(btns)
        dlg.setLayout(layout)

        chosen = {"path": None}
        def accept_choice():
            item = listw.currentItem()
            if item:
                chosen["path"] = item.text()
                dlg.accept()
        ok_btn.clicked.connect(accept_choice)
        cancel_btn.clicked.connect(dlg.reject)

        if dlg.exec_() == QDialog.Accepted and chosen["path"] and hasattr(self.node, "set_camera_source2"):
            choice = chosen["path"]
            print(f"[QtBridge] Second camera starting: {choice}")
            if getattr(self, "_sub_img2", None):
                try:
                    self.node.destroy_subscription(self._sub_img2)
                except Exception:
                    pass
                self._sub_img2 = None
            try:
                if self.node and hasattr(self.node, "set_camera_backend"):
                    self.node.set_camera_backend(self._camera_backend)
            except Exception:
                pass
            self.node.set_camera_source2(choice)
            self.start_camera2_sub()

    # =====================================================================
    #                      BLUETOOTH
    # =====================================================================

    @QtCore.pyqtSlot()
    def connectBluetooth(self):
        """Open system Bluetooth settings."""
        try:
            print("[QtBridge] Opening Bluetooth settings...")
            
            # Try different methods based on desktop environment
            commands = [
                ['gnome-control-center', 'bluetooth'],  # GNOME
                ['blueman-manager'],                     # Blueman
                ['systemsettings5', 'kcm_bluetooth'],   # KDE
                ['blueberry'],                           # Linux Mint
            ]
            
            success = False
            for cmd in commands:
                try:
                    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    success = True
                    print(f"[QtBridge] Opened Bluetooth settings using: {cmd[0]}")
                    self._send_toast_to_js("Bluetooth settings opened")
                    break
                except FileNotFoundError:
                    continue
            
            if not success:
                print("[QtBridge] No Bluetooth manager found. Install blueman or gnome-bluetooth.")
                self._send_toast_to_js("Bluetooth manager not found")
                
        except Exception as e:
            print(f"[QtBridge] Bluetooth error: {e}")
            self._send_toast_to_js("Failed to open Bluetooth settings")

    @QtCore.pyqtSlot()
    def stopCamera(self):
        """Stop camera streaming."""
        try:
            print("[QtBridge] Stopping camera...")
            
            # Stop ROS camera publisher
            if self.node and hasattr(self.node, "stop_camera"):
                self.node.stop_camera(join=True)
            self._stop_recording(second=False)

            # Tear down subscription to incoming frames
            if hasattr(self, '_sub_img') and self._sub_img:
                try:
                    self.node.destroy_subscription(self._sub_img)
                except Exception:
                    pass
                self._sub_img = None
            
            print("[QtBridge] Camera stopped")
            
        except Exception as e:
            print(f"[QtBridge] Error stopping camera: {e}")
            import traceback
            traceback.print_exc()

    @QtCore.pyqtSlot()
    def stopCamera2(self):
        """Stop second camera streaming."""
        try:
            print("[QtBridge] Stopping second camera...")

            if self.node and hasattr(self.node, "stop_camera2"):
                self.node.stop_camera2(join=True)
            self._stop_recording(second=True)

            if hasattr(self, '_sub_img2') and self._sub_img2:
                try:
                    self.node.destroy_subscription(self._sub_img2)
                except Exception:
                    pass
                self._sub_img2 = None

            print("[QtBridge] Second camera stopped")
            
        except Exception as e:
            print(f"[QtBridge] Error stopping second camera: {e}")
            import traceback
            traceback.print_exc()

    @QtCore.pyqtSlot(str)
    def openUrl(self, url: str):
        """Open URL in external browser (for report form)."""
        try:
            import webbrowser
            webbrowser.open(url, new=2, autoraise=True)
        except Exception as e:
            print(f"[QtBridge] openUrl failed: {e}")

    @QtCore.pyqtSlot(str, bool, str)
    @QtCore.pyqtSlot(str, bool, str, str)
    def saveSnapshot(self, name, second=False, mission="session", destination=""):
        """Save latest frame from camera 1 or 2 to laptop or robot storage."""
        target = self._sanitize_filename(name or "capture")
        mission_name = mission or self.current_mission_name or "session"
        requested = (destination or "").strip().lower()
        if requested not in ("laptop", "robot"):
            requested = "laptop" if self._is_laptop_mode() else "robot"

        if requested == "robot" and self._is_laptop_mode():
            if not self.node or not hasattr(self.node, "publish_camera_control"):
                self._send_toast_to_js("Robot snapshot save is not available right now.")
                return
            payload = {
                "action": "snapshot",
                "device": self._remote_camera2_device if second else self._remote_camera_device,
                "second": bool(second),
                "mission": mission_name,
                "name": target,
                "sender_id": getattr(self.node, "node_id", "outnav"),
            }
            self.node.publish_camera_control(payload)
            self._send_toast_to_js("Snapshot requested. Image will be saved on robot.")
            print(f"[QtBridge] Robot snapshot requested → {target}")
            return

        mission_dir = self._mission_folder(mission_name)
        if requested == "laptop":
            folder = os.path.join(os.path.expanduser("~"), "Videos", "outnav", mission_dir)
            location = "laptop"
        else:
            folder = os.path.join(self.base_dir, "videos", mission_dir)
            location = "robot"
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, f"{target}.jpg")
        jpg = self._last_jpg2 if second else self._last_jpg1
        if jpg is None:
            self._send_toast_to_js("No frame available yet.")
            return
        try:
            data = jpg.tobytes() if hasattr(jpg, "tobytes") else bytes(jpg)
            if not data:
                self._send_toast_to_js("No frame available yet.")
                return
            with open(path, "wb") as f:
                f.write(data)
            print(f"[QtBridge] Snapshot saved → {path}")
            self._send_toast_to_js(f"Snapshot saved on {location}: {path}")
        except Exception as e:
            print("[QtBridge] Snapshot failed:", e)
            self._send_toast_to_js("Failed to save snapshot")

    @QtCore.pyqtSlot(str, bool, str)
    def startCameraRecording(self, name, second=False, mission="session"):
        """Start writing incoming frames to mp4 for the active camera."""
        if self._is_laptop_mode():
            if second:
                self._send_toast_to_js("Second camera recording not available in remote mode.")
                return
            if not self.node or not hasattr(self.node, "publish_camera_control"):
                return
            mission_name = mission or self.current_mission_name or "session"
            payload = {
                "action": "record_start",
                "device": self._remote_camera_device or "/dev/video0",
                "width": int(self._camera_width) if self._camera_width else None,
                "height": int(self._camera_height) if self._camera_height else None,
                "fps": int(self._camera_fps) if self._camera_fps else None,
                "mjpeg": bool(getattr(self.node, "camera_use_mjpeg", True)),
                "backend": self._camera_backend,
                "mission": mission_name,
                "sender_id": getattr(self.node, "node_id", "outnav"),
            }
            self.node.publish_camera_control(payload)
            self._send_toast_to_js("Recording robot camera. Video will be saved on robot.")
            print("[QtBridge] Remote recording requested (robot-side save)")
            return

        safe = self._sanitize_filename(name or "recording")
        mission_dir = self._mission_folder(mission)
        folder = os.path.join(self.base_dir, "videos", mission_dir)
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, f"{safe}.mp4")
        if second:
            self._recording2 = True
            self._recording_path2 = path
            self._recording_fps2 = float(self._camera_fps or 20)
            self._recorder2 = None
            self._send_toast_to_js(f"Recording camera 2 → {path}")
            print(f"[QtBridge] Camera2 recording to {path}")
        else:
            self._recording1 = True
            self._recording_path1 = path
            self._recording_fps1 = float(self._camera_fps or 20)
            self._recorder1 = None
            self._send_toast_to_js(f"Recording camera → {path}")
            print(f"[QtBridge] Camera recording to {path}")

    @QtCore.pyqtSlot(bool)
    def stopCameraRecording(self, second=False):
        """Stop recording and close the writer."""
        if self._is_laptop_mode():
            if self.node and hasattr(self.node, "publish_camera_control"):
                payload = {
                    "action": "record_stop",
                    "sender_id": getattr(self.node, "node_id", "outnav"),
                }
                self.node.publish_camera_control(payload)
            return
        self._stop_recording(second=second)

    # ---------------- Internal recording helpers ----------------
    def _stop_recording(self, second: bool):
        if second:
            if self._recorder2:
                try:
                    self._recorder2.release()
                except Exception:
                    pass
            self._recorder2 = None
            self._recording_fps2 = None
            if self._recording2:
                self._send_toast_to_js("Camera 2 recording saved")
            self._recording2 = False
            self._recording_path2 = None
        else:
            if self._recorder1:
                try:
                    self._recorder1.release()
                except Exception:
                    pass
            self._recorder1 = None
            self._recording_fps1 = None
            if self._recording1:
                self._send_toast_to_js("Camera recording saved")
            self._recording1 = False
            self._recording_path1 = None

    def _write_recording_frame(self, frame, second: bool):
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        if second:
            if not self._recorder2 and self._recording_path2:
                fps = float(self._recording_fps2 or 20.0)
                self._recorder2 = cv2.VideoWriter(self._recording_path2, fourcc, fps, (w, h))
            if self._recorder2:
                self._recorder2.write(frame)
        else:
            if not self._recorder1 and self._recording_path1:
                fps = float(self._recording_fps1 or 20.0)
                self._recorder1 = cv2.VideoWriter(self._recording_path1, fourcc, fps, (w, h))
            if self._recorder1:
                self._recorder1.write(frame)

    def _sanitize_filename(self, name: str) -> str:
        safe = "".join(c for c in name if c.isalnum() or c in ("-", "_"))
        return safe or "capture"

    def _mission_folder(self, mission: str) -> str:
        name = mission or self.current_mission_name or "session"
        return self._sanitize_filename(name)

    @QtCore.pyqtSlot(str)
    def setActiveMissionName(self, name: str):
        """Persist current mission name for media storage."""
        self.current_mission_name = self._sanitize_filename(name or "session")

    # =====================================================================
    #                      SCREEN RECORDING
    # =====================================================================

    @QtCore.pyqtSlot()
    def startRecording(self):
        """Start screen recording with user-selected save location."""
        if self.recording_process:
            print("[QtBridge] Recording already in progress")
            return
        
        try:
            # Ask user where to save
            default_name = f"outnav_recording_{int(time.time())}.mp4"
            file_path, _ = QFileDialog.getSaveFileName(
                None,
                "Save Recording As",
                os.path.join(os.path.expanduser("~"), default_name),
                "Video Files (*.mp4 *.avi);;All Files (*)"
            )
            
            if not file_path:
                print("[QtBridge] Recording cancelled by user")
                return
            
            self.recording_file = file_path
            
            # Get window geometry if available
            if self.window:
                geom = self.window.geometry()
                width = geom.width()
                height = geom.height()
                x = geom.x()
                y = geom.y()
            else:
                width, height, x, y = 1280, 800, 0, 0
            
            # Get display info
            display = os.environ.get('DISPLAY', ':0')
            
            # Start ffmpeg recording
            cmd = [
                'ffmpeg',
                '-f', 'x11grab',
                '-r', '30',  # 30 fps
                '-s', f'{width}x{height}',
                '-i', f'{display}+{x},{y}',  # Capture specific window position
                '-vcodec', 'libx264',
                '-preset', 'ultrafast',
                '-crf', '23',
                '-pix_fmt', 'yuv420p',  #  Add this for compatibility
                '-y',  # overwrite
                file_path
            ]
            
            print(f"[QtBridge] Recording command: {' '.join(cmd)}")
            
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.recording = True
            print(f"[QtBridge]  Recording started: {file_path}")
            self._send_toast_to_js("Recording started")
            
        except Exception as e:
            print(f"[QtBridge] Recording start failed: {e}")
            import traceback
            traceback.print_exc()
            self._send_toast_to_js("Recording failed to start")

    @QtCore.pyqtSlot()
    def stopRecording(self):
        """Stop screen recording."""
        if not self.recording_process:
            print("[QtBridge] No recording in progress")
            return
        
        try:
            # Send SIGINT to ffmpeg for clean stop
            self.recording_process.terminate()
            self.recording_process.wait(timeout=5)
            
            self.recording = False
            print(f"[QtBridge]  Recording stopped: {self.recording_file}")
            self._send_toast_to_js(f"Recording saved")
            
            self.recording_process = None
            self.recording_file = None
            
        except Exception as e:
            print(f"[QtBridge] Recording stop failed: {e}")
            self._send_toast_to_js("Failed to stop recording")

    # =====================================================================
    #                      WAYPOINTS  /  MISSIONS
    # =====================================================================
    @QtCore.pyqtSlot(str, float, float, float, float, int, float, bool)
    def updateGPSData(self, fix_str, lat, lon, alt, heading, sats, hdop, can_proceed):
        """Forward GPS updates from GPSController to the HTML UI."""
        js = (
            "if(window.updateGPSData){"
            f"updateGPSData('{fix_str}', {lat}, {lon}, {alt}, {heading}, {sats}, {hdop}, {str(can_proceed).lower()});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))

    @QtCore.pyqtSlot(float, float)
    def addActualPathPointJS(self, lat, lon):
        """Safely add a breadcrumb point on the map from GPSController."""
        js = (
            "if(typeof addActualPathPoint === 'function'){"
            f"addActualPathPoint({lat}, {lon});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))

    @QtCore.pyqtSlot(str, result=bool)
    def activateGpsPort(self, port: str):
        """Accept a port/device selection from the UI and forward it to ROS/GPS controller."""
        try:
            cleaned = (port or "").strip()
            if not cleaned:
                print("[QtBridge] GPS port not provided")
                return False
            if cleaned != "auto" and not cleaned.startswith("/dev/"):
                cleaned = f"/dev/{cleaned}"

            self.gps_port = cleaned
            if self.node and hasattr(self.node, "set_gps_port"):
                self.node.set_gps_port(cleaned)
            if self.window and hasattr(self.window, "gps_controller") and hasattr(self.window.gps_controller, "set_port"):
                self.window.gps_controller.set_port(cleaned)
            if self.window and hasattr(self.window, "gps_serial_node") and hasattr(self.window.gps_serial_node, "set_port"):
                self.window.gps_serial_node.set_port(cleaned)

            print(f"[QtBridge] GPS activation requested on {cleaned}")
            return True
        except Exception as exc:
            print(f"[QtBridge] GPS activation failed: {exc}")
            return False

    @QtCore.pyqtSlot(str)
    def setWaypointsJson(self, json_str):
        try:
            self.waypoints = json.loads(json_str)
            print(f"[QtBridge] Received {len(self.waypoints)} waypoints.")
            if self.node:
                self.node.publish_waypoints(self.waypoints)
        except Exception as e:
            print("[QtBridge] Error parsing waypoints:", e)

    @QtCore.pyqtSlot(result=str)
    def getWaypoints(self):
        return json.dumps(getattr(self, "waypoints", []))

    @QtCore.pyqtSlot(str, float, float)
    def savePlace(self, name, lat, lng):
        self.saved_places = getattr(self, "saved_places", {})
        self.saved_places[name] = {"lat": lat, "lng": lng}
        print(f"[QtBridge] Saved place '{name}': {self.saved_places[name]}")

    @QtCore.pyqtSlot(result=str)
    def getSavedPlaces(self):
        return json.dumps(getattr(self, "saved_places", {}))

    # ---------------- Status / Telemetry ----------------
    @QtCore.pyqtSlot(str)
    def updateStatus(self, status):
        if self.window:
            self.window.update_status_label(status)

    @QtCore.pyqtSlot(str, str)
    def updateGPS(self, fix, latlon_str):
        if self.window:
            self.window.update_gps_label(fix, latlon_str)

    @QtCore.pyqtSlot(float)
    def updateSpeed(self, speed):
        if self.window:
            self.window.update_speed_label(speed)

    # ---------------- Exit ----------------
    @QtCore.pyqtSlot()
    def exitApp(self):
        # Stop recording if active
        if self.recording_process:
            self.stopRecording()
        
        if self.window:
            try:
                if hasattr(self.window, "stop_ros"):
                    self.window.stop_ros()
            except Exception:
                pass
            self.window.close()

    # ---------------- Mission save/load ----------------
    @QtCore.pyqtSlot(str)
    def saveMission(self, json_str):
        folder = os.path.join(self.base_dir, "missions", "saved_missions")
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "mission_latest.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write(json_str)
        print(f"[Bridge] Mission saved → {path}")
        self._send_mission_data_to_robot(json_str, "mission_latest", overwrite=True)

    @QtCore.pyqtSlot()
    def loadMission(self):
        folder = os.path.join(self.base_dir, "missions", "saved_missions")
        path = os.path.join(folder, "mission_latest.json")
        if not os.path.exists(path):
            print("[Bridge] No saved mission found.")
            return
        with open(path, "r", encoding="utf-8") as f:
            data = f.read()
        self._inject_js_load(data)
        print(f"[Bridge] Mission loaded from {path}")

    @QtCore.pyqtSlot(str, str)
    def saveMissionNamed(self, json_data, name):
        folder = os.path.join(self.base_dir, "missions", "saved_missions")
        os.makedirs(folder, exist_ok=True)
        safe = "".join(c for c in name if c.isalnum() or c in ("-", "_"))
        path = os.path.join(folder, f"{safe}.json")
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(json_data)
            latest_path = os.path.join(folder, "mission_latest.json")
            try:
                with open(latest_path, "w", encoding="utf-8") as f:
                    f.write(json_data)
            except Exception:
                pass
            print(f"[Bridge] Mission saved → {path}")
            self._send_mission_data_to_robot(json_data, safe, overwrite=False)
        except Exception as e:
            print("[Bridge] Failed to save mission:", e)

    def _send_mission_data_to_robot(self, json_data: str, name: str, overwrite: bool):
        try:
            if not self.node or not hasattr(self.node, "publish_mission_data"):
                return
            mode = getattr(self.node, "device_mode", "laptop")
            if mode != "laptop":
                return
            data_b64 = base64.b64encode(json_data.encode("utf-8")).decode("ascii")
            payload = {
                "name": name,
                "data_b64": data_b64,
                "overwrite": bool(overwrite),
                "sender_id": getattr(self.node, "node_id", "outnav"),
            }
            self.node.publish_mission_data(payload)
            print(f"[QtBridge] Mission data sent to robot: {name}")
        except Exception as e:
            print("[QtBridge] Failed to send mission data:", e)

    @QtCore.pyqtSlot(str)
    def syncSettings(self, json_str: str):
        try:
            if not self.node or not hasattr(self.node, "publish_settings"):
                return
            payload = json.loads(json_str)
            if "gpsUartEnabled" in payload and hasattr(self.node, "set_gps_enabled"):
                self.node.set_gps_enabled(payload.get("gpsUartEnabled"))
            if "gpsEnabled" in payload and hasattr(self.node, "set_gps_enabled"):
                self.node.set_gps_enabled(payload.get("gpsEnabled"))
            if "localizationPoseEnabled" in payload and hasattr(self.node, "set_localization_pose_enabled"):
                self.node.set_localization_pose_enabled(payload.get("localizationPoseEnabled"))
            if "localizationMotionEnabled" in payload and hasattr(self.node, "set_localization_motion_enabled"):
                self.node.set_localization_motion_enabled(payload.get("localizationMotionEnabled"))
            if "localizationCmdVelEnabled" in payload and hasattr(self.node, "set_localization_motion_enabled"):
                self.node.set_localization_motion_enabled(payload.get("localizationCmdVelEnabled"))
            if "cameraStreamEnabled" in payload:
                self._set_remote_stream_enabled(bool(payload.get("cameraStreamEnabled")))
            if "deviceMode" in payload:
                payload.pop("deviceMode", None)
            payload["sender_id"] = getattr(self.node, "node_id", "outnav")
            self.node.publish_settings(payload)
            print("[QtBridge] Settings synced to robot")
            self._send_sync_audit("settings", "Sent to robot")
        except Exception as e:
            print("[QtBridge] Settings sync failed:", e)
            self._send_sync_audit("settings", "Sync failed")

    @QtCore.pyqtSlot(str, str)
    def syncMissionData(self, name: str, json_str: str):
        try:
            if not self.node or not hasattr(self.node, "publish_mission_data"):
                return
            safe = "".join(c for c in name if c.isalnum() or c in ("-", "_")) or "mission_latest"
            self._send_mission_data_to_robot(json_str, safe, overwrite=True)
            self._send_sync_audit("mission", f"Sent {safe}")
        except Exception as e:
            print("[QtBridge] Mission sync failed:", e)
            self._send_sync_audit("mission", "Sync failed")

    @QtCore.pyqtSlot(int, int, int, bool)
    def setCameraProfile(self, width: int, height: int, fps: int, use_mjpeg: bool = True):
        try:
            if self.node and hasattr(self.node, "set_camera_profile"):
                self.node.set_camera_profile(width, height, fps, use_mjpeg)
        except Exception as e:
            print("[QtBridge] Camera profile set failed:", e)
        try:
            self._camera_width = int(width)
            self._camera_height = int(height)
            self._camera_fps = int(fps)
        except Exception:
            pass

    @QtCore.pyqtSlot(str, result=str)
    def detectCameraProfiles(self, device: str):
        try:
            cmd = ["v4l2-ctl", "--list-formats-ext", "-d", device]
            out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True, timeout=3)
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e)})
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
        prefer = None
        for candidate in ((640, 480, "MJPG"), (1280, 720, "MJPG"), (640, 480, "YUYV")):
            for f in formats:
                if f["width"] == candidate[0] and f["height"] == candidate[1] and f["fmt"] == candidate[2]:
                    prefer = f
                    break
            if prefer:
                break
        if not prefer and formats:
            prefer = formats[0]
        return json.dumps({"ok": True, "preferred": prefer, "formats": formats})

    def _send_sync_audit(self, kind: str, msg: str):
        if not self.window:
            return
        js = (
            "if(window.updateSyncAudit){"
            f"updateSyncAudit({json.dumps(kind)}, {json.dumps(msg)});"
            "}"
        )
        QtCore.QTimer.singleShot(0, lambda: self._run_js(js))

    @QtCore.pyqtSlot(str)
    def loadMissionNamed(self, name):
        folder = os.path.join(self.base_dir, "missions", "saved_missions")
        safe = "".join(c for c in name if c.isalnum() or c in ("-", "_"))
        path = os.path.join(folder, f"{safe}.json")
        if not os.path.exists(path):
            print(f"[Bridge] Mission '{name}' not found.")
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = f.read()
            self._inject_js_load(data)
            print(f"[Bridge] Mission loaded from {path}")
        except Exception as e:
            print("[Bridge] Failed to load mission:", e)

    @QtCore.pyqtSlot(result='QVariant')
    def listSavedMissions(self):
        folder = os.path.join(self.base_dir, "missions", "saved_missions")
        if not os.path.exists(folder):
            return []
        try:
            items = [
                os.path.splitext(f)[0]
                for f in os.listdir(folder)
                if f.endswith(".json")
            ]
            print(f"[Bridge] Listing missions: {items}")
            return items
        except Exception as e:
            print("[Bridge] Failed to list missions:", e)
            return []

    # ---------------- Mission scripts (right panel) ----------------
    @QtCore.pyqtSlot(result='QVariant')
    def getMissionFiles(self):
        try:
            files = self._list_mission_program_files()
            print(f"[Bridge] Found {len(files)} mission file(s): {files}")
            return files
        except Exception as e:
            print("[Bridge] getMissionFiles() error:", e)
            return []

    # ---------------- Task JSON → ROS publish ----------------
    @QtCore.pyqtSlot(str)
    def setTasksJson(self, tasks_json):
        try:
            print("[Bridge] Received tasks JSON from JS.")
            if self.node:
                self.node.publish_mission_tasks(tasks_json)
        except Exception as e:
            print("[Bridge] setTasksJson() error:", e)
    @QtCore.pyqtSlot(str)
    def publishMissionTasks(self, tasks_json):
        """Expose publish_mission_tasks directly to JS (expects JSON string)."""
        try:
            if self.node:
                self.node.publish_mission_tasks(tasks_json)
                print("[Bridge] publishMissionTasks forwarded to ROS")
        except Exception as e:
            print("[Bridge] publishMissionTasks error:", e)

    # ---------------- OSRM proxy ----------------
    @QtCore.pyqtSlot(float, float, float, float, result='QVariant')
    def osrmRoute(self, home_lat, home_lng, goal_lat, goal_lng):
        url = (
            "https://router.project-osrm.org/route/v1/driving/"
            f"{home_lng},{home_lat};{goal_lng},{goal_lat}"
            "?overview=full&geometries=geojson"
        )
        try:
            with urllib.request.urlopen(url, timeout=8) as r:
                body = r.read().decode("utf-8")
            j = json.loads(body)
            coords = j["routes"][0]["geometry"]["coordinates"]
            return [[c[1], c[0]] for c in coords]
        except Exception as e:
            print("[Bridge] OSRM fetch failed:", e)
            return []
    # ---------------- JS → ROS helpers expected by UI ----------------
    @QtCore.pyqtSlot()
    def clearWaypoints(self):
        if self.node:
            self.node.clear_waypoints()

    @QtCore.pyqtSlot(float)
    def setSpeed(self, speed):
        if self.node:
            self.node.set_speed(speed)

    @QtCore.pyqtSlot(float, float)
    def addWaypoint(self, lat, lng):
        if self.node:
            self.node.add_waypoint(lat, lng)

    @QtCore.pyqtSlot()
    def commitMission(self):
        if self.node:
            self.node.commit_mission()

    @QtCore.pyqtSlot(float, float)
    def setHome(self, lat, lng):
        if self.node:
            self.node.set_home(lat, lng)

    @QtCore.pyqtSlot(str)
    def publishRoute(self, coords_json):
        if self.node:
            self.node.publish_route_json(coords_json)

    @QtCore.pyqtSlot(str)
    def goRoad(self, coords_json):
        if self.node:
            self.node.go_road(coords_json)

    @QtCore.pyqtSlot()
    def resetMission(self):
        if self.node:
            self.node.reset_mission()

    @QtCore.pyqtSlot()
    def startMission(self):
        if self.node:
            self.node.start_mission()

    @QtCore.pyqtSlot()
    def resumeMission(self):
        if self.node:
            self.node.resume_mission()

    @QtCore.pyqtSlot()
    def stopMission(self):
        if self.node:
            self.node.stop_mission()

    @QtCore.pyqtSlot()
    def deactivateEmergencyStop(self):
        if self.node:
            self.node.deactivate_emergency_stop()

    @QtCore.pyqtSlot('QVariant')
    def onMissionError(self, cb):
        """Placeholder to accept JS callback; no ROS event hooked up yet."""
        self._mission_error_cb = cb
        print("[Bridge] onMissionError callback registered (not yet wired)")

    # ---------------- Helper utilities ----------------
    def _inject_js_load(self, json_text: str):
        if not self.window:
            return
        try:
            self.window.view.page().runJavaScript(
                f"if(window.loadMissionData) window.loadMissionData({json_text});"
                f"else if(window.queueMissionData) window.queueMissionData({json_text});"
            )
        except Exception as e:
            print("[Bridge] JS injection failed:", e)

    def sendMessageToJS(self, message: dict):
        try:
            self.sendToJS.emit(json.dumps(message))
        except Exception as e:
            print("[QtBridge] Failed to send to JS:", e)
            
    # ---------------- Text-to-Speech ----------------
    @QtCore.pyqtSlot(str)
    def speakText(self, text):
        """Text-to-speech using flite female voice (fast and clear)."""
        try:
            # Initialize lock if not exists
            if not hasattr(self, '_speech_lock'):
                self._speech_lock = threading.Lock()
            
            # Try to acquire lock (non-blocking)
            if not self._speech_lock.acquire(blocking=False):
                print(f"[QtBridge] ⏭️ Skipping speech (already speaking): \"{text}\"")
                return  # Skip if already speaking
            
            print(f"[QtBridge] 🔊 Speaking: \"{text}\"")
            
            def speak():
                try:
                    # Use flite with slt (female) voice and faster speed
                    import tempfile
                    
                    # Generate audio file with flite
                    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                        temp_file = f.name
                    
                    # Generate speech
                    subprocess.run(
                        ['flite', '-voice', 'slt', '-o', temp_file, text],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=10
                    )
                    
                    # Play at faster speed using sox
                    subprocess.run(
                        ['play', temp_file, 'tempo', '1.3'],  # 1.3x speed
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=10
                    )
                    
                    # Cleanup
                    import os
                    os.remove(temp_file)
                    
                except FileNotFoundError:
                    # Fallback if sox not installed - use regular flite
                    subprocess.run(
                        ['flite', '-voice', 'slt', text],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=10
                    )
                except Exception as e:
                    print(f"[QtBridge] TTS error: {e}")
                finally:
                    # Always release lock when done
                    self._speech_lock.release()
            
            threading.Thread(target=speak, daemon=True).start()
            
        except Exception as e:
            print(f"[QtBridge] speakText() failed: {e}")
            # Make sure to release lock on error
            if hasattr(self, '_speech_lock'):
                try:
                    self._speech_lock.release()
                except:
                    pass
    


    @QtCore.pyqtSlot()
    def emergencyStop(self):
        """Emergency stop - immediately halt all robot movement."""
        try:
            print("[QtBridge]  EMERGENCY STOP ACTIVATED")
            
            # Publish zero velocity to stop robot immediately
            if self.node:
                self.node.publish_cmd_vel(0.0, 0.0)
                
                # Publish emergency stop signal if method exists
                if hasattr(self.node, 'emergency_stop_pub'):
                    from std_msgs.msg import Bool
                    msg = Bool()
                    msg.data = True
                    self.node.emergency_stop_pub.publish(msg)
                    print("[QtBridge] Emergency stop signal published to ROS")
            
            # Visual feedback in UI
            self._send_toast_to_js(" EMERGENCY STOP ACTIVATED")
            
        except Exception as e:
            print(f"[QtBridge] Emergency stop failed: {e}")
            import traceback
            traceback.print_exc()

    @QtCore.pyqtSlot()
    def resumeFromEstop(self):
        """Resume from emergency stop."""
        try:
            print("[QtBridge]  Emergency stop cleared")
            
            if self.node and hasattr(self.node, 'emergency_stop_pub'):
                from std_msgs.msg import Bool
                msg = Bool()
                msg.data = False
                self.node.emergency_stop_pub.publish(msg)
                print("[QtBridge] Resume signal published to ROS")

            # Kick mission back into motion if requested
            if self.node:
                self.node.resume_mission()
            
            self._send_toast_to_js("Emergency stop cleared")
            
        except Exception as e:
            print(f"[QtBridge] Resume from e-stop failed: {e}")

    # ---------------- Manual drive ----------------
    @QtCore.pyqtSlot(float, float)
    def manualDrive(self, linear, angular):
        """Directly drive the robot via /cmd_vel."""
        try:
            if self.node:
                if hasattr(self.node, "publish_manual_drive"):
                    self.node.publish_manual_drive(linear, angular)
                else:
                    self.node.publish_cmd_vel(linear, angular)
        except Exception as e:
            print(f"[QtBridge] manualDrive failed: {e}")

    def _send_toast_to_js(self, message: str):
        """Send toast notification to JavaScript."""
        if not self.window:
            return
        
        def inject():
            try:
                page = self.window.view.page()
                # Escape single quotes in message
                safe_msg = message.replace("'", "\\'")
                js = f"if(window.showToast) {{ showToast('{safe_msg}'); }}"
                page.runJavaScript(js)
            except Exception as e:
                print(f"[QtBridge] Failed to send toast: {e}")
        
        QtCore.QTimer.singleShot(0, inject)
        
    @QtCore.pyqtSlot()
    def returnToHome(self):
        """Command robot to return to home position."""
        try:
            print("[QtBridge] 🏠 Return to Home activated")
            
            # This will be handled by the mission task system
            # The JavaScript sends a special return_to_home mission
            
            if self.node:
                print("[QtBridge] Robot returning to home position")
                
                # Speak announcement
                if hasattr(self, 'speakText'):
                    self.speakText("Returning to home position")
                
        except Exception as e:
            print(f"[QtBridge] Return to home error: {e}")
            import traceback
            traceback.print_exc()
