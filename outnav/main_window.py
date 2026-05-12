import os
import sys
import threading
from PyQt5 import QtCore, QtWidgets, QtWebEngineWidgets, QtWebChannel
from ros_node import OutdoorNavNode
from qt_bridge import QtBridge  
from gps_controller import GPSController
from gps_serial_node import GPSSerialNode
from PyQt5.QtWebEngineWidgets import QWebEngineSettings
from rclpy.executors import MultiThreadedExecutor
import rclpy

class OutdoorNavApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OutNav v1.0")
        self.adjust_size_to_monitor()

        # Initialize ROS2 node
        self.node = OutdoorNavNode()
        print("OutdoorNavApp -- ROS2 node started.")

        # Initialize web view FIRST
        self.view = QtWebEngineWidgets.QWebEngineView(self)
        self.setCentralWidget(self.view)

        # SINGLE WebChannel setup - CREATE BRIDGE FIRST
        self.channel = QtWebChannel.QWebChannel()
        self.bridge = QtBridge(self.node, window=self)   
        self.channel.registerObject("py", self.bridge)
        self.view.page().setWebChannel(self.channel)

        # NOW initialize GPS controller with bridge
        self.gps_controller = GPSController(bridge=self.bridge)
        print("OutdoorNavApp -- GPS Controller started.")

        # GPS serial reader (publishes /fix + /gps/heading) - robot side only by default.
        self.gps_serial_node = None
        if self._should_start_gps_serial():
            self.gps_serial_node = GPSSerialNode()
            print("OutdoorNavApp -- GPS Serial node started.")
        else:
            print("OutdoorNavApp -- GPS Serial node skipped on this host.")

        # START ROS SPIN THREAD
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        print("OutdoorNavApp -- ROS2 spin thread started.")

        # Load HTML file
        html_path = os.path.join(os.path.dirname(__file__), "map_template.html")
        self.view.load(QtCore.QUrl.fromLocalFile(html_path))
        print(f"OutdoorNavApp -- Loading {html_path}")
        
        # Configure page settings
        page = self.view.page()
        settings = page.settings()
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.LocalStorageEnabled, True)
        settings.setAttribute(QWebEngineSettings.JavascriptEnabled, True)
        settings.setAttribute(QWebEngineSettings.JavascriptCanAccessClipboard, True)

        # Enable JavaScript console logging
        def _js_console(level, msg, line, src):
            print(f"[JS] {msg}")
            if "ddsi_udp_conn_write" in msg or "dds" in msg.lower():
                try:
                    self.view.page().runJavaScript(
                        "if(window.updateDdsHealth) updateDdsHealth('warn', 'DDS write error');"
                    )
                except Exception:
                    pass
        self.view.page().javaScriptConsoleMessage = _js_console

        # Handle page load complete
        self.view.loadFinished.connect(self.on_page_loaded)

    def spin_ros(self):
        """Background thread to spin ROS2 node and process callbacks."""
        try:
            print("ROS Spin -- Starting to process ROS callbacks...")
            # Spin both nodes
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor.add_node(self.gps_controller)
            if self.gps_serial_node is not None:
                self.executor.add_node(self.gps_serial_node)
            self.executor.spin()
        except Exception as e:
            print(f" ROS Spin -- Error: {e}")
        finally:
            if hasattr(self, "executor") and self.executor:
                try:
                    self.executor.shutdown()
                except Exception:
                    pass

    def stop_ros(self):
        """Shutdown executor to stop spin thread."""
        if hasattr(self, "executor") and self.executor:
            try:
                self.executor.shutdown()
            except Exception:
                pass

    def on_page_loaded(self):
        print("OutdoorNavApp -- Page fully loaded")
        try:
            if self.bridge and hasattr(self.bridge, "start_camera_sub"):
                self.bridge.start_camera_sub()
        except Exception as e:
            print(f"OutdoorNavApp -- Auto camera subscribe failed: {e}")
        try:
            if self.bridge and hasattr(self.bridge, "uiReady"):
                QtCore.QTimer.singleShot(150, self.bridge.uiReady)
        except Exception as e:
            print(f"OutdoorNavApp -- UI ready notify failed: {e}")

    def closeEvent(self, event):
        print(" OutdoorNavApp -- Closing application")
        try:
            if hasattr(self, "bridge") and self.bridge:
                if hasattr(self.bridge, "stopCamera"):
                    self.bridge.stopCamera()
                if hasattr(self.bridge, "stopCamera2"):
                    self.bridge.stopCamera2()
            self.stop_ros()
            self.node.destroy_node()
            self.gps_controller.destroy_node()
            if hasattr(self, "gps_serial_node") and self.gps_serial_node is not None:
                self.gps_serial_node.stop()
                self.gps_serial_node.destroy_node()
        except Exception as e:
            print(f"OutdoorNavApp -- Error destroying node: {e}")
        event.accept()

    def _should_start_gps_serial(self) -> bool:
        """
        Start GPS serial only on robot hosts by default.

        Override with:
          OUTNAV_ENABLE_GPS_SERIAL=true  -> force start
          OUTNAV_ENABLE_GPS_SERIAL=false -> force skip
        """
        mode = (os.getenv("OUTNAV_ENABLE_GPS_SERIAL", "auto") or "auto").strip().lower()
        if mode in ("1", "true", "yes", "on"):
            return True
        if mode in ("0", "false", "no", "off"):
            return False
        if not self._is_robot_host():
            return False
        port = (os.getenv("OUTNAV_GPS_PORT", "/dev/ttyTHS1") or "/dev/ttyTHS1").strip()
        if port == "auto":
            return True
        return os.path.exists(port)

    @staticmethod
    def _is_robot_host() -> bool:
        """Best-effort detection for Jetson robot host."""
        try:
            if os.path.exists("/etc/nv_tegra_release"):
                return True
            if os.getenv("OUTNAV_HEADLESS", "").strip().lower() in ("1", "true", "yes", "on"):
                return True
        except Exception:
            return False
        return False

    def adjust_size_to_monitor(self):
        """Resize and center the window relative to the current monitor size."""
        screen = QtWidgets.QApplication.primaryScreen()
        if not screen:
            # Fallback to legacy fixed size if no screen info is available.
            self.resize(1550, 1500)
            return

        geometry = screen.availableGeometry()
        target_width = max(800, int(geometry.width() * 0.9))
        target_height = max(600, int(geometry.height() * 0.9))

        self.resize(target_width, target_height)

        # Center the window within the available screen area.
        frame = self.frameGeometry()
        frame.moveCenter(geometry.center())
        self.move(frame.topLeft())

# Note: main() is defined in app.py and used by run.sh. Keeping this file import-only
# prevents accidental double initialization of ROS/Qt.
