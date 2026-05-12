#!/usr/bin/env python3
"""
GPS Controller for OutdoorNav

- Listens to /fix          (sensor_msgs/NavSatFix) from your ROS GPS node
- Optionally listens to /gps/heading (std_msgs/Float64)
- Forwards data to the Qt/HTML UI via QtBridge (updateGPSData + addActualPathPoint)
"""

import math
import logging
import time
from PyQt5 import QtCore

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64


class GPSController(Node):
    def __init__(self, bridge=None):
        super().__init__("gps_controller")

        self.bridge = bridge

        # Current GPS state
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.num_satellites = 0
        self.fix_type = NavSatStatus.STATUS_NO_FIX
        self.hdop = 99.9
        self.heading = 0.0
        self.signal_lost = False
        self.port = "/dev/ttyTHS1"  # only for UI / logging
        self.last_update_time = 0.0
        self._last_log_wall = 0.0
        self._log_interval_sec = 5.0
        self._logger = logging.getLogger("gps_controller")

        # Subscribe to NavSatFix from external ROS GPS node
        self.create_subscription(
            NavSatFix,
            "/fix",
            self.gps_callback,
            10,
        )

        # Subscribe to heading (degrees)
        self.create_subscription(
            Float64,
            "/gps/heading",
            self.heading_callback,
            10,
        )

        # Periodic UI update (stale detection)
        self.create_timer(0.5, self.update_ui)

        self.get_logger().info("GPSController started (listening to /fix and /gps/heading).")

    # ----------------- ROS callbacks -----------------

    def gps_callback(self, msg: NavSatFix):
        """Update internal GPS state from NavSatFix."""
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        self.fix_type = msg.status.status
        self.last_update_time = self.get_clock().now().nanoseconds / 1e9

        # HDOP from covariance (gps_node encodes var = hdop^2 in [0] and [4])
        try:
            if (
                msg.position_covariance_type
                == NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                and len(msg.position_covariance) >= 5
                and msg.position_covariance[0] > 0.0
            ):
                self.hdop = math.sqrt(msg.position_covariance[0])
            else:
                self.hdop = 99.9
        except Exception:
            self.hdop = 99.9

        # Rough satellite estimate from HDOP
        if self.hdop < 1.0:
            self.num_satellites = 10
        elif self.hdop < 2.0:
            self.num_satellites = 8
        elif self.hdop < 3.0:
            self.num_satellites = 6
        elif self.hdop < 5.0:
            self.num_satellites = 4
        else:
            self.num_satellites = 3

        now = time.time()
        if (now - self._last_log_wall) >= self._log_interval_sec:
            self._last_log_wall = now
            self._logger.debug(
                "GPS update: lat=%.6f, lon=%.6f, alt=%.2f, fix_type=%s, hdop=%.2f, sats_est=%s, heading=%.1f°",
                self.latitude,
                self.longitude,
                self.altitude,
                self.fix_type,
                self.hdop,
                self.num_satellites,
                self.heading,
            )
        self.update_ui()

    def heading_callback(self, msg: Float64):
        """Optional heading / compass (degrees)."""
        self.heading = msg.data
        self.last_update_time = self.get_clock().now().nanoseconds / 1e9
        now = time.time()
        if (now - self._last_log_wall) >= self._log_interval_sec:
            self._last_log_wall = now
            self._logger.debug("Heading update: %.1f°", self.heading)
        self.update_ui()

    # ----------------- Settings → GPS port -----------------

    def set_port(self, port: str):
        self.port = port
        try:
            self.get_logger().info(f"GPS port (UI only) set to: {port}")
        except Exception:
            pass

    # ----------------- UI update -----------------

    def update_ui(self):
        """Send GPS data to QtBridge → JS HUD."""
        if not self.bridge:
            return

        try:
            # Consider data stale if we haven't heard from GPS recently
            now_sec = self.get_clock().now().nanoseconds / 1e9
            stale = (now_sec - self.last_update_time) > 3.0

            has_fix_flag = self.fix_type >= NavSatStatus.STATUS_FIX
            has_position = abs(self.latitude) > 0.0001 and abs(self.longitude) > 0.0001
            has_fix = (has_fix_flag or has_position) and not stale

            if has_fix:
                if self.num_satellites >= 8:
                    fix_str = f"3D Fix ({self.num_satellites} sats)"
                    can_proceed = True
                elif self.num_satellites >= 4:
                    fix_str = f"Weak ({self.num_satellites} sats)"
                    can_proceed = False
                else:
                    fix_str = "No Fix"
                    can_proceed = False
            else:
                fix_str = "No Fix"
                can_proceed = False

            self.signal_lost = not can_proceed

            # Push summary to JS HUD via QtBridge (queued to UI thread)
            if hasattr(self.bridge, "updateGPSData"):
                try:
                    QtCore.QMetaObject.invokeMethod(
                        self.bridge,
                        "updateGPSData",
                        QtCore.Qt.QueuedConnection,
                        QtCore.Q_ARG(str, fix_str),
                        QtCore.Q_ARG(float, 0.0 if stale else float(self.latitude)),
                        QtCore.Q_ARG(float, 0.0 if stale else float(self.longitude)),
                        QtCore.Q_ARG(float, 0.0 if stale else float(self.altitude)),
                        QtCore.Q_ARG(float, 0.0 if stale else float(self.heading)),
                        QtCore.Q_ARG(int, 0 if stale else int(self.num_satellites)),
                        QtCore.Q_ARG(float, 99.9 if stale else float(self.hdop)),
                        QtCore.Q_ARG(bool, False if stale else bool(can_proceed)),
                    )
                except Exception as e:
                    self.get_logger().warn(f"updateGPSData invoke failed: {e}")

            # Draw path point on map via QtBridge slot (also queued) only if fresh
            if has_position and not stale and hasattr(self.bridge, "addActualPathPointJS"):
                try:
                    QtCore.QMetaObject.invokeMethod(
                        self.bridge,
                        "addActualPathPointJS",
                        QtCore.Qt.QueuedConnection,
                        QtCore.Q_ARG(float, float(self.latitude)),
                        QtCore.Q_ARG(float, float(self.longitude)),
                    )
                except Exception as e:
                    try:
                        self.get_logger().warn(f"path point invoke failed: {e}")
                    except Exception:
                        print("GPSController path point invoke failed:", e)

        except Exception as e:
            try:
                self.get_logger().error(f"Failed to update UI: {e}")
            except Exception:
                print("GPSController update_ui error:", e)


def main():
    rclpy.init()
    controller = GPSController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
            pass
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
