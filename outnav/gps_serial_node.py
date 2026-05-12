#!/usr/bin/env python3
"""
GPS serial reader node for OutdoorNav.

Reads NMEA sentences from a UART (default /dev/ttyTHS1) and publishes:
- /fix (sensor_msgs/NavSatFix)
- /gps/heading (std_msgs/Float64)
"""

import os
import threading
import time
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

try:
    import serial
except Exception:  # pragma: no cover - handled at runtime
    serial = None


class GPSSerialNode(Node):
    def __init__(self, port="/dev/ttyTHS1", baudrate=9600):
        super().__init__("gps_serial_node")

        self._baudrate = baudrate
        self._desired_port = port
        self._port = None
        self._serial = None
        self._stop_event = threading.Event()
        self._port_lock = threading.Lock()
        self._enabled = True
        self._closing = False
        self._last_alt = 0.0
        self._last_hdop = 99.9
        self._last_fix_quality = 0
        self._last_error = None

        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.heading_pub = self.create_publisher(Float64, "/gps/heading", 10)
        self._settings_sub = self.create_subscription(
            String, "/outnav/settings", self._on_settings, 10
        )

        self.get_logger().info(f"GPSSerialNode starting on {port} @ {baudrate} baud")

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    # ----------------- Port handling -----------------
    def set_port(self, port: str):
        cleaned = (port or "").strip()
        if not cleaned:
            return
        if cleaned != "auto" and not cleaned.startswith("/dev/"):
            cleaned = f"/dev/{cleaned}"
        with self._port_lock:
            self._desired_port = cleaned
        self.get_logger().info(f"[GPS] Requested port change to {cleaned}")

    def set_enabled(self, enabled: bool):
        enabled = bool(enabled)
        with self._port_lock:
            if self._enabled == enabled:
                return
            self._enabled = enabled
        if not enabled:
            self._close_serial()
            self._log_once("[GPS] UART disabled; waiting for external GPS")
        else:
            self._last_error = None
            self._log_once("[GPS] UART enabled")

    def stop(self):
        self._closing = True
        self._stop_event.set()
        self._close_serial()
        if hasattr(self, "_thread") and self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _close_serial(self):
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _resolve_port(self, port: str) -> str:
        if port != "auto":
            return port
        for candidate in ("/dev/ttyTHS1", "/dev/ttyACM0", "/dev/ttyUSB0", "/dev/serial0"):
            if os.path.exists(candidate):
                return candidate
        return port

    def _open_serial(self, port: str) -> bool:
        if serial is None:
            self._log_once("[GPS] pyserial not available. Install python3-serial.")
            return False
        resolved = self._resolve_port(port)
        if resolved == "auto":
            self._log_once("[GPS] No GPS device found for auto port selection.")
            return False
        try:
            self._serial = serial.Serial(resolved, self._baudrate, timeout=0.5)
            self._port = resolved
            self._log_once(f"[GPS] Serial open on {resolved}")
            return True
        except Exception as exc:
            self._log_once(f"[GPS] Failed to open {resolved}: {exc}")
            self._serial = None
            return False

    def _log_once(self, message: str):
        if message == self._last_error:
            return
        self._last_error = message
        try:
            self.get_logger().warn(message)
        except Exception:
            print(message)

    def _on_settings(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if "gpsUartEnabled" in payload:
            self.set_enabled(bool(payload.get("gpsUartEnabled")))
        if "gpsPort" in payload:
            self.set_port(payload.get("gpsPort"))

    # ----------------- Read loop -----------------
    def _read_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            with self._port_lock:
                desired = self._desired_port
                enabled = self._enabled
            if not enabled:
                self._close_serial()
                time.sleep(0.2)
                continue
            if not self._serial or desired != self._port:
                self._close_serial()
                self._open_serial(desired)
                time.sleep(0.1)
                continue

            try:
                line = self._serial.readline()
            except Exception as exc:
                if self._stop_event.is_set() or self._closing:
                    break
                self._log_once(f"[GPS] Serial read error: {exc}")
                self._close_serial()
                time.sleep(0.2)
                continue

            if not line:
                continue
            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not text.startswith("$"):
                continue
            self._handle_nmea(text)

    # ----------------- NMEA parsing -----------------
    def _handle_nmea(self, sentence: str):
        body = sentence.split("*", 1)[0]
        parts = body.split(",")
        if not parts:
            return
        msg_type = parts[0].upper()

        if msg_type.endswith("GGA"):
            self._handle_gga(parts)
        elif msg_type.endswith("RMC"):
            self._handle_rmc(parts)

    def _handle_gga(self, parts):
        # $GxGGA,time,lat,NS,lon,EW,fix,sats,hdop,alt,M,...
        if len(parts) < 10:
            return
        lat = _parse_lat_lon(parts[2], parts[3], is_lat=True)
        lon = _parse_lat_lon(parts[4], parts[5], is_lat=False)
        fix_quality = _safe_int(parts[6])
        sats = _safe_int(parts[7])
        hdop = _safe_float(parts[8], default=99.9)
        alt = _safe_float(parts[9], default=0.0)

        if fix_quality <= 0 or lat is None or lon is None:
            return

        self._last_alt = alt
        self._last_hdop = hdop
        self._last_fix_quality = fix_quality
        self._publish_fix(lat, lon, alt, hdop, fix_quality)

    def _handle_rmc(self, parts):
        # $GxRMC,time,status,lat,NS,lon,EW,sog,cog,...
        if len(parts) < 9:
            return
        status = parts[2].upper() if parts[2] else "V"
        if status != "A":
            return
        lat = _parse_lat_lon(parts[3], parts[4], is_lat=True)
        lon = _parse_lat_lon(parts[5], parts[6], is_lat=False)
        course = _safe_float(parts[8], default=None)

        if lat is None or lon is None:
            return

        hdop = self._last_hdop
        alt = self._last_alt
        fix_quality = self._last_fix_quality or 1
        self._publish_fix(lat, lon, alt, hdop, fix_quality)

        if course is not None:
            msg = Float64()
            msg.data = float(course)
            self.heading_pub.publish(msg)

    def _publish_fix(self, lat, lon, alt, hdop, fix_quality):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.status.status = (
            NavSatStatus.STATUS_FIX if fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)

        if hdop and hdop > 0.0:
            var = float(hdop) ** 2
            msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)


def _safe_float(value, default=None):
    try:
        return float(value)
    except Exception:
        return default


def _safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return default


def _parse_lat_lon(value, hemi, is_lat):
    if not value:
        return None
    deg_len = 2 if is_lat else 3
    if len(value) < deg_len:
        return None
    try:
        deg = float(value[:deg_len])
        minutes = float(value[deg_len:])
    except Exception:
        return None
    coord = deg + (minutes / 60.0)
    if hemi in ("S", "W"):
        coord = -coord
    return coord


def main():
    rclpy.init()
    node = GPSSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
