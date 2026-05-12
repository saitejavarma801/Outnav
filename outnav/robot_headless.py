#!/usr/bin/env python3
import logging
import os

import rclpy
from rclpy.executors import MultiThreadedExecutor

from logging_utils import setup_logging
from ros_node import OutdoorNavNode
from gps_serial_node import GPSSerialNode


def main():
    log_path = setup_logging()
    logging.info("[Robot] Headless boot -> %s", log_path)

    rclpy.init(args=None)
    node = OutdoorNavNode()
    node.set_device_mode("robot")

    gps_port = os.getenv("OUTNAV_GPS_PORT", "/dev/ttyTHS1")
    gps_baud = int(os.getenv("OUTNAV_GPS_BAUD", "9600"))
    gps_node = GPSSerialNode(port=gps_port, baudrate=gps_baud)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(gps_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            gps_node.stop()
        except Exception:
            pass
        node.destroy_node()
        gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
