#!/usr/bin/env python3
import argparse
import os
import sys
import time
import signal

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraWorker(Node):
    def __init__(self, args):
        super().__init__(args.node_name)
        self.device = args.device
        self.topic = args.topic
        self.width = int(args.width)
        self.height = int(args.height)
        self.fps = max(1, int(args.fps))
        self.mjpeg = bool(args.mjpeg)
        self.backend = (args.backend or "v4l2").strip().lower()
        self.jpeg_quality = int(args.jpeg_quality)
        self.warmup_frames = int(args.warmup_frames)
        self.read_timeout_ms = int(args.read_timeout_ms)
        self.open_timeout_ms = int(args.open_timeout_ms)
        self.frame_delay = 1.0 / float(self.fps)
        self._stop = False

        self.pub = self.create_publisher(CompressedImage, self.topic, 10)

    def _apply_timeouts(self, cap):
        for prop_name, value in (
            ("CAP_PROP_OPEN_TIMEOUT_MSEC", self.open_timeout_ms),
            ("CAP_PROP_READ_TIMEOUT_MSEC", self.read_timeout_ms),
        ):
            prop = getattr(cv2, prop_name, None)
            if prop is None:
                continue
            try:
                cap.set(prop, float(value))
            except Exception:
                pass

    def _build_gstreamer_pipeline(self, use_mjpeg: bool) -> str:
        if use_mjpeg:
            return (
                f"v4l2src device={self.device} io-mode=2 do-timestamp=true ! "
                f"image/jpeg, width={self.width}, height={self.height}, framerate={self.fps}/1 ! "
                "jpegdec ! videoconvert ! video/x-raw, format=BGR ! "
                "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
                "appsink drop=1 max-buffers=1 sync=false"
            )
        return (
            f"v4l2src device={self.device} io-mode=2 do-timestamp=true ! "
            f"video/x-raw, width={self.width}, height={self.height}, framerate={self.fps}/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            "appsink drop=1 max-buffers=1 sync=false"
        )

    def _open_camera(self):
        cap = None
        if self.backend == "gstreamer" and self.device.startswith("/dev/video"):
            pipeline = self._build_gstreamer_pipeline(self.mjpeg)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap is not None and cap.isOpened():
                self._apply_timeouts(cap)
                return cap
            if cap:
                try:
                    cap.release()
                except Exception:
                    pass
            if self.mjpeg:
                pipeline = self._build_gstreamer_pipeline(False)
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if cap is not None and cap.isOpened():
                    self.mjpeg = False
                    self._apply_timeouts(cap)
                    self.get_logger().warn("[Worker] GStreamer MJPEG failed; using raw YUYV.")
                    return cap
            self.backend = "v4l2"
            self.get_logger().warn("[Worker] GStreamer failed; falling back to V4L2.")

        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(self.device)
        if cap is not None and cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if self.mjpeg:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
                cap.set(cv2.CAP_PROP_FPS, float(self.fps))
                self._apply_timeouts(cap)
            except Exception:
                pass
        return cap

    def run(self):
        failures = 0
        cap = self._open_camera()
        if cap is None or not cap.isOpened():
            self.get_logger().error(f"[Worker] Failed to open {self.device}. Retrying...")
            time.sleep(0.5)
            cap = self._open_camera()
            if cap is None or not cap.isOpened():
                self.get_logger().error(f"[Worker] Camera still unavailable: {self.device}")
                return

        warmup_left = self.warmup_frames
        while rclpy.ok() and not self._stop:
            ret, frame = cap.read()
            if not ret:
                failures += 1
                time.sleep(0.05)
                if failures >= 30:
                    try:
                        cap.release()
                    except Exception:
                        pass
                    if self.mjpeg:
                        self.mjpeg = False
                        self.get_logger().warn("[Worker] Read failures with MJPEG; retrying with YUYV.")
                    cap = self._open_camera()
                    failures = 0
                    if cap is None or not cap.isOpened():
                        self.get_logger().warn("[Worker] Reopen failed; retrying soon...")
                        time.sleep(0.5)
                continue
            failures = 0
            if warmup_left > 0:
                warmup_left -= 1
                time.sleep(self.frame_delay)
                continue
            try:
                ok, enc = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
                )
            except Exception:
                ok, enc = False, None
            if not ok or enc is None:
                time.sleep(self.frame_delay)
                continue
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = enc.tobytes()
            try:
                self.pub.publish(msg)
            except Exception:
                break
            time.sleep(self.frame_delay)

        try:
            cap.release()
        except Exception:
            pass


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--node-name", default="outnav_camera_worker")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--jpeg-quality", type=int, default=70)
    parser.add_argument("--warmup-frames", type=int, default=5)
    parser.add_argument("--backend", default="v4l2")
    parser.add_argument("--read-timeout-ms", type=int, default=2000)
    parser.add_argument("--open-timeout-ms", type=int, default=2000)
    parser.add_argument("--mjpeg", action="store_true")
    return parser.parse_args()


def main():
    args = _parse_args()
    rclpy.init(args=None)
    node = CameraWorker(args)
    def _handle_stop(_sig, _frame):
        node._stop = True
    signal.signal(signal.SIGTERM, _handle_stop)
    signal.signal(signal.SIGINT, _handle_stop)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
