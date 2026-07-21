#!/usr/bin/env python3
"""Keep the latest aligned RealSense RGB, depth and intrinsics as one pair."""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RealSenseImageBridge(Node):
    def __init__(self, output_dir: Path, pair_max_delta_sec: float) -> None:
        super().__init__("lite3_realsense_qnn_image_bridge")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.color_output = output_dir / "latest.jpg"
        self.depth_output = output_dir / "latest_depth.npy"
        self.meta_output = output_dir / "latest_meta.json"
        self.pair_max_delta_sec = pair_max_delta_sec
        self.frames = 0
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_depth_encoding = None
        self.camera_info = None
        self.create_subscription(Image, "/camera/realsense/color/image_raw", self._on_color, 10)
        self.create_subscription(Image, "/camera/realsense/aligned_depth_to_color/image_raw", self._on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/realsense/color/camera_info", self._on_camera_info, 10)
        self.get_logger().info("RealSense RGB/depth bridge output=%s" % output_dir)

    @staticmethod
    def _stamp_sec(msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding not in ("16UC1", "32FC1"):
            self.get_logger().warning("unsupported depth encoding: %s" % msg.encoding)
            return
        dtype = np.uint16 if msg.encoding == "16UC1" else np.float32
        pixels = np.frombuffer(msg.data, dtype=dtype)
        expected = msg.height * msg.width
        if pixels.size < expected:
            return
        self.latest_depth = pixels[:expected].reshape((msg.height, msg.width)).copy()
        self.latest_depth_stamp = self._stamp_sec(msg)
        self.latest_depth_encoding = msg.encoding

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = {
            "width": int(msg.width), "height": int(msg.height),
            "fx": float(msg.k[0]), "fy": float(msg.k[4]),
            "cx": float(msg.k[2]), "cy": float(msg.k[5]),
        }

    def _on_color(self, msg: Image) -> None:
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warning("unsupported encoding: %s" % msg.encoding)
            return
        if self.latest_depth is None or self.latest_depth_stamp is None or self.camera_info is None:
            return
        color_stamp = self._stamp_sec(msg)
        delta = abs(color_stamp - self.latest_depth_stamp)
        if delta > self.pair_max_delta_sec:
            return
        pixels = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if pixels.size < expected:
            return
        frame = pixels[:expected].reshape((msg.height, msg.step))[:, : msg.width * 3]
        frame = frame.reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            return
        temporary = self.color_output.with_suffix(".tmp.jpg")
        temporary.write_bytes(encoded.tobytes())
        os.replace(temporary, self.color_output)
        depth_temp = self.depth_output.with_suffix(".tmp.npy")
        with depth_temp.open("wb") as stream:
            np.save(stream, self.latest_depth, allow_pickle=False)
        os.replace(depth_temp, self.depth_output)
        metadata = dict(self.camera_info)
        metadata.update({
            "sequence": self.frames + 1,
            "color_stamp_sec": color_stamp,
            "depth_stamp_sec": self.latest_depth_stamp,
            "pair_delta_sec": delta,
            "depth_encoding": self.latest_depth_encoding,
            "depth_scale_to_m": 0.001 if self.latest_depth_encoding == "16UC1" else 1.0,
            "written_at_monotonic": time.monotonic(),
        })
        meta_temp = self.meta_output.with_suffix(".tmp.json")
        meta_temp.write_text(json.dumps(metadata, separators=(",", ":")))
        os.replace(meta_temp, self.meta_output)
        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info("RealSense pair forwarded frames=%d delta=%.4fs" % (self.frames, delta))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="/tmp/lite3_realsense_qnn")
    parser.add_argument("--pair-max-delta-sec", type=float, default=0.05)
    args = parser.parse_args()
    if args.pair_max_delta_sec <= 0.0:
        raise SystemExit("--pair-max-delta-sec must be positive")
    rclpy.init()
    node = RealSenseImageBridge(Path(args.output_dir), args.pair_max_delta_sec)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
