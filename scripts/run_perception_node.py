#!/usr/bin/env python3
"""Run the Perception node (image topic → detections).

With ROS2 (normal deployment — camera node must be running separately):
  PYTHONPATH=src python3 scripts/run_perception_node.py --ros

Local smoke without ROS (feeds a dummy JPEG once):
  PYTHONPATH=src python3 scripts/run_perception_node.py --smoke
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--target-fps", type=float, default=0.0)
    parser.add_argument(
        "--image-topic",
        default="/lite3/camera/image/compressed",
    )
    parser.add_argument(
        "--result-topic",
        default="/lite3/perception/result",
    )
    args = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    src = root / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))

    if args.ros:
        from lite3_ros.perception_rclpy_node import (
            PerceptionRosTopics,
            spin_perception_node,
        )

        spin_perception_node(
            topics=PerceptionRosTopics(
                image_topic=args.image_topic,
                result_topic=args.result_topic,
            ),
            target_fps=args.target_fps,
        )
        return 0

    from lite3_perception.perception_node import PerceptionNode, PerceptionNodeConfig
    from lite3_perception.udp_camera_receiver import CameraFrame

    node = PerceptionNode(PerceptionNodeConfig(target_fps=args.target_fps))

    if args.smoke:
        # Minimal JPEG (SOI+EOI) — detector is passthrough.
        frame = CameraFrame(
            jpeg_bytes=b"\xff\xd8\xff\xd9",
            timestamp_monotonic=time.monotonic(),
            width=1,
            height=1,
            sequence=1,
            source="smoke",
        )
        result = node.process_frame(frame)
        print(result.to_json())
        print(json.dumps(node.health(), separators=(",", ":")))
        return 0

    print(
        "Use --ros to subscribe to the camera topic, or --smoke for a dry run.\n"
        "Camera side: PYTHONPATH=src python3 scripts/run_udp_camera_node.py --ros",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
