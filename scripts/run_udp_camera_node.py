#!/usr/bin/env python3
"""Run the UDP camera node (GStreamer receive → frames).

Without ROS (smoke test / save JPEGs):
  cd /home/ubuntu/workspace/lite3
  PYTHONPATH=src python3 scripts/run_udp_camera_node.py --seconds 10

With ROS2 (publishes /lite3/camera/image/compressed):
  PYTHONPATH=src python3 scripts/run_udp_camera_node.py --ros
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
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--payload-type", type=int, default=26)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--save-dir", default="")
    parser.add_argument("--save-every", type=int, default=30)
    parser.add_argument("--ros", action="store_true")
    args = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    src = root / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))

    if args.ros:
        from lite3_ros.udp_camera_rclpy_node import spin_udp_camera_node

        spin_udp_camera_node(
            bind_host=args.bind,
            udp_port=args.port,
            payload_type=args.payload_type,
        )
        return 0

    from lite3_perception.camera_node import CameraNodeConfig, UdpCameraNode
    from lite3_perception.udp_camera_receiver import UdpCameraConfig

    save_dir = Path(args.save_dir) if args.save_dir else None
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)

    saved = {"n": 0}

    def on_frame(frame) -> None:
        if save_dir is None:
            return
        if frame.sequence % max(1, args.save_every) != 0:
            return
        path = save_dir / f"frame_{frame.sequence:06d}.jpg"
        path.write_bytes(frame.jpeg_bytes)
        saved["n"] += 1

    node = UdpCameraNode(
        CameraNodeConfig(
            udp=UdpCameraConfig(
                bind_host=args.bind,
                port=args.port,
                payload_type=args.payload_type,
            )
        ),
        on_frame=on_frame,
    )

    stop = False

    def _sig(_s, _f) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    node.start()
    print(
        f"udp camera node listening {args.bind}:{args.port} "
        f"backend={node.receiver.stats.backend}",
        flush=True,
    )
    start = time.monotonic()
    last = start
    try:
        while not stop:
            if args.seconds > 0 and (time.monotonic() - start) >= args.seconds:
                break
            time.sleep(0.5)
            now = time.monotonic()
            if now - last >= 1.0:
                print(json.dumps(node.health(), separators=(",", ":")), flush=True)
                last = now
    finally:
        node.stop()

    print(json.dumps(node.health(), separators=(",", ":")), flush=True)
    print(f"saved={saved['n']}", flush=True)
    return 0 if node.frames_forwarded > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
