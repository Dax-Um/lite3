#!/usr/bin/env python3
"""Live smoke test: receive Lite3 camera RTP/JPEG over UDP continuously.

Default matches motion-host push_udp.sh → IQ9 :5000.

Examples:
  PYTHONPATH=src python3 scripts/run_udp_camera_recv.py
  PYTHONPATH=src python3 scripts/run_udp_camera_recv.py --port 5000 --seconds 10
  PYTHONPATH=src python3 scripts/run_udp_camera_recv.py --save-dir /tmp/lite3_frames
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--port", type=int, default=5000, help="UDP port")
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Run duration (0 = until Ctrl-C)",
    )
    parser.add_argument(
        "--save-dir",
        default="",
        help="Optional directory to dump JPEG frames (every Nth frame)",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=30,
        help="Save every N completed frames when --save-dir is set",
    )
    parser.add_argument(
        "--payload-type",
        type=int,
        default=26,
        help="RTP payload type for JPEG (GStreamer default 26)",
    )
    args = parser.parse_args(argv)

    # Allow running from workspace root without install.
    root = Path(__file__).resolve().parents[1]
    src = root / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))

    from lite3_perception.udp_camera_receiver import UdpCameraConfig, UdpJpegCameraReceiver

    save_dir = Path(args.save_dir) if args.save_dir else None
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)

    config = UdpCameraConfig(
        bind_host=args.bind,
        port=args.port,
        payload_type=args.payload_type,
    )
    receiver = UdpJpegCameraReceiver(config)

    stop = False

    def _handle_sig(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    receiver.start()
    print(
        f"listening udp://{args.bind}:{args.port} "
        f"(payload_type={args.payload_type})",
        flush=True,
    )

    start = time.monotonic()
    last_report = start
    last_frames = 0
    saved = 0

    try:
        while not stop:
            if args.seconds > 0 and (time.monotonic() - start) >= args.seconds:
                break
            frame = receiver.wait_for_frame(timeout=0.5)
            if frame is not None and save_dir is not None:
                if receiver.stats.frames_completed % max(1, args.save_every) == 0:
                    path = save_dir / f"frame_{receiver.stats.frames_completed:06d}.jpg"
                    path.write_bytes(frame.jpeg_bytes)
                    saved += 1

            now = time.monotonic()
            if now - last_report >= 1.0:
                stats = receiver.stats
                delta_f = stats.frames_completed - last_frames
                fps = delta_f / (now - last_report)
                age = (
                    None
                    if stats.last_frame_monotonic is None
                    else now - stats.last_frame_monotonic
                )
                print(
                    " ".join(
                        [
                            f"frames={stats.frames_completed}",
                            f"fps={fps:.1f}",
                            f"bytes={stats.bytes_received}",
                            f"backend={stats.backend}",
                            f"age={age if age is not None else -1:.3f}",
                            f"err={stats.last_error!r}",
                        ]
                    ),
                    flush=True,
                )
                last_report = now
                last_frames = stats.frames_completed
    finally:
        receiver.stop()

    stats = receiver.stats
    print(
        f"done frames={stats.frames_completed} bytes={stats.bytes_received} saved={saved}",
        flush=True,
    )
    return 0 if stats.frames_completed > 0 or args.seconds == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
