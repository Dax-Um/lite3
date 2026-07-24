"""Tail Lite3 voice events and send speech requests to the host audio service."""
from __future__ import annotations

import argparse
import json
import os
import signal
import time
from pathlib import Path


def request_speech(request_dir: Path, request_id: str, text: str, *, action: str = "speak", cue: str | None = None) -> None:
    request_dir.mkdir(parents=True, exist_ok=True)
    target = request_dir / "{}-voice-speak-{}.json".format(int(time.time() * 1000), request_id or "unknown")
    temporary = target.with_suffix(".part")
    payload = {"action": action, "event_id": request_id}
    if action == "speak":
        payload["text"] = text
    if cue is not None:
        payload["cue"] = cue
    temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    os.replace(temporary, target)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--events", required=True, type=Path)
    parser.add_argument("--request-dir", required=True, type=Path)
    args = parser.parse_args()
    stopping = False
    def stop(*_args: object) -> None:
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with args.events.open("a+", encoding="utf-8") as stream:
        stream.seek(0, os.SEEK_END)
        while not stopping:
            line = stream.readline()
            if not line:
                time.sleep(0.1)
                continue
            try:
                event = json.loads(line)
                if event.get("type") == "voice_bark":
                    request_speech(args.request_dir, str(event.get("request_id", "")), "", action="play_once", cue="dog_bark")
                elif event.get("type") == "voice_tts" and str(event.get("text", "")).strip():
                    request_speech(args.request_dir, str(event.get("request_id", "")), str(event["text"]))
            except (ValueError, OSError):
                continue
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
