"""PipeWire microphone service that prints QNN ASR text to logs only."""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from .backend import AsrBackendError, QnnSenseVoiceBackend
from .config import AsrConfig
from .vad import EnergyVad

LOG = logging.getLogger("lite3_asr")
# Pin capture to the BOYA PipeWire source rather than the speaker microphone.
# PipeWire keeps the source name stable across its internal node-ID changes and
# converts the receiver's native 24-bit PCM to the service's 16-bit stream.
DEFAULT_TARGET = "alsa_input.usb-Shenzhen_jiayz_photo_industrial_ltd_BOYALINK_112004030501001585002101FFFFFFfg-01.analog-stereo"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Independent Lite3 microphone-to-text QNN ASR test")
    parser.add_argument("--asset-root", type=Path, required=True)
    parser.add_argument("--capture-backend", choices=("pipewire", "alsa"), default="pipewire")
    parser.add_argument("--target", default=DEFAULT_TARGET, help="PipeWire source name or ALSA device")
    parser.add_argument("--language", default="en")
    parser.add_argument("--vad-onset-rms", type=float, default=50.0)
    parser.add_argument("--vad-release-rms", type=float, default=40.0)
    parser.add_argument(
        "--silence-chunks", type=int, default=5,
        help="80 ms audio chunks required to finish an utterance (default: 5)",
    )
    parser.add_argument("--check", action="store_true", help="Validate assets/dependencies without recording")
    parser.add_argument("--result-jsonl", type=Path, help="Append each non-empty transcription as one JSON event.")
    return parser


def append_event(path: Path, record: dict) -> None:
    """Atomically append a small result record for tailing consumers."""
    path.parent.mkdir(parents=True, exist_ok=True)
    record = json.dumps({"timestamp": time.time(), **record}, ensure_ascii=False, separators=(",", ":")) + "\n"
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o640)
    try:
        os.write(fd, record.encode("utf-8"))
        os.fsync(fd)
    finally:
        os.close(fd)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config = AsrConfig(
        asset_root=args.asset_root.resolve(),
        pipewire_target=args.target,
        language=args.language,
        vad_onset_rms=args.vad_onset_rms,
        vad_release_rms=args.vad_release_rms,
        silence_chunks=args.silence_chunks,
    )
    backend = QnnSenseVoiceBackend(config)
    problems = backend.diagnose()
    if problems:
        for problem in problems:
            LOG.error("preflight: %s", problem)
        return 2
    LOG.info("preflight passed; provider=qnn, asset_root=%s", config.asset_root)
    if args.check:
        return 0

    stop = False
    capture: subprocess.Popen[bytes] | None = None

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop, capture
        stop = True
        if capture is not None and capture.poll() is None:
            capture.terminate()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    vad = EnergyVad(config)
    if args.capture_backend == "alsa":
        command = [
            "arecord", "-D", config.pipewire_target, "--rate", str(config.sample_rate),
            "--channels", "1", "--format", "S16_LE", "--file-type", "raw", "-",
        ]
    else:
        command = [
            "pw-record", "--latency", "16ms", "--rate", str(config.sample_rate),
            "--channels", "1", "--format", "s16", "--target", config.pipewire_target, "-",
        ]
    LOG.info(
        "listening: backend=%s target=%s onset=%.1f release=%.1f end_silence=%dms",
        args.capture_backend, args.target, args.vad_onset_rms, args.vad_release_rms,
        args.silence_chunks * config.chunk_frames * 1000 // config.sample_rate,
    )
    capture_env = os.environ.copy()
    capture_env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    capture = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        # PipeWire errors must not fill an unread pipe and stall audio capture.
        stderr=subprocess.DEVNULL,
        env=capture_env,
    )
    try:
        while not stop:
            assert capture.stdout is not None
            pcm = capture.stdout.read(config.chunk_frames * 2)
            if len(pcm) != config.chunk_frames * 2:
                raise RuntimeError("PipeWire capture ended before a complete audio frame arrived")
            utterance = vad.feed(pcm)
            if utterance is None:
                continue
            request_id = str(uuid.uuid4())
            if args.result_jsonl is not None:
                append_event(args.result_jsonl, {"type": "utterance_ended", "id": request_id})
            LOG.info("speech complete: %.2fs; QNN inference started", len(utterance) / 2 / config.sample_rate)
            try:
                result = backend.transcribe(utterance)
            except AsrBackendError as exc:
                LOG.error("inference failed: %s", exc)
                continue
            if config.language != "auto" and result.language not in (config.language, "nospeech"):
                LOG.info("text ignored: detected_language=%s target_language=%s text=%r", result.language, config.language, result.text)
            else:
                LOG.info("TEXT: %s", result.text or "(no speech)")
                if args.result_jsonl is not None and result.text.strip():
                    append_event(args.result_jsonl, {"type": "transcript", "id": request_id,
                                                     "text": result.text.strip(), "language": result.language})
    finally:
        if capture.poll() is None:
            capture.terminate()
        try:
            capture.wait(timeout=5)
        except subprocess.TimeoutExpired:
            capture.kill()
            capture.wait()
    return 0


if __name__ == "__main__":
    sys.exit(main())
