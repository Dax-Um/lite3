"""Serialized QNN inference backend for the bundled SenseVoice model."""

from __future__ import annotations

import json
import logging
import os
import shutil
import subprocess
import tempfile
import threading
import wave
from dataclasses import dataclass

from .config import AsrConfig

LOG = logging.getLogger(__name__)


class AsrBackendError(RuntimeError):
    """The QNN backend could not load or complete an inference."""


@dataclass(frozen=True)
class Transcription:
    text: str
    language: str


class QnnSenseVoiceBackend:
    """Runs one QNN inference at a time to protect the HTP runtime."""

    def __init__(self, config: AsrConfig) -> None:
        self._config = config
        self._inference_lock = threading.Lock()

    def diagnose(self) -> list[str]:
        problems = [f"missing asset: {path}" for path in self._config.missing_assets()]
        if problems:
            return problems
        ldd = shutil.which("ldd")
        if ldd:
            env = os.environ.copy()
            env["LD_LIBRARY_PATH"] = str(self._config.runtime_lib_dir) + ":" + env.get("LD_LIBRARY_PATH", "")
            result = subprocess.run(
                [ldd, str(self._config.executable)], text=True, capture_output=True, check=False, env=env
            )
            problems.extend(line.strip() for line in result.stdout.splitlines() if "not found" in line)
        return problems

    def transcribe(self, pcm: bytes) -> Transcription:
        problems = self.diagnose()
        if problems:
            raise AsrBackendError("QNN backend is not runnable: " + "; ".join(problems))

        with self._inference_lock:
            return self._transcribe_locked(pcm)

    def _transcribe_locked(self, pcm: bytes) -> Transcription:
        min_samples = int(self._config.sample_rate * self._config.min_asr_seconds)
        if len(pcm) // 2 < min_samples:
            pcm += b"\x00" * ((min_samples - len(pcm) // 2) * 2)
        with tempfile.NamedTemporaryFile(prefix="lite3_asr_", suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name
        try:
            self._write_wav(wav_path, pcm)
            env = os.environ.copy()
            env["ADSP_LIBRARY_PATH"] = "/usr/lib/rfsa/adsp"
            env["LD_LIBRARY_PATH"] = (
                f"{self._config.runtime_lib_dir}:/usr/lib:" + env.get("LD_LIBRARY_PATH", "")
            )
            command = [
                str(self._config.executable),
                f"--sense-voice-model={self._config.model}",
                f"--sense-voice.qnn-context-binary={self._config.context}",
                "--sense-voice.qnn-backend-lib=/usr/lib/libQnnHtp.so",
                "--sense-voice.qnn-system-lib=/usr/lib/libQnnSystem.so",
                f"--sense-voice-language={self._config.language}",
                "--provider=qnn",
                f"--tokens={self._config.tokens}",
                wav_path,
            ]
            result = subprocess.run(
                command,
                text=True,
                capture_output=True,
                timeout=self._config.inference_timeout_seconds,
                env=env,
                check=False,
            )
            if result.returncode:
                raise AsrBackendError(result.stderr.strip() or f"ASR exited with {result.returncode}")
            return self._parse_result(result.stdout + "\n" + result.stderr)
        except subprocess.TimeoutExpired as exc:
            raise AsrBackendError("QNN inference timed out") from exc
        finally:
            try:
                os.unlink(wav_path)
            except FileNotFoundError:
                pass

    def _parse_result(self, output: str) -> Transcription:
        for line in output.splitlines():
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                result = json.loads(line)
            except json.JSONDecodeError:
                continue
            language = str(result.get("lang", "")).strip("<|>")
            text = str(result.get("text", "")).strip()
            if language == "nospeech":
                return Transcription(text="", language=language)
            return Transcription(text=text, language=language)
        raise AsrBackendError("ASR completed without a JSON transcription result")

    def _write_wav(self, path: str, pcm: bytes) -> None:
        with wave.open(path, "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(self._config.sample_rate)
            wav.writeframes(pcm)
