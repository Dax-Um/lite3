"""A deterministic energy VAD with pre-roll buffering."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .config import AsrConfig


@dataclass
class EnergyVad:
    config: AsrConfig
    _pre_roll: deque[bytes] = field(init=False)
    _speech: list[bytes] = field(default_factory=list, init=False)
    _in_speech: bool = field(default=False, init=False)
    _silence_chunks: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self._pre_roll = deque(maxlen=self.config.pre_roll_chunks)

    def feed(self, pcm: bytes) -> bytes | None:
        """Return exactly one completed utterance, or ``None``.

        This method is intentionally called by one capture loop only; no queue or
        shared VAD state exists, which prevents overlapping utterances.
        """
        rms = self._rms(pcm)
        if not self._in_speech:
            self._pre_roll.append(pcm)
            if rms > self.config.vad_onset_rms:
                self._in_speech = True
                self._silence_chunks = 0
                self._speech = list(self._pre_roll)
                self._pre_roll.clear()
            return None

        self._speech.append(pcm)
        self._silence_chunks = self._silence_chunks + 1 if rms < self.config.vad_release_rms else 0
        is_complete = (
            self._silence_chunks >= self.config.silence_chunks
            or len(self._speech) >= self.config.max_speech_chunks
        )
        if not is_complete:
            return None

        utterance = b"".join(self._speech)
        valid = len(self._speech) >= self.config.min_speech_chunks
        self.reset()
        return utterance if valid else None

    def reset(self) -> None:
        self._pre_roll.clear()
        self._speech.clear()
        self._in_speech = False
        self._silence_chunks = 0

    @staticmethod
    def _rms(pcm: bytes) -> float:
        samples = np.frombuffer(pcm, dtype=np.int16)
        if not len(samples):
            return 0.0
        return float(np.sqrt(np.mean(samples.astype(np.float32) ** 2)))
