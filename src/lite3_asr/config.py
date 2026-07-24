"""Configuration and asset validation for the QNN ASR service."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class AsrConfig:
    asset_root: Path
    pipewire_target: str
    language: str = "en"
    sample_rate: int = 16_000
    chunk_frames: int = 1_280
    vad_onset_rms: float = 50.0
    vad_release_rms: float = 40.0
    silence_chunks: int = 5
    min_speech_chunks: int = 4
    max_speech_chunks: int = 300
    pre_roll_chunks: int = 10
    min_asr_seconds: float = 3.0
    inference_timeout_seconds: float = 20.0

    @property
    def executable(self) -> Path:
        return self.asset_root / "bin" / "sherpa-onnx-offline"

    @property
    def runtime_lib_dir(self) -> Path:
        """Private shared libraries shipped beside the ASR assets."""
        return self.asset_root / "lib"

    @property
    def model(self) -> Path:
        return self.asset_root / "model" / "libmodel_v4.so"

    @property
    def context(self) -> Path:
        return self.asset_root / "model" / "qnn" / "model_v4_context.bin"

    @property
    def tokens(self) -> Path:
        return self.asset_root / "model" / "tokens.txt"

    def missing_assets(self) -> list[Path]:
        return [path for path in (self.executable, self.model, self.context, self.tokens) if not path.is_file()]
