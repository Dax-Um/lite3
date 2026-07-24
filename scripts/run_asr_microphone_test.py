#!/usr/bin/env python3
"""Launch the independent Lite3 QNN microphone-to-text test."""

from pathlib import Path
import sys

# Keep this launcher runnable from a checkout without installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lite3_asr.service import main


if __name__ == "__main__":
    raise SystemExit(main())
