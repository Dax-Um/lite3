import sys
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _write_stage_01_assets(workspace_root: Path) -> Path:
    repo_root = workspace_root / "lite3"
    (workspace_root / "configs/lite3").mkdir(parents=True)
    (workspace_root / "configs/lite3/network.yaml").write_text("lite3: {}\n", encoding="utf-8")
    (workspace_root / "configs/lite3/safety_limits.yaml").write_text(
        "motion_limits: {}\n",
        encoding="utf-8",
    )
    (repo_root / "scripts").mkdir(parents=True)
    (repo_root / "scripts/patrol_preflight_check.py").write_text(
        "#!/usr/bin/env python3\n",
        encoding="utf-8",
    )
    return repo_root


def test_runtime_gate_allows_dry_run_when_stage_01_assets_exist(tmp_path):
    sys.path.insert(0, str(PROJECT_ROOT))
    from lite3_common.runtime_gate import verify_runtime_gate

    repo_root = _write_stage_01_assets(tmp_path)

    verify_runtime_gate(repo_root, real_robot=False, preflight_ok=False)


def test_runtime_gate_requires_stage_01_assets(tmp_path):
    sys.path.insert(0, str(PROJECT_ROOT))
    from lite3_common.runtime_gate import RuntimeGateError, verify_runtime_gate

    with pytest.raises(RuntimeGateError, match="missing required stage-01 asset"):
        verify_runtime_gate(tmp_path, real_robot=False, preflight_ok=False)


def test_runtime_gate_requires_preflight_before_real_robot_motion(tmp_path):
    sys.path.insert(0, str(PROJECT_ROOT))
    from lite3_common.runtime_gate import RuntimeGateError, verify_runtime_gate

    repo_root = _write_stage_01_assets(tmp_path)

    with pytest.raises(RuntimeGateError, match="preflight"):
        verify_runtime_gate(repo_root, real_robot=True, preflight_ok=False)

    verify_runtime_gate(repo_root, real_robot=True, preflight_ok=True)
