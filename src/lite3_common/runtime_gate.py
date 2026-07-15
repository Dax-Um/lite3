"""Runtime preflight guards shared by real-robot entrypoints."""

from __future__ import annotations

from pathlib import Path

from lite3_common.config import NETWORK_CONFIG_PATH, SAFETY_LIMITS_CONFIG_PATH, resolve_workspace_root

REQUIRED_WORKSPACE_PATHS = (
    NETWORK_CONFIG_PATH,
    SAFETY_LIMITS_CONFIG_PATH,
)
REQUIRED_REPO_PATHS = (
    Path("scripts/patrol_preflight_check.py"),
)


class RuntimeGateError(RuntimeError):
    """Raised when a real-robot entrypoint is missing a safety prerequisite."""


def verify_runtime_gate(
    project_root: str | Path,
    *,
    real_robot: bool,
    preflight_ok: bool,
) -> None:
    """Validate stage-01 assets and preflight state for an entrypoint.

    Dry-run modes need the documented config and preflight assets to exist, but
    do not require the interactive preflight command to have completed.
    """

    repo_root = Path(project_root)
    workspace_root = _workspace_root_for_repo(repo_root)
    missing = [
        str(path)
        for path in REQUIRED_WORKSPACE_PATHS
        if not (workspace_root / path).exists()
    ]
    missing.extend(
        str(path)
        for path in REQUIRED_REPO_PATHS
        if not (repo_root / path).exists()
    )
    if missing:
        raise RuntimeGateError(f"missing required stage-01 asset(s): {', '.join(missing)}")

    if real_robot and not preflight_ok:
        raise RuntimeGateError("preflight must complete successfully before real-robot motion")


def _workspace_root_for_repo(repo_root: Path) -> Path:
    return resolve_workspace_root(repo_root)
