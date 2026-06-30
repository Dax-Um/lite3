"""Configuration loading helpers for the Lite3 patrol workspace layout."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from lite3_common.types import MotionLimits


NETWORK_CONFIG_PATH = Path("configs/lite3/network.yaml")
SAFETY_LIMITS_CONFIG_PATH = Path("configs/lite3/safety_limits.yaml")


@dataclass(frozen=True)
class Lite3NetworkConfig:
    motion_host_ip: str
    motion_host_command_port: int
    iq9_management_ip: str
    iq9_robot_side_ip: str
    iq9_robot_side_interface: str = "end0"
    iq9_robot_side_ip_mode: str = "static"


def load_lite3_network_config(workspace_root: str | Path) -> Lite3NetworkConfig:
    root = resolve_workspace_root(workspace_root)
    data = _load_yaml(root / NETWORK_CONFIG_PATH)
    lite3 = _mapping(data.get("lite3"), "lite3")
    return Lite3NetworkConfig(
        motion_host_ip=str(lite3["motion_host_ip"]),
        motion_host_command_port=int(lite3["motion_host_command_port"]),
        iq9_management_ip=str(lite3["iq9_management_ip"]),
        iq9_robot_side_ip=str(lite3["iq9_robot_side_ip"]),
        iq9_robot_side_interface=str(lite3.get("iq9_robot_side_interface", "end0")),
        iq9_robot_side_ip_mode=str(lite3.get("iq9_robot_side_ip_mode", "static")),
    )


def load_motion_limits_config(workspace_root: str | Path) -> MotionLimits:
    root = resolve_workspace_root(workspace_root)
    data = _load_yaml(root / SAFETY_LIMITS_CONFIG_PATH)
    motion_limits = _mapping(data.get("motion_limits"), "motion_limits")
    return MotionLimits(
        max_vx_mps=float(motion_limits["max_vx_mps"]),
        max_vy_mps=float(motion_limits["max_vy_mps"]),
        max_wz_radps=float(motion_limits["max_wz_radps"]),
    )


def resolve_workspace_root(root: str | Path) -> Path:
    path = Path(root)
    if (path / "configs").exists():
        return path
    if (path.parent / "configs").exists():
        return path.parent
    return path


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return _mapping(data, str(path))


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value
