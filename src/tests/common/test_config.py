from pathlib import Path

import pytest

from lite3_common.config import load_lite3_network_config, load_motion_limits_config
from lite3_common.types import MotionLimits


EXAMPLE_MOTION_HOST = "203.0.113.10"
EXAMPLE_MANAGEMENT_HOST = "198.51.100.10"
EXAMPLE_ROBOT_SIDE_HOST = "198.51.100.11"
EXAMPLE_COMMAND_PORT = 12000


def write_configs(root: Path) -> None:
    (root / "configs/lite3").mkdir(parents=True)
    (root / "configs/lite3/network.yaml").write_text(
        "\n".join(
            [
                "lite3:",
                f'  motion_host_ip: "{EXAMPLE_MOTION_HOST}"',
                f"  motion_host_command_port: {EXAMPLE_COMMAND_PORT}",
                f'  iq9_management_ip: "{EXAMPLE_MANAGEMENT_HOST}"',
                f'  iq9_robot_side_ip: "{EXAMPLE_ROBOT_SIDE_HOST}"',
            ]
        ),
        encoding="utf-8",
    )
    (root / "configs/lite3/safety_limits.yaml").write_text(
        "\n".join(
            [
                "motion_limits:",
                "  max_vx_mps: 0.10",
                "  max_vy_mps: 0.05",
                "  max_wz_radps: 0.20",
            ]
        ),
        encoding="utf-8",
    )


def test_load_lite3_network_config_from_workspace_root(tmp_path):
    write_configs(tmp_path)

    config = load_lite3_network_config(tmp_path)

    assert config.motion_host_ip == EXAMPLE_MOTION_HOST
    assert config.motion_host_command_port == EXAMPLE_COMMAND_PORT
    assert config.iq9_management_ip == EXAMPLE_MANAGEMENT_HOST
    assert config.iq9_robot_side_ip == EXAMPLE_ROBOT_SIDE_HOST


def test_load_lite3_network_config_from_repo_root(tmp_path):
    write_configs(tmp_path)
    repo_root = tmp_path / "lite3"
    repo_root.mkdir()

    config = load_lite3_network_config(repo_root)

    assert config.motion_host_ip == EXAMPLE_MOTION_HOST


def test_load_motion_limits_config_from_workspace_root(tmp_path):
    write_configs(tmp_path)

    limits = load_motion_limits_config(tmp_path)

    assert limits == MotionLimits(max_vx_mps=0.10, max_vy_mps=0.05, max_wz_radps=0.20)


def test_load_config_rejects_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError, match="network.yaml"):
        load_lite3_network_config(tmp_path)
