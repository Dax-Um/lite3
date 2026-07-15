#!/usr/bin/env python3
"""Check or restore the last-known-good Lite3 waypoint Nav2 parameters."""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import tempfile
import time
from pathlib import Path
from typing import Dict, List, Tuple


DEFAULT_PATHS = (
    Path("/home/ysc/lite_cog_ros2/nav/src/dr_nav2/config/lite_nav2.yaml"),
    Path("/home/ysc/lite_cog_ros2/nav/install/dr_nav2/share/dr_nav2/config/lite_nav2.yaml"),
)

EXPECTED = {
    ("planner_server", "GridBased", "tolerance"): "1.0",
    ("controller_server", "", "controller_frequency"): "5.0",
    ("controller_server", "progress_checker", "required_movement_radius"): "0.5",
    ("controller_server", "progress_checker", "movement_time_allowance"): "10.0",
    ("controller_server", "goal_checker", "xy_goal_tolerance"): "0.3",
    ("controller_server", "goal_checker", "yaw_goal_tolerance"): "0.25",
    ("controller_server", "FollowPath", "min_vel_x"): "0.0",
    ("controller_server", "FollowPath", "max_vel_x"): "1.0",
    ("controller_server", "FollowPath", "min_speed_xy"): "0.0",
    ("controller_server", "FollowPath", "max_speed_xy"): "1.0",
    ("controller_server", "FollowPath", "vx_samples"): "21",
    ("controller_server", "FollowPath", "acc_lim_x"): "1.0",
    ("controller_server", "FollowPath", "decel_lim_x"): "-1.0",
    ("controller_server", "FollowPath", "PreferForward.strafe_x"): "0.3",
    ("controller_server", "FollowPath", "min_vel_y"): "0.0",
    ("controller_server", "FollowPath", "max_vel_y"): "0.0",
    ("controller_server", "FollowPath", "acc_lim_y"): "0.0",
    ("controller_server", "FollowPath", "decel_lim_y"): "0.0",
    ("controller_server", "FollowPath", "vy_samples"): "1",
}  # type: Dict[Tuple[str, str, str], str]


def rewrite_nav_config(source: str) -> Tuple[str, List[str]]:
    lines = source.splitlines(keepends=True)
    top_section = ""
    subsection = ""
    seen = {}  # type: Dict[Tuple[str, str, str], int]
    changes = []  # type: List[str]
    output = []

    for line in lines:
        top_match = re.match(r"^([A-Za-z0-9_]+):\s*(?:#.*)?$", line.rstrip("\n"))
        if top_match:
            top_section = top_match.group(1)
            subsection = ""
        sub_match = re.match(r"^    ([A-Za-z0-9_]+):\s*(?:#.*)?$", line.rstrip("\n"))
        if sub_match and top_section in {"planner_server", "controller_server"}:
            subsection = sub_match.group(1)

        key_match = re.match(
            r"^(\s+)([A-Za-z0-9_.]+):\s*([^#\n]*?)(\s*(?:#.*)?)(\n?)$",
            line,
        )
        if key_match:
            key = (top_section, subsection, key_match.group(2))
            expected = EXPECTED.get(key)
            if expected is not None:
                seen[key] = seen.get(key, 0) + 1
                current = key_match.group(3).strip()
                if current != expected:
                    changes.append(
                        "{}.{}.{}: {} -> {}".format(*key, current, expected)
                    )
                    line = "{}{}: {}{}{}".format(
                        key_match.group(1),
                        key_match.group(2),
                        expected,
                        key_match.group(4),
                        key_match.group(5),
                    )
        output.append(line)

    missing = [".".join(key) for key in EXPECTED if seen.get(key) != 1]
    if missing:
        raise ValueError("missing or duplicate Nav2 keys: {}".format(", ".join(missing)))
    return "".join(output), changes


def process_file(path: Path, *, apply: bool) -> bool:
    source = path.read_text(encoding="utf-8")
    updated, changes = rewrite_nav_config(source)
    if not changes:
        print("ready: {}".format(path))
        return False
    if not apply:
        print("unsafe: {}: {}".format(path, " | ".join(changes)))
        return True

    backup = Path(str(path) + ".backup_before_mqtt_nav_safety")
    if not backup.exists():
        shutil.copy2(str(path), str(backup))
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(updated)
        shutil.copymode(str(path), temporary)
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    print("updated: {}: {}".format(path, " | ".join(changes)))
    return True


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--live", action="store_true")
    parser.add_argument("paths", nargs="*", type=Path, default=list(DEFAULT_PATHS))
    return parser.parse_args(argv)


def check_live_parameters(timeout_sec: float = 5.0) -> bool:
    import rclpy
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters

    expected_by_node = {}  # type: Dict[str, List[Tuple[str, object]]]
    for (node_name, subsection, key), raw_value in EXPECTED.items():
        value = (
            int(raw_value)
            if key in {"vx_samples", "vy_samples"}
            else float(raw_value)
        )
        parameter_name = "{}.{}".format(subsection, key) if subsection else key
        expected_by_node.setdefault("/" + node_name, []).append(
            (parameter_name, value)
        )

    ready = True
    rclpy.init(args=None)
    node = rclpy.create_node("lite3_nav_live_config_check")
    try:
        for target, expected in expected_by_node.items():
            client = node.create_client(GetParameters, target + "/get_parameters")
            try:
                if not client.wait_for_service(timeout_sec=timeout_sec):
                    print("missing live parameter service: {}".format(target))
                    ready = False
                    continue
                request = GetParameters.Request()
                request.names = [name for name, _ in expected]
                future = client.call_async(request)
                deadline = time.monotonic() + timeout_sec
                while not future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                response = future.result() if future.done() else None
                if response is None or len(response.values) != len(expected):
                    print("live parameter query failed: {}".format(target))
                    ready = False
                    continue
                for (name, wanted), value in zip(expected, response.values):
                    if isinstance(wanted, int):
                        actual = value.integer_value
                        type_ok = value.type == ParameterType.PARAMETER_INTEGER
                    else:
                        actual = value.double_value
                        type_ok = value.type == ParameterType.PARAMETER_DOUBLE
                    if (
                        not type_ok
                        or not math.isfinite(float(actual))
                        or abs(float(actual) - float(wanted)) > 1e-6
                    ):
                        print(
                            "unsafe live parameter: {}.{}: {} != {}".format(
                                target, name, actual, wanted
                            )
                        )
                        ready = False
            finally:
                node.destroy_client(client)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return ready


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.live:
        return 0 if check_live_parameters() else 1
    needs_change = False
    for path in args.paths:
        if not path.is_file():
            print("missing: {}".format(path))
            return 2
        needs_change = process_file(path, apply=args.apply) or needs_change
    return 0 if args.apply or not needs_change else 1


if __name__ == "__main__":
    raise SystemExit(main())
