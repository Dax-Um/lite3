"""ROS2 LaserScan adapter."""

from __future__ import annotations


def scan_to_boundary_input(msg) -> tuple[list[float], float, float]:
    return list(msg.ranges), float(msg.angle_min), float(msg.angle_increment)
