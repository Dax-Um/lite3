"""ROS2 LaserScan adapter."""


def scan_to_boundary_input(msg) -> tuple[list[float], float, float]:
    return list(msg.ranges), float(msg.angle_min), float(msg.angle_increment)
