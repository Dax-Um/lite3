"""Extract compact local clearances from a LiDAR PointCloud2 frame."""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PointCloudClearanceConfig:
    front_half_angle_rad: float = math.radians(20.0)
    side_min_angle_rad: float = math.radians(15.0)
    # Ignore the ground plane while preserving low physical obstacles.
    min_z_m: float = -0.10
    max_z_m: float = 0.70
    point_stride: int = 4


def extract_clearances(message, config: PointCloudClearanceConfig = PointCloudClearanceConfig()):
    """Return ``front,left,right`` nearest ranges from a PointCloud2 message.

    The RoboSense frame has float32 ``x``/``y`` fields.  Field lookup keeps the
    converter independent of the remaining vendor-specific PointCloud layout.
    """
    fields = {field.name: field for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PointCloud2 must contain x, y and z fields")
    if any(field.datatype != 7 for field in (fields["x"], fields["y"], fields["z"])):
        raise ValueError("PointCloud2 x/y/z must be float32")
    endian = ">" if bool(message.is_bigendian) else "<"
    front = left = right = None
    data = memoryview(message.data)
    for offset in range(0, len(data) - message.point_step + 1, message.point_step * config.point_stride):
        x = struct.unpack_from(endian + "f", data, offset + fields["x"].offset)[0]
        y = struct.unpack_from(endian + "f", data, offset + fields["y"].offset)[0]
        z = struct.unpack_from(endian + "f", data, offset + fields["z"].offset)[0]
        if not all(math.isfinite(value) for value in (x, y, z)) or not config.min_z_m <= z <= config.max_z_m:
            continue
        distance = math.hypot(x, y)
        if distance <= 0.0:
            continue
        angle = math.atan2(y, x)
        if abs(angle) <= config.front_half_angle_rad:
            front = distance if front is None else min(front, distance)
        elif angle > config.side_min_angle_rad:
            left = distance if left is None else min(left, distance)
        elif angle < -config.side_min_angle_rad:
            right = distance if right is None else min(right, distance)
    return front, left, right
