from types import SimpleNamespace

from lite3_ros.lidar_adapter import scan_to_boundary_input


def test_scan_to_boundary_input_copies_ranges_and_angles():
    msg = SimpleNamespace(ranges=(1.0, 2.0, 3.0), angle_min=-0.5, angle_increment=0.1)

    ranges, angle_min, angle_increment = scan_to_boundary_input(msg)

    assert ranges == [1.0, 2.0, 3.0]
    assert angle_min == -0.5
    assert angle_increment == 0.1
