import ast
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "perception_host_probe_lidar.py"


def load_script():
    spec = importlib.util.spec_from_file_location("probe_lidar", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pointcloud(*, frame="rslidar", stamp=100.0, width=2, height=3, data_size=60):
    sec = int(stamp)
    nanosec = int((stamp - sec) * 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame,
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        width=width,
        height=height,
        point_step=10,
        row_step=20,
        fields=[
            SimpleNamespace(name="x"),
            SimpleNamespace(name="y"),
            SimpleNamespace(name="z"),
        ],
        data=bytes(data_size),
    )


def install_fake_ros(monkeypatch, *, message):
    events = {
        "destroyed_node": False,
        "destroyed_subscription": None,
        "shutdown": False,
        "spins": 0,
    }
    reliable = object()
    volatile = object()

    class FakeQoSProfile:
        def __init__(self, *, depth):
            self.depth = depth
            self.reliability = None
            self.durability = None

    class FakeReliabilityPolicy:
        RELIABLE = reliable

    class FakeDurabilityPolicy:
        VOLATILE = volatile

    class FakeNode:
        def __init__(self):
            self.callback = None

        def create_subscription(self, message_type, topic, callback, qos):
            self.callback = callback
            events["message_type"] = message_type
            events["topic"] = topic
            events["qos"] = qos
            events["subscription"] = object()
            return events["subscription"]

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=102_000_000_000)
            )

        def destroy_subscription(self, subscription):
            events["destroyed_subscription"] = subscription

        def destroy_node(self):
            events["destroyed_node"] = True

    node = FakeNode()
    pointcloud_type = type("PointCloud2", (), {})

    rclpy = ModuleType("rclpy")
    rclpy.__path__ = []
    rclpy.init = lambda args=None: events.setdefault("init_args", args)
    rclpy.create_node = lambda name: node
    rclpy.ok = lambda: events["spins"] < 1

    def spin_once(spin_node, *, timeout_sec):
        assert spin_node is node
        events["spin_timeout"] = timeout_sec
        events["spins"] += 1
        node.callback(message)

    rclpy.spin_once = spin_once

    def shutdown():
        events["shutdown"] = True

    rclpy.shutdown = shutdown

    qos_module = ModuleType("rclpy.qos")
    qos_module.QoSProfile = FakeQoSProfile
    qos_module.ReliabilityPolicy = FakeReliabilityPolicy
    qos_module.DurabilityPolicy = FakeDurabilityPolicy

    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs.__path__ = []
    sensor_msgs_msg = ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.PointCloud2 = pointcloud_type

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos_module)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs_msg)
    events["reliable"] = reliable
    events["volatile"] = volatile
    events["pointcloud_type"] = pointcloud_type
    return events


def test_pointcloud_probe_accepts_fresh_nonempty_expected_frame():
    module = load_script()

    assert module.validate_pointcloud(
        pointcloud(), now_sec=102.0, expected_frame="rslidar", max_age_sec=5.0
    ) == ""


def test_pointcloud_probe_rejects_wrong_frame_stale_empty_or_truncated_data():
    module = load_script()

    assert "frame" in module.validate_pointcloud(
        pointcloud(frame="map"),
        now_sec=102.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    )
    assert "age" in module.validate_pointcloud(
        pointcloud(stamp=90.0),
        now_sec=102.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    )
    assert "empty" in module.validate_pointcloud(
        pointcloud(width=0),
        now_sec=102.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    )
    assert "truncated" in module.validate_pointcloud(
        pointcloud(data_size=59),
        now_sec=102.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    )

    invalid_row_step = pointcloud()
    invalid_row_step.row_step = 1
    assert "row_step" in module.validate_pointcloud(
        invalid_row_step,
        now_sec=102.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    )

    missing_xyz = pointcloud()
    missing_xyz.fields = [SimpleNamespace(name="intensity")]
    assert "x/y/z" in module.validate_pointcloud(
        missing_xyz,
        now_sec=102.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    )


def test_probe_defaults_to_rslidar_and_rejects_nonfinite_timeouts():
    module = load_script()

    assert module.parse_args([]).expected_frame == "rslidar"
    for option, value in (
        ("--timeout-sec", "nan"),
        ("--timeout-sec", "inf"),
        ("--max-age-sec", "nan"),
        ("--max-age-sec", "inf"),
    ):
        with pytest.raises(SystemExit):
            module.parse_args([option, value])


def test_probe_exits_after_one_valid_sample_with_foxy_qos_and_cleans_up(monkeypatch):
    module = load_script()
    events = install_fake_ros(monkeypatch, message=pointcloud())

    assert module.wait_for_fresh_pointcloud(
        timeout_sec=1.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    ) is True
    assert events["spins"] == 1
    assert events["topic"] == "/rslidar_points"
    assert events["message_type"] is events["pointcloud_type"]
    assert events["qos"].depth == 1
    assert events["qos"].reliability is events["reliable"]
    assert events["qos"].durability is events["volatile"]
    assert events["destroyed_subscription"] is events["subscription"]
    assert events["destroyed_node"] is True
    assert events["shutdown"] is True


def test_probe_rejects_invalid_sample_and_requests_power_on_frame_stamp_check(
    monkeypatch, capsys
):
    module = load_script()
    events = install_fake_ros(monkeypatch, message=pointcloud(frame="map"))

    assert module.wait_for_fresh_pointcloud(
        timeout_sec=1.0,
        expected_frame="rslidar",
        max_age_sec=5.0,
    ) is False
    output = capsys.readouterr().out
    assert "frame 'map' != 'rslidar'" in output
    assert "power-on verification required" in output
    assert "header.frame_id and header.stamp" in output
    assert events["destroyed_node"] is True
    assert events["shutdown"] is True


def test_probe_and_live_config_helpers_parse_as_python_3_8():
    prepare_config = SCRIPT.with_name("perception_host_prepare_nav_config.py")

    ast.parse(SCRIPT.read_text(encoding="utf-8"), feature_version=(3, 8))
    ast.parse(prepare_config.read_text(encoding="utf-8"), feature_version=(3, 8))
