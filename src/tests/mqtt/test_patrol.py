import time
import math

import pytest

from lite3_mqtt.patrol import ContinuousPatrolController, MockPatrolBackend, Waypoint
from lite3_mqtt.patrol import PatrolConfig


def test_start_uses_current_pose_plus_two_offsets_and_repeats(tmp_path):
    config = tmp_path / "patrol.yaml"
    config.write_text(
        """
route_id: mqtt_loop
frame_id: map
min_distance_m: 1.0
offsets:
  - dx: 2.0
    dy: 0.0
  - dx: 2.0
    dy: 2.0
""",
        encoding="utf-8",
    )
    backend = MockPatrolBackend(
        home=Waypoint(id="home", x=10.0, y=20.0, yaw=0.0),
        route_duration_sec=0.01,
    )
    controller = ContinuousPatrolController(backend=backend, patrol_config=config)

    assert controller.start()
    deadline = time.monotonic() + 1.0
    while len(backend.routes) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.close()

    assert len(backend.routes) >= 2
    first = backend.routes[0]
    assert first.loop is True
    assert [item.id for item in first.waypoints] == ["home", "p1", "p2"]
    assert first.waypoints[0].x == pytest.approx(10.0)
    assert first.waypoints[0].y == pytest.approx(20.0)
    assert first.waypoints[1].x == pytest.approx(12.0)
    assert first.waypoints[2].x == pytest.approx(12.0)
    assert first.waypoints[2].y == pytest.approx(22.0)


def test_duplicate_start_is_idempotent(tmp_path):
    config = tmp_path / "patrol.yaml"
    config.write_text(
        "route_id: x\nframe_id: map\nmin_distance_m: 1.0\noffsets:\n  - {dx: 1.0, dy: 0.0}\n",
        encoding="utf-8",
    )
    controller = ContinuousPatrolController(
        backend=MockPatrolBackend(route_duration_sec=0.2),
        patrol_config=config,
    )
    assert controller.start() is True
    assert controller.start() is False
    controller.close()


def test_startup_gate_runs_before_current_pose_capture(tmp_path):
    config = tmp_path / "patrol.yaml"
    config.write_text(
        "route_id: x\nframe_id: map\nmin_distance_m: 1.0\noffsets:\n  - {dx: 1.0, dy: 0.0}\n",
        encoding="utf-8",
    )
    calls = []

    class Gate:
        def ensure_ready(self):
            calls.append("gate")

    class Backend(MockPatrolBackend):
        def capture_current_pose(self, *, waypoint_id):
            calls.append("pose")
            return super().capture_current_pose(waypoint_id=waypoint_id)

    controller = ContinuousPatrolController(
        backend=Backend(route_duration_sec=0.01),
        patrol_config=config,
        startup_gate=Gate(),
    )
    controller.start()
    deadline = time.monotonic() + 1.0
    while len(calls) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.close()
    assert calls[:2] == ["gate", "pose"]


def test_forward_patrol_has_no_lateral_offset_and_faces_travel_direction(tmp_path):
    config = tmp_path / "forward.yaml"
    config.write_text(
        """
route_id: forward
frame_id: map
min_distance_m: 1.0
forward_distances_m:
  - 2.0
  - 4.0
""",
        encoding="utf-8",
    )
    home = Waypoint(id="home", x=10.0, y=20.0, yaw=math.pi / 2.0)

    route = PatrolConfig.from_yaml(config).build_route(home)

    assert [item.id for item in route.waypoints] == [
        "home",
        "p1",
        "p2",
        "p1_return",
        "home_return",
    ]
    assert route.waypoints[1].x == pytest.approx(10.0)
    assert route.waypoints[1].y == pytest.approx(22.0)
    assert route.waypoints[2].x == pytest.approx(10.0)
    assert route.waypoints[2].y == pytest.approx(24.0)
    assert route.waypoints[1].yaw == pytest.approx(home.yaw)
    assert route.waypoints[2].yaw == pytest.approx(-math.pi / 2.0)
    assert route.waypoints[3].x == pytest.approx(route.waypoints[1].x)
    assert route.waypoints[3].y == pytest.approx(route.waypoints[1].y)
    assert route.waypoints[3].yaw == pytest.approx(-math.pi / 2.0)
    assert route.waypoints[4].x == pytest.approx(home.x)
    assert route.waypoints[4].y == pytest.approx(home.y)
    assert route.waypoints[4].yaw == pytest.approx(home.yaw)


def test_forward_patrol_rejects_lateral_fields_and_requires_two_distances(tmp_path):
    config = tmp_path / "bad.yaml"
    config.write_text(
        """
route_id: mixed
frame_id: map
min_distance_m: 1.0
forward_distances_m: [2.0, 4.0]
offsets:
  - {dx: 2.0, dy: 1.0}
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="exactly one"):
        PatrolConfig.from_yaml(config)


def test_equilateral_triangle_uses_current_pose_and_faces_each_edge(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        """
route_id: triangle
frame_id: map
min_distance_m: 1.0
equilateral_triangle_side_m: 2.0
""",
        encoding="utf-8",
    )
    home = Waypoint(id="home", x=1.0, y=2.0, yaw=0.0)

    route = PatrolConfig.from_yaml(config).build_route(home)

    assert [item.id for item in route.waypoints] == ["p1", "p2", "home_return"]
    assert route.waypoints[0].x == pytest.approx(3.0)
    assert route.waypoints[0].y == pytest.approx(2.0)
    assert route.waypoints[1].x == pytest.approx(2.0)
    assert route.waypoints[1].y == pytest.approx(2.0 + math.sqrt(3.0))
    assert route.waypoints[0].yaw == pytest.approx(2.0 * math.pi / 3.0)
    assert route.waypoints[1].yaw == pytest.approx(-2.0 * math.pi / 3.0)
    physical_route = [home] + route.waypoints
    for start, end in zip(physical_route, physical_route[1:]):
        assert math.hypot(end.x - start.x, end.y - start.y) == pytest.approx(2.0)


def test_equilateral_triangle_can_use_safe_absolute_map_heading(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        """
route_id: triangle
frame_id: map
min_distance_m: 1.0
equilateral_triangle_side_m: 2.0
equilateral_triangle_heading_deg: 240.0
""",
        encoding="utf-8",
    )
    home = Waypoint(id="home", x=8.0, y=-1.0, yaw=0.5)

    route = PatrolConfig.from_yaml(config).build_route(home)

    assert route.waypoints[0].x == pytest.approx(7.0)
    assert route.waypoints[0].y == pytest.approx(-1.0 - math.sqrt(3.0))
    assert route.waypoints[1].x == pytest.approx(9.0)
    assert route.waypoints[1].y == pytest.approx(-1.0 - math.sqrt(3.0))
    assert route.waypoints[2].yaw == pytest.approx(math.radians(240.0))


def test_max_loop_limit_stops_after_one_route(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        "route_id: triangle\nframe_id: map\nmin_distance_m: 1.0\n"
        "equilateral_triangle_side_m: 2.0\n",
        encoding="utf-8",
    )
    backend = MockPatrolBackend(route_duration_sec=0.01)
    controller = ContinuousPatrolController(
        backend=backend,
        patrol_config=config,
        max_loops=1,
    )
    controller.start()
    deadline = time.monotonic() + 1.0
    while controller.active and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.close()
    assert len(backend.routes) == 1
