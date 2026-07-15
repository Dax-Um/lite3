import time
import math
import threading

import pytest
from types import SimpleNamespace

from lite3_mqtt.patrol import (
    ContinuousPatrolController,
    MockPatrolBackend,
    NavSafetyState,
    Waypoint,
    WaypointRoute,
    _goal_progressed,
    _twist_is_finite,
    _validate_computed_path,
    _validate_route_on_costmap,
)
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
    assert [item.id for item in first.waypoints] == ["p1", "p2", "home_return"]
    assert first.waypoints[0].x == pytest.approx(12.0)
    assert first.waypoints[1].x == pytest.approx(12.0)
    assert first.waypoints[1].y == pytest.approx(22.0)
    assert first.waypoints[2].x == pytest.approx(10.0)
    assert first.waypoints[2].y == pytest.approx(20.0)


def test_absolute_waypoints_keep_map_coordinates_and_face_each_leg(tmp_path):
    config = tmp_path / "absolute.yaml"
    config.write_text(
        """
route_id: known_good
frame_id: map
min_distance_m: 1.0
absolute_waypoints:
  - {x: 4.13, y: 1.41}
  - {x: 1.71, y: 2.93}
""",
        encoding="utf-8",
    )
    home = Waypoint(id="home", x=1.03, y=0.28, yaw=0.0)

    route = PatrolConfig.from_yaml(config).build_route(home)

    assert [(item.x, item.y) for item in route.waypoints] == pytest.approx(
        [(4.13, 1.41), (1.71, 2.93), (1.03, 0.28)]
    )
    assert route.waypoints[0].yaw == pytest.approx(math.atan2(1.13, 3.10))
    assert route.waypoints[1].yaw == pytest.approx(math.atan2(1.52, -2.42))
    assert route.waypoints[2].yaw == pytest.approx(math.atan2(-2.65, -0.68))


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


def test_prepare_motion_runs_startup_gate_without_sending_route(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        "route_id: triangle\nframe_id: map\nmin_distance_m: 1.0\n"
        "equilateral_triangle_side_m: 2.0\n",
        encoding="utf-8",
    )
    calls = []

    class Gate:
        def ensure_ready(self):
            calls.append("gate")

    backend = MockPatrolBackend()
    controller = ContinuousPatrolController(
        backend=backend,
        patrol_config=config,
        startup_gate=Gate(),
    )

    controller.prepare_motion()
    controller.close()

    assert calls == ["gate"]
    assert backend.routes == []


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
        "p1",
        "p2",
        "p1_return",
        "home_return",
    ]
    assert route.waypoints[0].x == pytest.approx(10.0)
    assert route.waypoints[0].y == pytest.approx(22.0)
    assert route.waypoints[1].x == pytest.approx(10.0)
    assert route.waypoints[1].y == pytest.approx(24.0)
    assert route.waypoints[0].yaw == pytest.approx(home.yaw)
    assert route.waypoints[1].yaw == pytest.approx(-math.pi / 2.0)
    assert route.waypoints[2].x == pytest.approx(route.waypoints[0].x)
    assert route.waypoints[2].y == pytest.approx(route.waypoints[0].y)
    assert route.waypoints[2].yaw == pytest.approx(-math.pi / 2.0)
    assert route.waypoints[3].x == pytest.approx(home.x)
    assert route.waypoints[3].y == pytest.approx(home.y)
    assert route.waypoints[3].yaw == pytest.approx(home.yaw)


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


def test_equilateral_triangle_uses_current_pose_and_last_known_good_headings(tmp_path):
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
    assert route.waypoints[2].yaw == pytest.approx(0.0)
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
    assert route.waypoints[0].yaw == pytest.approx(0.0)
    assert route.waypoints[1].yaw == pytest.approx(math.radians(120.0))
    assert route.waypoints[2].yaw == pytest.approx(math.radians(-120.0))


def test_triangle_preflight_tries_alternate_headings(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        "route_id: triangle\nframe_id: map\nmin_distance_m: 1.0\n"
        "equilateral_triangle_side_m: 2.0\n",
        encoding="utf-8",
    )

    class FallbackBackend(MockPatrolBackend):
        def __init__(self):
            super().__init__(route_duration_sec=0.01)
            self.validations = 0

        def validate_route(self, route, *, start):
            self.validations += 1
            if self.validations == 1:
                raise ValueError("blocked heading")
            super().validate_route(route, start=start)

    backend = FallbackBackend()
    controller = ContinuousPatrolController(
        backend=backend,
        patrol_config=config,
        max_loops=1,
    )
    controller.start()
    deadline = time.monotonic() + 1.0
    while controller.active and time.monotonic() < deadline:
        time.sleep(0.01)
    assert backend.validations == 2
    assert len(backend.routes) == 1


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


def test_emergency_stop_is_latched_until_reset(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        "route_id: triangle\nframe_id: map\nmin_distance_m: 1.0\n"
        "equilateral_triangle_side_m: 2.0\n",
        encoding="utf-8",
    )
    backend = MockPatrolBackend(route_duration_sec=0.2)
    controller = ContinuousPatrolController(backend=backend, patrol_config=config)

    assert controller.start()
    controller.emergency_stop()
    assert controller.emergency_latched
    assert controller.start() is False
    controller.reset()
    assert not controller.emergency_latched
    deadline = time.monotonic() + 1.0
    while controller.active and time.monotonic() < deadline:
        time.sleep(0.01)
    assert controller.start() is True
    controller.close()


def test_stop_between_prepare_and_send_is_not_lost(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        "route_id: triangle\nframe_id: map\nmin_distance_m: 1.0\n"
        "equilateral_triangle_side_m: 2.0\n",
        encoding="utf-8",
    )

    class BarrierBackend(MockPatrolBackend):
        def __init__(self):
            super().__init__()
            self.prepare_count = 0
            self.before_send = threading.Event()
            self.release = threading.Event()

        def prepare_route(self):
            super().prepare_route()
            self.prepare_count += 1
            if self.prepare_count == 2:
                self.before_send.set()
                self.release.wait(timeout=1.0)

    backend = BarrierBackend()
    controller = ContinuousPatrolController(backend=backend, patrol_config=config)
    assert controller.start()
    assert backend.before_send.wait(timeout=1.0)
    controller.stop()
    backend.release.set()
    deadline = time.monotonic() + 1.0
    while controller.active and time.monotonic() < deadline:
        time.sleep(0.01)
    assert backend.routes == []


def test_return_home_sends_only_the_home_goal(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        "route_id: triangle\nframe_id: map\nmin_distance_m: 1.0\n"
        "equilateral_triangle_side_m: 2.0\n",
        encoding="utf-8",
    )
    backend = MockPatrolBackend(route_duration_sec=0.1)
    controller = ContinuousPatrolController(backend=backend, patrol_config=config)
    assert controller.start()
    deadline = time.monotonic() + 1.0
    while controller.home is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert controller.return_home()
    assert controller.start() is False
    deadline = time.monotonic() + 2.0
    while controller.active and time.monotonic() < deadline:
        time.sleep(0.01)
    assert [item.id for item in backend.routes[-1].waypoints] == ["home_return"]


def test_succeeded_status_with_missed_waypoint_does_not_repeat(tmp_path):
    config = tmp_path / "triangle.yaml"
    config.write_text(
        "route_id: triangle\nframe_id: map\nmin_distance_m: 1.0\n"
        "equilateral_triangle_side_m: 2.0\n",
        encoding="utf-8",
    )

    class MissedBackend(MockPatrolBackend):
        def send_route(self, route):
            self.routes.append(route)
            return {"accepted": True, "status": "SUCCEEDED", "missed_waypoints": [1]}

    backend = MissedBackend()
    controller = ContinuousPatrolController(backend=backend, patrol_config=config)
    controller.start()
    deadline = time.monotonic() + 1.0
    while controller.active and time.monotonic() < deadline:
        time.sleep(0.01)
    assert len(backend.routes) == 1


def test_nav_safety_state_uses_reception_age_and_lateral_velocity():
    state = NavSafetyState()
    state.mark_odom(now=10.0, frame_id="map", x=1.0, y=2.0, yaw=0.0)
    state.mark_localization(now=10.0, converged=True)
    state.mark("local_costmap", 10.0)
    state.mark("global_costmap", 10.0)
    assert state.blocking_reasons(
        now=10.5, max_age_sec=1.0, max_lateral_speed_mps=0.02
    ) == []
    state.mark_cmd_vel(0.03)
    reasons = state.blocking_reasons(
        now=11.5, max_age_sec=1.0, max_lateral_speed_mps=0.02
    )
    assert "odom_stale" in reasons
    assert "lateral_cmd_vel" in reasons


def test_cmd_vel_rejects_non_finite_value_in_any_twist_component():
    def vector(x=0.0, y=0.0, z=0.0):
        return SimpleNamespace(x=x, y=y, z=z)

    assert _twist_is_finite(
        SimpleNamespace(linear=vector(), angular=vector())
    )
    assert not _twist_is_finite(
        SimpleNamespace(linear=vector(x=float("nan")), angular=vector())
    )
    assert not _twist_is_finite(
        SimpleNamespace(linear=vector(), angular=vector(z=float("inf")))
    )

    state = NavSafetyState()
    state.mark_cmd_vel(0.0, valid=False)
    assert "cmd_vel_invalid" in state.blocking_reasons(
        now=0.0,
        max_age_sec=1.0,
        max_lateral_speed_mps=0.02,
    )


def test_route_preflight_rejects_unknown_or_occupied_cells():
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    position = SimpleNamespace(x=0.0, y=0.0)
    info = SimpleNamespace(
        resolution=1.0,
        width=5,
        height=5,
        origin=SimpleNamespace(position=position, orientation=orientation),
    )
    route = WaypointRoute(
        route_id="test",
        frame_id="map",
        loop=False,
        waypoints=[Waypoint("p1", 2.0, 1.0, 0.0)],
    )
    start = Waypoint("home", 1.0, 1.0, 0.0)
    header = SimpleNamespace(frame_id="map")
    free = SimpleNamespace(header=header, info=info, data=[0] * 25)
    _validate_route_on_costmap(route, start=start, costmap=free)

    blocked_data = [0] * 25
    blocked_data[1 * 5 + 2] = 100
    blocked = SimpleNamespace(header=header, info=info, data=blocked_data)
    with pytest.raises(ValueError, match="not known free"):
        _validate_route_on_costmap(route, start=start, costmap=blocked)

    wrong_frame = SimpleNamespace(
        header=SimpleNamespace(frame_id="odom"), info=info, data=[0] * 25
    )
    with pytest.raises(ValueError, match="does not match route frame"):
        _validate_route_on_costmap(route, start=start, costmap=wrong_frame)


def test_route_preflight_rejects_inflated_cost_and_requires_clearance():
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    info = SimpleNamespace(
        resolution=0.1,
        width=40,
        height=40,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0),
            orientation=orientation,
        ),
    )
    route = WaypointRoute(
        route_id="test",
        frame_id="map",
        loop=False,
        waypoints=[Waypoint("p1", 2.0, 2.0, 0.0)],
    )
    start = Waypoint("home", 1.0, 1.0, 0.0)
    inflated = [0] * (info.width * info.height)
    inflated[20 * info.width + 20] = 50
    with pytest.raises(ValueError, match="not known free"):
        _validate_route_on_costmap(
            route,
            start=start,
            costmap=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"), info=info, data=inflated
            ),
            clearance_m=0.0,
        )

    obstacle = [0] * (info.width * info.height)
    obstacle[20 * info.width + 22] = 100
    with pytest.raises(ValueError, match="corridor home->p1 is not known free"):
        _validate_route_on_costmap(
            route,
            start=start,
            costmap=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"), info=info, data=obstacle
            ),
            clearance_m=0.35,
        )


def test_route_preflight_rejects_blocked_future_leg_corridor():
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    info = SimpleNamespace(
        resolution=0.1,
        width=60,
        height=60,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0),
            orientation=orientation,
        ),
    )
    route = WaypointRoute(
        route_id="test",
        frame_id="map",
        loop=False,
        waypoints=[
            Waypoint("p1", 3.0, 1.0, 0.0),
            Waypoint("p2", 3.0, 3.0, math.pi / 2.0),
        ],
    )
    start = Waypoint("home", 1.0, 1.0, 0.0)
    data = [0] * (info.width * info.height)
    data[20 * info.width + 30] = 100

    with pytest.raises(ValueError, match="corridor p1->p2 is not known free"):
        _validate_route_on_costmap(
            route,
            start=start,
            costmap=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"), info=info, data=data
            ),
            clearance_m=0.0,
        )


def test_nav_safety_state_reports_arrival_position_and_yaw_error():
    state = NavSafetyState()
    state.mark_odom(now=10.0, frame_id="map", x=1.1, y=1.2, yaw=0.4)

    position_error, yaw_error = state.arrival_error(
        Waypoint("p1", 1.0, 1.0, 0.1)
    )

    assert position_error == pytest.approx(math.hypot(0.1, 0.2))
    assert yaw_error == pytest.approx(0.3)

    state.mark_odom(now=10.5, frame_id="odom", x=1.0, y=1.0, yaw=0.1)
    assert state.arrival_error(Waypoint("p1", 1.0, 1.0, 0.1)) is None

    state.mark_odom(now=11.0, frame_id="map", x=float("nan"), y=1.2, yaw=0.4)
    assert state.arrival_error(Waypoint("p1", 1.0, 1.0, 0.1)) is None
    reasons = state.blocking_reasons(
        now=11.0, max_age_sec=1.0, max_lateral_speed_mps=0.02
    )
    assert "odom_pose_invalid" in reasons


def test_progress_requires_goal_approach_or_goal_yaw_improvement_near_goal():
    goal = Waypoint("p1", 3.0, 2.0, 1.0)
    reference = Waypoint("odom", 1.0, 2.0, 0.5)

    assert not _goal_progressed(
        reference,
        Waypoint("odom", 1.02, 2.01, 0.53),
        goal,
        distance_m=0.10,
        yaw_rad=0.15,
        yaw_progress_position_m=0.30,
    )
    assert _goal_progressed(
        reference,
        Waypoint("odom", 1.11, 2.0, 0.5),
        goal,
        distance_m=0.10,
        yaw_rad=0.15,
        yaw_progress_position_m=0.30,
    )
    assert not _goal_progressed(
        reference,
        Waypoint("odom", 0.89, 2.0, 0.66),
        goal,
        distance_m=0.10,
        yaw_rad=0.15,
        yaw_progress_position_m=0.30,
    )

    near_reference = Waypoint("odom", 2.8, 2.0, 0.5)
    assert _goal_progressed(
        near_reference,
        Waypoint("odom", 2.8, 2.0, 0.66),
        goal,
        distance_m=0.10,
        yaw_rad=0.15,
        yaw_progress_position_m=0.30,
    )
    assert not _goal_progressed(
        near_reference,
        Waypoint("odom", 2.8, 2.0, 0.34),
        goal,
        distance_m=0.10,
        yaw_rad=0.15,
        yaw_progress_position_m=0.30,
    )


def test_computed_path_requires_reachable_goal_and_bounded_detour():
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    info = SimpleNamespace(
        resolution=0.1,
        width=60,
        height=60,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0),
            orientation=orientation,
        ),
    )
    costmap = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=info,
        data=[0] * (info.width * info.height),
    )
    start = Waypoint("home", 1.0, 1.0, 0.0)
    goal = Waypoint("p1", 3.0, 1.0, 0.0)

    def pose(x, y, frame_id="map"):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame_id),
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
        )

    path = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        poses=[pose(1.0, 1.0), pose(2.0, 1.0), pose(3.0, 1.0)],
    )
    _validate_computed_path(
        path,
        frame_id="map",
        start=start,
        goal=goal,
        leg_index=1,
        costmap=costmap,
        max_detour_ratio=3.0,
    )

    empty = SimpleNamespace(header=SimpleNamespace(frame_id="map"), poses=[])
    with pytest.raises(ValueError, match="empty path"):
        _validate_computed_path(
            empty,
            frame_id="map",
            start=start,
            goal=goal,
            leg_index=1,
            costmap=costmap,
            max_detour_ratio=3.0,
        )

    wrong_endpoint = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        poses=[pose(1.0, 1.0), pose(2.0, 1.0)],
    )
    with pytest.raises(ValueError, match="from goal"):
        _validate_computed_path(
            wrong_endpoint,
            frame_id="map",
            start=start,
            goal=goal,
            leg_index=1,
            costmap=costmap,
            max_detour_ratio=3.0,
        )

    wrong_start = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        poses=[pose(2.0, 1.0), pose(3.0, 1.0)],
    )
    with pytest.raises(ValueError, match="captured pose"):
        _validate_computed_path(
            wrong_start,
            frame_id="map",
            start=start,
            goal=goal,
            leg_index=1,
            costmap=costmap,
            max_detour_ratio=3.0,
        )

    detour = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        poses=[
            pose(1.0, 1.0),
            pose(1.0, 5.0),
            pose(3.0, 5.0),
            pose(3.0, 1.0),
        ],
    )
    with pytest.raises(ValueError, match="detour is too long"):
        _validate_computed_path(
            detour,
            frame_id="map",
            start=start,
            goal=goal,
            leg_index=1,
            costmap=costmap,
            max_detour_ratio=3.0,
        )
