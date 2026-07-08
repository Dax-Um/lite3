import math

import pytest

from lite3_iq9.waypoint_patrol import (
    PatrolModeState,
    PatrolOffset,
    PatrolSegment,
    WaypointPatrolConfig,
    WaypointPatrolPlanner,
)
from lite3_iq9.waypoint_route import Waypoint, WaypointRoute


def test_start_patrol_builds_default_three_point_route_from_home_pose():
    planner = WaypointPatrolPlanner(
        default_offsets=[
            PatrolOffset(dx=3.0, dy=0.0, yaw_offset=0.0),
            PatrolOffset(dx=3.0, dy=3.0, yaw_offset=math.pi / 2.0),
        ]
    )
    home = Waypoint(id="home", x=1.0, y=2.0, yaw=0.25)

    route = planner.start_patrol(home)

    assert planner.state == PatrolModeState.ACTIVE
    assert planner.home == home
    assert route.route_id == "default_patrol"
    assert [waypoint.id for waypoint in route.waypoints] == ["home", "p1", "p2", "home_return"]
    assert route.waypoints[0] == home
    assert route.waypoints[-1].x == pytest.approx(home.x)
    assert route.waypoints[-1].y == pytest.approx(home.y)
    assert route.waypoints[-1].yaw == pytest.approx(home.yaw)
    assert _distance(home, route.waypoints[1]) >= 3.0
    assert _distance(home, route.waypoints[2]) >= 3.0


def test_start_patrol_rotates_default_offsets_by_home_yaw():
    planner = WaypointPatrolPlanner(default_offsets=[PatrolOffset(dx=3.0, dy=0.0)])
    home = Waypoint(id="home", x=10.0, y=-2.0, yaw=math.pi / 2.0)

    route = planner.start_patrol(home)

    assert route.waypoints[1].x == pytest.approx(10.0)
    assert route.waypoints[1].y == pytest.approx(1.0)
    assert route.waypoints[1].yaw == pytest.approx(math.pi / 2.0)


def test_start_patrol_builds_sequential_segment_route():
    planner = WaypointPatrolPlanner(
        default_offsets=[],
        default_segments=[
            PatrolSegment(distance_m=2.0, turn_rad=0.0),
            PatrolSegment(distance_m=2.0, turn_rad=math.radians(120.0)),
        ],
        min_distance_m=0.1,
    )
    home = Waypoint(id="home", x=0.0, y=0.0, yaw=0.0)

    route = planner.start_patrol(home)

    assert [waypoint.id for waypoint in route.waypoints] == ["home", "p1", "p2", "home_return"]
    assert route.waypoints[1].x == pytest.approx(2.0)
    assert route.waypoints[1].y == pytest.approx(0.0)
    assert route.waypoints[1].yaw == pytest.approx(0.0)
    assert route.waypoints[2].x == pytest.approx(1.0)
    assert route.waypoints[2].y == pytest.approx(math.sqrt(3.0))
    assert route.waypoints[2].yaw == pytest.approx(math.radians(120.0))


def test_stop_patrol_returns_to_initial_home_from_current_pose():
    planner = WaypointPatrolPlanner(default_offsets=[PatrolOffset(dx=3.0, dy=0.0)])
    home = Waypoint(id="home", x=1.0, y=2.0, yaw=0.5)
    planner.start_patrol(home)

    return_route = planner.stop_patrol(Waypoint(id="current", x=4.0, y=2.0, yaw=0.0))

    assert planner.state == PatrolModeState.RETURNING_HOME
    assert return_route.route_id == "return_home"
    assert [waypoint.id for waypoint in return_route.waypoints] == ["current", "home_return"]
    assert return_route.waypoints[-1].x == pytest.approx(home.x)
    assert return_route.waypoints[-1].y == pytest.approx(home.y)
    assert return_route.waypoints[-1].yaw == pytest.approx(home.yaw)


def test_stop_patrol_without_home_is_rejected():
    planner = WaypointPatrolPlanner(default_offsets=[PatrolOffset(dx=3.0, dy=0.0)])

    with pytest.raises(RuntimeError, match="home"):
        planner.stop_patrol(Waypoint(id="current", x=0.0, y=0.0, yaw=0.0))


def test_start_patrol_uses_external_route_when_provided():
    planner = WaypointPatrolPlanner(default_offsets=[PatrolOffset(dx=3.0, dy=0.0)])
    home = Waypoint(id="home", x=0.0, y=0.0, yaw=0.0)
    external_route = WaypointRoute(
        route_id="web_route",
        frame_id="map",
        loop=False,
        waypoints=[
            Waypoint(id="web_p1", x=2.0, y=0.0, yaw=0.0),
            Waypoint(id="web_p2", x=2.0, y=2.0, yaw=1.57),
            Waypoint(id="web_p3", x=0.0, y=2.0, yaw=3.14),
        ],
    )

    route = planner.start_patrol(home, route=external_route)

    assert planner.state == PatrolModeState.ACTIVE
    assert planner.home == home
    assert route is external_route


def test_default_offsets_must_create_points_at_least_min_distance_from_home():
    with pytest.raises(ValueError, match="min_distance"):
        WaypointPatrolPlanner(default_offsets=[PatrolOffset(dx=2.0, dy=0.0)], min_distance_m=3.0)


def test_loads_patrol_config_from_yaml(tmp_path):
    config_file = tmp_path / "patrol.yaml"
    config_file.write_text(
        """
route_id: field_demo
frame_id: map
min_distance_m: 3.0
offsets:
  - id: p1
    dx: 3.5
    dy: 0.0
    yaw_offset: 0.0
    dwell_sec: 1.0
  - id: p2
    dx: 3.5
    dy: 3.5
    yaw_offset: 1.57
""",
        encoding="utf-8",
    )

    config = WaypointPatrolConfig.from_yaml(config_file)

    assert config.route_id == "field_demo"
    assert config.frame_id == "map"
    assert config.min_distance_m == 3.0
    assert config.offsets[0].dx == 3.5
    assert config.offsets[0].dwell_sec == 1.0
    assert config.offsets[1].dwell_sec == 0.0


def test_loads_segment_patrol_config_from_yaml(tmp_path):
    config_file = tmp_path / "patrol.yaml"
    config_file.write_text(
        """
route_id: turn_demo
frame_id: map
min_distance_m: 0.1
segments:
  - id: p1
    distance_m: 2.0
    turn_deg: 0.0
  - id: p2
    distance_m: 2.0
    turn_deg: 120.0
""",
        encoding="utf-8",
    )

    config = WaypointPatrolConfig.from_yaml(config_file)

    assert config.route_id == "turn_demo"
    assert config.offsets == []
    assert config.segments[0].distance_m == 2.0
    assert config.segments[1].turn_rad == pytest.approx(math.radians(120.0))


def _distance(a: Waypoint, b: Waypoint) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)
