import math
import sys
import time
from types import ModuleType, SimpleNamespace

import pytest

from lite3_mqtt.direct_patrol import (
    DirectMockPatrolBackend,
    DirectPatrolController,
    _pose_stamped,
)
from lite3_mqtt.patrol import PatrolConfig, Waypoint


def triangle_config(tmp_path, side=1.2):
    path = tmp_path / "triangle.yaml"
    path.write_text(
        "route_id: triangle\n"
        "frame_id: map\n"
        "min_distance_m: 1.0\n"
        "equilateral_triangle_side_m: {}\n".format(side),
        encoding="utf-8",
    )
    return path


def test_triangle_has_three_equal_legs_and_last_known_good_headings(tmp_path):
    home = Waypoint("home", 4.0, -2.0, 0.4)
    route = PatrolConfig.from_yaml(triangle_config(tmp_path)).build_route(home)

    assert [point.id for point in route.waypoints] == ["p1", "p2", "home_return"]
    physical = [home] + route.waypoints
    for start, goal in zip(physical, physical[1:]):
        dx = goal.x - start.x
        dy = goal.y - start.y
        assert math.hypot(dx, dy) == pytest.approx(1.2)
    assert route.waypoints[0].yaw == pytest.approx(0.4 + 2.0 * math.pi / 3.0)
    assert route.waypoints[1].yaw == pytest.approx(0.4 - 2.0 * math.pi / 3.0)
    assert route.waypoints[2].yaw == pytest.approx(0.4)


def test_start_captures_home_once_and_sends_one_three_goal_route(tmp_path):
    backend = DirectMockPatrolBackend(
        home=Waypoint("home", 10.0, 20.0, math.pi / 2.0),
        route_duration_sec=0.01,
    )
    controller = DirectPatrolController(
        backend=backend,
        patrol_config=triangle_config(tmp_path),
        max_loops=1,
    )

    assert controller.start() is True
    deadline = time.monotonic() + 1.0
    while controller.active and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.close()

    assert len(backend.routes) == 1
    assert len(backend.routes[0].waypoints) == 3
    assert backend.routes[0].waypoints[-1].x == pytest.approx(10.0)
    assert backend.routes[0].waypoints[-1].y == pytest.approx(20.0)


def test_duplicate_start_does_not_create_second_patrol_thread(tmp_path):
    controller = DirectPatrolController(
        backend=DirectMockPatrolBackend(route_duration_sec=0.2),
        patrol_config=triangle_config(tmp_path),
    )

    assert controller.start() is True
    assert controller.start() is False
    controller.close()


def test_mission_home_can_be_captured_without_starting_auto_patrol(tmp_path):
    backend = DirectMockPatrolBackend(
        home=Waypoint("current", 3.0, -1.0, 0.25),
    )
    controller = DirectPatrolController(
        backend=backend,
        patrol_config=triangle_config(tmp_path),
    )

    assert controller.capture_home() is True
    assert controller.home == Waypoint("home", 3.0, -1.0, 0.25)
    assert controller.active is False


def test_coyote_return_home_spins_then_calls_completion(tmp_path):
    backend = DirectMockPatrolBackend(
        home=Waypoint("home", 1.0, 2.0, 0.0),
        route_duration_sec=0.01,
    )
    controller = DirectPatrolController(
        backend=backend,
        patrol_config=triangle_config(tmp_path),
    )
    assert controller.capture_home()
    completed = []

    assert controller.return_home(
        spin_after_arrival=True,
        on_complete=lambda: completed.append(True),
    )
    deadline = time.monotonic() + 1.0
    while not completed and time.monotonic() < deadline:
        time.sleep(0.01)

    assert backend.spins == [pytest.approx(math.pi)]
    assert completed == [True]


def test_nav2_goal_uses_zero_stamp_across_unsynchronized_hosts(monkeypatch):
    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(
                frame_id="",
                stamp=SimpleNamespace(sec=99, nanosec=99),
            )
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(z=0.0, w=0.0),
            )

    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs.__path__ = []
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = PoseStamped
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)

    node = SimpleNamespace(get_clock=lambda: pytest.fail("clock must not be read"))
    pose = _pose_stamped(node, "map", Waypoint("p1", 4.13, 1.41, 0.3))

    assert pose.header.frame_id == "map"
    assert pose.header.stamp.sec == 0
    assert pose.header.stamp.nanosec == 0


def test_nav2_backend_sends_all_three_poses_in_one_follow_waypoints_goal(monkeypatch):
    sent = []

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(
                frame_id="", stamp=SimpleNamespace(sec=99, nanosec=99)
            )
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(z=0.0, w=0.0),
            )

    class FollowWaypoints:
        class Goal:
            def __init__(self):
                self.poses = []

    class Future:
        def __init__(self, value):
            self.value = value

        def done(self):
            return True

        def result(self):
            return self.value

    class GoalHandle:
        accepted = True

        def get_result_async(self):
            result = SimpleNamespace(missed_waypoints=[])
            return Future(SimpleNamespace(status=4, result=result))

        def cancel_goal_async(self):
            return Future(SimpleNamespace())

    class ActionClient:
        def __init__(self, node, action_type, action_name):
            assert action_type is FollowWaypoints
            assert action_name == "/FollowWaypoints"

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal):
            sent.append(goal)
            return Future(GoalHandle())

        def destroy(self):
            pass

    class Node:
        def destroy_node(self):
            pass

    rclpy = ModuleType("rclpy")
    rclpy.__path__ = []
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True
    rclpy.spin_once = lambda node, timeout_sec: None
    rclpy.create_node = lambda name: Node()
    rclpy_action = ModuleType("rclpy.action")
    rclpy_action.ActionClient = ActionClient
    nav2_msgs = ModuleType("nav2_msgs")
    nav2_msgs.__path__ = []
    nav2_action = ModuleType("nav2_msgs.action")
    nav2_action.FollowWaypoints = FollowWaypoints
    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs.__path__ = []
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = PoseStamped
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.action", rclpy_action)
    monkeypatch.setitem(sys.modules, "nav2_msgs", nav2_msgs)
    monkeypatch.setitem(sys.modules, "nav2_msgs.action", nav2_action)
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)

    from lite3_mqtt.direct_patrol import DirectNav2PatrolBackend
    from lite3_mqtt.patrol import WaypointRoute

    route = WaypointRoute(
        route_id="triangle",
        frame_id="map",
        loop=True,
        waypoints=[
            Waypoint("p1", 2.0, 0.0, 0.0),
            Waypoint("p2", 1.0, math.sqrt(3.0), 2.0),
            Waypoint("home_return", 0.0, 0.0, -2.0),
        ],
    )
    backend = DirectNav2PatrolBackend()

    result = backend.send_route(route)

    assert result["status"] == 4
    assert len(sent) == 1
    assert len(sent[0].poses) == 3
    assert all(pose.header.stamp.sec == 0 for pose in sent[0].poses)
    assert all(pose.header.stamp.nanosec == 0 for pose in sent[0].poses)
