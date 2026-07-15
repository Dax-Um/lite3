import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "run_nav_watchdog.py"


def test_watchdog_cancels_current_and_legacy_navigation_actions():
    source = SCRIPT.read_text(encoding="utf-8")

    assert '"/navigate_to_pose/_action/cancel_goal"' in source
    assert '"/FollowWaypoints/_action/cancel_goal"' in source
    assert '"/follow_path/_action/cancel_goal"' in source
    assert '"/spin/_action/cancel_goal"' in source
    assert '"/backup/_action/cancel_goal"' in source
    assert '"/wait/_action/cancel_goal"' in source


def test_watchdog_holds_zero_until_reset_ack_and_acks_heartbeat_arm():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "self.stale = True" in source
    assert "self.zero_pub.publish(Twist())" in source
    assert "if msg.data == 0:" in source
    assert '"/lite3/nav/watchdog_reset"' in source
    assert '"/lite3/nav/watchdog_reset_ack"' in source
    assert '"/lite3/nav/watchdog_arm_ack"' in source
    assert "self._harvest_cancel_futures()" in source
    assert "nonzero heartbeat ignored while fail-safe is latched" in source
    assert "self.next_cancel_attempt = now + 1.0" in source


def test_watchdog_repeatedly_cancels_top_level_controller_and_recovery_actions(
    monkeypatch,
):
    clock = SimpleNamespace(now=0.0)
    calls = []

    class Future:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(return_code=0)

    class Client:
        def __init__(self, service):
            self.service = service

        def service_is_ready(self):
            return True

        def wait_for_service(self, timeout_sec=0.0):
            _ = timeout_sec
            return True

        def call_async(self, request):
            _ = request
            calls.append(self.service)
            return Future()

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Logger:
        def error(self, message):
            _ = message

        def info(self, message):
            _ = message

    class Node:
        def __init__(self, name):
            self.name = name
            self.publisher = None

        def create_client(self, service_type, service):
            _ = service_type
            return Client(service)

        def create_publisher(self, message_type, topic, qos):
            _ = message_type, topic, qos
            self.publisher = Publisher()
            return self.publisher

        def create_subscription(self, *args):
            _ = args
            return object()

        def create_timer(self, *args):
            _ = args
            return object()

        def get_logger(self):
            return Logger()

    class CancelGoal:
        class Request:
            def __init__(self):
                self.goal_info = None

    class GoalInfo:
        pass

    class Twist:
        pass

    class UInt64:
        pass

    rclpy = ModuleType("rclpy")
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = Node
    action_msgs_msg = ModuleType("action_msgs.msg")
    action_msgs_msg.GoalInfo = GoalInfo
    action_msgs_srv = ModuleType("action_msgs.srv")
    action_msgs_srv.CancelGoal = CancelGoal
    action_msgs = ModuleType("action_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = Twist
    geometry_msgs = ModuleType("geometry_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.UInt64 = UInt64
    std_msgs = ModuleType("std_msgs")
    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "action_msgs": action_msgs,
        "action_msgs.msg": action_msgs_msg,
        "action_msgs.srv": action_msgs_srv,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    spec = importlib.util.spec_from_file_location("run_nav_watchdog_runtime_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module,
        "time",
        SimpleNamespace(monotonic=lambda: clock.now),
    )

    watchdog = module.NavHeartbeatWatchdog(timeout_sec=1.0, check_period_sec=0.2)
    watchdog._on_heartbeat(SimpleNamespace(data=1))
    assert [message.data for message in watchdog.arm_ack_pub.messages] == [1]
    clock.now = 2.0
    watchdog._check()

    assert set(calls) == set(module.NavHeartbeatWatchdog.CANCEL_SERVICES)
    assert len(watchdog.zero_pub.messages) == 1

    watchdog._on_heartbeat(SimpleNamespace(data=2))
    assert [message.data for message in watchdog.arm_ack_pub.messages] == [1]
    clock.now = 3.1
    watchdog._check()
    for service in module.NavHeartbeatWatchdog.CANCEL_SERVICES:
        assert calls.count(service) == 2

    watchdog._on_reset_request(SimpleNamespace(data=99))
    clock.now = 3.2
    watchdog._check()
    for service in module.NavHeartbeatWatchdog.CANCEL_SERVICES:
        assert calls.count(service) == 2
    assert watchdog.stale is True
    assert watchdog.reset_ack_pub.messages == []

    clock.now = 3.3
    watchdog._on_reset_request(SimpleNamespace(data=99))
    watchdog._check()
    assert watchdog.reset_ack_pub.messages == []

    clock.now = 3.5
    watchdog._check()

    assert watchdog.stale is False
    assert [message.data for message in watchdog.reset_ack_pub.messages] == [99]
