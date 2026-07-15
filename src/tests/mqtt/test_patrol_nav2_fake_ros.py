import logging
import sys
from types import ModuleType, SimpleNamespace

import pytest

import lite3_mqtt.patrol as patrol


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def monotonic_ns(self):
        return int(self.now * 1_000_000_000) + 1


class FakeFuture:
    def __init__(self, result=None, *, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result


class FakePublisher:
    def __init__(self, env, topic):
        self.env = env
        self.topic = topic
        self.values = []

    def publish(self, message):
        self.values.append(message.data)
        self.env.events.append(("publish", message.data))
        self.env.events.append(("publish_topic", self.topic, message.data))
        if self.topic == "/lite3/nav/watchdog_reset" and self.env.emit_reset_ack:
            self.env.pending_reset_ack = int(message.data)
        elif (
            self.topic == "/lite3/nav/heartbeat"
            and int(message.data) != 0
            and self.env.emit_arm_ack
        ):
            self.env.pending_arm_ack = int(message.data)


def install_fake_ros(
    monkeypatch,
    *,
    mode,
    status_sequences=None,
    emit_action_status=True,
    cancel_service_ready=True,
    status_publisher_ready=True,
    emit_reset_ack=True,
    emit_arm_ack=True,
):
    env = SimpleNamespace(
        mode=mode,
        clock=FakeClock(),
        state=None,
        backend=None,
        current_goal=None,
        current_goal_sample_count=0,
        cancel_calls=0,
        sent_goals=[],
        publishers=[],
        events=[],
        status_sequences=list(status_sequences or []),
        last_status=[],
        goal_status_callbacks={},
        cancel_service_calls=[],
        cancel_requests=[],
        emit_action_status=emit_action_status,
        cancel_service_ready=cancel_service_ready,
        status_publisher_ready=status_publisher_ready,
        emit_reset_ack=emit_reset_ack,
        emit_arm_ack=emit_arm_ack,
        watchdog_reset_ack_callback=None,
        watchdog_arm_ack_callback=None,
        pending_reset_ack=None,
        pending_arm_ack=None,
    )

    class UInt64:
        def __init__(self):
            self.data = 0

    class NavigateGoal:
        def __init__(self):
            self.pose = None

    class NavigateToPose:
        Goal = NavigateGoal

    class GoalStatusArray:
        pass

    class UUID:
        def __init__(self, value=None):
            self.uuid = list(value or ([0] * 16))

    class Stamp:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class GoalInfo:
        def __init__(self, value=None):
            self.goal_id = UUID(value)
            self.stamp = Stamp()

    class CancelGoalRequest:
        def __init__(self):
            self.goal_info = GoalInfo()

    class CancelGoal:
        Request = CancelGoalRequest

    class QoSProfile:
        def __init__(self, depth):
            self.depth = depth
            self.reliability = None
            self.durability = None

    class ReliabilityPolicy:
        RELIABLE = 1

    class DurabilityPolicy:
        TRANSIENT_LOCAL = 1

    class GoalHandle:
        def __init__(self, waypoint, *, accepted=True):
            self.waypoint = waypoint
            self.accepted = accepted

        def get_result_async(self):
            if env.mode in {
                "success",
                "success_no_fresh",
                "arrival_one_fresh",
                "arrival_first_sample_mismatch",
                "arrival_mismatch",
                "arrival_mismatch_then_success",
                "arrival_mismatch_outside_clearance",
                "arrival_pose_missing",
                "arrival_retry_acceptance_timeout",
            }:
                return FakeFuture(SimpleNamespace(status=4))
            if env.mode == "abort":
                return FakeFuture(SimpleNamespace(status=6))
            if env.mode == "cancel_terminal":
                class CancelResultFuture:
                    def done(self):
                        return env.cancel_calls > 0

                    def result(self):
                        return SimpleNamespace(status=5)

                return CancelResultFuture()
            return FakeFuture(done=False)

        def cancel_goal_async(self):
            env.cancel_calls += 1
            return FakeFuture(SimpleNamespace(return_code=0))

    class Node:
        def __init__(self):
            self.publisher = None

        def create_publisher(self, message_type, topic, qos):
            _ = message_type, topic, qos
            self.publisher = FakePublisher(env, topic)
            env.publishers.append(self.publisher)
            return self.publisher

        def create_subscription(self, message_type, topic, callback, qos):
            _ = qos
            if message_type is GoalStatusArray and topic.endswith("/_action/status"):
                env.goal_status_callbacks[topic] = callback
            elif message_type is UInt64 and topic == "/lite3/nav/watchdog_reset_ack":
                env.watchdog_reset_ack_callback = callback
            elif message_type is UInt64 and topic == "/lite3/nav/watchdog_arm_ack":
                env.watchdog_arm_ack_callback = callback
            return SimpleNamespace(topic=topic)

        def create_client(self, service_type, service_name):
            _ = service_type
            return CancelClient(service_name)

        def get_publishers_info_by_topic(self, topic):
            _ = topic
            return [object()] if env.status_publisher_ready else []

        def destroy_subscription(self, subscription):
            _ = subscription

        def destroy_publisher(self, publisher):
            _ = publisher

        def destroy_client(self, client):
            _ = client

        def destroy_node(self):
            pass

    class ActionClient:
        def __init__(self, node, action_type, action_name):
            _ = action_type, action_name
            self.node = node

        def wait_for_server(self, timeout_sec=0.0):
            _ = timeout_sec
            return True

        def send_goal_async(self, goal):
            env.sent_goals.append(goal.pose)
            env.current_goal = goal.pose
            env.current_goal_sample_count = 0
            same_goal_attempts = sum(
                item.id == goal.pose.id for item in env.sent_goals
            )
            if (
                env.mode == "arrival_retry_acceptance_timeout"
                and goal.pose.id == "p1"
                and same_goal_attempts == 2
            ):
                return FakeFuture(done=False)
            if env.mode in {"acceptance_timeout", "acceptance_unsafe"}:
                return FakeFuture(done=False)
            if env.mode == "acceptance_unsafe_late_accept":
                handle = GoalHandle(goal.pose)

                class LateAcceptanceFuture:
                    def done(self):
                        return env.clock.now >= 0.2

                    def result(self):
                        return handle

                return LateAcceptanceFuture()
            if env.mode == "goal_rejected":
                return FakeFuture(GoalHandle(goal.pose, accepted=False))
            return FakeFuture(GoalHandle(goal.pose))

        def destroy(self):
            pass

    def status_values(topic):
        if isinstance(env.last_status, dict):
            return list(env.last_status.get(topic, []))
        return list(env.last_status)

    def goal_uuid(action_name, index):
        seed = (sum(action_name.encode("utf-8")) + index + 1) % 255
        return [seed or 1] + ([index % 255] * 15)

    class CancelClient:
        def __init__(self, service_name):
            self.service_name = service_name
            self.action_name = service_name[: -len("/_action/cancel_goal")]

        def service_is_ready(self):
            return env.cancel_service_ready

        def wait_for_service(self, timeout_sec=0.0):
            _ = timeout_sec
            return env.cancel_service_ready

        def call_async(self, request):
            env.cancel_service_calls.append(self.action_name)
            env.cancel_requests.append(request)
            topic = self.action_name + "/_action/status"
            active = [
                (index, value)
                for index, value in enumerate(status_values(topic))
                if value not in patrol.NavGoalStatusState.TERMINAL_STATUSES
            ]
            goals = [GoalInfo(goal_uuid(self.action_name, index)) for index, _ in active]
            return FakeFuture(
                SimpleNamespace(return_code=0, goals_canceling=goals)
            )

    def refresh_safety():
        if env.state is None:
            return
        env.state.mark_odom(
            now=env.clock.now,
            frame_id="map",
            x=env.state.pose.x,
            y=env.state.pose.y,
            yaw=env.state.pose.yaw,
        )
        env.state.mark_localization(now=env.clock.now, converged=True)
        env.state.mark("local_costmap", env.clock.now)
        env.state.mark("global_costmap", env.clock.now)

    def spin_once(node, timeout_sec=0.0):
        _ = node
        env.clock.now += max(float(timeout_sec), 0.05)
        if env.pending_reset_ack is not None and env.watchdog_reset_ack_callback:
            token = env.pending_reset_ack
            env.pending_reset_ack = None
            env.events.append(("watchdog_reset_ack", token))
            env.watchdog_reset_ack_callback(SimpleNamespace(data=token))
        if env.pending_arm_ack is not None and env.watchdog_arm_ack_callback:
            token = env.pending_arm_ack
            env.pending_arm_ack = None
            env.events.append(("watchdog_arm_ack", token))
            env.watchdog_arm_ack_callback(SimpleNamespace(data=token))
        if env.mode == "readiness":
            if env.status_sequences:
                env.last_status = env.status_sequences.pop(0)
            if env.emit_action_status:
                for topic, callback in env.goal_status_callbacks.items():
                    statuses = status_values(topic)
                    action_name = topic[: -len("/_action/status")]
                    env.events.append(("status", topic, tuple(statuses)))
                    callback(
                        SimpleNamespace(
                            status_list=[
                                SimpleNamespace(
                                    status=value,
                                    goal_info=GoalInfo(goal_uuid(action_name, index)),
                                )
                                for index, value in enumerate(statuses)
                            ]
                        )
                    )
            refresh_safety()
            return
        if env.mode in {"acceptance_unsafe", "acceptance_unsafe_late_accept"}:
            refresh_safety()
            env.state.mark_cmd_vel(float("nan"))
            return
        if env.mode in {
            "acceptance_timeout",
            "arrival_retry_acceptance_timeout",
            "cancel_result",
            "cancel_terminal",
            "stall",
        }:
            refresh_safety()
        if (
            env.mode in {"cancel_result", "cancel_terminal"}
            and env.current_goal is not None
            and env.clock.now >= 0.1
        ):
            env.backend.cancel_active()
        if env.mode == "success" and env.current_goal is not None:
            env.state.mark_odom(
                now=env.clock.now,
                frame_id="map",
                x=env.current_goal.x,
                y=env.current_goal.y,
                yaw=env.current_goal.yaw,
            )
        if env.current_goal is not None:
            env.current_goal_sample_count += 1
        if env.mode == "arrival_one_fresh" and env.current_goal is not None:
            if env.current_goal_sample_count == 1:
                env.state.mark_odom(
                    now=env.clock.now,
                    frame_id="map",
                    x=env.current_goal.x,
                    y=env.current_goal.y,
                    yaw=env.current_goal.yaw,
                )
        if (
            env.mode == "arrival_first_sample_mismatch"
            and env.current_goal is not None
        ):
            offset = 1.0 if env.current_goal_sample_count == 1 else 0.0
            env.state.mark_odom(
                now=env.clock.now,
                frame_id="map",
                x=env.current_goal.x + offset,
                y=env.current_goal.y,
                yaw=env.current_goal.yaw,
            )
        if env.mode == "arrival_mismatch" and env.current_goal is not None:
            env.state.mark_odom(
                now=env.clock.now,
                frame_id="map",
                x=env.current_goal.x + 0.35,
                y=env.current_goal.y,
                yaw=env.current_goal.yaw,
            )
        if (
            env.mode == "arrival_mismatch_then_success"
            and env.current_goal is not None
        ):
            same_goal_attempts = sum(
                item.id == env.current_goal.id for item in env.sent_goals
            )
            offset = (
                0.35
                if env.current_goal.id == "p1" and same_goal_attempts == 1
                else 0.0
            )
            env.state.mark_odom(
                now=env.clock.now,
                frame_id="map",
                x=env.current_goal.x + offset,
                y=env.current_goal.y,
                yaw=env.current_goal.yaw,
            )
        if (
            env.mode == "arrival_mismatch_outside_clearance"
            and env.current_goal is not None
        ):
            env.state.mark_odom(
                now=env.clock.now,
                frame_id="map",
                x=env.current_goal.x + 0.51,
                y=env.current_goal.y,
                yaw=env.current_goal.yaw,
            )
        if (
            env.mode == "arrival_pose_missing"
            and env.current_goal is not None
        ):
            env.state.mark_odom(
                now=env.clock.now,
                frame_id="map",
                x=float("nan"),
                y=env.current_goal.y,
                yaw=env.current_goal.yaw,
            )
        if (
            env.mode == "arrival_retry_acceptance_timeout"
            and env.current_goal is not None
        ):
            same_goal_attempts = sum(
                item.id == env.current_goal.id for item in env.sent_goals
            )
            if same_goal_attempts == 1:
                env.state.mark_odom(
                    now=env.clock.now,
                    frame_id="map",
                    x=env.current_goal.x + 0.35,
                    y=env.current_goal.y,
                    yaw=env.current_goal.yaw,
                )

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True
    rclpy.create_node = lambda name: Node()
    rclpy.spin_once = spin_once
    rclpy_action = ModuleType("rclpy.action")
    rclpy_action.ActionClient = ActionClient
    rclpy_qos = ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = ReliabilityPolicy
    rclpy_qos.DurabilityPolicy = DurabilityPolicy
    nav2_action = ModuleType("nav2_msgs.action")
    nav2_action.NavigateToPose = NavigateToPose
    nav2_msgs = ModuleType("nav2_msgs")
    nav2_msgs.action = nav2_action
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.UInt64 = UInt64
    std_msgs = ModuleType("std_msgs")
    std_msgs.msg = std_msgs_msg
    action_msgs_msg = ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatusArray = GoalStatusArray
    action_msgs = ModuleType("action_msgs")
    action_msgs.msg = action_msgs_msg
    action_msgs_srv = ModuleType("action_msgs.srv")
    action_msgs_srv.CancelGoal = CancelGoal
    action_msgs.srv = action_msgs_srv

    for name, module in {
        "rclpy": rclpy,
        "rclpy.action": rclpy_action,
        "rclpy.qos": rclpy_qos,
        "nav2_msgs": nav2_msgs,
        "nav2_msgs.action": nav2_action,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "action_msgs": action_msgs,
        "action_msgs.msg": action_msgs_msg,
        "action_msgs.srv": action_msgs_srv,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    monkeypatch.setattr(patrol, "time", env.clock)
    monkeypatch.setattr(
        patrol,
        "_waypoint_pose_stamped",
        lambda node, waypoint, frame_id: waypoint,
    )

    def create_safety_subscriptions(node, state, odom_topic):
        _ = node, odom_topic
        env.state = state
        state.mark_odom(now=env.clock.now, frame_id="map", x=0.0, y=0.0, yaw=0.0)
        state.mark_localization(now=env.clock.now, converged=True)
        state.mark("local_costmap", env.clock.now)
        state.mark("global_costmap", env.clock.now)
        return []

    monkeypatch.setattr(
        patrol,
        "_create_safety_subscriptions",
        create_safety_subscriptions,
    )
    monkeypatch.setattr(patrol, "_wait_for_safe_samples", lambda *args, **kwargs: None)
    monkeypatch.setattr(patrol, "_nav_graph_reasons", lambda *args, **kwargs: [])
    monkeypatch.setattr(patrol, "_controller_nonholonomic_reasons", lambda node: [])
    return env


def route():
    return patrol.WaypointRoute(
        "route",
        "map",
        False,
        [
            patrol.Waypoint("p1", 1.0, 0.0, 0.0),
            patrol.Waypoint("p2", 1.0, 1.0, 1.57),
        ],
    )


def backend(env, **overrides):
    values = {
        "timeout_sec": 1.0,
        "max_data_age_sec": 0.3,
        "route_timeout_sec": 3.0,
        "cancel_timeout_sec": 0.2,
        "goal_acceptance_timeout_sec": 0.3,
        "progress_timeout_sec": 0.5,
        "nav_idle_quiet_sec": 0.1,
    }
    values.update(overrides)
    instance = patrol.Nav2PatrolBackend(**values)
    env.backend = instance
    return instance


def test_fake_ros_sequential_success_requires_post_result_odom_and_disarms(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="success")
    instance = backend(env)

    result = instance.send_route(route())

    assert result == {
        "accepted": True,
        "status": 4,
        "missed_waypoints": [],
        "reason": None,
    }
    assert [item.id for item in env.sent_goals] == ["p1", "p2"]
    assert instance._goal_state_uncertain is False
    assert env.publishers[-1].values[-1] == 0


def test_fake_ros_terminal_result_without_new_odom_fails_closed(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="success_no_fresh")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["status"] == "ARRIVAL_MISMATCH"
    assert result["reason"] == "arrival_pose_stale"
    assert result["fresh_odom_samples"] == 0
    assert result["retry_count"] == 0
    assert len(env.sent_goals) == 1
    assert instance._goal_state_uncertain is False
    assert env.publishers[-1].values[-1] == 0


def test_fake_ros_arrival_requires_two_post_result_odom_samples(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="arrival_one_fresh")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["status"] == "ARRIVAL_MISMATCH"
    assert result["reason"] == "arrival_pose_stale"
    assert result["fresh_odom_samples"] == 1
    assert len(env.sent_goals) == 1


def test_fake_ros_arrival_uses_latest_of_two_settle_samples(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="arrival_first_sample_mismatch")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["status"] == 4
    assert [item.id for item in env.sent_goals] == ["p1", "p2"]


def test_fake_ros_explicit_goal_rejection_is_terminal_and_disarms(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="goal_rejected")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["accepted"] is False
    assert result["reason"] == "goal_rejected"
    assert instance._goal_state_uncertain is False
    assert env.publishers[-1].values[-1] == 0


def test_fake_ros_abort_is_terminal_and_does_not_advance_to_next_leg(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="abort")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["status"] == 6
    assert result["missed_waypoints"] == [0]
    assert len(env.sent_goals) == 1
    assert instance._goal_state_uncertain is False
    assert env.publishers[-1].values[-1] == 0


def test_fake_ros_retry_recovers_same_waypoint_before_advancing(
    monkeypatch,
    caplog,
):
    env = install_fake_ros(monkeypatch, mode="arrival_mismatch_then_success")
    instance = backend(env)

    with caplog.at_level(logging.WARNING):
        result = instance.send_route(route())

    assert result["status"] == 4
    assert [item.id for item in env.sent_goals] == ["p1", "p1", "p2"]
    assert "retry_count=1/1; retrying same waypoint" in caplog.text
    assert instance._goal_state_uncertain is False
    assert env.publishers[-1].values[-1] == 0


def test_fake_ros_persistent_arrival_mismatch_exhausts_one_retry(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="arrival_mismatch")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["status"] == "ARRIVAL_MISMATCH"
    assert result["reason"] == "arrival_mismatch"
    assert result["position_error_m"] == pytest.approx(0.35)
    assert result["retry_count"] == 1
    assert result["waypoint_index"] == 0
    assert result["waypoint_id"] == "p1"
    assert result["target_pose"] == {"x": 1.0, "y": 0.0, "yaw": 0.0}
    assert result["actual_pose"] == {"x": 1.35, "y": 0.0, "yaw": 0.0}
    assert [item.id for item in env.sent_goals] == ["p1", "p1"]
    assert instance._goal_state_uncertain is False
    assert env.publishers[-1].values[-1] == 0


def test_fake_ros_arrival_mismatch_outside_clearance_is_not_retried(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="arrival_mismatch_outside_clearance")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["status"] == "ARRIVAL_MISMATCH"
    assert result["position_error_m"] == pytest.approx(0.51)
    assert result["retry_count"] == 0
    assert len(env.sent_goals) == 1


def test_fake_ros_zero_arrival_retry_limit_preserves_immediate_failure(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="arrival_mismatch")
    instance = backend(env, arrival_retry_limit=0)

    result = instance.send_route(route())

    assert result["status"] == "ARRIVAL_MISMATCH"
    assert result["retry_count"] == 0
    assert len(env.sent_goals) == 1


def test_fake_ros_missing_arrival_pose_is_never_retried(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="arrival_pose_missing")
    instance = backend(env)

    result = instance.send_route(route())

    assert result["reason"] == "arrival_pose_missing"
    assert result["actual_pose"] is None
    assert result["retry_count"] == 0
    assert len(env.sent_goals) == 1


def test_fake_ros_retry_acceptance_timeout_latches_uncertainty(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="arrival_retry_acceptance_timeout")
    instance = backend(env, max_data_age_sec=10.0)

    result = instance.send_route(route())

    assert result["status"] == "ACCEPTANCE_TIMEOUT"
    assert result["accepted"] is True
    assert [item.id for item in env.sent_goals] == ["p1", "p1"]
    assert instance._goal_state_uncertain is True
    assert env.publishers[-1].values[-1] != 0


@pytest.mark.parametrize("retry_limit", [-1, 2, True])
def test_arrival_retry_limit_is_strictly_bounded(retry_limit):
    with pytest.raises(ValueError, match="arrival_retry_limit must be 0 or 1"):
        patrol.Nav2PatrolBackend(arrival_retry_limit=retry_limit)


def test_fake_ros_acceptance_has_its_own_short_timeout_and_latches_uncertainty(
    monkeypatch,
):
    env = install_fake_ros(monkeypatch, mode="acceptance_timeout")
    instance = backend(
        env,
        timeout_sec=9.0,
        max_data_age_sec=10.0,
        goal_acceptance_timeout_sec=0.3,
    )

    result = instance.send_route(route())

    assert result["status"] == "ACCEPTANCE_TIMEOUT"
    assert env.clock.now < 1.0
    assert instance._goal_state_uncertain is True
    assert env.publishers[-1].values[-1] != 0
    with pytest.raises(RuntimeError, match="goal state is uncertain"):
        instance.prepare_route()


def test_fake_ros_unsafe_acceptance_wait_stops_heartbeat_and_fails_closed(
    monkeypatch,
):
    env = install_fake_ros(monkeypatch, mode="acceptance_unsafe")
    instance = backend(
        env,
        max_data_age_sec=10.0,
        cancel_timeout_sec=0.6,
        goal_acceptance_timeout_sec=1.0,
    )

    result = instance.send_route(route())

    assert result["status"] == "CANCEL_TIMEOUT"
    assert result["reason"] == "safety:cmd_vel_invalid"
    assert len(env.publishers[-1].values) == 1
    assert env.publishers[-1].values[0] != 0
    assert instance._goal_state_uncertain is True


def test_late_accept_after_unsafe_wait_never_restarts_heartbeat(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="acceptance_unsafe_late_accept")
    instance = backend(
        env,
        max_data_age_sec=10.0,
        cancel_timeout_sec=0.6,
        goal_acceptance_timeout_sec=1.0,
    )

    result = instance.send_route(route())

    assert result["status"] == "CANCEL_TIMEOUT"
    assert result["reason"] == "safety:cmd_vel_invalid"
    assert env.cancel_calls == 1
    assert len(env.publishers[-1].values) == 1
    assert env.publishers[-1].values[0] != 0
    assert instance._goal_state_uncertain is True


def test_fake_ros_cancel_timeout_latches_uncertainty_without_disarm(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="cancel_result")
    instance = backend(env, max_data_age_sec=10.0)

    result = instance.send_route(route())

    assert result["status"] == "CANCEL_TIMEOUT"
    assert result["reason"] == "operator_cancel"
    assert env.cancel_calls == 1
    assert instance._goal_state_uncertain is True
    assert env.publishers[-1].values[-1] != 0


def test_fake_ros_stall_requests_cancel_then_latches_if_result_never_arrives(
    monkeypatch,
):
    env = install_fake_ros(monkeypatch, mode="stall")
    instance = backend(
        env,
        max_data_age_sec=10.0,
        progress_timeout_sec=0.2,
        cancel_timeout_sec=0.2,
    )

    result = instance.send_route(route())

    assert result["status"] == "CANCEL_TIMEOUT"
    assert result["reason"] == "stalled"
    assert env.cancel_calls == 1
    assert instance._goal_state_uncertain is True
    assert env.publishers[-1].values[-1] != 0


def test_fake_ros_operator_cancel_terminal_result_disarms(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="cancel_terminal")
    instance = backend(env, max_data_age_sec=10.0)

    result = instance.send_route(route())

    assert result["status"] == 5
    assert result["reason"] == "operator_cancel"
    assert env.cancel_calls == 1
    assert instance._goal_state_uncertain is False
    assert env.publishers[-1].values[-1] == 0


def test_fake_ros_never_sends_goal_without_watchdog_arm_ack(monkeypatch):
    env = install_fake_ros(monkeypatch, mode="success", emit_arm_ack=False)
    instance = backend(env, timeout_sec=0.3, max_data_age_sec=10.0)

    result = instance.send_route(route())

    assert result["status"] == "WATCHDOG_ARM_FAILED"
    assert result["reason"] == "watchdog_arm_ack_timeout"
    assert env.sent_goals == []


def test_restart_readiness_resets_watchdog_only_after_all_goals_terminal(
    monkeypatch,
):
    env = install_fake_ros(
        monkeypatch,
        mode="readiness",
        status_sequences=[[2], [3], [5]],
    )
    instance = backend(env, max_data_age_sec=10.0)

    instance.wait_until_ready(timeout_sec=1.0)

    active_index = env.events.index(
        ("status", "/navigate_to_pose/_action/status", (2,))
    )
    terminal_index = env.events.index(
        ("status", "/navigate_to_pose/_action/status", (5,))
    )
    reset_index = next(
        index
        for index, event in enumerate(env.events)
        if event[0] == "watchdog_reset_ack"
    )
    assert active_index < terminal_index < reset_index


def test_fresh_never_used_action_servers_pass_after_clean_cancel_barrier(
    monkeypatch,
):
    env = install_fake_ros(
        monkeypatch,
        mode="readiness",
        emit_action_status=False,
    )
    instance = backend(env, max_data_age_sec=10.0)

    instance.wait_until_ready(timeout_sec=1.0)

    assert set(env.cancel_service_calls) == {
        "/navigate_to_pose",
        "/FollowWaypoints",
        "/follow_path",
        "/spin",
        "/backup",
        "/wait",
    }
    assert all(request.goal_info.goal_id.uuid == [0] * 16 for request in env.cancel_requests)
    assert all(request.goal_info.stamp.sec == 0 for request in env.cancel_requests)
    assert all(request.goal_info.stamp.nanosec == 0 for request in env.cancel_requests)
    assert any(event[0] == "watchdog_reset_ack" for event in env.events)


def test_readiness_never_returns_without_watchdog_reset_ack(monkeypatch):
    env = install_fake_ros(
        monkeypatch,
        mode="readiness",
        emit_action_status=False,
        emit_reset_ack=False,
    )
    instance = backend(env, max_data_age_sec=10.0)

    with pytest.raises(TimeoutError, match="watchdog_reset_ack_missing"):
        instance.wait_until_ready(timeout_sec=0.8)

    assert env.sent_goals == []


def test_restart_readiness_never_resets_watchdog_while_goal_is_active(monkeypatch):
    env = install_fake_ros(
        monkeypatch,
        mode="readiness",
        status_sequences=[[2]],
    )
    instance = backend(env, max_data_age_sec=10.0)

    with pytest.raises(TimeoutError, match="navigation_goal_not_terminal"):
        instance.wait_until_ready(timeout_sec=0.3)

    assert not any(
        event[0] == "publish_topic" and event[1] == "/lite3/nav/watchdog_reset"
        for event in env.events
    )


def test_restart_readiness_blocks_orphan_follow_path_goal(monkeypatch):
    env = install_fake_ros(
        monkeypatch,
        mode="readiness",
        status_sequences=[
            {
                "/navigate_to_pose/_action/status": [],
                "/follow_path/_action/status": [2],
            }
        ],
    )
    instance = backend(env, max_data_age_sec=10.0)

    with pytest.raises(TimeoutError, match="/follow_path"):
        instance.wait_until_ready(timeout_sec=0.3)

    assert not any(
        event[0] == "publish_topic" and event[1] == "/lite3/nav/watchdog_reset"
        for event in env.events
    )


def test_restart_readiness_blocks_orphan_wait_goal(monkeypatch):
    env = install_fake_ros(
        monkeypatch,
        mode="readiness",
        status_sequences=[{"/wait/_action/status": [3]}],
    )
    instance = backend(env, max_data_age_sec=10.0)

    with pytest.raises(TimeoutError, match="/wait"):
        instance.wait_until_ready(timeout_sec=0.3)

    assert not any(
        event[0] == "publish_topic" and event[1] == "/lite3/nav/watchdog_reset"
        for event in env.events
    )


def test_restart_readiness_fails_closed_when_cancel_service_is_missing(monkeypatch):
    env = install_fake_ros(
        monkeypatch,
        mode="readiness",
        cancel_service_ready=False,
    )
    instance = backend(env, max_data_age_sec=10.0)

    with pytest.raises(TimeoutError, match="navigation_cancel_service_missing"):
        instance.wait_until_ready(timeout_sec=0.3)

    assert env.cancel_requests == []
    assert not any(
        event[0] == "publish_topic" and event[1] == "/lite3/nav/watchdog_reset"
        for event in env.events
    )
