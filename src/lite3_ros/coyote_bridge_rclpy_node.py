"""Foxy nodes for coyote spool → internal ROS topics → MQTT bridge."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from lite3_mqtt.coyote_bridge import (
    CoyoteMediaWorker,
    CoyoteMotionController,
    CoyoteSpoolReader,
)
from lite3_mqtt.contract import DetectionType, InternalRosTopics, PatrolAction
from lite3_motion.pointcloud_clearance import extract_clearances
from lite3_perception.coyote_spool import (
    INTERNAL_IMAGE_TOPIC,
    INTERNAL_STATUS_TOPIC,
)


MAX_CONTROL_EVENTS_PER_TICK = 8


try:
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import LaserScan, PointCloud2
    from std_msgs.msg import Int32, String, UInt64
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object  # type: ignore[misc, assignment]
    String = None
    Int32 = None


@dataclass(frozen=True)
class CoyoteRosTopics:
    status_topic: str = INTERNAL_STATUS_TOPIC
    complete_topic: str = "/lite3/data/coyote/complete"
    glass_status_topic: str = "/lite3/data/glass/status"
    image_topic: str = INTERNAL_IMAGE_TOPIC
    # Direct IQ9 receiver; never depend on the perception-host motion_receiver.
    robot_basic_state_topic: str = "/lite3/motion/robot_basic_state"
    robot_motion_state_topic: str = "/lite3/motion/robot_motion_state"
    motion_state_topic: str = "/lite3/motion/state"


class CoyoteMotionOutputNode(Node):
    """Provide exclusive, sensor-gated `/cmd_vel` output for search behavior."""

    GUARDED_ACTIONS = (
        "/navigate_to_pose",
        "/FollowWaypoints",
        "/follow_path",
        "/spin",
        "/backup",
    )
    ACTIVE_GOAL_STATES = {
        1,  # STATUS_ACCEPTED
        2,  # STATUS_EXECUTING
        3,  # STATUS_CANCELING
    }

    def __init__(
        self,
        *,
        cmd_vel_topic: str = "/cmd_vel",
        scan_topic: str = "/scan",
        pointcloud_topic: str = "/rslidar_points",
        motion_state_topic: str = "/lite3/motion/state",
        nav_idle_quiet_sec: float = 0.50,
        release_delay_sec: float = 0.25,
    ) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is required to create CoyoteMotionOutputNode")
        if min(nav_idle_quiet_sec, release_delay_sec) <= 0.0:
            raise ValueError("motion output timing must be positive")
        super().__init__("lite3_coyote_motion_output")
        self.cmd_vel_topic = cmd_vel_topic
        self.nav_idle_quiet_sec = nav_idle_quiet_sec
        self.release_delay_sec = release_delay_sec
        self.controller = None  # type: Optional[CoyoteMotionController]
        self.output_lock = threading.RLock()
        self.cmd_vel_pub = None
        self.release_deadline = None  # type: Optional[float]
        self.navigate_graph_ready = False
        self.last_graph_check = 0.0
        self.action_active = {
            action_name: False for action_name in self.GUARDED_ACTIONS
        }
        self.nav_idle_since = None  # type: Optional[float]
        self.handoff_token = None  # type: Optional[int]
        self.handoff_acked = False
        self.next_handoff_publish = 0.0
        self._last_pointcloud_clearance_log = 0.0
        self.watchdog_reset_pub = self.create_publisher(
            UInt64,
            "/lite3/nav/watchdog_reset",
            10,
        )
        self.create_subscription(
            UInt64,
            "/lite3/nav/watchdog_reset_ack",
            self._on_watchdog_reset_ack,
            10,
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, motion_state_topic, self._on_motion_state, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_subscriptions = []
        for action_name in self.GUARDED_ACTIONS:
            self.status_subscriptions.append(
                self.create_subscription(
                    GoalStatusArray,
                    action_name + "/_action/status",
                    self._status_callback(action_name),
                    status_qos,
                )
            )
        self.create_timer(0.05, self._release_if_due)

    def bind_controller(self, controller: CoyoteMotionController) -> None:
        if self.controller is not None:
            raise RuntimeError("coyote motion controller is already bound")
        self.controller = controller

    def motion_ready(self) -> bool:
        self._refresh_nav_graph()
        now = time.monotonic()
        return (
            self.navigate_graph_ready
            and self.handoff_acked
            and not any(self.action_active.values())
            and self.nav_idle_since is not None
            and now - self.nav_idle_since >= self.nav_idle_quiet_sec
        )

    def acquire(self) -> None:
        with self.output_lock:
            if self.cmd_vel_pub is None:
                self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            self.send_cmd_vel(0.0, 0.0, 0.0)
            self.release_deadline = None
            token = int(time.time_ns() & ((1 << 63) - 1)) or 1
            self.handoff_token = token
            self.handoff_acked = False
            self.next_handoff_publish = 0.0
            self.nav_idle_since = None

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            raise ValueError("cmd_vel values must be finite")
        with self.output_lock:
            moving = any(abs(value) > 1e-9 for value in (vx, vy, wz))
            # Do not create a second /cmd_vel publisher merely to repeat idle zeroes.
            # If this behavior already owns output, however, publish zero before
            # relinquishing it.
            if self.cmd_vel_pub is None and not moving:
                return
            if self.cmd_vel_pub is None:
                self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            if moving:
                self.release_deadline = None
            message = Twist()
            message.linear.x = float(vx)
            message.linear.y = float(vy)
            message.angular.z = float(wz)
            self.cmd_vel_pub.publish(message)

    def release(self) -> None:
        with self.output_lock:
            if self.cmd_vel_pub is None:
                return
            self.send_cmd_vel(0.0, 0.0, 0.0)
            if self.release_deadline is None:
                self.release_deadline = time.monotonic() + self.release_delay_sec
            self.handoff_token = None
            self.handoff_acked = False
            self.next_handoff_publish = 0.0

    def _on_scan(self, message) -> None:
        if self.controller is None:
            return
        try:
            self.controller.update_scan(
                message.ranges,
                float(message.angle_min),
                float(message.angle_increment),
            )
        except Exception as exc:
            self.get_logger().error("coyote LaserScan rejected: {}".format(exc))

    def _on_pointcloud(self, message) -> None:
        """Translate perception-host RoboSense PointCloud2 into clearances."""
        if self.controller is None:
            return
        try:
            front_m, left_m, right_m = extract_clearances(message)
            self.controller.update_clearances(
                front_m=front_m,
                left_m=left_m,
                right_m=right_m,
            )
            now = time.monotonic()
            if now - self._last_pointcloud_clearance_log >= 2.0:
                self._last_pointcloud_clearance_log = now
                self.get_logger().info(
                    "POINTCLOUD_CLEARANCE front=%s left=%s right=%s"
                    % tuple(
                        "%.2fm" % value if value is not None else "none"
                        for value in (front_m, left_m, right_m)
                    )
                )
        except Exception as exc:
            self.get_logger().error("coyote PointCloud2 rejected: {}".format(exc))

    def _on_motion_state(self, message) -> None:
        if self.controller is None:
            return
        try:
            payload = json.loads(message.data)
            position = payload["pos_world"]
            if not isinstance(position, list) or len(position) != 3:
                raise ValueError("pos_world must have exactly three values")
            yaw = float(payload["yaw_rad"])
            self.controller.update_odom(
                float(position[0]),
                float(position[1]),
                yaw,
            )
        except Exception as exc:
            self.get_logger().error("coyote direct motion state rejected: {}".format(exc))

    def _status_callback(self, action_name: str):
        def on_status(message) -> None:
            active = any(
                int(item.status) in self.ACTIVE_GOAL_STATES
                for item in message.status_list
            )
            self.action_active[action_name] = active
            if action_name == "/navigate_to_pose":
                self.navigate_graph_ready = True
            if any(self.action_active.values()):
                self.nav_idle_since = None
            elif self.nav_idle_since is None:
                self.nav_idle_since = time.monotonic()

        return on_status

    def _on_watchdog_reset_ack(self, message) -> None:
        token = self.handoff_token
        if token is not None and int(message.data) == token:
            self.handoff_acked = True

    def _refresh_nav_graph(self) -> None:
        now = time.monotonic()
        if now - self.last_graph_check < 0.20:
            return
        self.last_graph_check = now
        try:
            publishers = self.get_publishers_info_by_topic(
                "/navigate_to_pose/_action/status"
            )
        except Exception:
            publishers = []
        self.navigate_graph_ready = bool(publishers)
        if not self.navigate_graph_ready:
            self.nav_idle_since = None
        elif not any(self.action_active.values()) and self.nav_idle_since is None:
            self.nav_idle_since = now

    def _publish_handoff_reset(self) -> None:
        token = self.handoff_token
        if token is None or self.handoff_acked:
            return
        now = time.monotonic()
        if now < self.next_handoff_publish:
            return
        message = UInt64()
        message.data = int(token)
        self.watchdog_reset_pub.publish(message)
        self.next_handoff_publish = now + 0.20

    def _release_if_due(self) -> None:
        with self.output_lock:
            self._refresh_nav_graph()
            self._publish_handoff_reset()
            if (
                self.cmd_vel_pub is None
                or self.release_deadline is None
                or time.monotonic() < self.release_deadline
            ):
                return
            publisher = self.cmd_vel_pub
            self.cmd_vel_pub = None
            self.release_deadline = None
            self.destroy_publisher(publisher)

    def destroy_node(self) -> None:
        try:
            if self.cmd_vel_pub is not None:
                self.send_cmd_vel(0.0, 0.0, 0.0)
        finally:
            super().destroy_node()


class CoyoteSpoolAdapterNode(Node):
    """Expose the native QNN process spool as small internal ROS messages."""

    def __init__(
        self,
        *,
        reader: CoyoteSpoolReader,
        media_ready: Callable[[], bool],
        topics: Optional[CoyoteRosTopics] = None,
        poll_period_sec: float = 0.05,
    ) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is required to create CoyoteSpoolAdapterNode")
        super().__init__("lite3_coyote_spool_adapter")
        self.reader = reader
        self.media_ready = media_ready
        self.topics = topics or CoyoteRosTopics()
        self.status_pub = self.create_publisher(String, self.topics.status_topic, 10)
        self.image_pub = self.create_publisher(String, self.topics.image_topic, 10)
        self.create_timer(poll_period_sec, self._poll)

    def _poll(self) -> None:
        try:
            status = self.reader.read_status_if_changed()
        except Exception as exc:
            self.get_logger().error("invalid coyote status spool: {}".format(exc))
        else:
            if status is not None:
                message = String()
                message.data = status
                self.status_pub.publish(message)

        if not self.media_ready():
            return
        for _ready_path, manifest in self.reader.ready_manifests():
            message = String()
            message.data = json.dumps(
                manifest,
                ensure_ascii=False,
                separators=(",", ":"),
            )
            if manifest["kind"] == "image":
                self.image_pub.publish(message)
            else:
                self.get_logger().warning(
                    "unsupported coyote media kind: {}".format(manifest["kind"])
                )


class CoyoteMqttBridgeNode(Node):
    """Consume internal ROS events and keep motion/media work off callbacks."""

    def __init__(
        self,
        *,
        reader: CoyoteSpoolReader,
        media_worker: CoyoteMediaWorker,
        motion_controller: CoyoteMotionController,
        search_events=None,
        on_coyote_mission_start: Optional[Callable[[str], None]] = None,
        on_coyote_home_reached: Optional[Callable[[str], None]] = None,
        on_robot_basic_state: Optional[Callable[[int], None]] = None,
        on_robot_motion_state: Optional[Callable[[int], None]] = None,
        on_motion_state: Optional[Callable[[dict], None]] = None,
        on_coyote_status: Optional[Callable[[object], None]] = None,
        on_coyote_complete_event: Optional[Callable[[str, str], None]] = None,
        topics: Optional[CoyoteRosTopics] = None,
        control_hz: float = 20.0,
    ) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is required to create CoyoteMqttBridgeNode")
        if control_hz < 20.0:
            raise ValueError("coyote control_hz must be at least 20")
        super().__init__("lite3_coyote_mqtt_bridge")
        self.reader = reader
        self.media_worker = media_worker
        self.motion_controller = motion_controller
        self.search_events = search_events
        self.on_coyote_mission_start = on_coyote_mission_start
        self.on_coyote_home_reached = on_coyote_home_reached
        self.on_robot_basic_state = on_robot_basic_state
        self.on_robot_motion_state = on_robot_motion_state
        self.on_motion_state = on_motion_state
        self.on_coyote_status = on_coyote_status
        self.on_coyote_complete_event = on_coyote_complete_event
        self.topics = topics or CoyoteRosTopics()
        self._last_motion_trace = None
        self.mission_event_pub = self.create_publisher(
            String,
            InternalRosTopics.MISSION_EVENT,
            10,
        )
        self.coyote_complete_pub = self.create_publisher(
            String,
            self.topics.complete_topic,
            10,
        )
        self.create_subscription(
            String,
            self.topics.status_topic,
            self._on_coyote_status,
            10,
        )
        self.create_subscription(
            String,
            self.topics.complete_topic,
            self._on_coyote_complete_event,
            10,
        )
        self.create_subscription(
            String,
            self.topics.glass_status_topic,
            self._on_glass_status,
            10,
        )
        self.create_subscription(String, self.topics.image_topic, self._on_image, 10)
        self.create_subscription(
            Int32,
            self.topics.robot_basic_state_topic,
            self._on_robot_basic_state,
            10,
        )
        self.create_subscription(
            Int32,
            self.topics.robot_motion_state_topic,
            self._on_robot_motion_state,
            10,
        )
        self.create_subscription(
            String,
            self.topics.motion_state_topic,
            self._on_motion_state,
            10,
        )
        self.create_subscription(
            String,
            InternalRosTopics.MISSION_START,
            self._on_mission_start,
            10,
        )
        self.create_subscription(
            String,
            InternalRosTopics.MISSION_HOME_REACHED,
            self._on_mission_home_reached,
            10,
        )
        self.create_timer(1.0 / control_hz, self._tick_motion)

    def _on_coyote_status(self, message) -> None:
        self._handle_status(message, DetectionType.COYOTE)

    def publish_coyote_complete_event(self, event_id: str, completion_reason: str) -> None:
        """Publish the terminal Coyote handoff before any completion motion."""
        message = String()
        message.data = json.dumps(
            {
                "event_id": event_id,
                "completion_reason": completion_reason,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.coyote_complete_pub.publish(message)
        self.get_logger().info(
            "coyote COMPLETE ROS event published topic={} event_id={} reason={}".format(
                self.topics.complete_topic,
                event_id,
                completion_reason,
            )
        )

    def _on_coyote_complete_event(self, message) -> None:
        try:
            event = json.loads(message.data)
            if (
                not isinstance(event, dict)
                or not isinstance(event.get("event_id"), str)
                or not event["event_id"]
                or not isinstance(event.get("completion_reason"), str)
                or not event["completion_reason"]
            ):
                raise ValueError("invalid coyote COMPLETE event")
            if self.on_coyote_complete_event is None:
                raise RuntimeError("coyote COMPLETE handler is not configured")
            self.on_coyote_complete_event(
                event["event_id"],
                event["completion_reason"],
            )
        except Exception as exc:
            self.get_logger().error("coyote COMPLETE event rejected: {}".format(exc))

    def _on_glass_status(self, message) -> None:
        self._handle_status(message, DetectionType.BROKEN_CUP)

    def _handle_status(self, message, detection_type: DetectionType) -> None:
        try:
            status = self.motion_controller.handle_status(message.data, detection_type)
        except Exception as exc:
            self.get_logger().error("coyote status rejected: {}".format(exc))
            return
        if detection_type is DetectionType.COYOTE and self.on_coyote_status is not None:
            try:
                self.on_coyote_status(status)
            except Exception as exc:
                # An auxiliary observer (for example RealSense return-vector
                # capture) must never interrupt the primary motion stream.
                self.get_logger().error("coyote status observer failed: {}".format(exc))

    def _on_image(self, message) -> None:
        self._submit_manifest(message.data, expected_kind="image")

    def _on_robot_basic_state(self, message) -> None:
        if self.on_robot_basic_state is None:
            return
        try:
            self.on_robot_basic_state(int(message.data))
        except Exception as exc:
            self.get_logger().error("robot basic state rejected: {}".format(exc))

    def _on_robot_motion_state(self, message) -> None:
        if self.on_robot_motion_state is None:
            return
        try:
            self.on_robot_motion_state(int(message.data))
        except Exception as exc:
            self.get_logger().error("robot motion state rejected: {}".format(exc))

    def _on_motion_state(self, message) -> None:
        if self.on_motion_state is None:
            return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("motion state must be a JSON object")
            self.on_motion_state(payload)
        except Exception as exc:
            self.get_logger().error("robot motion payload rejected: {}".format(exc))

    def _on_mission_start(self, message) -> None:
        try:
            event = json.loads(message.data)
            if (
                not isinstance(event, dict)
                or event.get("source") != "coyote"
                or event.get("requested_action") != "START_SEARCH"
                or not isinstance(event.get("event_id"), str)
                or not event["event_id"]
            ):
                return
            if self.on_coyote_mission_start is None:
                raise RuntimeError("coyote mission start handler is not configured")
            self.on_coyote_mission_start(event["event_id"])
        except Exception as exc:
            self.get_logger().error("coyote mission start rejected: {}".format(exc))

    def _on_mission_home_reached(self, message) -> None:
        try:
            event = json.loads(message.data)
            if (
                not isinstance(event, dict)
                or event.get("source") != "coyote"
                or event.get("state") != "HOME_REACHED"
                or not isinstance(event.get("event_id"), str)
                or not event["event_id"]
            ):
                return
            if self.on_coyote_home_reached is None:
                raise RuntimeError("coyote home-reached handler is not configured")
            self.on_coyote_home_reached(event["event_id"])
        except Exception as exc:
            self.get_logger().error("coyote home-reached rejected: {}".format(exc))

    def publish_return_home_request(self, event_id: str, *, source: str) -> None:
        """Handoff only after a detection motion session reaches COMPLETE."""
        if source not in {"coyote", "broken_cup"}:
            raise ValueError("unsupported mission event source")
        message = String()
        message.data = json.dumps(
            {
                "event_id": event_id,
                "source": source,
                "state": "COMPLETED",
                "requested_action": "RETURN_HOME",
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.mission_event_pub.publish(message)
        self.get_logger().info(
            "mission event published topic={} source={} event_id={} action=RETURN_HOME".format(
                InternalRosTopics.MISSION_EVENT,
                source,
                event_id,
            )
        )

    def _submit_manifest(self, body: str, *, expected_kind: str) -> None:
        try:
            manifest = json.loads(body)
            if not isinstance(manifest, dict) or manifest.get("kind") != expected_kind:
                raise ValueError("internal coyote media kind mismatch")
            claim = self.reader.claim(
                str(manifest.get("event_id", "")),
                expected_kind,
            )
            if claim is None:
                return
            if not self.media_worker.submit(claim):
                self.reader.release(claim)
                raise RuntimeError("coyote media queue is full")
        except Exception as exc:
            self.get_logger().error("coyote media event rejected: {}".format(exc))

    def _tick_motion(self) -> None:
        try:
            stop_barrier = False
            processed_count = 0
            if self.search_events is not None:
                while processed_count < MAX_CONTROL_EVENTS_PER_TICK:
                    try:
                        item = self.search_events.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        processed_count += 1
                        if isinstance(item, tuple) and item and item[0] == "patrol":
                            action = PatrolAction(item[1])
                            timestamp = int(item[2])
                            accepted = self.motion_controller.handle_patrol_command(
                                action,
                                timestamp,
                            )
                            stop_barrier = stop_barrier or accepted
                        else:
                            if isinstance(item, tuple) and item and item[0] == "search":
                                event_id = str(item[1])
                                detection_type = DetectionType(item[2])
                            else:
                                event_id = str(item)
                                detection_type = DetectionType.COYOTE
                            started = self.motion_controller.start_search(
                                event_id,
                                detection_type,
                            )
                            stop_barrier = stop_barrier or started
                            if started:
                                self.get_logger().info(
                                    "detection search armed event_id={} type={}".format(
                                        event_id,
                                        detection_type.value,
                                    )
                                )
                    except Exception as exc:
                        self.motion_controller.stop()
                        stop_barrier = True
                        self.get_logger().error(
                            "coyote control event rejected: {}".format(exc)
                        )
                    finally:
                        self.search_events.task_done()
            # start_search()/stop() already emitted zero. Do not emit a
            # non-zero command in the same control callback.
            if stop_barrier:
                return
            command = self.motion_controller.tick()
            trace = (self.motion_controller.last_reason, command)
            if trace != self._last_motion_trace:
                self._last_motion_trace = trace
                self.get_logger().info(
                    "coyote motion reason={} cmd_vel=({}, {}, {})".format(
                        trace[0], trace[1][0], trace[1][1], trace[1][2]
                    )
                )
        except Exception as exc:
            self.get_logger().error("coyote motion output failed: {}".format(exc))

    def destroy_node(self) -> None:
        try:
            self.motion_controller.stop()
        except Exception as exc:
            self.get_logger().error("coyote final stop failed: {}".format(exc))
        finally:
            super().destroy_node()
