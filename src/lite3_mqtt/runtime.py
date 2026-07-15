"""Route inbound MQTT commands to patrol and detection services."""

from __future__ import annotations

import logging
import threading
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from typing import Deque, Optional, Set

from lite3_mqtt.contract import (
    DetectionType,
    PatrolAction,
    Topics,
    parse_detection_trigger,
    parse_patrol_command,
)
from lite3_mqtt.media import DetectionMediaPublisher
from lite3_mqtt.patrol import ContinuousPatrolController


class Lite3MqttRuntime:
    def __init__(
        self,
        *,
        patrol: ContinuousPatrolController,
        detection_publisher: DetectionMediaPublisher,
        max_pending_detections: int = 2,
        publish_trigger_media: bool = True,
        patrol_only: bool = False,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if max_pending_detections <= 0:
            raise ValueError("max_pending_detections must be positive")
        self.patrol = patrol
        self.detection_publisher = detection_publisher
        self.publish_trigger_media = bool(publish_trigger_media)
        self.patrol_only = bool(patrol_only)
        self.logger = logger or logging.getLogger(__name__)
        self._detection_worker = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="mqtt-detection-media",
        )
        self._motion_prepare_worker = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="mqtt-detection-motion-prepare",
        )
        self._events_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._closed = False
        self._motion_prepare_pending = False
        self._detection_slots = threading.BoundedSemaphore(max_pending_detections)
        self._recent_events = deque(maxlen=256)  # type: Deque[str]
        self._recent_event_set = set()  # type: Set[str]
        self._recent_patrol_commands = deque(maxlen=64)
        self._recent_patrol_command_set = set()
        self._last_patrol_timestamp = None  # type: Optional[int]

    def handle_message(self, topic: str, payload: bytes) -> None:
        try:
            if topic == Topics.AUTO_PATROL:
                command = parse_patrol_command(payload)
                with self._state_lock:
                    if self._closed:
                        self.logger.warning(
                            "MQTT patrol command ignored while runtime is closing"
                        )
                        return
                    if self.patrol_only:
                        self._handle_patrol(command.action)
                        return
                    if not self._accept_patrol_command(
                        command.timestamp,
                        command.action,
                    ):
                        return
                    try:
                        self._handle_patrol(command.action)
                    except Exception:
                        # The command was not completed. Allow an operator to
                        # retry the exact payload, including safety commands.
                        self._forget_patrol_command(
                            command.timestamp,
                            command.action,
                        )
                        raise
                return
            if topic in {Topics.SOUND_DETECT, Topics.COYOTE_DETECT}:
                if self.patrol_only:
                    self.logger.info("detection ignored while patrol-only mode is active")
                    return
                trigger = parse_detection_trigger(topic, payload)
                # Detection/search takes motion ownership from patrol. Cancel
                # Nav2 before the bridge is allowed to issue any search step.
                self.patrol.stop()
                self.prepare_detection_motion()
                self.logger.info(
                    "patrol stopped for detection search event_id=%s type=%s",
                    trigger.event_id,
                    trigger.detection_type.value,
                )
                if self.publish_trigger_media:
                    self.report_detection(
                        trigger.detection_type,
                        event_id=trigger.event_id,
                    )
                else:
                    self.logger.info(
                        "trigger media suppressed; awaiting perception spool "
                        "event_id=%s type=%s",
                        trigger.event_id,
                        trigger.detection_type.value,
                    )
                return
            self.logger.warning("ignored unsupported MQTT topic=%s", topic)
        except Exception:
            self.logger.exception("MQTT message rejected topic=%s payload=%r", topic, payload[:256])

    def report_detection(
        self,
        detection_type: DetectionType,
        *,
        event_id: Optional[str] = None,
    ) -> bool:
        if not self._detection_slots.acquire(blocking=False):
            self.logger.warning("detection dropped: media queue is full event_id=%s", event_id)
            return False
        with self._state_lock:
            if self._closed:
                self._detection_slots.release()
                return False
            if event_id is not None and not self._remember_event(event_id):
                self._detection_slots.release()
                self.logger.info("duplicate detection ignored event_id=%s", event_id)
                return False
            try:
                self._detection_worker.submit(
                    self._publish_detection_task,
                    detection_type,
                    event_id,
                )
            except Exception:
                if event_id is not None:
                    self._forget_event(event_id)
                self._detection_slots.release()
                raise
        return True

    def prepare_detection_motion(self) -> bool:
        """Prepare Nav2/watchdog asynchronously without sending a goal."""
        with self._state_lock:
            if self._closed or self._motion_prepare_pending:
                return False
            self._motion_prepare_pending = True
            try:
                self._motion_prepare_worker.submit(self._prepare_motion_task)
            except Exception:
                self._motion_prepare_pending = False
                raise
        return True

    def close(self) -> None:
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
        try:
            self.patrol.close()
        finally:
            # Executor threads are non-daemon. Always drain them even when a
            # patrol backend reports an error during shutdown.
            try:
                self._motion_prepare_worker.shutdown(wait=True)
            finally:
                self._detection_worker.shutdown(wait=True)

    def handle_connection_lost(self) -> None:
        with self._state_lock:
            if self._closed:
                return
        self.logger.error("MQTT connection lost; stopping active patrol")
        self.patrol.stop()

    def _handle_patrol(self, action: PatrolAction) -> None:
        if action is PatrolAction.START:
            started = self.patrol.start()
            self.logger.info("patrol START received started=%s", started)
        elif action is PatrolAction.STOP:
            was_active = bool(getattr(self.patrol, "active", False))
            self.patrol.stop()
            self.logger.info("patrol STOP received was_active=%s", was_active)
        elif action is PatrolAction.RETURN_HOME:
            started = self.patrol.return_home()
            self.logger.info("patrol RETURN_HOME received started=%s", started)
        elif action is PatrolAction.EMERGENCY_STOP:
            self.patrol.emergency_stop()
            self.logger.error("patrol EMERGENCY_STOP received latched=True")
        elif action is PatrolAction.RESET:
            self.patrol.reset()
            self.logger.info("patrol RESET received")

    def _accept_patrol_command(self, timestamp: int, action: PatrolAction) -> bool:
        key = (timestamp, action.value)
        if key in self._recent_patrol_command_set:
            self.logger.info("duplicate patrol command ignored action=%s", action.value)
            return False
        if (
            action in {PatrolAction.START, PatrolAction.RETURN_HOME, PatrolAction.RESET}
            and self._last_patrol_timestamp is not None
            and timestamp <= self._last_patrol_timestamp
        ):
            self.logger.warning(
                "out-of-order patrol command ignored action=%s timestamp=%s last=%s",
                action.value,
                timestamp,
                self._last_patrol_timestamp,
            )
            return False
        if len(self._recent_patrol_commands) == self._recent_patrol_commands.maxlen:
            evicted = self._recent_patrol_commands.popleft()
            self._recent_patrol_command_set.discard(evicted)
        self._recent_patrol_commands.append(key)
        self._recent_patrol_command_set.add(key)
        if self._last_patrol_timestamp is None:
            self._last_patrol_timestamp = timestamp
        else:
            self._last_patrol_timestamp = max(self._last_patrol_timestamp, timestamp)
        return True

    def _forget_patrol_command(self, timestamp: int, action: PatrolAction) -> None:
        key = (timestamp, action.value)
        self._recent_patrol_command_set.discard(key)
        try:
            self._recent_patrol_commands.remove(key)
        except ValueError:
            pass
        self._last_patrol_timestamp = max(
            (item_timestamp for item_timestamp, _ in self._recent_patrol_commands),
            default=None,
        )

    def _remember_event(self, event_id: str) -> bool:
        with self._events_lock:
            if event_id in self._recent_event_set:
                return False
            if len(self._recent_events) == self._recent_events.maxlen:
                evicted = self._recent_events.popleft()
                self._recent_event_set.discard(evicted)
            self._recent_events.append(event_id)
            self._recent_event_set.add(event_id)
            return True

    def _forget_event(self, event_id: str) -> None:
        with self._events_lock:
            if event_id not in self._recent_event_set:
                return
            self._recent_event_set.discard(event_id)
            try:
                self._recent_events.remove(event_id)
            except ValueError:
                pass

    def _publish_detection_task(
        self,
        detection_type: DetectionType,
        event_id: Optional[str],
    ) -> None:
        try:
            self.detection_publisher.publish_detection(
                detection_type,
                event_id=event_id,
            )
        except Exception:
            if event_id is not None:
                self._forget_event(event_id)
            self.logger.exception("detection media publish failed event_id=%s", event_id)
        finally:
            self._detection_slots.release()

    def _prepare_motion_task(self) -> None:
        try:
            self.patrol.prepare_motion()
            self.logger.info("detection motion stack is ready; no goal was sent")
        except Exception:
            self.logger.exception("detection motion stack preparation failed")
        finally:
            with self._state_lock:
                self._motion_prepare_pending = False
