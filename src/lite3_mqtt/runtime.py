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
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.patrol = patrol
        self.detection_publisher = detection_publisher
        self.logger = logger or logging.getLogger(__name__)
        self._detection_worker = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="mqtt-detection-media",
        )
        self._events_lock = threading.Lock()
        self._recent_events = deque(maxlen=256)  # type: Deque[str]
        self._recent_event_set = set()  # type: Set[str]

    def handle_message(self, topic: str, payload: bytes) -> None:
        try:
            if topic == Topics.AUTO_PATROL:
                self._handle_patrol(parse_patrol_command(payload).action)
                return
            if topic in {Topics.SOUND_DETECT, Topics.COYOTE_DETECT}:
                trigger = parse_detection_trigger(topic, payload)
                self.report_detection(trigger.detection_type, event_id=trigger.event_id)
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
        if event_id is not None and not self._remember_event(event_id):
            self.logger.info("duplicate detection ignored event_id=%s", event_id)
            return False
        self._detection_worker.submit(
            self.detection_publisher.publish_detection,
            detection_type,
            event_id=event_id,
        )
        return True

    def close(self) -> None:
        self.patrol.close()
        self._detection_worker.shutdown(wait=True)

    def _handle_patrol(self, action: PatrolAction) -> None:
        if action is PatrolAction.START:
            started = self.patrol.start()
            self.logger.info("patrol START received started=%s", started)
        elif action is PatrolAction.STOP:
            self.patrol.stop()
        elif action is PatrolAction.RETURN_HOME:
            self.patrol.return_home()
        elif action is PatrolAction.EMERGENCY_STOP:
            self.patrol.emergency_stop()
        elif action is PatrolAction.RESET:
            self.patrol.reset()

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
