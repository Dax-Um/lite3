"""ROS2 perception node: image topic in → result topic out.

Does **not** open the UDP socket. It only subscribes to the camera node:

  /lite3/camera/image/compressed  →  process  →  /lite3/perception/result
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

from lite3_perception.perception_node import (
    PerceptionNode,
    PerceptionNodeConfig,
    PerceptionResult,
)


try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object  # type: ignore[misc, assignment]
    CompressedImage = None
    String = None


@dataclass(frozen=True)
class PerceptionRosTopics:
    image_topic: str = "/lite3/camera/image/compressed"
    result_topic: str = "/lite3/perception/result"
    status_topic: str = "/lite3/perception/status"


class PerceptionRclpyNode(Node):
    def __init__(
        self,
        *,
        topics: PerceptionRosTopics | None = None,
        target_fps: float = 0.0,
        status_period_sec: float = 1.0,
        perception: PerceptionNode | None = None,
    ):
        if rclpy is None:
            raise RuntimeError("rclpy is required to create PerceptionRclpyNode")

        super().__init__("lite3_perception")
        self.topics = topics or PerceptionRosTopics()
        self.perception = perception or PerceptionNode(
            PerceptionNodeConfig(target_fps=target_fps),
            on_result=self._publish_result,
        )
        if perception is not None and perception.on_result is None:
            perception.on_result = self._publish_result

        self.result_pub = self.create_publisher(String, self.topics.result_topic, 10)
        self.status_pub = self.create_publisher(String, self.topics.status_topic, 10)
        self.create_subscription(
            CompressedImage, self.topics.image_topic, self._on_image, 10
        )
        self.create_timer(status_period_sec, self._publish_status)
        self._images_received = 0
        self._last_image_monotonic: float | None = None
        self.get_logger().info(
            f"perception node listening on {self.topics.image_topic} → "
            f"{self.topics.result_topic}"
        )

    def _on_image(self, msg) -> None:
        self._images_received += 1
        self._last_image_monotonic = time.monotonic()
        jpeg = bytes(msg.data)
        # CompressedImage has no width/height; detector may fill after decode.
        self.perception.process_jpeg(
            jpeg,
            sequence=self._images_received,
            frame_timestamp_monotonic=time.monotonic(),
        )

    def _publish_result(self, result: PerceptionResult) -> None:
        msg = String()
        msg.data = result.to_json()
        self.result_pub.publish(msg)

    def _publish_status(self) -> None:
        health = self.perception.health()
        health["images_received"] = self._images_received
        if self._last_image_monotonic is not None:
            health["last_image_age_sec"] = time.monotonic() - self._last_image_monotonic
        else:
            health["last_image_age_sec"] = None
        health["image_topic"] = self.topics.image_topic
        msg = String()
        msg.data = json.dumps(health, separators=(",", ":"))
        self.status_pub.publish(msg)


def spin_perception_node(**kwargs) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to spin PerceptionRclpyNode")
    rclpy.init(args=None)
    node = PerceptionRclpyNode(**kwargs)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
