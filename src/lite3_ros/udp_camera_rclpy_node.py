"""ROS2 node: UDP camera → CompressedImage topic.

Topics (defaults):
  /lite3/camera/image/compressed   sensor_msgs/CompressedImage
  /lite3/camera/status             std_msgs/String (JSON health)
"""

from __future__ import annotations

from dataclasses import dataclass

from lite3_perception.camera_node import CameraNodeConfig, UdpCameraNode
from lite3_perception.udp_camera_receiver import CameraFrame, UdpCameraConfig


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
class UdpCameraRosTopics:
    image_topic: str = "/lite3/camera/image/compressed"
    status_topic: str = "/lite3/camera/status"


class UdpCameraRclpyNode(Node):
    def __init__(
        self,
        *,
        bind_host: str = "0.0.0.0",
        udp_port: int = 5000,
        payload_type: int = 26,
        topics: UdpCameraRosTopics | None = None,
        status_period_sec: float = 1.0,
        camera_node: UdpCameraNode | None = None,
    ):
        if rclpy is None:
            raise RuntimeError("rclpy is required to create UdpCameraRclpyNode")

        super().__init__("lite3_udp_camera")
        self.topics = topics or UdpCameraRosTopics()

        self.image_pub = self.create_publisher(
            CompressedImage, self.topics.image_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.topics.status_topic, 10)

        if camera_node is None:
            camera_node = UdpCameraNode(
                CameraNodeConfig(
                    udp=UdpCameraConfig(
                        bind_host=bind_host,
                        port=udp_port,
                        payload_type=payload_type,
                    ),
                    status_period_sec=status_period_sec,
                ),
                on_frame=self._on_frame,
                on_status=self._on_status,
            )
        else:
            camera_node.on_frame = self._on_frame
            camera_node.on_status = self._on_status

        self.camera = camera_node
        self.camera.start()
        self.get_logger().info(
            f"UDP camera node up: {bind_host}:{udp_port} → {self.topics.image_topic}"
        )

    def _on_frame(self, frame: CameraFrame) -> None:
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = frame.jpeg_bytes
        try:
            msg.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass
        msg.header.frame_id = "lite3_camera"
        # Stash sequence in frame_id suffix is ugly; use header stamp only.
        self.image_pub.publish(msg)

    def _on_status(self, health: dict) -> None:
        msg = String()
        msg.data = self.camera.health_json()
        self.status_pub.publish(msg)

    def destroy_node(self) -> None:
        try:
            self.camera.stop()
        finally:
            super().destroy_node()


def spin_udp_camera_node(**kwargs) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to spin UdpCameraRclpyNode")
    rclpy.init(args=None)
    node = UdpCameraRclpyNode(**kwargs)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
