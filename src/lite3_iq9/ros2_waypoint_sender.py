"""ROS2 FollowWaypoints sender for the IQ9 runtime.

The module imports rclpy only inside live functions so unit tests can run on
machines without ROS2.
"""

from __future__ import annotations

import time

from lite3_iq9.waypoint_route import Waypoint, WaypointRoute


def capture_current_pose(
    *,
    odom_topic: str = "/odom",
    timeout_sec: float = 5.0,
    waypoint_id: str = "home",
) -> Waypoint:
    import rclpy
    from nav_msgs.msg import Odometry

    from lite3_iq9.ros2_state_bridge import _pose_from_odom

    rclpy.init(args=None)
    node = rclpy.create_node("lite3_waypoint_patrol_pose_capture")
    captured = {"pose": None}

    def on_msg(msg):
        captured["pose"] = _pose_from_odom(msg)

    subscription = node.create_subscription(Odometry, odom_topic, on_msg, 10)
    deadline = time.monotonic() + timeout_sec
    try:
        while rclpy.ok() and captured["pose"] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if captured["pose"] is None:
            raise TimeoutError(f"timed out waiting for {odom_topic}")
        pose = captured["pose"]
        return Waypoint(id=waypoint_id, x=pose.x, y=pose.y, yaw=pose.yaw, dwell_sec=0.0)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def send_follow_waypoints(
    route: WaypointRoute,
    *,
    action_name: str = "/FollowWaypoints",
    timeout_sec: float = 10.0,
) -> dict[str, object]:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowWaypoints
    from rclpy.action import ActionClient

    from lite3_iq9.nav2_waypoint_client import _yaw_to_quaternion

    rclpy.init(args=None)
    node = rclpy.create_node("lite3_waypoint_patrol_sender")
    client = ActionClient(node, FollowWaypoints, action_name)
    try:
        if not client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError(f"{action_name} action server is not available")

        goal = FollowWaypoints.Goal()
        for waypoint in route.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = route.frame_id
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = 0.0
            quat = _yaw_to_quaternion(waypoint.yaw)
            pose.pose.orientation.x = quat["x"]
            pose.pose.orientation.y = quat["y"]
            pose.pose.orientation.z = quat["z"]
            pose.pose.orientation.w = quat["w"]
            goal.poses.append(pose)

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"accepted": False, "status": None, "missed_waypoints": []}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        return {
            "accepted": True,
            "status": result.status,
            "missed_waypoints": list(result.result.missed_waypoints),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()
