from lite3_behavior.patrol_controller import ControllerOutput
from lite3_common.types import StopReason, Twist2D
from lite3_control.readiness_gate import ReadinessResult
from lite3_ros.patrol_rclpy_node import (
    RuntimeFlags,
    TopicTimestamps,
    build_readiness_input,
    format_status_text,
)


def test_build_readiness_input_copies_topic_timestamps_and_flags():
    item = build_readiness_input(
        now=10.0,
        timestamps=TopicTimestamps(scan=9.9, odom=9.8, imu=9.7),
        flags=RuntimeFlags(
            motion_host_reachable=True,
            preflight_ok=True,
            auto_mode_ok=True,
            stand_ready_ok=True,
        ),
    )

    assert item.now == 10.0
    assert item.scan_last_seen == 9.9
    assert item.odom_last_seen == 9.8
    assert item.imu_last_seen == 9.7
    assert item.motion_host_reachable is True
    assert item.preflight_ok is True
    assert item.auto_mode_ok is True
    assert item.stand_ready_ok is True


def test_format_status_text_includes_output_and_readiness_reasons():
    output = ControllerOutput(
        raw_cmd=Twist2D(0.08, 0.0, 0.0),
        safe_cmd=Twist2D(0.0, 0.0, 0.0),
        state="move_along_lane",
        stop_reason=StopReason.LIDAR_TIMEOUT,
        lane_index=2,
        return_home_active=False,
        boundary_min_front_m=0.51,
    )
    readiness = ReadinessResult(ready=False, reasons=("scan_stale", "auto_mode"))

    text = format_status_text(output, readiness)

    assert "ready=false" in text
    assert "reasons=scan_stale|auto_mode" in text
    assert "state=move_along_lane" in text
    assert "safe_cmd=0.000,0.000,0.000" in text
    assert "stop_reason=lidar_timeout" in text
    assert "boundary_min_front_m=0.510" in text
