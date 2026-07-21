import importlib.util
import subprocess
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "run_coyote_mqtt_bridge.py"
SYSTEM_HELPER = SCRIPT.with_name("lite3_system_docker.sh")
DOCKERFILE = SCRIPT.parents[1] / "deploy" / "mqtt" / "Dockerfile.foxy"
ENTRYPOINT = SCRIPT.parents[1] / "deploy" / "mqtt" / "entrypoint.sh"
SAMPLE_HELPER = SCRIPT.with_name("run_mqtt_sample_peer_docker.sh")
BRINGUP = (
    SCRIPT.parents[1]
    / "deploy"
    / "mqtt"
    / "lite3_bringup"
    / "launch"
    / "lite3_system.launch.py"
)
ROS_BRIDGE = SCRIPT.parents[1] / "src" / "lite3_ros" / "coyote_bridge_rclpy_node.py"


def load_script():
    spec = importlib.util.spec_from_file_location("run_coyote_mqtt_bridge", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_bridge_defaults_to_no_robot_motion_and_20hz():
    script = load_script()
    args = script.parse_args([])

    script.validate_args(args)

    assert args.motion_output == "disabled"
    assert args.control_hz >= 20.0
    assert args.status_timeout_sec <= 1.0
    assert args.forward_speed_mps == pytest.approx(1.50)
    assert args.turn_speed_radps == pytest.approx(0.65)
    assert args.search_turn_speed_radps == pytest.approx(1.45)


def test_target_reached_completion_uses_direct_turn_hello_then_navigation():
    script = load_script()
    calls = []
    finished = []

    class FakeDriver:
        def send_cmd_vel(self, vx, vy, wz):
            calls.append(("cmd_vel", vx, vy, wz))

        def send_simple_command(self, code, value=0):
            calls.append(("simple", code, value))

    routine = script.CompletionActionRoutine(
        FakeDriver(),
        on_finished=finished.append,
        logger=script.logging.getLogger("test-completion-action"),
        sleep=lambda _seconds: None,
        wait_for_robot_basic_state=lambda _state, _timeout: True,
        direct_turn_steps=1,
    )
    routine._run("coyote-action-event")

    assert calls == [
        ("simple", script.CMD_MANUAL_MODE, 0),
        ("simple", script.CMD_FLAT_GAIT_FAST, 0),
        ("cmd_vel", 0.0, 0.0, 0.0),
        ("cmd_vel", 0.0, 0.0, 0.0),
        ("simple", script.CMD_FLAT_GAIT_SLOW, 0),
        ("simple", script.CMD_NAVIGATION_MODE, 0),
        ("cmd_vel", 0.0, 0.0, script.COMPLETION_DIRECT_TURN_WZ_RADPS),
        ("simple", script.CMD_MANUAL_MODE, 0),
        ("simple", script.CMD_STAND_SIT, 0),
        ("simple", script.CMD_HELLO, 0),
        ("simple", script.CMD_STAND_SIT, 0),
        ("simple", script.CMD_NAVIGATION_MODE, 0),
    ]
    assert finished == ["coyote-action-event"]


def test_gated_udp_motion_sink_stops_udp_after_release_until_next_search():
    script = load_script()
    calls = []

    class FakeDriver:
        def send_cmd_vel(self, vx, vy, wz):
            calls.append((vx, vy, wz))

    sink = script.GatedUdpMotionSink(FakeDriver())
    sink.send_cmd_vel(0.0, 0.0, 0.0)
    sink.acquire()
    sink.send_cmd_vel(1.5, 0.0, 0.0)
    sink.send_cmd_vel(0.0, 0.0, 0.0)
    sink.release()
    sink.send_cmd_vel(0.0, 0.0, 0.0)

    assert calls == [(1.5, 0.0, 0.0), (0.0, 0.0, 0.0)]
    assert sink.wait_released(0.0)


def test_coyote_mission_stands_before_search_and_sits_after_home():
    script = load_script()
    calls = []
    ready = []

    class FakeDriver:
        def send_cmd_vel(self, vx, vy, wz):
            calls.append(("cmd_vel", vx, vy, wz))

        def send_simple_command(self, code, value=0):
            calls.append(("simple", code, value))

    routine = script.CoyoteMissionStartRoutine(
        FakeDriver(),
        on_search_ready=ready.append,
        logger=script.logging.getLogger("test-mission-start"),
        sleep=lambda _seconds: None,
    )
    routine.update_robot_basic_state(script.ROBOT_BASIC_STATE_SITTING)
    routine._active_event_id = "mission-event"
    # The fake driver cannot change Motion Host state, so emulate it after
    # the Stand/Sit command as the real ROS RobotState stream does.
    original_send = routine.driver.send_simple_command
    def send_simple_command(code, value=0):
        original_send(code, value)
        if code == script.CMD_STAND_SIT:
            next_state = (
                script.ROBOT_BASIC_STATE_STANDING
                if routine._robot_basic_state == script.ROBOT_BASIC_STATE_SITTING
                else script.ROBOT_BASIC_STATE_SITTING
            )
            routine.update_robot_basic_state(next_state)
    routine.driver.send_simple_command = send_simple_command
    routine._run("mission-event")
    routine.sit_after_home("mission-event")

    assert calls == [
        ("cmd_vel", 0.0, 0.0, 0.0),
        ("simple", script.CMD_STAND_SIT, 0),
        ("simple", script.CMD_NAVIGATION_MODE, 0),
        ("simple", script.CMD_MANUAL_MODE, 0),
        ("simple", script.CMD_STAND_SIT, 0),
    ]
    assert ready == ["mission-event"]


def test_bridge_uses_the_same_container_mqtt_environment(monkeypatch):
    monkeypatch.setenv("MQTT_HOST", "broker.internal")
    monkeypatch.setenv("MQTT_PORT", "2883")
    monkeypatch.setenv("MQTT_USER", "robot")
    monkeypatch.setenv("MQTT_PASS", "secret")

    args = load_script().parse_args([])

    assert args.broker_host == "broker.internal"
    assert args.broker_port == 2883
    assert args.username == "robot"
    assert args.password == "secret"


def test_udp_motion_requires_all_four_operator_gates():
    script = load_script()
    args = script.parse_args(["--motion-output", "udp"])

    with pytest.raises(SystemExit, match="--allow-robot-motion"):
        script.validate_args(args)


def test_udp_motion_accepts_explicit_exclusive_gates():
    script = load_script()
    args = script.parse_args(
        [
            "--motion-output",
            "udp",
            "--allow-robot-motion",
            "--preflight-ok",
            "--auto-mode-ok",
            "--exclusive-motion-ok",
        ]
    )

    script.validate_args(args)


def test_ros_motion_requires_explicit_enable_but_uses_runtime_nav_sensor_gates():
    script = load_script()
    args = script.parse_args(["--motion-output", "ros"])

    with pytest.raises(SystemExit, match="--allow-robot-motion"):
        script.validate_args(args)

    args = script.parse_args(
        ["--motion-output", "ros", "--allow-robot-motion"]
    )
    script.validate_args(args)
    assert args.forward_speed_mps == pytest.approx(1.00)


def test_ros_tracking_allows_one_mps_but_udp_keeps_shared_limit():
    script = load_script()
    ros_args = script.parse_args(
        [
            "--motion-output",
            "ros",
            "--allow-robot-motion",
            "--forward-speed-mps",
            "1.00",
        ]
    )
    script.validate_args(ros_args)
    assert ros_args.search_turn_speed_radps == pytest.approx(1.35)

    udp_args = script.parse_args(
        [
            "--motion-output",
            "udp",
            "--allow-robot-motion",
            "--preflight-ok",
            "--auto-mode-ok",
            "--exclusive-motion-ok",
            "--forward-speed-mps",
            "1.00",
        ]
    )
    with pytest.raises(SystemExit, match="--forward-speed-mps"):
        script.validate_args(udp_args)


def test_udp_fallback_keeps_shared_turn_limit():
    args = load_script().parse_args(["--motion-output", "udp"])

    assert args.turn_speed_radps == pytest.approx(0.20)
    assert args.search_turn_speed_radps == pytest.approx(0.20)


def test_ros_rejects_search_turn_speed_above_requested_value():
    script = load_script()
    args = script.parse_args(
        [
            "--motion-output",
            "ros",
            "--allow-robot-motion",
            "--search-turn-speed-radps",
            "1.351",
        ]
    )

    with pytest.raises(SystemExit, match="--search-turn-speed-radps"):
        script.validate_args(args)


def test_ros_rejects_alignment_turn_speed_above_requested_value():
    script = load_script()
    args = script.parse_args(
        [
            "--motion-output",
            "ros",
            "--allow-robot-motion",
            "--turn-speed-radps",
            "0.601",
        ]
    )

    with pytest.raises(SystemExit, match="--turn-speed-radps"):
        script.validate_args(args)


def test_bridge_rejects_control_rate_below_motion_host_requirement():
    script = load_script()
    args = script.parse_args(["--control-hz", "19.9"])

    with pytest.raises(SystemExit, match="at least 20"):
        script.validate_args(args)


def test_single_container_helper_uses_one_foxy_bringup():
    body = SYSTEM_HELPER.read_text()

    subprocess.run(["bash", "-n", str(SYSTEM_HELPER)], check=True)
    assert 'NAME="lite3-system"' in body
    assert body.count("docker run") == 1
    assert "ros2 launch lite3_bringup lite3_system.launch.py" in body
    assert "--network host" in body


def test_single_container_helper_removes_legacy_split_runtimes():
    body = SYSTEM_HELPER.read_text()

    assert "lite3-mqtt-runtime" in body
    assert "lite3-coyote-mqtt-bridge" in body
    assert "stop_and_remove" in body
    assert not SCRIPT.with_name("run_mqtt_runtime_docker.sh").exists()
    assert not SCRIPT.with_name("start_coyote_mqtt_bridge_docker.sh").exists()


def test_foxy_launch_starts_patrol_and_coyote_ros_bridge_fail_closed():
    body = BRINGUP.read_text()

    assert "run_mqtt_runtime.py" in body
    assert "run_nav_watchdog.py" in body
    assert "run_coyote_mqtt_bridge.py" in body
    assert "pointcloud_to_laserscan_node" in body
    assert '"cloud_in:=/rslidar_points"' in body
    assert '"scan:=/scan"' in body
    assert '"target_frame:=base_link"' in body
    assert '"--motion-output"' in body
    assert '"ros"' in body
    assert '"--allow-robot-motion"' in body
    assert '"ROS_LOCALHOST_ONLY": "0"' in body
    assert '"nav_network_interface"' in body
    assert body.count('"CYCLONEDDS_URI"') == 4
    assert '"ROS_LOG_DIR": "/tmp/ros/mqtt"' in body
    assert '"ROS_LOG_DIR": "/tmp/ros/coyote"' in body
    assert body.count("_shutdown_if_process_exits(") == 5


def test_ros_motion_output_uses_remote_sensors_and_dynamic_cmd_vel_ownership():
    body = ROS_BRIDGE.read_text()
    script = SCRIPT.read_text()

    assert "LaserScan" in body
    assert "Odometry" in body
    assert '"/navigate_to_pose"' in body
    assert 'cmd_vel_topic: str = "/cmd_vel"' in body
    assert "if self.cmd_vel_pub is None and not moving" in body
    assert "self.destroy_publisher(publisher)" in body
    assert "def motion_ready(self)" in body
    assert '"/lite3/nav/watchdog_reset"' in body
    assert '"/lite3/nav/watchdog_reset_ack"' in body
    assert "self.handoff_acked" in body
    assert "self.get_publishers_info_by_topic" in body
    assert "self.navigate_graph_ready\n            and self.handoff_acked" in body
    assert "MAX_CONTROL_EVENTS_PER_TICK = 8" in body
    assert "while processed_count < MAX_CONTROL_EVENTS_PER_TICK" in body
    assert "if stop_barrier:\n                return" in body
    assert "patrol_command_seen" not in body
    assert "handle_patrol_command" in body
    assert 'glass_status_topic: str = "/lite3/data/glass/status"' in body
    assert "Topics.SOUND_DETECT" in script
    assert "Topics.COYOTE_DETECT" in script
    assert "Topics.AUTO_PATROL" in script
    assert "parse_detection_trigger" in script
    assert "parse_patrol_command" in script
    assert "search_events.put_nowait" in script
    assert "command.action is PatrolAction.EMERGENCY_STOP" in script
    assert '("patrol", command.action.value, command.timestamp)' in script
    assert "coyote control queue full; emergency latched" in script


def test_single_container_enables_restart_only_after_both_processes_are_ready():
    body = SYSTEM_HELPER.read_text()

    run_index = body.index("--restart no")
    patrol_ready_index = body.rindex(
        'current_generation_has_log "${NAME}" "runtime ready"'
    )
    bridge_ready_index = body.rindex(
        'current_generation_has_log "${NAME}" "coyote bridge ready"'
    )
    update_index = body.index("docker update --restart unless-stopped")

    assert run_index < patrol_ready_index < update_index
    assert run_index < bridge_ready_index < update_index
    assert "STARTUP_TIMEOUT_SEC=15" in body
    assert 'while container_running "${NAME}"' in body
    assert "sleep 3" not in body
    assert 'docker kill --signal SIGINT "${container}"' in body
    assert 'docker update --restart no "${container}"' in body
    assert "shutdown_deadline=$((SECONDS + 20))" in body
    assert 'docker stop --timeout 5 "${container}"' in body


def test_single_container_readiness_is_scoped_to_current_start_generation():
    body = SYSTEM_HELPER.read_text()

    assert "current_generation_has_log()" in body
    assert "docker inspect -f '{{.State.StartedAt}}'" in body
    assert 'docker logs --since "${started_at}"' in body
    assert 'docker logs "${NAME}" 2>&1 | grep' not in body


def test_all_mqtt_docker_runtimes_use_cyclone_dds():
    helper = SYSTEM_HELPER.read_text()
    dockerfile = DOCKERFILE.read_text()
    entrypoint = ENTRYPOINT.read_text()
    sample_helper = SAMPLE_HELPER.read_text()

    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in helper
    assert "rmw_fastrtps_cpp" not in helper
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in dockerfile
    assert "ros-foxy-rmw-cyclonedds-cpp" in dockerfile
    assert '"${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp"' in entrypoint
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in sample_helper


def test_bringup_package_is_built_into_the_foxy_overlay():
    body = DOCKERFILE.read_text()

    assert "COPY lite3_bringup /tmp/lite3_overlay_src/src/lite3_bringup" in body
    assert "colcon build" in body
    assert "mkdir -p /tmp/ros/launch /tmp/ros/mqtt /tmp/ros/coyote" in ENTRYPOINT.read_text()
    assert "ros-foxy-pointcloud-to-laserscan" in DOCKERFILE.read_text()
