"""Bring up all Lite3 ROS-facing processes in one Foxy container."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _shutdown_if_process_exits(process, label):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=[
                LogInfo(msg="{} exited; stopping the Lite3 system".format(label)),
                EmitEvent(
                    event=Shutdown(
                        reason="required process exited: {}".format(label)
                    )
                ),
            ],
        )
    )


def generate_launch_description():
    workspace_root = LaunchConfiguration("workspace_root")
    broker_host = LaunchConfiguration("broker_host")
    broker_port = LaunchConfiguration("broker_port")
    spool_dir = LaunchConfiguration("coyote_spool_dir")
    nav_network_interface = LaunchConfiguration("nav_network_interface")
    voice_runtime_dir = LaunchConfiguration("voice_runtime_dir")

    motion_state_receiver = ExecuteProcess(
        name="lite3_motion_state_receiver",
        cmd=[
            "python3",
            PathJoinSubstitution(
                [workspace_root, "scripts", "run_motion_state_receiver.py"]
            ),
            "--bind-host",
            "0.0.0.0",
            "--port",
            "43897",
            "--state-file",
            PathJoinSubstitution([voice_runtime_dir, "motion_state.json"]),
        ],
        cwd=workspace_root,
        output="screen",
    )

    coyote_bridge = ExecuteProcess(
        name="lite3_coyote_bridge",
        cmd=[
            "python3",
            PathJoinSubstitution(
                [workspace_root, "scripts", "run_coyote_mqtt_bridge.py"]
            ),
            "--broker-host",
            broker_host,
            "--broker-port",
            broker_port,
            "--spool-dir",
            spool_dir,
            "--motion-output",
            "udp",
            "--allow-robot-motion",
            "--preflight-ok",
            "--auto-mode-ok",
            "--exclusive-motion-ok",
            "--forward-speed-mps",
            "2.00",
        ],
        cwd=workspace_root,
        output="screen",
        additional_env={
            # The bridge, state receiver and RealSense bridge are all in this
            # single container.  Foxy/Cyclone crashes on this IQ9 when it
            # arbitrarily picks the Wi-Fi interface for timer-driven nodes.
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_LOG_DIR": "/tmp/ros/coyote",
        },
    )

    realsense_camera = ExecuteProcess(
        name="lite3_realsense_camera",
        cmd=[
            "bash",
            PathJoinSubstitution(
                [workspace_root, "scripts", "run_realsense_ros_camera.sh"]
            ),
        ],
        cwd=workspace_root,
        output="screen",
        additional_env={"REALSENSE_OUTPUT_DIR": "/home/ubuntu/iq9_coyote/outputs/realsense"},
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workspace_root",
                default_value="/workspace/lite3",
            ),
            DeclareLaunchArgument(
                "broker_host",
                default_value="127.0.0.1",
            ),
            DeclareLaunchArgument(
                "broker_port",
                default_value="1883",
            ),
            DeclareLaunchArgument(
                "coyote_spool_dir",
                default_value="/home/ubuntu/iq9_coyote/outputs/spool",
            ),
            DeclareLaunchArgument(
                "nav_network_interface",
                default_value="end0",
            ),
            DeclareLaunchArgument(
                "voice_runtime_dir",
                default_value="/home/ubuntu/iq9_coyote/outputs/voice_control",
            ),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            motion_state_receiver,
            coyote_bridge,
            realsense_camera,
            _shutdown_if_process_exits(
                motion_state_receiver, "Motion Host state receiver"
            ),
            _shutdown_if_process_exits(coyote_bridge, "coyote perception bridge"),
            _shutdown_if_process_exits(realsense_camera, "RealSense camera"),
        ]
    )
