from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def start_playback(context):
    bag_path = Path(
        context.perform_substitution(LaunchConfiguration("bag"))
    ).expanduser().resolve()
    if not bag_path.exists():
        raise RuntimeError(f"RGB-D dataset does not exist: {bag_path}")
    playback_rate = float(context.perform_substitution(LaunchConfiguration("rate")))
    if playback_rate <= 0.0:
        raise RuntimeError("Playback rate must be greater than zero")

    playback = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            str(bag_path),
            "--clock",
            "100",
            "--delay",
            "2.0",
            "--rate",
            str(playback_rate),
            "--disable-keyboard-controls",
        ],
        output="screen",
        additional_env={
            "FASTRTPS_DEFAULT_PROFILES_FILE": LaunchConfiguration(
                "rgbd_dds_profiles_file"
            )
        },
    )
    return [
        LogInfo(msg=f"Playing fixed RGB-D inputs: {bag_path}"),
        playback,
        RegisterEventHandler(
            OnProcessExit(
                target_action=playback,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="RGB-D playback completed"))
                ],
            )
        ),
    ]


def generate_launch_description():
    default_dds_profile = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "fastdds_rgbd.xml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="Path to a fixed RGB-D dataset."),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_dds_profile,
                ),
            ),
            OpaqueFunction(function=start_playback),
        ]
    )
