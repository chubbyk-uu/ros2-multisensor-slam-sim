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
from launch.substitutions import LaunchConfiguration


def start_playback(context):
    bag_path = Path(
        context.perform_substitution(LaunchConfiguration("bag"))
    ).expanduser().resolve()
    if not bag_path.exists():
        raise RuntimeError(f"3D SLAM dataset does not exist: {bag_path}")

    playback_rate = float(context.perform_substitution(LaunchConfiguration("rate")))
    if playback_rate <= 0.0:
        raise RuntimeError("Playback rate must be greater than zero")

    command = [
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
    ]
    loop = context.perform_substitution(LaunchConfiguration("loop")).lower()
    if loop in ("1", "true", "yes", "on"):
        command.append("--loop")

    playback_process = ExecuteProcess(cmd=command, output="screen")
    return [
        LogInfo(msg=f"Playing fixed 3D SLAM inputs: {bag_path}"),
        playback_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=playback_process,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="3D dataset playback completed")
                    )
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="Path to a fixed 3D dataset."),
            DeclareLaunchArgument(
                "rate", default_value="1.0", description="Rosbag playback rate."
            ),
            DeclareLaunchArgument(
                "loop", default_value="false", description="Replay continuously."
            ),
            OpaqueFunction(function=start_playback),
        ]
    )
