from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def start_playback(context):
    bag_path = Path(
        context.perform_substitution(LaunchConfiguration("bag"))
    ).expanduser().resolve()
    if not bag_path.exists():
        raise RuntimeError(f"SLAM dataset does not exist: {bag_path}")

    playback_rate = float(
        context.perform_substitution(LaunchConfiguration("rate"))
    )
    if playback_rate <= 0.0:
        raise RuntimeError("Playback rate must be greater than zero")

    loop = context.perform_substitution(LaunchConfiguration("loop")).lower()
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
    if loop in ("1", "true", "yes", "on"):
        command.append("--loop")

    playback_process = ExecuteProcess(cmd=command, output="screen")
    return [
        LogInfo(msg=f"Playing SLAM dataset: {bag_path}"),
        playback_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=playback_process,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="SLAM dataset playback completed"
                        )
                    )
                ],
            )
        ),
    ]


def generate_launch_description():
    custom_slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam"), "launch", "custom_slam.launch.py"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam"), "rviz", "custom_slam.rviz"]
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_wsl_gpu = LaunchConfiguration("use_wsl_gpu")
    wsl_gpu_adapter = LaunchConfiguration("wsl_gpu_adapter")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag",
                description="Path to a recorded SLAM dataset.",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="1.0",
                description="Rosbag playback rate.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                description="Replay the dataset repeatedly.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start the custom SLAM RViz view.",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
                description="Use Mesa D3D12 rendering in WSL.",
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter",
                default_value="NVIDIA",
                description="GPU adapter selected by Mesa D3D12 in WSL.",
            ),
            SetEnvironmentVariable(
                "GALLIUM_DRIVER",
                "d3d12",
                condition=IfCondition(use_wsl_gpu),
            ),
            SetEnvironmentVariable(
                "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                wsl_gpu_adapter,
                condition=IfCondition(use_wsl_gpu),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(custom_slam_launch),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            ),
            OpaqueFunction(function=start_playback),
        ]
    )
