from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    structured_regression = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "structured_loop_regression.launch.py",
        ]
    )
    recorder = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "record_3d_slam_data.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value=str(Path.cwd() / "bags" / "structured_3d_reference"),
                description="Output directory for the fixed 3D MCAP dataset.",
            ),
            DeclareLaunchArgument("laps", default_value="2"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "smoke",
                default_value="false",
                description="Start dependencies without running the driving regression.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/slam_robot_structured_dataset_recording.db",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(structured_regression),
                        launch_arguments={
                            "laps": LaunchConfiguration("laps"),
                            "gui": LaunchConfiguration("gui"),
                            "rviz": LaunchConfiguration("rviz"),
                            "smoke": LaunchConfiguration("smoke"),
                            "database_path": LaunchConfiguration(
                                "database_path"
                            ),
                            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
                            "wsl_gpu_adapter": LaunchConfiguration(
                                "wsl_gpu_adapter"
                            ),
                        }.items(),
                    )
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(recorder),
                        launch_arguments={
                            "output": LaunchConfiguration("output")
                        }.items(),
                    )
                ],
            ),
        ]
    )
