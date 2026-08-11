"""Run the shared frontier regression against the RTAB-Map baseline."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
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


def generate_launch_description():
    smoke = LaunchConfiguration("smoke")
    exploration_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "rtabmap_3d_exploration_simulation.launch.py",
        ]
    )
    structured_world = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "worlds",
            "structured_loop_3d.sdf",
        ]
    )
    regression = Node(
        package="slam_robot_slam_3d",
        executable="frontier_exploration_regression",
        name="frontier_exploration_regression",
        output="screen",
        arguments=[
            "--wall-timeout",
            LaunchConfiguration("wall_timeout"),
            "--maximum-recovery-events",
            LaunchConfiguration("maximum_recovery_events"),
        ],
        remappings=[("/map", "/rtabmap/map")],
        condition=UnlessCondition(smoke),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("smoke", default_value="false"),
            DeclareLaunchArgument("wall_timeout", default_value="900"),
            DeclareLaunchArgument(
                "maximum_recovery_events", default_value="12"
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/slam_robot_rtabmap_frontier.db",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
            # Pinned so the regression is reproducible, declared so a campaign
            # can sweep it without editing this file. Zero draws a fresh seed
            # per run, which is a batch-level choice, not a regression one.
            DeclareLaunchArgument("exploration_seed", default_value="20260811"),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(exploration_launch),
                        launch_arguments={
                            "world": structured_world,
                            "gui": LaunchConfiguration("gui"),
                            "rviz": LaunchConfiguration("rviz"),
                            "database_path": LaunchConfiguration("database_path"),
                            "reset_database": "true",
                            "exploration_seed": LaunchConfiguration("exploration_seed"),
                            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
                            "wsl_gpu_adapter": LaunchConfiguration(
                                "wsl_gpu_adapter"
                            ),
                        }.items(),
                    )
                ],
            ),
            regression,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=regression,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="RTAB-Map frontier regression finished"
                            )
                        )
                    ],
                )
            ),
        ]
    )
