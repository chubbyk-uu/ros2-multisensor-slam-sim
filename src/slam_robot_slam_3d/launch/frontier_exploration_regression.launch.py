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
            "custom_3d_exploration_simulation.launch.py",
        ]
    )
    default_world = PathJoinSubstitution(
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
            "--world",
            LaunchConfiguration("world"),
            "--world-profile",
            LaunchConfiguration("world_profile"),
            "--spawn-x",
            LaunchConfiguration("spawn_x"),
            "--spawn-y",
            LaunchConfiguration("spawn_y"),
            "--spawn-yaw",
            LaunchConfiguration("spawn_yaw"),
            "--maximum-recovery-events",
            LaunchConfiguration("maximum_recovery_events"),
        ],
        condition=UnlessCondition(smoke),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("smoke", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument(
                "world_profile", default_value="structured_loop_3d"
            ),
            # Matches the RTAB-Map entry point so neither run carries a
            # different time budget into a coverage comparison.
            DeclareLaunchArgument("wall_timeout", default_value="900"),
            DeclareLaunchArgument(
                "maximum_recovery_events", default_value="5"
            ),
            DeclareLaunchArgument(
                "snapshot_path",
                default_value="/tmp/slam_robot_frontier_exploration.snapshot",
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
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            # Declared rather than left to leak through the scoped group: an
            # undeclared value does reach the include, but only by inheritance
            # that a later refactor would silently remove, and the fault
            # injection regression would then quietly stop injecting anything.
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(exploration_launch),
                        launch_arguments={
                            "world": LaunchConfiguration("world"),
                            "gui": LaunchConfiguration("gui"),
                            "rviz": LaunchConfiguration("rviz"),
                            "snapshot_path": LaunchConfiguration("snapshot_path"),
                            "load_snapshot": "false",
                            "spawn_x": LaunchConfiguration("spawn_x"),
                            "spawn_y": LaunchConfiguration("spawn_y"),
                            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                            "exploration_seed": LaunchConfiguration("exploration_seed"),
                            "nav2_autostart": LaunchConfiguration("nav2_autostart"),
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
                                reason="frontier exploration regression finished"
                            )
                        )
                    ],
                )
            ),
        ]
    )
