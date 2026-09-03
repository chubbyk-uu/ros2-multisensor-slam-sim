"""Run the shared dynamic-obstacle or blocked-road criteria on custom 3D SLAM."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename):
    return PathJoinSubstitution(
        [FindPackageShare(package), "launch", filename]
    )


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def finish_regression(event, _context):
    """Keep the regression exit status instead of hiding it during shutdown."""
    if event.returncode != 0:
        raise RuntimeError(
            "custom 3D navigation safety regression exited with code "
            f"{event.returncode}"
        )
    return [
        EmitEvent(
            event=Shutdown(
                reason="custom 3D navigation safety regression passed"
            )
        )
    ]


def launch_scenario(context):
    scenario = context.perform_substitution(LaunchConfiguration("scenario"))
    if scenario not in ("dynamic-obstacle", "blocked-road"):
        raise RuntimeError(
            "scenario must be 'dynamic-obstacle' or 'blocked-road', got "
            f"{scenario!r}"
        )
    world_name = (
        "blocked_road_world" if scenario == "blocked-road" else "slam_world"
    )
    world = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "worlds",
            f"{world_name}.sdf",
        ]
    )
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_slam_3d",
                "custom_3d_navigation_simulation.launch.py",
            )
        ),
        launch_arguments={
            "world": world,
            "world_name": world_name,
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "mode": "mapping",
            "load_snapshot": "false",
            "save_on_shutdown": "false",
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
            "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
        }.items(),
    )
    regression = Node(
        package="slam_robot_navigation",
        executable="navigation_regression.py",
        name="custom_3d_navigation_safety_regression",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("smoke")),
        arguments=[
            "--scenario", scenario,
            "--localization-mode", "online-slam",
            "--world-name", world_name,
            "--timeout", LaunchConfiguration("timeout"),
            "--activation-timeout", LaunchConfiguration("activation_timeout"),
            "--wall-watchdog-timeout",
            LaunchConfiguration("wall_watchdog_timeout"),
            "--dependency-grace", LaunchConfiguration("dependency_grace"),
            "--give-up-budget", LaunchConfiguration("give_up_budget"),
            "--robot-circumscribed-radius",
            LaunchConfiguration("robot_circumscribed_radius"),
            "--seal-offset-x", LaunchConfiguration("seal_offset_x"),
            "--obstacle-offset-y", LaunchConfiguration("obstacle_offset_y"),
        ],
    )
    return [
        GroupAction(scoped=True, actions=[stack]),
        TimerAction(
            period=LaunchConfiguration("startup_delay"),
            actions=[regression],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=regression,
                on_exit=finish_regression,
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario", default_value="dynamic-obstacle"
            ),
            DeclareLaunchArgument(
                "smoke",
                default_value="false",
                description="Start dependencies without driving the robot.",
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("timeout", default_value="300.0"),
            DeclareLaunchArgument(
                "activation_timeout", default_value="120.0"
            ),
            DeclareLaunchArgument(
                "wall_watchdog_timeout", default_value="300.0"
            ),
            DeclareLaunchArgument("dependency_grace", default_value="15.0"),
            DeclareLaunchArgument("give_up_budget", default_value="180.0"),
            DeclareLaunchArgument(
                "robot_circumscribed_radius", default_value="0.336"
            ),
            DeclareLaunchArgument("seal_offset_x", default_value="0.0"),
            DeclareLaunchArgument("obstacle_offset_y", default_value="0.0"),
            DeclareLaunchArgument("startup_delay", default_value="50.0"),
            DeclareLaunchArgument(
                "snapshot_path",
                default_value="/tmp/custom_3d_navigation_safety.snapshot",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
            OpaqueFunction(function=launch_scenario),
        ]
    )
