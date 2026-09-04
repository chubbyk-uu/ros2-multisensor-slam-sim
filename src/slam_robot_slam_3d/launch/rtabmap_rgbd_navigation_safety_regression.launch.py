"""Verify RGB-D online SLAM with 3D-LiDAR dynamic-obstacle sensing."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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
    """Preserve the regression result instead of hiding it during shutdown."""
    if event.returncode != 0:
        raise RuntimeError(
            "RTAB-Map RGB-D navigation safety regression exited with code "
            f"{event.returncode}"
        )
    return [
        EmitEvent(
            event=Shutdown(
                reason="RTAB-Map RGB-D navigation safety regression passed"
            )
        )
    ]


def generate_launch_description():
    world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    default_dds_profile = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "fastdds_rgbd.xml"]
    )
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_slam_3d",
                "rtabmap_rgbd_navigation_simulation.launch.py",
            )
        ),
        launch_arguments={
            "world": world,
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "database_path": LaunchConfiguration("database_path"),
            "reset_database": "true",
            "rgbd_dds_profiles_file": LaunchConfiguration(
                "rgbd_dds_profiles_file"
            ),
            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
            "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
        }.items(),
    )
    regression = Node(
        package="slam_robot_navigation",
        executable="navigation_regression.py",
        name="rtabmap_rgbd_navigation_safety_regression",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("smoke")),
        arguments=[
            "--scenario", "dynamic-obstacle",
            "--localization-mode", "online-slam",
            "--world-name", "slam_world",
            "--timeout", LaunchConfiguration("timeout"),
            "--activation-timeout", LaunchConfiguration("activation_timeout"),
            "--wall-watchdog-timeout",
            LaunchConfiguration("wall_watchdog_timeout"),
            "--dependency-grace", LaunchConfiguration("dependency_grace"),
            "--robot-circumscribed-radius",
            LaunchConfiguration("robot_circumscribed_radius"),
            # RGB-D is directional. Expand known free space incrementally,
            # then apply the unchanged, already verified obstacle geometry.
            "--pre-map-dynamic-route",
        ],
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                "robot_circumscribed_radius", default_value="0.336"
            ),
            DeclareLaunchArgument("startup_delay", default_value="50.0"),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/slam_robot_rtabmap_rgbd_navigation.db",
            ),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_dds_profile,
                ),
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
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
    )
