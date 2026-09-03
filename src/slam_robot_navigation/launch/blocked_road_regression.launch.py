"""
Bring up blocked_road_world and score how Nav2 gives up on a sealed alcove.

The world, its <world name> and its map are set together here because they
have to agree: the entity services Gazebo exposes are namespaced by the world
name, and the map has to be the one generated from this world. Leaving those to
the caller means three chances to get it wrong for no benefit, and two of them
fail quietly -- a mismatched world name simply leaves the spawn bridge pointing
at nothing.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WORLD_NAME = "blocked_road_world"


def shutdown_after_regression(event, _context):
    """Propagate the scorer's result instead of turning every exit into 0."""
    if event.returncode != 0:
        raise RuntimeError(
            "blocked-road regression exited with code "
            f"{event.returncode}"
        )
    return [
        EmitEvent(
            event=Shutdown(reason="blocked-road regression passed")
        )
    ]


def generate_launch_description():
    world = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "worlds",
            f"{WORLD_NAME}.sdf",
        ]
    )
    default_map = str(
        Path.cwd() / "maps" / "reference" / "blocked_road_map.yaml"
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_robot_bringup"),
                    "launch",
                    "navigation_simulation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world,
            "world_name": WORLD_NAME,
            "map": LaunchConfiguration("map"),
            "gui": LaunchConfiguration("gui"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    regression = Node(
        package="slam_robot_navigation",
        executable="navigation_regression.py",
        name="navigation_regression",
        output="screen",
        arguments=[
            "--scenario", "blocked-road",
            "--world-name", WORLD_NAME,
            "--give-up-budget", LaunchConfiguration("give_up_budget"),
            "--activation-timeout", LaunchConfiguration("activation_timeout"),
            "--wall-watchdog-timeout",
            LaunchConfiguration("wall_watchdog_timeout"),
            "--dependency-grace", LaunchConfiguration("dependency_grace"),
            "--robot-circumscribed-radius",
            LaunchConfiguration("robot_circumscribed_radius"),
            "--seal-offset-x", LaunchConfiguration("seal_offset_x"),
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("give_up_budget", default_value="180.0"),
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
            DeclareLaunchArgument("seal_offset_x", default_value="0.0"),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="50.0",
                description=(
                    "Seconds to let Nav2 and AMCL come up before the "
                    "regression starts asking them for a plan."
                ),
            ),
            simulation,
            TimerAction(
                period=LaunchConfiguration("startup_delay"),
                actions=[regression],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=regression,
                    on_exit=shutdown_after_regression,
                )
            ),
        ]
    )
