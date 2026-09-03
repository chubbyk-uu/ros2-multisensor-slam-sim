from functools import partial
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
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


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def prepare_output_files(context):
    """Remove reports from an earlier run so they cannot satisfy this run."""
    for argument_name in (
        "trajectory_output",
        "map_output",
        "verdict_output",
    ):
        output_path = Path(
            LaunchConfiguration(argument_name).perform(context)
        ).expanduser()
        if output_path.is_file():
            output_path.unlink()
    return []


def finish_after_driver(event, context, checker):
    del context
    if event.returncode != 0:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "online RGB-D route regression failed with exit code "
                        f"{event.returncode}"
                    )
                )
            )
        ]
    return [TimerAction(period=5.0, actions=[checker])]


def generate_launch_description():
    simulation_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "rtabmap_rgbd_simulation.launch.py",
        ]
    )
    structured_world = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "worlds",
            "structured_loop_3d.sdf",
        ]
    )
    default_dds_profile = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "fastdds_rgbd.xml"]
    )
    driver = Node(
        package="slam_robot_slam_3d",
        executable="structured_loop_regression",
        name="rtabmap_rgbd_online_route_regression",
        output="screen",
        arguments=["--laps", "2"],
    )
    checker = ExecuteProcess(
        cmd=[
            "ros2", "run", "slam_robot_slam_3d",
            "rtabmap_rgbd_fixed_regression_check",
            "--label", "online active loop",
            "--trajectory", LaunchConfiguration("trajectory_output"),
            "--map", LaunchConfiguration("map_output"),
            "--output", LaunchConfiguration("verdict_output"),
            "--minimum-loop-closures",
            LaunchConfiguration("minimum_loop_closures"),
            "--minimum-info-messages",
            LaunchConfiguration("minimum_info_messages"),
            "--minimum-map-updates",
            LaunchConfiguration("minimum_map_updates"),
            "--minimum-camera-info-messages",
            LaunchConfiguration("minimum_camera_info_messages"),
            "--minimum-camera-info-rate-hz",
            LaunchConfiguration("minimum_camera_info_rate_hz"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/slam_robot_rtabmap_rgbd_online.db",
            ),
            DeclareLaunchArgument(
                "trajectory_output",
                default_value="/tmp/rtabmap_rgbd_online_trajectory.json",
            ),
            DeclareLaunchArgument(
                "map_output",
                default_value="/tmp/rtabmap_rgbd_online_map.json",
            ),
            DeclareLaunchArgument(
                "verdict_output",
                default_value="/tmp/rtabmap_rgbd_online_verdict.json",
            ),
            DeclareLaunchArgument("minimum_loop_closures", default_value="20"),
            DeclareLaunchArgument("minimum_info_messages", default_value="500"),
            DeclareLaunchArgument("minimum_map_updates", default_value="450"),
            DeclareLaunchArgument(
                "minimum_camera_info_messages", default_value="9000"
            ),
            DeclareLaunchArgument(
                "minimum_camera_info_rate_hz", default_value="27.0"
            ),
            DeclareLaunchArgument("wall_timeout", default_value="900.0"),
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
            OpaqueFunction(function=prepare_output_files),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(simulation_launch),
                        launch_arguments={
                            "world": structured_world,
                            "gui": LaunchConfiguration("gui"),
                            "rviz": LaunchConfiguration("rviz"),
                            "database_path": LaunchConfiguration("database_path"),
                            "reset_database": "true",
                            "rgbd_dds_profiles_file": LaunchConfiguration(
                                "rgbd_dds_profiles_file"
                            ),
                            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
                            "wsl_gpu_adapter": LaunchConfiguration(
                                "wsl_gpu_adapter"
                            ),
                        }.items(),
                    )
                ],
            ),
            driver,
            Node(
                package="slam_robot_slam_3d",
                executable="stack_trajectory_census",
                name="rtabmap_rgbd_online_trajectory_census",
                output="screen",
                arguments=[
                    "--label", "rtabmap_rgbd_online",
                    "--output", LaunchConfiguration("trajectory_output"),
                    "--process-match", "/rtabmap_sync/rgbd_sync",
                    "--process-match", "/rtabmap_slam/rtabmap",
                    "--info-topic", "/rtabmap/info",
                    "--camera-info-topic", "/camera/color/camera_info",
                    "--database-path", LaunchConfiguration("database_path"),
                    "--wall-timeout", LaunchConfiguration("wall_timeout"),
                ],
            ),
            Node(
                package="slam_robot_slam_3d",
                executable="map_projection_census",
                name="rtabmap_rgbd_online_map_census",
                output="screen",
                arguments=[
                    "--topic", "/rtabmap/map",
                    "--label", "rtabmap_rgbd_online",
                    "--output", LaunchConfiguration("map_output"),
                    "--wall-timeout", LaunchConfiguration("wall_timeout"),
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=driver,
                    on_exit=partial(finish_after_driver, checker=checker),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=checker,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="RTAB-Map RGB-D online verdict completed"
                            )
                        )
                    ],
                )
            ),
        ]
    )
