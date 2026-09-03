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


def generate_launch_description():
    rtabmap_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam_3d"), "launch", "rtabmap_rgbd.launch.py"]
    )
    default_bag = str(Path.cwd() / "bags" / "structured_rgbd_reference")
    default_dds_profile = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "fastdds_rgbd.xml"]
    )
    playback = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag"),
            "--clock",
            "100",
            "--rate",
            LaunchConfiguration("rate"),
            "--disable-keyboard-controls",
        ],
        output="screen",
        additional_env={
            "FASTRTPS_DEFAULT_PROFILES_FILE": LaunchConfiguration(
                "rgbd_dds_profiles_file"
            )
        },
    )
    checker = Node(
        package="slam_robot_slam_3d",
        executable="rtabmap_rgbd_fixed_regression_check",
        name="rtabmap_rgbd_fixed_regression_check",
        output="screen",
        arguments=[
            "--trajectory", LaunchConfiguration("trajectory_output"),
            "--map", LaunchConfiguration("map_output"),
            "--output", LaunchConfiguration("verdict_output"),
            "--minimum-loop-closures",
            LaunchConfiguration("minimum_loop_closures"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", default_value=default_bag),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/slam_robot_rtabmap_rgbd_fixed.db",
            ),
            DeclareLaunchArgument(
                "trajectory_output",
                default_value="/tmp/rtabmap_rgbd_trajectory.json",
            ),
            DeclareLaunchArgument(
                "map_output",
                default_value="/tmp/rtabmap_rgbd_map.json",
            ),
            DeclareLaunchArgument(
                "verdict_output",
                default_value="/tmp/rtabmap_rgbd_fixed_verdict.json",
            ),
            DeclareLaunchArgument("minimum_loop_closures", default_value="20"),
            DeclareLaunchArgument("wall_timeout", default_value="600.0"),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_dds_profile,
                ),
            ),
            OpaqueFunction(function=prepare_output_files),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(rtabmap_launch),
                        launch_arguments={
                            "database_path": LaunchConfiguration("database_path"),
                            "reset_database": "true",
                            "rgbd_dds_profiles_file": LaunchConfiguration(
                                "rgbd_dds_profiles_file"
                            ),
                        }.items(),
                    )
                ],
            ),
            Node(
                package="slam_robot_slam_3d",
                executable="recorded_odom_tf_publisher",
                name="recorded_odom_tf_publisher",
                output="screen",
                parameters=[{"use_sim_time": True, "odom_topic": "/odom"}],
            ),
            Node(
                package="slam_robot_slam_3d",
                executable="stack_trajectory_census",
                name="rtabmap_rgbd_trajectory_census",
                output="screen",
                arguments=[
                    "--label", "rtabmap_rgbd",
                    "--output", LaunchConfiguration("trajectory_output"),
                    "--process-match", "/rtabmap_sync/rgbd_sync",
                    "--process-match", "/rtabmap_slam/rtabmap",
                    "--info-topic", "/rtabmap/info",
                    "--database-path", LaunchConfiguration("database_path"),
                    "--wall-timeout", LaunchConfiguration("wall_timeout"),
                ],
            ),
            Node(
                package="slam_robot_slam_3d",
                executable="map_projection_census",
                name="rtabmap_rgbd_map_census",
                output="screen",
                arguments=[
                    "--topic", "/rtabmap/map",
                    "--label", "rtabmap_rgbd",
                    "--output", LaunchConfiguration("map_output"),
                    "--wall-timeout", LaunchConfiguration("wall_timeout"),
                ],
            ),
            TimerAction(period=6.0, actions=[playback]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=playback,
                    on_exit=[
                        TimerAction(
                            period=20.0,
                            actions=[checker],
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=checker,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="RTAB-Map RGB-D fixed verdict completed"
                            )
                        )
                    ],
                )
            ),
        ]
    )
