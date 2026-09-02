"""Replay the fixed dataset and require a successful custom 3D loop closure."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
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


def generate_launch_description():
    smoke = LaunchConfiguration("smoke")
    front_end_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "custom_3d_front_end.launch.py",
        ]
    )
    default_bag = str(Path.cwd() / "bags" / "structured_3d_reference")
    playback = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", LaunchConfiguration("bag"),
            "--clock", "--rate", LaunchConfiguration("rate"),
        ],
        output="screen",
        condition=UnlessCondition(smoke),
    )
    regression = Node(
        package="slam_robot_slam_3d",
        executable="front_end_regression",
        name="custom_3d_loop_regression",
        output="screen",
        parameters=[
            {
                "minimum_samples": 300,
                "minimum_truth_distance": 50.0,
                "minimum_sim_time": 300.0,
                "minimum_pose_graph_commits": 1,
                "maximum_pose_graph_discards": 0,
                "maximum_pose_graph_failures": 0,
                "timeout_sec": 480.0,
            }
        ],
        condition=UnlessCondition(smoke),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", default_value=default_bag),
            DeclareLaunchArgument("rate", default_value="1.0"),
            # Kept for a uniform headless composite-launch interface.  This
            # fixed-bag regression starts no simulator or RViz process.
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
                FindPackageShare("slam_robot_slam_3d"), "config",
                "custom_3d_slam.yaml",
            ])),
            DeclareLaunchArgument("smoke", default_value="false"),
            DeclareLaunchArgument(
                "preprocessor_overrides_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("slam_robot_slam_3d"), "config",
                    "preprocessing_experiments", "baseline.yaml",
                ]),
                description="PointCloudPreprocessor experimental override file.",
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(front_end_launch),
                        launch_arguments={
                            "params_file": LaunchConfiguration("params_file"),
                            "use_sim_time": "true",
                            "preprocessor_overrides_file": LaunchConfiguration(
                                "preprocessor_overrides_file"
                            ),
                        }.items(),
                    )
                ],
            ),
            TimerAction(period=2.0, actions=[regression]),
            TimerAction(period=4.0, actions=[playback]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=regression,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="custom 3D loop regression finished"))
                    ],
                )
            ),
        ]
    )
