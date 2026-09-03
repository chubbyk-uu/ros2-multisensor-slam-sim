"""
Replay one fixed dataset through a single mapping stack and measure it.

Run this once per stack against the same bag. Because both passes consume
byte-identical sensor data, the resulting difference is attributable to the
stack alone: no exploration policy runs, the trajectories are the same
recorded trajectory, and neither pass depends on real-time factor.

One pass produces the four quantities a stack comparison needs. The grid census
answers what the 2D projection contains; the trajectory census answers how far
the estimate drifts from ground truth, how many loop events the back end
committed, and what the stack costs in CPU, memory and on-disk map.
"""

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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_launch(filename):
    return PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam_3d"), "launch", filename]
    )


def generate_launch_description():
    stack = LaunchConfiguration("stack")
    is_custom = IfCondition(EqualsSubstitution(stack, "custom"))
    is_rtabmap = IfCondition(EqualsSubstitution(stack, "rtabmap"))
    default_bag = str(Path.cwd() / "bags" / "structured_3d_reference")
    default_output = str(Path.cwd() / "bags" / "map_projection_census.json")
    default_trajectory = str(Path.cwd() / "bags" / "stack_trajectory_census.json")

    custom_front_end = GroupAction(
        scoped=True,
        condition=is_custom,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    package_launch("custom_3d_front_end.launch.py")
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": "true",
                }.items(),
            )
        ],
    )
    # Started directly rather than through rtabmap_3d.launch.py, which gates
    # RTAB-Map behind a point-cloud contract check that cannot pass until the
    # bag is already playing. Waiting for it would let RTAB-Map miss the first
    # seconds of a dataset the custom stack received in full, which is exactly
    # the asymmetry this comparison exists to avoid. The same config file is
    # used, so the mapping parameters are unchanged.
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        condition=is_rtabmap,
        parameters=[
            LaunchConfiguration("rtabmap_params_file"),
            {"database_path": LaunchConfiguration("database_path")},
        ],
        remappings=[("odom", "/odom"), ("scan_cloud", "/lidar_3d/points")],
        arguments=["-d"],
    )
    # Started for both stacks even though only RTAB-Map looks the transform up,
    # so the two passes differ in nothing but the stack under test.
    odom_tf = Node(
        package="slam_robot_slam_3d",
        executable="recorded_odom_tf_publisher",
        name="recorded_odom_tf_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "odom_topic": "/odom"}],
    )
    census = Node(
        package="slam_robot_slam_3d",
        executable="map_projection_census",
        name="map_projection_census",
        output="screen",
        arguments=[
            "--topic", LaunchConfiguration("map_topic"),
            "--label", stack,
            "--output", LaunchConfiguration("output"),
            "--wall-timeout", LaunchConfiguration("wall_timeout"),
        ],
    )
    # Two nodes rather than one: the process to sample out of /proc and the
    # database to size differ per stack, and only one of them ever runs.
    custom_trajectory = Node(
        package="slam_robot_slam_3d",
        executable="stack_trajectory_census",
        name="stack_trajectory_census",
        output="screen",
        condition=is_custom,
        arguments=[
            "--label", stack,
            "--output", LaunchConfiguration("trajectory_output"),
            "--process-match", "/scan_to_map_odometry_node",
            "--wall-timeout", LaunchConfiguration("wall_timeout"),
        ],
    )
    rtabmap_trajectory = Node(
        package="slam_robot_slam_3d",
        executable="stack_trajectory_census",
        name="stack_trajectory_census",
        output="screen",
        condition=is_rtabmap,
        arguments=[
            "--label", stack,
            "--output", LaunchConfiguration("trajectory_output"),
            "--process-match", "/rtabmap_slam/rtabmap",
            "--info-topic", "/rtabmap/info",
            "--database-path", LaunchConfiguration("database_path"),
            "--wall-timeout", LaunchConfiguration("wall_timeout"),
        ],
    )
    playback = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", LaunchConfiguration("bag"),
            "--clock", "100", "--rate", LaunchConfiguration("rate"),
            "--disable-keyboard-controls",
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stack",
                default_value="custom",
                description="Mapping stack to census: custom or rtabmap.",
            ),
            DeclareLaunchArgument("bag", default_value=default_bag),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("output", default_value=default_output),
            DeclareLaunchArgument(
                "trajectory_output", default_value=default_trajectory
            ),
            DeclareLaunchArgument("wall_timeout", default_value="1800.0"),
            DeclareLaunchArgument(
                "map_topic",
                default_value="/map",
                description="Use /rtabmap/map when stack:=rtabmap.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("slam_robot_slam_3d"),
                        "config",
                        "custom_3d_slam.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/slam_robot_map_projection_census.db",
            ),
            # Overridable so a run can match RTAB-Map's detection rate to the
            # custom front end's. The shipped config processes 1 Hz of a 10 Hz
            # cloud, which is a navigation choice rather than an algorithmic
            # limit; comparing accuracy against it without saying so would
            # report a configured rate as a difference in scan matching.
            DeclareLaunchArgument(
                "rtabmap_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("slam_robot_slam_3d"),
                        "config",
                        "rtabmap_3d.yaml",
                    ]
                ),
            ),
            custom_front_end,
            rtabmap,
            odom_tf,
            census,
            custom_trajectory,
            rtabmap_trajectory,
            # RTAB-Map only starts after its point-cloud contract check passes,
            # which itself needs data; give both stacks time to subscribe
            # before the recorded clock starts advancing.
            TimerAction(period=6.0, actions=[playback]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=playback,
                    on_exit=[
                        # The census node writes on every update, so the file is
                        # already final here; the delay only lets the last
                        # in-flight rebuild land.
                        TimerAction(
                            period=20.0,
                            actions=[
                                EmitEvent(
                                    event=Shutdown(
                                        reason="map projection census completed"
                                    )
                                )
                            ],
                        )
                    ],
                )
            ),
        ]
    )
