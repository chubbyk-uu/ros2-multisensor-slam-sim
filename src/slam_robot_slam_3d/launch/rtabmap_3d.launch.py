from functools import partial
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def start_rtabmap_after_contract(
    event, context, reset_database, database_path, lidar_topic
):
    if event.returncode != 0:
        reason = (
            "3D point-cloud input contract failed with exit code "
            f"{event.returncode}; RTAB-Map was not started."
        )
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]

    database_arguments = ["-d"] if reset_database else []

    return [
        LogInfo(
            msg=(
                "3D point-cloud contract passed; starting RTAB-Map online "
                "3D LiDAR SLAM with external local odometry."
            )
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            namespace="rtabmap",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("slam_robot_slam_3d"),
                        "config",
                        "rtabmap_3d.yaml",
                    ]
                ),
                {"database_path": database_path},
            ],
            remappings=[
                ("odom", "/odom"),
                ("scan_cloud", lidar_topic),
            ],
            arguments=database_arguments,
        ),
    ]


def register_rtabmap_start(context, contract_check):
    """
    Resolve every value the deferred handler needs while the scope still exists.

    The handler runs when the contract check exits, which is long after this
    launch description has been visited. A caller that wraps the include in a
    scoped GroupAction has popped the surrounding scope by then, so resolving
    LaunchConfiguration inside the handler aborts the whole launch with
    "launch configuration 'reset_database' does not exist". Capturing the plain
    values here keeps the file correct under any scoping the caller chooses.
    """
    reset_database = context.perform_substitution(
        LaunchConfiguration("reset_database")
    ).lower() in ("1", "true", "yes", "on")
    database_path = context.perform_substitution(
        LaunchConfiguration("database_path")
    )
    lidar_topic = context.perform_substitution(LaunchConfiguration("lidar_topic"))
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=contract_check,
                on_exit=partial(
                    start_rtabmap_after_contract,
                    reset_database=reset_database,
                    database_path=database_path,
                    lidar_topic=lidar_topic,
                ),
            )
        )
    ]


def generate_launch_description():
    lidar_topic = LaunchConfiguration("lidar_topic")
    expected_lidar_frame = LaunchConfiguration("expected_lidar_frame")
    contract_check = Node(
        package="slam_robot_slam_3d",
        executable="pointcloud_contract_check",
        name="rtabmap_pointcloud_contract_check",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "topic": lidar_topic,
                "expected_frame": expected_lidar_frame,
                "minimum_points": 100,
                "require_point_time": False,
                "timeout_sec": LaunchConfiguration("contract_timeout"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/lidar_3d/points",
                description="PointCloud2 topic consumed by RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "expected_lidar_frame",
                default_value="lidar_3d_link",
                description="Required PointCloud2 frame_id.",
            ),
            DeclareLaunchArgument(
                "contract_timeout",
                default_value="30.0",
                description="Wall-clock seconds to wait for a valid point cloud.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=str(Path.home() / ".ros" / "rtabmap_3d.db"),
                description="RTAB-Map SQLite database path.",
            ),
            DeclareLaunchArgument(
                "reset_database",
                default_value="true",
                description="Delete database_path before starting a fresh mapping run.",
            ),
            OpaqueFunction(
                function=register_rtabmap_start,
                args=[contract_check],
            ),
            contract_check,
        ]
    )
