from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_rtabmap_node(context, large_message_environment):
    reset_database = context.perform_substitution(
        LaunchConfiguration("reset_database")
    ).lower() in ("1", "true", "yes", "on")
    return [
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
                        "rtabmap_rgbd.yaml",
                    ]
                ),
                {"database_path": LaunchConfiguration("database_path")},
            ],
            remappings=[
                ("odom", "/odom"),
                ("rgbd_image", LaunchConfiguration("rgbd_topic")),
            ],
            arguments=["-d"] if reset_database else [],
            additional_env=large_message_environment,
        )
    ]


def generate_launch_description():
    default_dds_profile = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "config",
            "fastdds_rgbd.xml",
        ]
    )
    rgbd_dds_profiles_file = LaunchConfiguration("rgbd_dds_profiles_file")
    large_message_environment = {
        "FASTRTPS_DEFAULT_PROFILES_FILE": rgbd_dds_profiles_file
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value=str(Path.home() / ".ros" / "rtabmap_rgbd.db"),
                description="RTAB-Map RGB-D SQLite database path.",
            ),
            DeclareLaunchArgument(
                "reset_database",
                default_value="true",
                description="Delete the RGB-D database before mapping.",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/depth/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "rgbd_topic",
                default_value="/rtabmap/rgbd_image",
            ),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_dds_profile,
                ),
                description="Fast DDS profile used by RGB-D large-message nodes.",
            ),
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name="rgbd_sync",
                namespace="rtabmap",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "approx_sync": False,
                        "topic_queue_size": 10,
                        "sync_queue_size": 10,
                        "qos": 2,
                        "qos_camera_info": 2,
                    }
                ],
                remappings=[
                    ("rgb/image", LaunchConfiguration("rgb_topic")),
                    ("depth/image", LaunchConfiguration("depth_topic")),
                    (
                        "rgb/camera_info",
                        LaunchConfiguration("camera_info_topic"),
                    ),
                    ("rgbd_image", LaunchConfiguration("rgbd_topic")),
                ],
                additional_env=large_message_environment,
            ),
            OpaqueFunction(
                function=create_rtabmap_node,
                args=[large_message_environment],
            ),
        ]
    )
