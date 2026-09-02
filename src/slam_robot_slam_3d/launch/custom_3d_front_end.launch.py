from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "config",
            "custom_3d_slam.yaml",
        ]
    )
    # Sizes the shared-memory segment for this node's large map topics; see
    # the profile itself for the payloads and the stall it prevents. An
    # operator who already exports a profile keeps theirs, because a second
    # profile silently replacing the first is worse than a large payload.
    default_profiles_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "config",
            "fastdds_large_cloud.xml",
        ]
    )
    parameters = [
        LaunchConfiguration("params_file"),
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "persistence.mode": LaunchConfiguration("mode"),
            "persistence.snapshot_path": LaunchConfiguration("snapshot_path"),
            "persistence.load_snapshot": LaunchConfiguration("load_snapshot"),
            "persistence.save_on_shutdown": LaunchConfiguration(
                "save_on_shutdown"
            ),
        },
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_parameters,
                description="Custom 3D front-end parameter file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("mode", default_value="mapping"),
            DeclareLaunchArgument(
                "snapshot_path",
                default_value=str(
                    Path.home() / ".ros" / "custom_slam_3d.snapshot"
                ),
            ),
            DeclareLaunchArgument("load_snapshot", default_value="false"),
            DeclareLaunchArgument(
                "dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_profiles_file,
                ),
                description=(
                    "Fast DDS XML profile applied to the front-end nodes. "
                    "Defaults to this package's large-cloud profile unless the "
                    "environment already names one."
                ),
            ),
            DeclareLaunchArgument("save_on_shutdown", default_value="false"),
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                LaunchConfiguration("dds_profiles_file"),
            ),
            Node(
                package="slam_robot_slam_3d",
                executable="point_cloud_preprocessor_node",
                name="point_cloud_preprocessor_3d",
                output="screen",
                parameters=parameters,
            ),
            Node(
                package="slam_robot_slam_3d",
                executable="scan_to_map_odometry_node",
                name="scan_to_map_odometry_3d",
                output="screen",
                parameters=parameters,
            ),
        ]
    )
