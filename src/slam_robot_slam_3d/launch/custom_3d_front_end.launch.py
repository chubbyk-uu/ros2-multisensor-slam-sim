from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    parameters = [
        LaunchConfiguration("params_file"),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_parameters,
                description="Custom 3D front-end parameter file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
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
