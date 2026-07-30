from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam"),
            "config",
            "laser_preprocessor.yaml",
        ]
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Laser preprocessing parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time.",
            ),
            Node(
                package="slam_robot_slam",
                executable="laser_scan_preprocessor_node",
                name="laser_scan_preprocessor",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
