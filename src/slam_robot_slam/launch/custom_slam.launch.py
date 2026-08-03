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
    default_scan_matcher_params_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam"),
            "config",
            "scan_matcher.yaml",
        ]
    )

    params_file = LaunchConfiguration("params_file")
    scan_matcher_params_file = LaunchConfiguration("scan_matcher_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    reject_degenerate_loop_closures = LaunchConfiguration(
        "reject_degenerate_loop_closures"
    )

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
            DeclareLaunchArgument(
                "scan_matcher_params_file",
                default_value=default_scan_matcher_params_file,
                description="Correlative scan matcher parameter file.",
            ),
            DeclareLaunchArgument(
                "reject_degenerate_loop_closures",
                default_value="true",
                description="Reject translation-degenerate loop matches.",
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
            Node(
                package="slam_robot_slam",
                executable="scan_matcher_odometry_node",
                name="scan_matcher_odometry",
                output="screen",
                parameters=[
                    scan_matcher_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "loop_closure.reject_degenerate_matches":
                            reject_degenerate_loop_closures,
                    },
                ],
            ),
        ]
    )
