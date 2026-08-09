from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_navigation"),
            "config",
            "frontier_exploration.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=default_parameters
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "save_snapshot_on_completion", default_value="true"
            ),
            DeclareLaunchArgument(
                "snapshot_service",
                default_value="/scan_to_map_odometry_3d/save_snapshot",
            ),
            Node(
                package="slam_robot_navigation",
                executable="frontier_explorer_node",
                name="frontier_explorer",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "map_topic": LaunchConfiguration("map_topic"),
                        "save_snapshot_on_completion": LaunchConfiguration(
                            "save_snapshot_on_completion"
                        ),
                        "snapshot_service": LaunchConfiguration(
                            "snapshot_service"
                        ),
                    },
                ],
            ),
        ]
    )
