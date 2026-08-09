"""RTAB-Map, Nav2, and autonomous frontier exploration baseline."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename):
    return PathJoinSubstitution([FindPackageShare(package), "launch", filename])


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    default_exploration_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_navigation"),
            "config",
            "frontier_exploration.yaml",
        ]
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_slam_3d", "rtabmap_navigation_simulation.launch.py"
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "database_path": LaunchConfiguration("database_path"),
            "reset_database": LaunchConfiguration("reset_database"),
            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
            "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
        }.items(),
    )
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch("slam_robot_navigation", "frontier_exploration.launch.py")
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("exploration_params_file"),
            "use_sim_time": "true",
            "map_topic": "/rtabmap/map",
            "save_snapshot_on_completion": "false",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "database_path",
                default_value=str(Path.home() / ".ros" / "rtabmap_3d.db"),
            ),
            DeclareLaunchArgument("reset_database", default_value="true"),
            DeclareLaunchArgument(
                "exploration_params_file",
                default_value=default_exploration_parameters,
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
            GroupAction(scoped=True, actions=[navigation]),
            GroupAction(scoped=True, actions=[exploration]),
        ]
    )
