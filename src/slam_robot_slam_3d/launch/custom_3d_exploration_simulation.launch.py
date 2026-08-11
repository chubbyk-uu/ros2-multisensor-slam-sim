"""Self-developed 3D SLAM, Nav2, and autonomous frontier exploration."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename):
    return PathJoinSubstitution(
        [FindPackageShare(package), "launch", filename]
    )


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    default_slam_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "config",
            "custom_3d_slam.yaml",
        ]
    )
    default_exploration_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_navigation"),
            "config",
            "frontier_exploration.yaml",
        ]
    )
    default_snapshot = str(
        Path.home() / ".ros" / "custom_slam_3d.snapshot"
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_slam_3d",
                "custom_3d_navigation_simulation.launch.py",
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "mode": "mapping",
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "load_snapshot": LaunchConfiguration("load_snapshot"),
            "save_on_shutdown": "true",
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
            "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
        }.items(),
    )
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_navigation", "frontier_exploration.launch.py"
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("exploration_params_file"),
            "use_sim_time": "true",
            "selection_random_seed": LaunchConfiguration(
                "exploration_seed"
            ),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "slam_params_file", default_value=default_slam_parameters
            ),
            DeclareLaunchArgument(
                "exploration_params_file",
                default_value=default_exploration_parameters,
            ),
            DeclareLaunchArgument(
                "snapshot_path", default_value=default_snapshot
            ),
            DeclareLaunchArgument("load_snapshot", default_value="false"),
            DeclareLaunchArgument("exploration_seed", default_value="0"),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter", default_value="NVIDIA"
            ),
            GroupAction(scoped=True, actions=[navigation]),
            GroupAction(scoped=True, actions=[exploration]),
        ]
    )
