"""Self-developed 3D SLAM with Nav2 online navigation."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        release = Path("/proc/sys/kernel/osrelease").read_text().lower()
        return "microsoft" in release
    except OSError:
        return False


def package_launch(package, filename):
    return PathJoinSubstitution(
        [FindPackageShare(package), "launch", filename]
    )


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    default_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "config",
            "custom_3d_slam.yaml",
        ]
    )
    default_snapshot = str(
        Path.home() / ".ros" / "custom_slam_3d.snapshot"
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch("slam_robot_gazebo", "simulation.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": "false",
            "sensor_variant": "3d",
            "odometry_mode": "wheel_imu",
            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
            "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
        }.items(),
    )
    front_end = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_slam_3d", "custom_3d_front_end.launch.py"
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "true",
            "mode": LaunchConfiguration("mode"),
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "load_snapshot": LaunchConfiguration("load_snapshot"),
            "save_on_shutdown": LaunchConfiguration("save_on_shutdown"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_navigation", "online_slam_navigation.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": LaunchConfiguration("rviz"),
            "map_topic": "/map",
            "lidar_topic": "/lidar_3d/points",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "params_file", default_value=default_parameters
            ),
            DeclareLaunchArgument("mode", default_value="mapping"),
            DeclareLaunchArgument(
                "snapshot_path", default_value=default_snapshot
            ),
            DeclareLaunchArgument("load_snapshot", default_value="false"),
            DeclareLaunchArgument("save_on_shutdown", default_value="true"),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter", default_value="NVIDIA"
            ),
            GroupAction(scoped=True, actions=[simulation]),
            GroupAction(scoped=True, actions=[front_end]),
            navigation,
        ]
    )
