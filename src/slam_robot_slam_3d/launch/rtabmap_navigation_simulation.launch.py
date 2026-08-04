from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    rtabmap_simulation_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "rtabmap_3d_simulation.launch.py",
        ]
    )
    navigation_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_navigation"),
            "launch",
            "online_slam_navigation.launch.py",
        ]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("odometry_mode", default_value="wheel_imu"),
            DeclareLaunchArgument(
                "database_path",
                default_value=str(Path.home() / ".ros" / "rtabmap_3d.db"),
            ),
            DeclareLaunchArgument("reset_database", default_value="true"),
            DeclareLaunchArgument("contract_timeout", default_value="30.0"),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_simulation_launch),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "rviz": "false",
                    "odometry_mode": LaunchConfiguration("odometry_mode"),
                    "database_path": LaunchConfiguration("database_path"),
                    "reset_database": LaunchConfiguration("reset_database"),
                    "contract_timeout": LaunchConfiguration("contract_timeout"),
                    "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
                    "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "use_rviz": LaunchConfiguration("rviz"),
                    "autostart": LaunchConfiguration("autostart"),
                    "use_composition": LaunchConfiguration("use_composition"),
                    "map_topic": "/rtabmap/map",
                    "lidar_topic": "/lidar_3d/points",
                }.items(),
            ),
        ]
    )
