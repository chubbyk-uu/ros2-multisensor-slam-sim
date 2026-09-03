from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    rgbd_simulation_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "rtabmap_rgbd_simulation.launch.py",
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
        [
            FindPackageShare("slam_robot_gazebo"),
            "worlds",
            "structured_loop_3d.sdf",
        ]
    )
    default_rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "rviz",
            "rtabmap_rgbd_navigation.rviz",
        ]
    )
    default_dds_profile = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "fastdds_rgbd.xml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("odometry_mode", default_value="wheel_imu"),
            DeclareLaunchArgument(
                "database_path",
                default_value=str(Path.home() / ".ros" / "rtabmap_rgbd.db"),
            ),
            DeclareLaunchArgument("reset_database", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_dds_profile,
                ),
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(rgbd_simulation_launch),
                        launch_arguments={
                            "world": LaunchConfiguration("world"),
                            "gui": LaunchConfiguration("gui"),
                            "rviz": "false",
                            "odometry_mode": LaunchConfiguration("odometry_mode"),
                            "database_path": LaunchConfiguration("database_path"),
                            "reset_database": LaunchConfiguration("reset_database"),
                            "rgbd_dds_profiles_file": LaunchConfiguration(
                                "rgbd_dds_profiles_file"
                            ),
                            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
                            "wsl_gpu_adapter": LaunchConfiguration(
                                "wsl_gpu_adapter"
                            ),
                        }.items(),
                    )
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(navigation_launch),
                        launch_arguments={
                            "use_sim_time": "true",
                            "use_rviz": LaunchConfiguration("rviz"),
                            "rviz_config": LaunchConfiguration("rviz_config"),
                            "autostart": LaunchConfiguration("autostart"),
                            "use_composition": LaunchConfiguration(
                                "use_composition"
                            ),
                            "map_topic": "/rtabmap/map",
                            "lidar_topic": "/lidar_3d/points",
                        }.items(),
                    )
                ],
            ),
        ]
    )
