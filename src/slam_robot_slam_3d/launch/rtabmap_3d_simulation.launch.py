from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "launch", "simulation.launch.py"]
    )
    rtabmap_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam_3d"), "launch", "rtabmap_3d.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam_3d"), "rviz", "rtabmap_3d.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument(
                "odometry_mode",
                default_value="wheel_imu",
                description=(
                    "Local odometry source: wheel_imu (default EKF fusion) "
                    "or wheel."
                ),
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=str(Path.home() / ".ros" / "rtabmap_3d.db"),
                description="RTAB-Map SQLite database path.",
            ),
            DeclareLaunchArgument("reset_database", default_value="true"),
            DeclareLaunchArgument("contract_timeout", default_value="30.0"),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(simulation_launch),
                        launch_arguments={
                            "world": LaunchConfiguration("world"),
                            "gui": LaunchConfiguration("gui"),
                            "rviz": "false",
                            "sensor_variant": "3d",
                            "odometry_mode": LaunchConfiguration("odometry_mode"),
                            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
                            "wsl_gpu_adapter": LaunchConfiguration(
                                "wsl_gpu_adapter"
                            ),
                        }.items(),
                    )
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch),
                launch_arguments={
                    "lidar_topic": "/lidar_3d/points",
                    "expected_lidar_frame": "lidar_3d_link",
                    "database_path": LaunchConfiguration("database_path"),
                    "reset_database": LaunchConfiguration("reset_database"),
                    "contract_timeout": LaunchConfiguration("contract_timeout"),
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rtabmap_rviz",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
