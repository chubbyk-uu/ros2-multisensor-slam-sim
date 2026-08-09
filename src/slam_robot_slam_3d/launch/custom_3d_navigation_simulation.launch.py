"""Self-developed 3D SLAM with Nav2 online navigation."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    def launch(package, name):
        return PathJoinSubstitution([FindPackageShare(package), "launch", name])
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"])),
        DeclareLaunchArgument("gui", default_value="true"), DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([FindPackageShare("slam_robot_slam_3d"), "config", "custom_3d_slam.yaml"])),
        DeclareLaunchArgument("use_wsl_gpu", default_value="true" if _wsl() else "false"),
        DeclareLaunchArgument("wsl_gpu_adapter", default_value="NVIDIA"),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(launch("slam_robot_gazebo", "simulation.launch.py")), launch_arguments={"world": LaunchConfiguration("world"), "gui": LaunchConfiguration("gui"), "rviz": "false", "sensor_variant": "3d", "odometry_mode": "wheel_imu", "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"), "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter")}.items())]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(launch("slam_robot_slam_3d", "custom_3d_front_end.launch.py")), launch_arguments={"params_file": LaunchConfiguration("params_file"), "use_sim_time": "true"}.items())]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch("slam_robot_navigation", "online_slam_navigation.launch.py")), launch_arguments={"use_sim_time": "true", "use_rviz": LaunchConfiguration("rviz"), "map_topic": "/map", "lidar_topic": "/lidar_3d/points"}.items()),
    ])
