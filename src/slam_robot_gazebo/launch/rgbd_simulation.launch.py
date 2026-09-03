from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "launch", "simulation.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "rviz", "rgbd.rviz"]
    )
    default_rgbd_dds_profiles_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "config",
            "fastdds_rgbd.xml",
        ]
    )

    forwarded_arguments = (
        "world",
        "world_name",
        "gui",
        "rviz",
        "rviz_config",
        "use_wsl_gpu",
        "wsl_gpu_adapter",
        "sensor_variant",
        "camera_variant",
        "odometry_mode",
        "rgbd_width",
        "rgbd_height",
        "rgbd_update_rate",
        "rgbd_horizontal_fov",
        "rgbd_minimum_depth",
        "rgbd_maximum_depth",
        "rgbd_pointcloud",
        "rgbd_dds_profiles_file",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Absolute path to the Gazebo Sim world file.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="slam_world",
                description="The <world name> inside the selected SDF file.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with RGB, depth, and point-cloud displays.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_config,
                description="Absolute path to the RGB-D RViz configuration.",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
                description="Use Mesa's D3D12 renderer in WSL.",
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter",
                default_value="NVIDIA",
                description="GPU adapter selected by Mesa D3D12 in WSL.",
            ),
            DeclareLaunchArgument(
                "sensor_variant",
                default_value="3d",
                description="LiDAR model variant; RGB-D validation defaults to 3d.",
            ),
            DeclareLaunchArgument(
                "camera_variant",
                default_value="rgbd",
                description="Camera model variant; this entry defaults to rgbd.",
            ),
            DeclareLaunchArgument(
                "odometry_mode",
                default_value="wheel_imu",
                description="Odometry source: wheel or wheel_imu.",
            ),
            DeclareLaunchArgument(
                "rgbd_width",
                default_value="640",
                description="RGB and depth image width in pixels.",
            ),
            DeclareLaunchArgument(
                "rgbd_height",
                default_value="480",
                description="RGB and depth image height in pixels.",
            ),
            DeclareLaunchArgument(
                "rgbd_update_rate",
                default_value="30.0",
                description="RGB-D frame rate in Hz.",
            ),
            DeclareLaunchArgument(
                "rgbd_horizontal_fov",
                default_value="1.2217304763960306",
                description="RGB-D horizontal field of view in radians.",
            ),
            DeclareLaunchArgument(
                "rgbd_minimum_depth",
                default_value="0.20",
                description="Minimum valid RGB-D depth in metres.",
            ),
            DeclareLaunchArgument(
                "rgbd_maximum_depth",
                default_value="6.0",
                description="Maximum valid RGB-D depth in metres.",
            ),
            DeclareLaunchArgument(
                "rgbd_pointcloud",
                default_value="false",
                description=(
                    "Bridge the dense RGB-D cloud for an explicit spatial check; "
                    "disabled by default to preserve the 30 Hz image streams."
                ),
            ),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_rgbd_dds_profiles_file,
                ),
                description=(
                    "Fast DDS XML profile for RGB-D large-message writers."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    name: LaunchConfiguration(name) for name in forwarded_arguments
                }.items(),
            ),
        ]
    )
