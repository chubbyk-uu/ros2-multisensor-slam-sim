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
    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "launch", "simulation.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "rviz", "lidar_3d.rviz"]
    )

    forwarded_arguments = (
        "world",
        "gui",
        "rviz",
        "rviz_config",
        "use_wsl_gpu",
        "wsl_gpu_adapter",
        "sensor_variant",
        "odometry_mode",
        "left_wheel_friction",
        "right_wheel_friction",
        "lidar_3d_x",
        "lidar_3d_y",
        "lidar_3d_z",
        "lidar_3d_roll",
        "lidar_3d_pitch",
        "lidar_3d_yaw",
        "lidar_3d_horizontal_samples",
        "lidar_3d_horizontal_min_angle",
        "lidar_3d_horizontal_max_angle",
        "lidar_3d_vertical_samples",
        "lidar_3d_vertical_min_angle",
        "lidar_3d_vertical_max_angle",
        "lidar_3d_update_rate",
        "lidar_3d_minimum_range",
        "lidar_3d_maximum_range",
        "lidar_3d_noise_stddev",
        "imu_update_rate",
        "imu_angular_velocity_noise_stddev",
        "imu_angular_velocity_bias_stddev",
        "imu_linear_acceleration_noise_stddev",
        "imu_linear_acceleration_bias_stddev",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Absolute path to the Gazebo Sim world file.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the robot and 3D point-cloud displays.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_config,
                description="Absolute path to the 3D LiDAR RViz configuration.",
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
                description="LiDAR model variant; this entry defaults to 3d.",
            ),
            DeclareLaunchArgument(
                "odometry_mode",
                default_value="wheel",
                description="Odometry source: wheel or wheel_imu.",
            ),
            DeclareLaunchArgument(
                "left_wheel_friction",
                default_value="1.2",
                description="Left drive-wheel friction coefficient.",
            ),
            DeclareLaunchArgument(
                "right_wheel_friction",
                default_value="1.2",
                description="Right drive-wheel friction coefficient.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_x",
                default_value="-0.07",
                description="3D LiDAR sensing origin x in base_link, in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_y",
                default_value="0.0",
                description="3D LiDAR sensing origin y in base_link, in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_z",
                default_value="0.18",
                description="3D LiDAR sensing origin z in base_link, in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_roll",
                default_value="0.0",
                description="3D LiDAR mounting roll in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_pitch",
                default_value="0.0",
                description="3D LiDAR mounting pitch in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_yaw",
                default_value="0.0",
                description="3D LiDAR mounting yaw in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_horizontal_samples",
                default_value="720",
                description="Number of horizontal samples in each 3D scan.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_horizontal_min_angle",
                default_value="-3.141592653589793",
                description="Minimum horizontal scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_horizontal_max_angle",
                default_value="3.1328660073298216",
                description="Maximum horizontal scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_vertical_samples",
                default_value="16",
                description="Number of vertical scan channels.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_vertical_min_angle",
                default_value="-0.2617993877991494",
                description="Minimum vertical scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_vertical_max_angle",
                default_value="0.2617993877991494",
                description="Maximum vertical scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_update_rate",
                default_value="10.0",
                description="3D LiDAR update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_minimum_range",
                default_value="0.20",
                description="3D LiDAR minimum range in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_maximum_range",
                default_value="20.0",
                description="3D LiDAR maximum range in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_noise_stddev",
                default_value="0.01",
                description="Gaussian range noise standard deviation in metres.",
            ),
            DeclareLaunchArgument(
                "imu_update_rate",
                default_value="100.0",
                description="Raw IMU update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "imu_angular_velocity_noise_stddev",
                default_value="0.002",
                description="Gyroscope white-noise standard deviation in rad/s.",
            ),
            DeclareLaunchArgument(
                "imu_angular_velocity_bias_stddev",
                default_value="0.0002",
                description="Gyroscope startup-bias standard deviation in rad/s.",
            ),
            DeclareLaunchArgument(
                "imu_linear_acceleration_noise_stddev",
                default_value="0.02",
                description="Accelerometer white-noise standard deviation in m/s^2.",
            ),
            DeclareLaunchArgument(
                "imu_linear_acceleration_bias_stddev",
                default_value="0.005",
                description="Accelerometer startup-bias standard deviation in m/s^2.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    name: LaunchConfiguration(name)
                    for name in forwarded_arguments
                }.items(),
            ),
        ]
    )
