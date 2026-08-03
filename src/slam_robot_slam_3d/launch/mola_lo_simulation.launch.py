from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    simulation_launch = PathJoinSubstitution(
        [FindPackageShare('slam_robot_gazebo'), 'launch', 'simulation.launch.py']
    )
    mola_launch = PathJoinSubstitution(
        [FindPackageShare('slam_robot_slam_3d'), 'launch', 'mola_lo.launch.py']
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare('slam_robot_gazebo'), 'worlds', 'slam_world.sdf']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=default_world,
                description='Absolute path to the Gazebo Sim world file.',
            ),
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                description='Start the Gazebo graphical client.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description="Start MOLA's official RViz configuration.",  # noqa: Q000
            ),
            DeclareLaunchArgument(
                'mola_gui',
                default_value='false',
                description='Start the separate MolaViz GUI.',
            ),
            DeclareLaunchArgument(
                'odometry_mode',
                default_value='wheel',
                description='Gazebo odometry source: wheel or wheel_imu.',
            ),
            DeclareLaunchArgument(
                'enforce_planar_motion',
                default_value='false',
                description='Constrain the 3D pose estimate to planar motion.',
            ),
            DeclareLaunchArgument(
                'use_imu_gravity',
                default_value='false',
                description='Use the IMU as a gravity prior without enabling LIO.',
            ),
            DeclareLaunchArgument(
                'contract_timeout',
                default_value='30.0',
                description='Wall-clock seconds to wait for a valid point cloud.',
            ),
            DeclareLaunchArgument(
                'use_wsl_gpu',
                default_value='true' if running_in_wsl() else 'false',
                description="Use Mesa's D3D12 renderer in WSL.",  # noqa: Q000
            ),
            DeclareLaunchArgument(
                'wsl_gpu_adapter',
                default_value='NVIDIA',
                description='GPU adapter selected by Mesa D3D12 in WSL.',
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(simulation_launch),
                        launch_arguments={
                            'world': LaunchConfiguration('world'),
                            'gui': LaunchConfiguration('gui'),
                            'rviz': 'false',
                            'sensor_variant': '3d',
                            'odometry_mode': LaunchConfiguration('odometry_mode'),
                            'use_wsl_gpu': LaunchConfiguration('use_wsl_gpu'),
                            'wsl_gpu_adapter': LaunchConfiguration(
                                'wsl_gpu_adapter'
                            ),
                        }.items(),
                    )
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mola_launch),
                launch_arguments={
                    'lidar_topic': '/lidar_3d/points',
                    'expected_lidar_frame': 'lidar_3d_link',
                    'contract_timeout': LaunchConfiguration('contract_timeout'),
                    'rviz': LaunchConfiguration('rviz'),
                    'mola_gui': LaunchConfiguration('mola_gui'),
                    'enforce_planar_motion': LaunchConfiguration(
                        'enforce_planar_motion'
                    ),
                    'use_imu_gravity': LaunchConfiguration('use_imu_gravity'),
                }.items(),
            ),
        ]
    )
