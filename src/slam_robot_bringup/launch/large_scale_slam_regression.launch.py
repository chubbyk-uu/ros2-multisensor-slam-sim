from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    custom_slam_development = PathJoinSubstitution(
        [
            FindPackageShare('slam_robot_bringup'),
            'launch',
            'custom_slam_development.launch.py',
        ]
    )
    large_warehouse_world = PathJoinSubstitution(
        [
            FindPackageShare('slam_robot_gazebo'),
            'worlds',
            'large_warehouse.sdf',
        ]
    )

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_wsl_gpu = LaunchConfiguration('use_wsl_gpu')
    wsl_gpu_adapter = LaunchConfiguration('wsl_gpu_adapter')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gui',
                default_value='false',
                description='Start the Gazebo graphical client.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='Start the custom SLAM RViz view.',
            ),
            DeclareLaunchArgument(
                'use_wsl_gpu',
                default_value='true' if running_in_wsl() else 'false',
                description='Use Mesa D3D12 rendering in WSL.',
            ),
            DeclareLaunchArgument(
                'wsl_gpu_adapter',
                default_value='NVIDIA',
                description='GPU adapter selected by Mesa D3D12 in WSL.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(custom_slam_development),
                launch_arguments={
                    'world': large_warehouse_world,
                    'gui': gui,
                    'use_rviz': use_rviz,
                    'use_wsl_gpu': use_wsl_gpu,
                    'wsl_gpu_adapter': wsl_gpu_adapter,
                    # Keep the recorded large-scale baseline reproducible.
                    'odometry_mode': 'wheel',
                }.items(),
            ),
        ]
    )
