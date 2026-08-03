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
    large_scale_regression = PathJoinSubstitution(
        [
            FindPackageShare('slam_robot_bringup'),
            'launch',
            'large_scale_slam_regression.launch.py',
        ]
    )
    record_slam_data = PathJoinSubstitution(
        [
            FindPackageShare('slam_robot_slam'),
            'launch',
            'record_slam_data.launch.py',
        ]
    )

    output = LaunchConfiguration('output')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_wsl_gpu = LaunchConfiguration('use_wsl_gpu')
    wsl_gpu_adapter = LaunchConfiguration('wsl_gpu_adapter')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'output',
                default_value=str(
                    Path.cwd() / 'bags' / 'large_scale_reference'
                ),
                description='Output directory for the fixed MCAP dataset.',
            ),
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
                PythonLaunchDescriptionSource(large_scale_regression),
                launch_arguments={
                    'gui': gui,
                    'use_rviz': use_rviz,
                    'use_wsl_gpu': use_wsl_gpu,
                    'wsl_gpu_adapter': wsl_gpu_adapter,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(record_slam_data),
                launch_arguments={
                    'output': output,
                    # /tf would contain the live SLAM map -> odom transform.
                    # The fixed dataset must contain inputs, not old outputs.
                    'include_dynamic_tf': 'false',
                }.items(),
            ),
        ]
    )
