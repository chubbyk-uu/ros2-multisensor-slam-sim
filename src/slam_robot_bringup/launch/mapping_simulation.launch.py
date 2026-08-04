from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
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
    mapping_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam"), "launch", "mapping.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_robot_bringup"), "rviz", "mapping.rviz"]
    )

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_wsl_gpu = LaunchConfiguration("use_wsl_gpu")
    wsl_gpu_adapter = LaunchConfiguration("wsl_gpu_adapter")
    odometry_mode = LaunchConfiguration("odometry_mode")
    left_wheel_friction = LaunchConfiguration("left_wheel_friction")
    right_wheel_friction = LaunchConfiguration("right_wheel_friction")
    auto_save_map = LaunchConfiguration("auto_save_map")
    map_output_prefix = LaunchConfiguration("map_output_prefix")

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
                "use_rviz",
                default_value="true",
                description="Start RViz with the mapping configuration.",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
                description=(
                    "Use Mesa's D3D12 renderer instead of llvmpipe in WSL."
                ),
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter",
                default_value="NVIDIA",
                description="GPU adapter name selected by Mesa D3D12 in WSL.",
            ),
            DeclareLaunchArgument(
                "odometry_mode",
                default_value="wheel_imu",
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
                "auto_save_map",
                default_value="true",
                description="Save the map and pose graph before shutdown.",
            ),
            DeclareLaunchArgument(
                "map_output_prefix",
                default_value=str(Path.cwd() / "maps" / "slam_map"),
                description=(
                    "Auto-save output path without a file extension."
                ),
            ),
            SetEnvironmentVariable(
                "GALLIUM_DRIVER",
                "d3d12",
                condition=IfCondition(use_wsl_gpu),
            ),
            SetEnvironmentVariable(
                "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                wsl_gpu_adapter,
                condition=IfCondition(use_wsl_gpu),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    "world": world,
                    "gui": gui,
                    "rviz": "false",
                    "use_wsl_gpu": use_wsl_gpu,
                    "wsl_gpu_adapter": wsl_gpu_adapter,
                    "odometry_mode": odometry_mode,
                    "left_wheel_friction": left_wheel_friction,
                    "right_wheel_friction": right_wheel_friction,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "auto_save_map": auto_save_map,
                    "map_output_prefix": map_output_prefix,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
