from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
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
    custom_slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam"), "launch", "custom_slam.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_robot_slam"), "rviz", "custom_slam.rviz"]
    )

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_wsl_gpu = LaunchConfiguration("use_wsl_gpu")
    wsl_gpu_adapter = LaunchConfiguration("wsl_gpu_adapter")
    odometry_mode = LaunchConfiguration("odometry_mode")
    left_wheel_friction = LaunchConfiguration("left_wheel_friction")
    right_wheel_friction = LaunchConfiguration("right_wheel_friction")
    reject_degenerate_loop_closures = LaunchConfiguration(
        "reject_degenerate_loop_closures"
    )

    custom_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_slam_launch),
        launch_arguments={
            "use_sim_time": "true",
            "reject_degenerate_loop_closures":
                reject_degenerate_loop_closures,
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
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
                "use_rviz",
                default_value="true",
                description="Start the custom SLAM development RViz view.",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
                description="Use Mesa D3D12 rendering in WSL.",
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter",
                default_value="NVIDIA",
                description="GPU adapter selected by Mesa D3D12 in WSL.",
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
                "reject_degenerate_loop_closures",
                default_value="true",
                description="Reject translation-degenerate loop matches.",
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
            # Let Gazebo spawn the robot and begin publishing TF first.
            # RViz then fills its TF cache before filtered clouds are emitted.
            TimerAction(period=2.0, actions=[rviz]),
            TimerAction(
                period=2.5,
                actions=[custom_slam],
            ),
        ]
    )
