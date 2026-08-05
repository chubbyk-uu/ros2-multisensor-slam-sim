from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "launch", "simulation.launch.py"]
    )
    front_end_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "custom_3d_front_end.launch.py",
        ]
    )
    structured_world = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "worlds",
            "structured_loop_3d.sdf",
        ]
    )
    left_wheel_friction = PythonExpression(
        ["'0.15' if '", profile, "' == 'slip' else '1.2'"]
    )
    regression = Node(
        package="slam_robot_slam_3d",
        executable="front_end_motion_regression",
        name="front_end_motion_regression",
        output="screen",
        arguments=["--profile", profile],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="rotation",
                choices=["rotation", "slip"],
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
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
                            "world": structured_world,
                            "gui": LaunchConfiguration("gui"),
                            "rviz": LaunchConfiguration("rviz"),
                            "sensor_variant": "3d",
                            "odometry_mode": "wheel_imu",
                            "left_wheel_friction": left_wheel_friction,
                            "right_wheel_friction": "1.2",
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
                        PythonLaunchDescriptionSource(front_end_launch)
                    )
                ],
            ),
            TimerAction(period=5.0, actions=[regression]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=regression,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="custom 3D motion regression finished"
                            )
                        )
                    ],
                )
            ),
        ]
    )
