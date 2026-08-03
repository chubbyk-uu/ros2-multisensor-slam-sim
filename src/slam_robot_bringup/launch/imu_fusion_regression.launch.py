from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    custom_slam_development = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_bringup"),
            "launch",
            "custom_slam_development.launch.py",
        ]
    )

    odometry_mode = LaunchConfiguration("odometry_mode")
    profile = LaunchConfiguration("profile")
    left_wheel_friction = LaunchConfiguration("left_wheel_friction")
    right_wheel_friction = LaunchConfiguration("right_wheel_friction")

    regression = Node(
        package="slam_robot_gazebo",
        executable="imu_fusion_regression",
        name="imu_fusion_regression",
        output="screen",
        arguments=[
            "--odometry-mode",
            odometry_mode,
            "--profile",
            profile,
            "--require-slam",
        ],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odometry_mode",
                default_value="wheel_imu",
                description="Odometry source: wheel or wheel_imu.",
            ),
            DeclareLaunchArgument(
                "profile",
                default_value="normal",
                description="Result label: normal or slip.",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(custom_slam_development),
                launch_arguments={
                    "gui": "false",
                    "use_rviz": "false",
                    "odometry_mode": odometry_mode,
                    "left_wheel_friction": left_wheel_friction,
                    "right_wheel_friction": right_wheel_friction,
                }.items(),
            ),
            TimerAction(period=5.0, actions=[regression]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=regression,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="IMU fusion regression completed"
                            )
                        )
                    ],
                )
            ),
        ]
    )
