from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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
    recorder_launch = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "launch",
            "record_rgbd_slam_data.launch.py",
        ]
    )
    structured_world = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "worlds",
            "structured_loop_3d.sdf",
        ]
    )
    default_dds_profile = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "fastdds_rgbd.xml"]
    )
    smoke = LaunchConfiguration("smoke")
    driver = Node(
        package="slam_robot_slam_3d",
        executable="structured_loop_regression",
        name="structured_rgbd_route_driver",
        output="screen",
        arguments=["--drive-only", "--laps", LaunchConfiguration("laps")],
        condition=UnlessCondition(smoke),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value=str(Path.cwd() / "bags" / "structured_rgbd_reference"),
            ),
            DeclareLaunchArgument("laps", default_value="1"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("smoke", default_value="false"),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_dds_profile,
                ),
            ),
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
                            "camera_variant": "rgbd",
                            "odometry_mode": "wheel_imu",
                            "rgbd_update_rate": "10.0",
                            "rgbd_pointcloud": "false",
                            "rgbd_dds_profiles_file": LaunchConfiguration(
                                "rgbd_dds_profiles_file"
                            ),
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
                        PythonLaunchDescriptionSource(recorder_launch),
                        launch_arguments={
                            "output": LaunchConfiguration("output"),
                            "route_laps": LaunchConfiguration("laps"),
                            "rgbd_dds_profiles_file": LaunchConfiguration(
                                "rgbd_dds_profiles_file"
                            ),
                        }.items(),
                    )
                ],
            ),
            driver,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=driver,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="structured RGB-D route recording finished"
                            )
                        )
                    ],
                )
            ),
        ]
    )
