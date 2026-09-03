from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def shutdown_after_regression(event, _context):
    """Preserve the sensor regression result across launch shutdown."""
    if event.returncode != 0:
        raise RuntimeError(
            f"RGB-D sensor regression exited with code {event.returncode}"
        )
    return [
        EmitEvent(event=Shutdown(reason="RGB-D sensor regression passed"))
    ]


def generate_launch_description():
    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "launch", "simulation.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )

    width = LaunchConfiguration("rgbd_width")
    height = LaunchConfiguration("rgbd_height")
    update_rate = LaunchConfiguration("rgbd_update_rate")
    minimum_depth = LaunchConfiguration("rgbd_minimum_depth")
    maximum_depth = LaunchConfiguration("rgbd_maximum_depth")
    minimum_rate = LaunchConfiguration("minimum_rate")
    minimum_median_rate = LaunchConfiguration("minimum_median_rate")
    maximum_p95_gap = LaunchConfiguration("maximum_p95_gap")
    minimum_paired_fraction = LaunchConfiguration("minimum_paired_fraction")
    require_pointcloud = LaunchConfiguration("require_pointcloud")

    regression = Node(
        package="slam_robot_gazebo",
        executable="rgbd_sensor_regression",
        name="rgbd_sensor_regression",
        output="screen",
        arguments=[
            "--width",
            width,
            "--height",
            height,
            "--minimum-rate",
            minimum_rate,
            "--minimum-median-rate",
            minimum_median_rate,
            "--maximum-p95-gap",
            maximum_p95_gap,
            "--minimum-paired-fraction",
            minimum_paired_fraction,
            "--minimum-depth",
            minimum_depth,
            "--maximum-depth",
            maximum_depth,
            "--wall-timeout",
            "30.0",
        ],
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(require_pointcloud),
    )
    pointcloud_regression = Node(
        package="slam_robot_gazebo",
        executable="rgbd_sensor_regression",
        name="rgbd_sensor_regression",
        output="screen",
        arguments=[
            "--width",
            width,
            "--height",
            height,
            "--minimum-rate",
            minimum_rate,
            "--minimum-median-rate",
            minimum_median_rate,
            "--maximum-p95-gap",
            maximum_p95_gap,
            "--minimum-paired-fraction",
            minimum_paired_fraction,
            "--minimum-depth",
            minimum_depth,
            "--maximum-depth",
            maximum_depth,
            "--wall-timeout",
            "30.0",
            "--require-pointcloud",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(require_pointcloud),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rgbd_width", default_value="640"),
            DeclareLaunchArgument("rgbd_height", default_value="480"),
            DeclareLaunchArgument("rgbd_update_rate", default_value="30.0"),
            DeclareLaunchArgument("rgbd_minimum_depth", default_value="0.20"),
            DeclareLaunchArgument("rgbd_maximum_depth", default_value="6.0"),
            DeclareLaunchArgument("minimum_rate", default_value="24.0"),
            DeclareLaunchArgument("minimum_median_rate", default_value="27.0"),
            DeclareLaunchArgument("maximum_p95_gap", default_value="0.07"),
            DeclareLaunchArgument("minimum_paired_fraction", default_value="0.95"),
            DeclareLaunchArgument("require_pointcloud", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("world_name", default_value="slam_world"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "world_name": LaunchConfiguration("world_name"),
                    "gui": "false",
                    "rviz": "false",
                    "sensor_variant": "3d",
                    "camera_variant": "rgbd",
                    "odometry_mode": "wheel_imu",
                    "rgbd_width": width,
                    "rgbd_height": height,
                    "rgbd_update_rate": update_rate,
                    "rgbd_minimum_depth": minimum_depth,
                    "rgbd_maximum_depth": maximum_depth,
                    "rgbd_pointcloud": require_pointcloud,
                }.items(),
            ),
            TimerAction(
                period=5.0,
                actions=[regression, pointcloud_regression],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=regression,
                    on_exit=shutdown_after_regression,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=pointcloud_regression,
                    on_exit=shutdown_after_regression,
                )
            ),
        ]
    )
