"""Self-developed 3D SLAM with Nav2 online navigation."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        release = Path("/proc/sys/kernel/osrelease").read_text().lower()
        return "microsoft" in release
    except OSError:
        return False


def package_launch(package, filename):
    return PathJoinSubstitution(
        [FindPackageShare(package), "launch", filename]
    )


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    default_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "config",
            "custom_3d_slam.yaml",
        ]
    )
    default_rviz = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "rviz",
            "rtabmap_navigation_3d.rviz",
        ]
    )
    default_nav2_parameters = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"]
    )
    default_snapshot = str(
        Path.home() / ".ros" / "custom_slam_3d.snapshot"
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch("slam_robot_gazebo", "simulation.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": "false",
            "sensor_variant": "3d",
            "odometry_mode": "wheel_imu",
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
            "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
        }.items(),
    )
    front_end = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_slam_3d", "custom_3d_front_end.launch.py"
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("slam_params_file"),
            "use_sim_time": "true",
            "mode": LaunchConfiguration("mode"),
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "load_snapshot": LaunchConfiguration("load_snapshot"),
            "save_on_shutdown": LaunchConfiguration("save_on_shutdown"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_navigation", "online_slam_navigation.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": LaunchConfiguration("nav2_params_file"),
            "use_rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "map_topic": "/map",
            "lidar_topic": "/lidar_3d/points",
            "autostart": LaunchConfiguration("nav2_autostart"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            # Exposed for the fault-injection regression only. Held false, the
            # lifecycle manager waits for an explicit manage_nodes call, which
            # is the one way to park Nav2 in the configured-but-not-active
            # state where its action servers exist and reject every goal. That
            # window is a real race during normal autostart; this is how it is
            # held still long enough to assert on.
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument(
                "slam_params_file", default_value=default_parameters
            ),
            DeclareLaunchArgument(
                "nav2_params_file", default_value=default_nav2_parameters
            ),
            DeclareLaunchArgument("mode", default_value="mapping"),
            DeclareLaunchArgument(
                "snapshot_path", default_value=default_snapshot
            ),
            DeclareLaunchArgument("load_snapshot", default_value="false"),
            DeclareLaunchArgument("save_on_shutdown", default_value="true"),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter", default_value="NVIDIA"
            ),
            GroupAction(scoped=True, actions=[simulation]),
            GroupAction(scoped=True, actions=[front_end]),
            navigation,
        ]
    )
