from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


ROBOT_FOOTPRINT = (
    "[[0.155, 0.160], [0.075, 0.188], [-0.075, 0.188], "
    "[-0.295, 0.160], [-0.295, -0.160], [-0.075, -0.188], "
    "[0.075, -0.188], [0.155, -0.160]]"
)


def generate_launch_description():
    nav2_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )
    nav2_rviz_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "rviz_launch.py"]
    )
    official_params = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"]
    )
    official_rviz = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    # Start from the Nav2 Jazzy official configuration and only adapt values
    # that depend on this robot's frames, footprint, sensor, and spawn pose.
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "amcl.ros__parameters.laser_max_range": "12.0",
            "amcl.ros__parameters.max_beams": "120",
            "amcl.ros__parameters.set_initial_pose": "true",
            "amcl.ros__parameters.always_reset_initial_pose": "true",
            "amcl.ros__parameters.initial_pose.x": initial_pose_x,
            "amcl.ros__parameters.initial_pose.y": initial_pose_y,
            "amcl.ros__parameters.initial_pose.z": "0.0",
            "amcl.ros__parameters.initial_pose.yaw": initial_pose_yaw,
            "bt_navigator.ros__parameters.robot_base_frame":
                "base_footprint",
            "local_costmap.local_costmap.ros__parameters."
            "robot_base_frame": "base_footprint",
            "local_costmap.local_costmap.ros__parameters."
            "footprint": ROBOT_FOOTPRINT,
            "global_costmap.global_costmap.ros__parameters."
            "robot_base_frame": "base_footprint",
            "global_costmap.global_costmap.ros__parameters."
            "footprint": ROBOT_FOOTPRINT,
            "behavior_server.ros__parameters.robot_base_frame":
                "base_footprint",
            "docking_server.ros__parameters.base_frame":
                "base_footprint",
            "controller_server.ros__parameters.FollowPath.ax_max":
                "0.8",
            "controller_server.ros__parameters.FollowPath.ax_min":
                "-0.8",
            "controller_server.ros__parameters.FollowPath.az_max":
                "2.0",
            "controller_server.ros__parameters.FollowPath.wz_max":
                "1.5",
            "controller_server.ros__parameters.FollowPath."
            "CostCritic.consider_footprint": "true",
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=str(Path.cwd() / "maps" / "slam_map.yaml"),
                description="Absolute path to the saved occupancy map YAML.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=official_params,
                description="Base Nav2 parameter file; defaults to Jazzy official.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=official_rviz,
                description="RViz configuration used for navigation.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start the official Nav2 RViz view.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically activate all Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description=(
                    "Use the official composed Nav2 bringup; disabled by default "
                    "for reliable interactive shutdown."
                ),
            ),
            DeclareLaunchArgument(
                "initial_pose_x",
                default_value="0.0",
                description="Known initial robot X coordinate in the map.",
            ),
            DeclareLaunchArgument(
                "initial_pose_y",
                default_value="0.0",
                description="Known initial robot Y coordinate in the map.",
            ),
            DeclareLaunchArgument(
                "initial_pose_yaw",
                default_value="0.0",
                description="Known initial robot yaw in the map.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    "slam": "False",
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": configured_params,
                    "autostart": autostart,
                    "use_composition": use_composition,
                    "use_respawn": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_rviz_launch),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "rviz_config": rviz_config,
                }.items(),
            ),
        ]
    )
