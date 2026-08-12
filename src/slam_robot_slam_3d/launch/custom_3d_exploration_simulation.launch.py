"""Self-developed 3D SLAM, Nav2, and autonomous frontier exploration."""

import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename):
    return PathJoinSubstitution(
        [FindPackageShare(package), "launch", filename]
    )


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def boolean_value(text):
    """Parse a launch boolean without accepting a misspelling as false."""
    lowered = text.strip().lower()
    if lowered in ("true", "1", "yes", "on"):
        return True
    if lowered in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"invalid boolean launch value: {text!r}")


def resolve_spawn(context):
    """Return an explicit spawn pose, sampling it before Gazebo starts."""
    if not boolean_value(context.perform_substitution(
            LaunchConfiguration("random_spawn"))):
        return {
            "x": context.perform_substitution(LaunchConfiguration("spawn_x")),
            "y": context.perform_substitution(LaunchConfiguration("spawn_y")),
            "yaw": context.perform_substitution(LaunchConfiguration("spawn_yaw")),
            "record": None,
        }
    executable = (
        Path(get_package_prefix("slam_robot_gazebo"))
        / "lib" / "slam_robot_gazebo" / "safe_spawn_sampler"
    )
    command = [
        str(executable),
        "--world",
        context.perform_substitution(LaunchConfiguration("world")),
        "--count",
        "1",
        "--seed",
        context.perform_substitution(LaunchConfiguration("spawn_seed")),
        "--safety-margin",
        context.perform_substitution(LaunchConfiguration("spawn_safety_margin")),
        "--robot-height",
        context.perform_substitution(LaunchConfiguration("spawn_robot_height")),
        "--vertical-margin",
        context.perform_substitution(LaunchConfiguration("spawn_vertical_margin")),
        "--minimum-separation",
        context.perform_substitution(LaunchConfiguration("spawn_minimum_separation")),
    ]
    completed = subprocess.run(
        command, check=True, capture_output=True, text=True, timeout=30.0
    )
    document = json.loads(completed.stdout)
    pose = document["poses"][0]
    return {
        "x": str(pose["x"]),
        "y": str(pose["y"]),
        "yaw": str(pose["yaw"]),
        "record": document,
    }


def launch_exploration(context):
    """Resolve one spawn pose and hand the same value to every consumer."""
    spawn = resolve_spawn(context)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_slam_3d",
                "custom_3d_navigation_simulation.launch.py",
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "mode": "mapping",
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "load_snapshot": LaunchConfiguration("load_snapshot"),
            "save_on_shutdown": "true",
            "spawn_x": spawn["x"],
            "spawn_y": spawn["y"],
            "spawn_yaw": spawn["yaw"],
            "nav2_autostart": LaunchConfiguration("nav2_autostart"),
            "use_wsl_gpu": LaunchConfiguration("use_wsl_gpu"),
            "wsl_gpu_adapter": LaunchConfiguration("wsl_gpu_adapter"),
        }.items(),
    )
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch(
                "slam_robot_navigation", "frontier_exploration.launch.py"
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("exploration_params_file"),
            "use_sim_time": "true",
            "selection_random_seed": LaunchConfiguration("exploration_seed"),
        }.items(),
    )
    # The whole sampler record, not just the coordinates: without the envelope,
    # the separation and the reference point, a log cannot tell a rerun from a
    # different sampler. It locates the source, it does not prove the binary --
    # the sampler runs from install/ and may predate the checkout.
    if spawn["record"] is None:
        message = (
            "SAFE SPAWN RECORD explicit world="
            + context.perform_substitution(LaunchConfiguration("world"))
            + f" x={spawn['x']} y={spawn['y']} yaw={spawn['yaw']}"
        )
    else:
        message = "SAFE SPAWN RECORD " + json.dumps(
            spawn["record"], sort_keys=True)
    return [
        LogInfo(msg=message),
        GroupAction(scoped=True, actions=[navigation]),
        GroupAction(scoped=True, actions=[exploration]),
    ]


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    default_slam_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam_3d"),
            "config",
            "custom_3d_slam.yaml",
        ]
    )
    default_exploration_parameters = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_navigation"),
            "config",
            "frontier_exploration.yaml",
        ]
    )
    default_snapshot = str(
        Path.home() / ".ros" / "custom_slam_3d.snapshot"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("random_spawn", default_value="false"),
            DeclareLaunchArgument("spawn_seed", default_value="0"),
            DeclareLaunchArgument("spawn_safety_margin", default_value="0.15"),
            # 0.35 m is the top of the 3D LiDAR above base_footprint; the
            # extra 0.10 m keeps random spawns clear of low overhead geometry.
            DeclareLaunchArgument("spawn_robot_height", default_value="0.35"),
            DeclareLaunchArgument("spawn_vertical_margin", default_value="0.10"),
            # Only meaningful for a batch drawing several poses at once; a
            # single interactive spawn has nothing to be separated from.
            DeclareLaunchArgument("spawn_minimum_separation", default_value="2.0"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "slam_params_file", default_value=default_slam_parameters
            ),
            DeclareLaunchArgument(
                "exploration_params_file",
                default_value=default_exploration_parameters,
            ),
            DeclareLaunchArgument(
                "snapshot_path", default_value=default_snapshot
            ),
            DeclareLaunchArgument("load_snapshot", default_value="false"),
            # See custom_3d_navigation_simulation.launch.py: fault injection
            # only, so the Nav2 startup race can be held still and asserted on.
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            DeclareLaunchArgument("exploration_seed", default_value="0"),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter", default_value="NVIDIA"
            ),
            OpaqueFunction(function=launch_exploration),
        ]
    )
