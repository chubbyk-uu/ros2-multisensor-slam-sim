from pathlib import Path
import signal
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def save_custom_map_before_shutdown(
    context,
    scan_matcher_node,
    auto_save,
    output_prefix_value,
    map_topic,
):
    """Save the latest custom occupancy grid before stopping its publisher."""
    signal_scan_matcher = [
        EmitEvent(
            event=SignalProcess(
                signal_number=signal.SIGINT,
                process_matcher=matches_action(scan_matcher_node),
            )
        )
    ]
    if not auto_save:
        return signal_scan_matcher

    output_prefix = Path(output_prefix_value).expanduser().resolve()
    package_prefix = Path(
        context.perform_substitution(FindPackagePrefix("nav2_map_server"))
    )
    save_executable = (
        package_prefix / "lib" / "nav2_map_server" / "map_saver_cli"
    )

    print(
        f"\n[custom_auto_save_map] Saving {map_topic} to: {output_prefix}",
        flush=True,
    )
    try:
        output_prefix.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            [
                str(save_executable),
                "-t",
                map_topic,
                "-f",
                str(output_prefix),
                "--occ",
                "0.65",
                "--free",
                "0.196",
            ],
            check=False,
            env=dict(context.environment),
            timeout=30.0,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        print(
            f"[custom_auto_save_map] Save failed to run: {error}",
            flush=True,
        )
        return signal_scan_matcher

    if result.returncode == 0:
        print(
            "[custom_auto_save_map] Save completed; shutting down.",
            flush=True,
        )
    else:
        print(
            "[custom_auto_save_map] Save failed; shutting down without a new map.",
            flush=True,
        )
    return signal_scan_matcher


def register_custom_map_shutdown_handler(context, scan_matcher_node):
    """Capture scoped launch values before registering the shutdown hook."""
    auto_save_value = context.perform_substitution(
        LaunchConfiguration("auto_save_map")
    ).lower()
    auto_save = auto_save_value in ("1", "true", "yes", "on")
    output_prefix = context.perform_substitution(
        LaunchConfiguration("map_output_prefix")
    )
    map_topic = context.perform_substitution(LaunchConfiguration("map_topic"))
    return [
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    OpaqueFunction(
                        function=save_custom_map_before_shutdown,
                        args=[
                            scan_matcher_node,
                            auto_save,
                            output_prefix,
                            map_topic,
                        ],
                    )
                ]
            )
        )
    ]


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam"),
            "config",
            "laser_preprocessor.yaml",
        ]
    )
    default_scan_matcher_params_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam"),
            "config",
            "scan_matcher.yaml",
        ]
    )

    params_file = LaunchConfiguration("params_file")
    scan_matcher_params_file = LaunchConfiguration("scan_matcher_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_save_map = LaunchConfiguration("auto_save_map")
    map_output_prefix = LaunchConfiguration("map_output_prefix")
    map_topic = LaunchConfiguration("map_topic")
    reject_degenerate_loop_closures = LaunchConfiguration(
        "reject_degenerate_loop_closures"
    )

    # Keep the map publisher outside the terminal's process group until the
    # shutdown handler finishes saving its transient-local map snapshot.
    scan_matcher_node = Node(
        package="slam_robot_slam",
        executable="scan_matcher_odometry_node",
        name="scan_matcher_odometry",
        output="screen",
        prefix="setsid",
        parameters=[
            scan_matcher_params_file,
            {
                "use_sim_time": use_sim_time,
                "map_topic": map_topic,
                "loop_closure.reject_degenerate_matches":
                    reject_degenerate_loop_closures,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Laser preprocessing parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "scan_matcher_params_file",
                default_value=default_scan_matcher_params_file,
                description="Correlative scan matcher parameter file.",
            ),
            DeclareLaunchArgument(
                "reject_degenerate_loop_closures",
                default_value="true",
                description="Reject translation-degenerate loop matches.",
            ),
            DeclareLaunchArgument(
                "auto_save_map",
                default_value="true",
                description="Save the custom occupancy grid before shutdown.",
            ),
            DeclareLaunchArgument(
                "map_output_prefix",
                default_value=str(Path.cwd() / "maps" / "custom_slam_map"),
                description="Auto-save path without a file extension.",
            ),
            DeclareLaunchArgument(
                "map_topic",
                default_value="/custom_slam/map",
                description="Custom occupancy grid topic to publish and save.",
            ),
            LogInfo(
                condition=IfCondition(auto_save_map),
                msg=[
                    "Custom map auto-save enabled. Ctrl+C will save to: ",
                    map_output_prefix,
                ],
            ),
            Node(
                package="slam_robot_slam",
                executable="laser_scan_preprocessor_node",
                name="laser_scan_preprocessor",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            scan_matcher_node,
            OpaqueFunction(
                function=register_custom_map_shutdown_handler,
                args=[scan_matcher_node],
            ),
        ]
    )
