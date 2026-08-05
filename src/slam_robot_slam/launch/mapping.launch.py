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
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from lifecycle_msgs.msg import Transition


def save_map_before_shutdown(context, slam_toolbox_node):
    """Save while SLAM Toolbox services are still alive."""
    signal_slam = [
        EmitEvent(
            event=SignalProcess(
                signal_number=signal.SIGINT,
                process_matcher=matches_action(slam_toolbox_node),
            )
        )
    ]
    auto_save = context.perform_substitution(
        LaunchConfiguration("auto_save_map")
    ).lower()
    if auto_save not in ("1", "true", "yes", "on"):
        return signal_slam

    try:
        output_prefix = Path(
            context.perform_substitution(
                LaunchConfiguration("map_output_prefix")
            )
        ).expanduser().resolve()
        package_prefix = Path(
            context.perform_substitution(FindPackagePrefix("slam_robot_slam"))
        )
        save_executable = (
            package_prefix / "lib" / "slam_robot_slam" / "save_slam_map"
        )

        print(
            f"\n[auto_save_map] Saving map to: {output_prefix}",
            flush=True,
        )
        result = subprocess.run(
            [str(save_executable), str(output_prefix)],
            check=False,
            env=dict(context.environment),
            timeout=45.0,
        )
    except Exception as error:  # Keep the detached SLAM process recoverable.
        print(
            f"[auto_save_map] Save failed: {error}",
            flush=True,
        )
        return signal_slam

    if result.returncode == 0:
        print("[auto_save_map] Save completed; shutting down.", flush=True)
    else:
        print(
            "[auto_save_map] Save failed; shutting down without a new map.",
            flush=True,
        )
    return signal_slam


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_slam"),
            "config",
            "mapper_params_online_async.yaml",
        ]
    )
    params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_save_map = LaunchConfiguration("auto_save_map")
    map_output_prefix = LaunchConfiguration("map_output_prefix")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    # A separate session prevents terminal Ctrl+C from reaching SLAM Toolbox
    # before the launch shutdown handler has saved the map. The handler sends
    # SIGINT explicitly after the save completes.
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        prefix="setsid",
        parameters=[
            params_file,
            {
                "use_lifecycle_manager": use_lifecycle_manager,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(
                autostart,
                NotSubstitution(use_lifecycle_manager),
            )
        ),
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(
                autostart,
                NotSubstitution(use_lifecycle_manager),
            )
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_params_file,
                description="Absolute path to the SLAM Toolbox parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Configure and activate SLAM Toolbox.",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="Let an external lifecycle manager control SLAM.",
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
            LogInfo(
                condition=IfCondition(auto_save_map),
                msg=[
                    "Map auto-save enabled. Ctrl+C will save to: ",
                    map_output_prefix,
                ],
            ),
            slam_toolbox_node,
            configure_slam,
            activate_slam,
            # Keep this handler after SLAM Toolbox so it is registered with
            # higher shutdown priority and can use the services before the
            # SLAM process receives SIGINT.
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(
                            function=save_map_before_shutdown,
                            args=[slam_toolbox_node],
                        )
                    ]
                ),
            ),
        ]
    )
