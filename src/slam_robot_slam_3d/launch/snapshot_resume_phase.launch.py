"""
Run one phase of the snapshot save/restore regression on the fixed dataset.

The regression needs the front end to actually die and come back, which one
launch cannot express, so it runs this twice: `phase:=record` maps the first
part of the bag and saves; `phase:=resume` restarts from that snapshot in
mapping mode and plays the rest; `phase:=localize` restarts from it in
localization mode instead. Use `snapshot_resume_regression` or
`snapshot_localization_regression` rather than driving the phases by hand;
they are what compare the reports.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def phase_actions(context):
    phase = context.perform_substitution(LaunchConfiguration("phase"))
    if phase not in ("record", "resume", "localize"):
        raise RuntimeError(
            f"phase must be record, resume or localize, not {phase!r}")
    split = float(context.perform_substitution(LaunchConfiguration("split")))
    window = float(context.perform_substitution(LaunchConfiguration("window")))
    bag = context.perform_substitution(LaunchConfiguration("bag"))
    rate = context.perform_substitution(LaunchConfiguration("rate"))
    report = context.perform_substitution(LaunchConfiguration("report"))
    previous_report = context.perform_substitution(
        LaunchConfiguration("previous_report")
    )

    playback = [
        "ros2", "bag", "play", bag, "--clock", "100", "--rate", rate,
        "--disable-keyboard-controls",
    ]
    if phase == "record":
        playback += ["--playback-duration", str(split)]
    else:
        # The offset is what makes the two phases join: playback resumes at the
        # sim time the recorded phase stopped at, so odometry and ground truth
        # continue from where the saved session left them rather than restarting
        # at the origin.
        playback += ["--start-offset", str(split)]
        if window > 0.0:
            playback += ["--playback-duration", str(window)]

    front_end = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_robot_slam_3d"),
                    "launch",
                    "custom_3d_front_end.launch.py",
                ]
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "true",
            # localize is the whole point of the third phase: the same
            # snapshot, restored down the read-only branch instead of the one
            # that keeps mapping.
            "mode": "localization" if phase == "localize" else "mapping",
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "load_snapshot": "false" if phase == "record" else "true",
            # Saving is requested through the service the exploration stack
            # calls on completion, so the regression covers that path and does
            # not depend on how much time a shutdown happens to allow.
            "save_on_shutdown": "false",
        }.items(),
    )
    probe = Node(
        package="slam_robot_slam_3d",
        executable="snapshot_resume_probe",
        name="snapshot_resume_probe",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "phase": phase,
                "report_path": report,
                "previous_report_path": previous_report,
                "timeout_sec": LaunchConfiguration("timeout_sec"),
            }
        ],
    )
    return [
        GroupAction(scoped=True, actions=[front_end]),
        # The dataset records /odom but not /tf, so odom -> base_footprint has
        # to be republished or the map -> base_footprint lookups this regression
        # is built on can never resolve.
        Node(
            package="slam_robot_slam_3d",
            executable="recorded_odom_tf_publisher",
            name="recorded_odom_tf_publisher",
            output="screen",
        ),
        # The dataset carries one /tf_static message, at the head of the bag,
        # which a resumed phase seeks straight past. Held for the life of the
        # run, and started in both phases: running it only where it is needed
        # would leave that phase as the only one whose startup order was never
        # exercised.
        Node(
            package="slam_robot_slam_3d",
            executable="recorded_static_tf_publisher",
            name="recorded_static_tf_publisher",
            output="screen",
            parameters=[{"bag": bag}],
        ),
        TimerAction(period=2.0, actions=[probe]),
        TimerAction(
            period=6.0, actions=[ExecuteProcess(cmd=playback, output="screen")]
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=probe,
                on_exit=[
                    EmitEvent(event=Shutdown(reason=f"{phase} phase finished"))
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "phase", description="record, resume or localize."),
            DeclareLaunchArgument("bag"),
            DeclareLaunchArgument("report"),
            DeclareLaunchArgument("snapshot_path"),
            DeclareLaunchArgument("previous_report", default_value=""),
            DeclareLaunchArgument("split", default_value="240.0"),
            DeclareLaunchArgument("window", default_value="0.0"),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("timeout_sec", default_value="1200.0"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("slam_robot_slam_3d"),
                        "config",
                        "custom_3d_slam.yaml",
                    ]
                ),
            ),
            # Kept for a uniform headless interface. This fixed-bag regression
            # starts no simulator or RViz process.
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=phase_actions),
        ]
    )
