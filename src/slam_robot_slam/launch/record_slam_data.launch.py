from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


RECORDED_TOPICS = [
    "/clock",
    "/scan",
    "/odom",
    "/ground_truth/odom",
    "/tf",
    "/tf_static",
    "/robot_description",
]


def start_recorder(context):
    output = Path(
        context.perform_substitution(LaunchConfiguration("output"))
    ).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    return [
        LogInfo(msg=f"Recording SLAM dataset to: {output}"),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--storage",
                "mcap",
                "--use-sim-time",
                "--disable-keyboard-controls",
                "--output",
                str(output),
                "--topics",
                *RECORDED_TOPICS,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output = Path.cwd() / "bags" / f"slam_data_{timestamp}"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value=str(default_output),
                description="Output directory for the MCAP rosbag.",
            ),
            OpaqueFunction(function=start_recorder),
        ]
    )
