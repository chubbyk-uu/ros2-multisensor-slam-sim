from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


RECORDED_TOPICS = [
    "/clock",
    "/lidar_3d/points",
    "/wheel/odom",
    "/imu/data",
    "/odom",
    "/ground_truth/odom",
    "/tf_static",
    "/robot_description",
]


def start_recorder(context):
    output = Path(
        context.perform_substitution(LaunchConfiguration("output"))
    ).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise RuntimeError(
            f"3D SLAM dataset already exists: {output}. "
            "Move it aside or choose a different output path."
        )

    return [
        LogInfo(msg=f"Recording fixed 3D SLAM inputs to: {output}"),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--storage",
                "mcap",
                "--storage-preset-profile",
                "zstd_fast",
                "--use-sim-time",
                "--disable-keyboard-controls",
                "--output",
                str(output),
                "--custom-data",
                "dataset=structured_3d_reference",
                "contract_version=1",
                "--topics",
                *RECORDED_TOPICS,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output = Path.cwd() / "bags" / f"slam_3d_data_{timestamp}"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value=str(default_output),
                description="Output directory for the fixed 3D MCAP dataset.",
            ),
            OpaqueFunction(function=start_recorder),
        ]
    )
