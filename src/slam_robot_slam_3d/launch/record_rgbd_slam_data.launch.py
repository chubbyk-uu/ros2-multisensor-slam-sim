from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


RECORDED_TOPICS = [
    "/clock",
    "/lidar_3d/points",
    "/camera/color/image_raw",
    "/camera/depth/image_raw",
    "/camera/color/camera_info",
    "/camera/depth/camera_info",
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
            f"RGB-D dataset already exists: {output}. "
            "Move it aside or choose a different output path."
        )
    route_laps = context.perform_substitution(LaunchConfiguration("route_laps"))
    if not route_laps.isdigit() or int(route_laps) < 1:
        raise RuntimeError("route_laps must be a positive integer")

    return [
        LogInfo(msg=f"Recording fixed RGB-D inputs to: {output}"),
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
                "dataset=structured_rgbd_reference",
                "contract_version=1",
                "camera_rate_hz=10.0",
                "storage_profile=zstd_fast",
                f"route_laps={route_laps}",
                "--topics",
                *RECORDED_TOPICS,
            ],
            output="screen",
            additional_env={
                "FASTRTPS_DEFAULT_PROFILES_FILE": LaunchConfiguration(
                    "rgbd_dds_profiles_file"
                )
            },
            # ros2 bag record does not exit on the first launch SIGINT in this
            # composition. Escalate promptly to SIGTERM, which still performs
            # an orderly MCAP close, then retain a long no-SIGKILL flush window.
            sigterm_timeout="10",
            sigkill_timeout="60",
        ),
    ]


def generate_launch_description():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output = Path.cwd() / "bags" / f"slam_rgbd_data_{timestamp}"
    default_dds_profile = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "fastdds_rgbd.xml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value=str(default_output),
                description="Output directory for the fixed RGB-D MCAP dataset.",
            ),
            DeclareLaunchArgument(
                "rgbd_dds_profiles_file",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE",
                    default_value=default_dds_profile,
                ),
                description="Fast DDS profile used by the large-message recorder.",
            ),
            DeclareLaunchArgument(
                "route_laps",
                default_value="1",
                description="Number of structured route laps represented by the bag.",
            ),
            OpaqueFunction(function=start_recorder),
        ]
    )
