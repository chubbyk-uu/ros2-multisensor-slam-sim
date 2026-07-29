from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def running_in_wsl():
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()
    except OSError:
        return False


def generate_launch_description():
    model_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_description"), "urdf", "slam_robot.urdf.xacro"]
    )
    default_world_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "worlds", "slam_world.sdf"]
    )
    bridge_config_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "bridge.yaml"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "rviz", "simulation.rviz"]
    )
    gz_sim_launch_path = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_wsl_gpu = LaunchConfiguration("use_wsl_gpu")
    wsl_gpu_adapter = LaunchConfiguration("wsl_gpu_adapter")
    robot_description = ParameterValue(
        Command(["xacro ", model_path]),
        value_type=str,
    )

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        condition=IfCondition(gui),
        launch_arguments={
            "gz_args": [world, " -r -v 3"],
        }.items(),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        condition=UnlessCondition(gui),
        launch_arguments={
            "gz_args": [world, " -s -r -v 3"],
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world_path,
                description="Absolute path to the Gazebo Sim world file.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the robot and laser scan displays.",
            ),
            DeclareLaunchArgument(
                "use_wsl_gpu",
                default_value="true" if running_in_wsl() else "false",
                description=(
                    "Use Mesa's D3D12 renderer instead of llvmpipe in WSL."
                ),
            ),
            DeclareLaunchArgument(
                "wsl_gpu_adapter",
                default_value="NVIDIA",
                description="GPU adapter name selected by Mesa D3D12 in WSL.",
            ),
            SetEnvironmentVariable(
                "GALLIUM_DRIVER",
                "d3d12",
                condition=IfCondition(use_wsl_gpu),
            ),
            SetEnvironmentVariable(
                "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                wsl_gpu_adapter,
                condition=IfCondition(use_wsl_gpu),
            ),
            gazebo_with_gui,
            gazebo_headless,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_slam_robot",
                output="screen",
                parameters=[
                    {
                        "name": "slam_robot",
                        "topic": "/robot_description",
                        "z": 0.03,
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                parameters=[{"config_file": bridge_config_path}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(rviz),
                arguments=["-d", rviz_config_path],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
