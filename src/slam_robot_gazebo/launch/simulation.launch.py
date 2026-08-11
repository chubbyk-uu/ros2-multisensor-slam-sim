from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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
    wheel_imu_bridge_config_path = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "config",
            "bridge_wheel_imu.yaml",
        ]
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "config", "ekf_2d.yaml"]
    )
    covariance_config_path = PathJoinSubstitution(
        [
            FindPackageShare("slam_robot_gazebo"),
            "config",
            "sensor_covariance.yaml",
        ]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_gazebo"), "rviz", "simulation.rviz"]
    )
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_wsl_gpu = LaunchConfiguration("use_wsl_gpu")
    wsl_gpu_adapter = LaunchConfiguration("wsl_gpu_adapter")
    sensor_variant = LaunchConfiguration("sensor_variant")
    odometry_mode = LaunchConfiguration("odometry_mode")
    left_wheel_friction = LaunchConfiguration("left_wheel_friction")
    right_wheel_friction = LaunchConfiguration("right_wheel_friction")
    lidar_3d_x = LaunchConfiguration("lidar_3d_x")
    lidar_3d_y = LaunchConfiguration("lidar_3d_y")
    lidar_3d_z = LaunchConfiguration("lidar_3d_z")
    lidar_3d_roll = LaunchConfiguration("lidar_3d_roll")
    lidar_3d_pitch = LaunchConfiguration("lidar_3d_pitch")
    lidar_3d_yaw = LaunchConfiguration("lidar_3d_yaw")
    lidar_3d_horizontal_samples = LaunchConfiguration(
        "lidar_3d_horizontal_samples"
    )
    lidar_3d_horizontal_min_angle = LaunchConfiguration(
        "lidar_3d_horizontal_min_angle"
    )
    lidar_3d_horizontal_max_angle = LaunchConfiguration(
        "lidar_3d_horizontal_max_angle"
    )
    lidar_3d_vertical_samples = LaunchConfiguration("lidar_3d_vertical_samples")
    lidar_3d_vertical_min_angle = LaunchConfiguration(
        "lidar_3d_vertical_min_angle"
    )
    lidar_3d_vertical_max_angle = LaunchConfiguration(
        "lidar_3d_vertical_max_angle"
    )
    lidar_3d_update_rate = LaunchConfiguration("lidar_3d_update_rate")
    lidar_3d_minimum_range = LaunchConfiguration("lidar_3d_minimum_range")
    lidar_3d_maximum_range = LaunchConfiguration("lidar_3d_maximum_range")
    lidar_3d_noise_stddev = LaunchConfiguration("lidar_3d_noise_stddev")
    imu_update_rate = LaunchConfiguration("imu_update_rate")
    imu_angular_velocity_noise_stddev = LaunchConfiguration(
        "imu_angular_velocity_noise_stddev"
    )
    imu_angular_velocity_bias_stddev = LaunchConfiguration(
        "imu_angular_velocity_bias_stddev"
    )
    imu_linear_acceleration_noise_stddev = LaunchConfiguration(
        "imu_linear_acceleration_noise_stddev"
    )
    imu_linear_acceleration_bias_stddev = LaunchConfiguration(
        "imu_linear_acceleration_bias_stddev"
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                model_path,
                " sensor_variant:=",
                sensor_variant,
                " odometry_mode:=",
                odometry_mode,
                " left_wheel_friction:=",
                left_wheel_friction,
                " right_wheel_friction:=",
                right_wheel_friction,
                " lidar_3d_x:=",
                lidar_3d_x,
                " lidar_3d_y:=",
                lidar_3d_y,
                " lidar_3d_z:=",
                lidar_3d_z,
                " lidar_3d_roll:=",
                lidar_3d_roll,
                " lidar_3d_pitch:=",
                lidar_3d_pitch,
                " lidar_3d_yaw:=",
                lidar_3d_yaw,
                " lidar_3d_horizontal_samples:=",
                lidar_3d_horizontal_samples,
                " lidar_3d_horizontal_min_angle:=",
                lidar_3d_horizontal_min_angle,
                " lidar_3d_horizontal_max_angle:=",
                lidar_3d_horizontal_max_angle,
                " lidar_3d_vertical_samples:=",
                lidar_3d_vertical_samples,
                " lidar_3d_vertical_min_angle:=",
                lidar_3d_vertical_min_angle,
                " lidar_3d_vertical_max_angle:=",
                lidar_3d_vertical_max_angle,
                " lidar_3d_update_rate:=",
                lidar_3d_update_rate,
                " lidar_3d_minimum_range:=",
                lidar_3d_minimum_range,
                " lidar_3d_maximum_range:=",
                lidar_3d_maximum_range,
                " lidar_3d_noise_stddev:=",
                lidar_3d_noise_stddev,
                " imu_update_rate:=",
                imu_update_rate,
                " imu_angular_velocity_noise_stddev:=",
                imu_angular_velocity_noise_stddev,
                " imu_angular_velocity_bias_stddev:=",
                imu_angular_velocity_bias_stddev,
                " imu_linear_acceleration_noise_stddev:=",
                imu_linear_acceleration_noise_stddev,
                " imu_linear_acceleration_bias_stddev:=",
                imu_linear_acceleration_bias_stddev,
            ]
        ),
        value_type=str,
    )

    # Launch Gazebo without an intermediate shell so SIGINT reaches it directly.
    # The Jazzy ros_gz_sim wrapper uses shell=True, which can leave the parent
    # launch process waiting after its ROS nodes have already stopped.
    gazebo_with_gui = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            world,
            "-r",
            "-v",
            "3",
            "--force-version",
            "8",
        ],
        name="gazebo",
        output="screen",
        condition=IfCondition(gui),
        shell=False,
    )
    gazebo_headless = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            world,
            "-s",
            "-r",
            "-v",
            "3",
            "--force-version",
            "8",
        ],
        name="gazebo",
        output="screen",
        condition=UnlessCondition(gui),
        shell=False,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
                description="Robot spawn x in the world frame.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="0.0",
                description="Robot spawn y in the world frame.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Robot spawn yaw in radians.",
            ),
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
                "rviz_config",
                default_value=rviz_config_path,
                description="Absolute path to the RViz configuration file.",
            ),
            DeclareLaunchArgument(
                "sensor_variant",
                default_value="2d",
                description="LiDAR model variant: 2d or 3d.",
            ),
            DeclareLaunchArgument(
                "odometry_mode",
                default_value="wheel_imu",
                description="Odometry source: wheel or wheel_imu.",
            ),
            DeclareLaunchArgument(
                "left_wheel_friction",
                default_value="1.2",
                description="Left drive-wheel friction coefficient.",
            ),
            DeclareLaunchArgument(
                "right_wheel_friction",
                default_value="1.2",
                description="Right drive-wheel friction coefficient.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_x",
                default_value="-0.07",
                description="3D LiDAR sensing origin x in base_link, in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_y",
                default_value="0.0",
                description="3D LiDAR sensing origin y in base_link, in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_z",
                default_value="0.18",
                description="3D LiDAR sensing origin z in base_link, in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_roll",
                default_value="0.0",
                description="3D LiDAR mounting roll in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_pitch",
                default_value="0.0",
                description="3D LiDAR mounting pitch in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_yaw",
                default_value="0.0",
                description="3D LiDAR mounting yaw in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_horizontal_samples",
                default_value="720",
                description="Number of horizontal samples in each 3D scan.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_horizontal_min_angle",
                default_value="-3.141592653589793",
                description="Minimum horizontal scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_horizontal_max_angle",
                default_value="3.1328660073298216",
                description="Maximum horizontal scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_vertical_samples",
                default_value="16",
                description="Number of vertical scan channels.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_vertical_min_angle",
                default_value="-0.2617993877991494",
                description="Minimum vertical scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_vertical_max_angle",
                default_value="0.2617993877991494",
                description="Maximum vertical scan angle in radians.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_update_rate",
                default_value="10.0",
                description="3D LiDAR update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_minimum_range",
                default_value="0.20",
                description="3D LiDAR minimum range in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_maximum_range",
                default_value="20.0",
                description="3D LiDAR maximum range in metres.",
            ),
            DeclareLaunchArgument(
                "lidar_3d_noise_stddev",
                default_value="0.01",
                description="Gaussian range noise standard deviation in metres.",
            ),
            DeclareLaunchArgument(
                "imu_update_rate",
                default_value="100.0",
                description="Raw IMU update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "imu_angular_velocity_noise_stddev",
                default_value="0.002",
                description="Gyroscope white-noise standard deviation in rad/s.",
            ),
            DeclareLaunchArgument(
                "imu_angular_velocity_bias_stddev",
                default_value="0.0002",
                description="Gyroscope startup-bias standard deviation in rad/s.",
            ),
            DeclareLaunchArgument(
                "imu_linear_acceleration_noise_stddev",
                default_value="0.02",
                description="Accelerometer white-noise standard deviation in m/s^2.",
            ),
            DeclareLaunchArgument(
                "imu_linear_acceleration_bias_stddev",
                default_value="0.005",
                description="Accelerometer startup-bias standard deviation in m/s^2.",
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
                        # Varying where a run begins changes the whole route
                        # without ever asking the robot to take a goal it knows
                        # is worse, which is what randomising goal selection
                        # does. Defaults keep the origin every existing
                        # measurement was taken from.
                        "x": ParameterValue(
                            LaunchConfiguration("spawn_x"), value_type=float),
                        "y": ParameterValue(
                            LaunchConfiguration("spawn_y"), value_type=float),
                        "z": 0.03,
                        "Y": ParameterValue(
                            LaunchConfiguration("spawn_yaw"), value_type=float),
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                condition=UnlessCondition(
                    PythonExpression(
                        ["'", odometry_mode, "' == 'wheel_imu'"]
                    )
                ),
                arguments=[
                    (
                        "/world/slam_world/create@"
                        "ros_gz_interfaces/srv/SpawnEntity"
                    ),
                    (
                        "/world/slam_world/remove@"
                        "ros_gz_interfaces/srv/DeleteEntity"
                    ),
                ],
                parameters=[{"config_file": bridge_config_path}],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", odometry_mode, "' == 'wheel_imu'"]
                    )
                ),
                arguments=[
                    (
                        "/world/slam_world/create@"
                        "ros_gz_interfaces/srv/SpawnEntity"
                    ),
                    (
                        "/world/slam_world/remove@"
                        "ros_gz_interfaces/srv/DeleteEntity"
                    ),
                ],
                parameters=[{"config_file": wheel_imu_bridge_config_path}],
            ),
            Node(
                package="slam_robot_gazebo",
                executable="sensor_covariance_adapter_node",
                name="sensor_covariance_adapter",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", odometry_mode, "' == 'wheel_imu'"]
                    )
                ),
                parameters=[covariance_config_path],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", odometry_mode, "' == 'wheel_imu'"]
                    )
                ),
                parameters=[
                    ekf_config_path,
                    {"use_sim_time": True},
                ],
                remappings=[("odometry/filtered", "/odom")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
