from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mola_launch_path = PathJoinSubstitution(
        [
            FindPackageShare('mola_lidar_odometry'),
            'ros2-launchs',
            'ros2-lidar-odometry.launch.py',
        ]
    )
    mola_pipeline_path = PathJoinSubstitution(
        [
            FindPackageShare('mola_lidar_odometry'),
            'pipelines',
            'lidar3d-gicp.yaml',
        ]
    )

    lidar_topic = LaunchConfiguration('lidar_topic')
    expected_lidar_frame = LaunchConfiguration('expected_lidar_frame')
    contract_timeout = LaunchConfiguration('contract_timeout')
    use_rviz = LaunchConfiguration('rviz')
    use_mola_gui = LaunchConfiguration('mola_gui')
    enforce_planar_motion = LaunchConfiguration('enforce_planar_motion')
    use_imu_gravity = LaunchConfiguration('use_imu_gravity')
    imu_topic = PythonExpression(
        [
            '"/imu/data_raw" if "',
            use_imu_gravity,
            '".lower() == "true" else "/mola/disabled_imu"',
        ]
    )

    mola = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mola_launch_path),
        launch_arguments={
            'lidar_topic_name': lidar_topic,
            'lidar_topic_type': 'PointCloud2',
            'lidar_qos_reliability': 'best_effort',
            'lidar_qos_depth': '10',
            'use_sim_time': 'true',
            'mola_lo_pipeline': mola_pipeline_path,
            'mola_deskew_method': 'MotionCompensationMethod::None',
            'use_imu_for_lio': 'False',
            'imu_gravity_correction': use_imu_gravity,
            'imu_topic_name': imu_topic,
            'ignore_lidar_pose_from_tf': 'false',
            'mola_tf_base_link': 'base_link',
            'mola_lo_reference_frame': 'map',
            'mola_state_estimator_reference_frame': 'map',
            'mola_bridge_odometry_frame': 'odom',
            'publish_localization_following_rep105': 'True',
            'forward_ros_tf_odom_to_mola': 'False',
            'odom_topic_name': '',
            'use_state_estimator': 'False',
            'enforce_planar_motion': enforce_planar_motion,
            'start_active': 'True',
            'start_mapping_enabled': 'True',
            'use_diagnostic_aggregator': 'False',
            'use_mola_gui': use_mola_gui,
            'use_rviz': use_rviz,
        }.items(),
    )

    contract_check = Node(
        package='slam_robot_slam_3d',
        executable='pointcloud_contract_check',
        name='pointcloud_contract_check',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'topic': lidar_topic,
                'expected_frame': expected_lidar_frame,
                'minimum_points': 100,
                'require_point_time': False,
                'timeout_sec': contract_timeout,
            }
        ],
    )

    def start_mola_after_contract(event, _context):
        if event.returncode != 0:
            reason = (
                '3D point-cloud input contract failed with exit code '
                f'{event.returncode}; MOLA-LO was not started.'
            )
            return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
        return [
            LogInfo(
                msg=(
                    '3D point-cloud contract passed; starting MOLA GICP in '
                    'LiDAR-only mode with per-point deskew disabled.'
                )
            ),
            mola,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'lidar_topic',
                default_value='/lidar_3d/points',
                description='PointCloud2 topic used by MOLA-LO.',
            ),
            DeclareLaunchArgument(
                'expected_lidar_frame',
                default_value='lidar_3d_link',
                description='Required PointCloud2 frame_id.',
            ),
            DeclareLaunchArgument(
                'contract_timeout',
                default_value='30.0',
                description='Wall-clock seconds to wait for a valid point cloud.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description="Start MOLA's official RViz configuration.",  # noqa: Q000
            ),
            DeclareLaunchArgument(
                'mola_gui',
                default_value='false',
                description='Start the separate MolaViz GUI.',
            ),
            DeclareLaunchArgument(
                'enforce_planar_motion',
                default_value='false',
                description='Constrain the 3D pose estimate to planar motion.',
            ),
            DeclareLaunchArgument(
                'use_imu_gravity',
                default_value='false',
                description=(
                    'Use IMU only as an ICP gravity prior. This does not enable '
                    'per-point IMU deskew or full LIO.'
                ),
            ),
            SetEnvironmentVariable('MOLA_IGNORE_NO_POINT_STAMPS', 'true'),
            SetEnvironmentVariable('MOLA_TF_FOOTPRINT_LINK', ''),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=contract_check,
                    on_exit=start_mola_after_contract,
                )
            ),
            contract_check,
        ]
    )
