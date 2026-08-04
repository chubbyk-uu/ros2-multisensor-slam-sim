from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


# Nav2 uses `footprint` and ignores `robot_radius` whenever the polygon is not
# empty, so the 0.22 m radius left in the official parameters is inert.
ROBOT_FOOTPRINT = (
    "[[0.155, 0.160], [0.075, 0.188], [-0.075, 0.188], "
    "[-0.295, 0.160], [-0.295, -0.160], [-0.075, -0.188], "
    "[0.075, -0.188], [0.155, -0.160]]"
)

# Single source of truth for the navigation height band. RTAB-Map projects the
# same band into /rtabmap/map via Grid/MaxGroundHeight and
# Grid/MaxObstacleHeight, so these must be kept in step with rtabmap_3d.yaml.
MIN_OBSTACLE_HEIGHT = "0.05"
MAX_OBSTACLE_HEIGHT = "1.00"

# The local costmap uses a voxel layer, whose vertical extent is
# origin_z + z_voxels * z_resolution. Nav2's defaults span only 0.80 m, so
# points between 0.80 m and MAX_OBSTACLE_HEIGHT would pass the observation
# filter and then be dropped by VoxelLayer::worldToMap3D without any warning.
# 20 voxels x 0.05 m makes the voxel ceiling match the configured band.
VOXEL_ORIGIN_Z = "0.0"
VOXEL_RESOLUTION_Z = "0.05"
VOXEL_COUNT_Z = "20"


def generate_launch_description():
    navigation_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )
    official_params = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"]
    )
    official_rviz = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    params_file = LaunchConfiguration("params_file")
    map_topic = LaunchConfiguration("map_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")

    # Keep Nav2's Jazzy defaults and only adapt frame, footprint, and standard
    # map / PointCloud2 interfaces for online RTAB-Map SLAM. Map Server and
    # AMCL are deliberately absent: RTAB-Map already owns map -> odom.
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "bt_navigator.ros__parameters.robot_base_frame": "base_footprint",
            "behavior_server.ros__parameters.robot_base_frame": "base_footprint",
            "docking_server.ros__parameters.base_frame": "base_footprint",
            "local_costmap.local_costmap.ros__parameters.robot_base_frame":
                "base_footprint",
            "local_costmap.local_costmap.ros__parameters.footprint":
                ROBOT_FOOTPRINT,
            "global_costmap.global_costmap.ros__parameters.robot_base_frame":
                "base_footprint",
            "global_costmap.global_costmap.ros__parameters.footprint":
                ROBOT_FOOTPRINT,
            "global_costmap.global_costmap.ros__parameters.static_layer.map_topic":
                map_topic,
            "global_costmap.global_costmap.ros__parameters."
            "static_layer.subscribe_to_updates": "false",
            "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic":
                lidar_topic,
            "local_costmap.local_costmap.ros__parameters."
            "voxel_layer.scan.data_type": "PointCloud2",
            "local_costmap.local_costmap.ros__parameters."
            "voxel_layer.scan.min_obstacle_height": MIN_OBSTACLE_HEIGHT,
            "local_costmap.local_costmap.ros__parameters."
            "voxel_layer.scan.max_obstacle_height": MAX_OBSTACLE_HEIGHT,
            "local_costmap.local_costmap.ros__parameters."
            "voxel_layer.origin_z": VOXEL_ORIGIN_Z,
            "local_costmap.local_costmap.ros__parameters."
            "voxel_layer.z_resolution": VOXEL_RESOLUTION_Z,
            "local_costmap.local_costmap.ros__parameters."
            "voxel_layer.z_voxels": VOXEL_COUNT_Z,
            # Layer-level ceiling also bounds raytrace clearing, so returns
            # above the navigation band neither mark nor clear.
            "local_costmap.local_costmap.ros__parameters."
            "voxel_layer.max_obstacle_height": MAX_OBSTACLE_HEIGHT,
            "global_costmap.global_costmap.ros__parameters."
            "obstacle_layer.scan.topic": lidar_topic,
            "global_costmap.global_costmap.ros__parameters."
            "obstacle_layer.scan.data_type": "PointCloud2",
            "global_costmap.global_costmap.ros__parameters."
            "obstacle_layer.scan.min_obstacle_height": MIN_OBSTACLE_HEIGHT,
            "global_costmap.global_costmap.ros__parameters."
            "obstacle_layer.scan.max_obstacle_height": MAX_OBSTACLE_HEIGHT,
            "collision_monitor.ros__parameters.scan.type": "pointcloud",
            "collision_monitor.ros__parameters.scan.topic": lidar_topic,
            "collision_monitor.ros__parameters.scan.min_height":
                MIN_OBSTACLE_HEIGHT,
            "collision_monitor.ros__parameters.scan.max_height":
                MAX_OBSTACLE_HEIGHT,
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("params_file", default_value=official_params),
            DeclareLaunchArgument("map_topic", default_value="/rtabmap/map"),
            DeclareLaunchArgument("lidar_topic", default_value="/lidar_3d/points"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "namespace": "",
                    "use_sim_time": use_sim_time,
                    "params_file": configured_params,
                    "autostart": autostart,
                    "use_composition": use_composition,
                    "use_respawn": "false",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rtabmap_navigation_rviz",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", official_rviz],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("/map", map_topic)],
            ),
        ]
    )
