from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_description"), "urdf", "slam_robot.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("slam_robot_description"), "rviz", "display.rviz"]
    )

    model = LaunchConfiguration("model")
    use_gui = LaunchConfiguration("use_gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=model_path,
                description="Absolute path to the robot Xacro file.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Use the joint state publisher GUI.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock when Gazebo is running.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=UnlessCondition(use_gui),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher",
                condition=IfCondition(use_gui),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
