"""Contract tests for RGB-D launch defaults, bridges, and result propagation."""

from importlib.machinery import SourceFileLoader
import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest
import yaml


PACKAGE = Path(__file__).parents[1]
PROFILE_VARIABLE = "FASTRTPS_DEFAULT_PROFILES_FILE"
PROFILE = PACKAGE / "config" / "fastdds_rgbd.xml"
LARGEST_RGBD_PAYLOAD = 640 * 480 * 32


def load_launch(filename):
    path = PACKAGE / "launch" / filename
    loader = SourceFileLoader(filename.replace(".", "_"), str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def launch_arguments(module):
    return {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def default_text(argument):
    return "".join(substitution.text for substitution in argument.default_value)


def resolve(value, context):
    if isinstance(value, str):
        return value
    return "".join(part.perform(context) for part in value)


def node_environments(operator_profile=None):
    previous = os.environ.pop(PROFILE_VARIABLE, None)
    if operator_profile is not None:
        os.environ[PROFILE_VARIABLE] = operator_profile
    try:
        module = load_launch("simulation.launch.py")
        description = module.generate_launch_description()
        context = LaunchContext()
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument) and entity.default_value:
                context.launch_configurations[entity.name] = resolve(
                    entity.default_value, context
                )

        environments = {}
        for entity in description.entities:
            if not isinstance(entity, Node):
                continue
            node_name = resolve(entity._Node__node_name, context)
            process = entity._ExecuteLocal__process_description
            additional = process._Executable__additional_env or []
            if hasattr(additional, "items"):
                additional = list(additional.items())
            environments[node_name] = {
                resolve(key, context): resolve(value, context)
                for key, value in additional
            }
        return environments
    finally:
        os.environ.pop(PROFILE_VARIABLE, None)
        if previous is not None:
            os.environ[PROFILE_VARIABLE] = previous


def test_base_simulation_keeps_camera_and_dense_cloud_opt_in():
    module = load_launch("simulation.launch.py")
    arguments = launch_arguments(module)
    assert default_text(arguments["camera_variant"]) == "none"
    assert default_text(arguments["rgbd_width"]) == "640"
    assert default_text(arguments["rgbd_height"]) == "480"
    assert default_text(arguments["rgbd_update_rate"]) == "30.0"
    assert default_text(arguments["rgbd_minimum_depth"]) == "0.20"
    assert default_text(arguments["rgbd_maximum_depth"]) == "6.0"
    assert default_text(arguments["rgbd_pointcloud"]) == "false"


def test_visual_entry_enables_rgbd_but_not_dense_cloud():
    module = load_launch("rgbd_simulation.launch.py")
    arguments = launch_arguments(module)
    assert default_text(arguments["sensor_variant"]) == "3d"
    assert default_text(arguments["camera_variant"]) == "rgbd"
    assert default_text(arguments["rgbd_pointcloud"]) == "false"


def test_rgbd_profile_can_hold_several_complete_cloud_samples():
    root = ElementTree.parse(PROFILE).getroot()
    namespace = {"dds": "http://www.eprosima.com"}
    shared_memory = root.findall(
        './/dds:transport_descriptor[dds:type="SHM"]', namespace
    )
    assert shared_memory
    assert min(
        int(item.find("dds:segment_size", namespace).text)
        for item in shared_memory
    ) >= 4 * LARGEST_RGBD_PAYLOAD
    assert all(
        int(item.find("dds:maxMessageSize", namespace).text) <= 512 * 1024
        for item in shared_memory
    )
    descriptors = root.findall(".//dds:transport_descriptor", namespace)
    declared = {
        item.find("dds:transport_id", namespace).text for item in descriptors
    }
    used = {
        item.text
        for item in root.findall(
            ".//dds:participant/dds:rtps/dds:userTransports/dds:transport_id",
            namespace,
        )
    }
    transport_types = {
        item.find("dds:type", namespace).text for item in descriptors
    }
    assert used and used <= declared
    assert "SHM" in transport_types and "UDPv4" in transport_types
    assert [
        item.text
        for item in root.findall(
            ".//dds:participant/dds:rtps/dds:useBuiltinTransports", namespace
        )
    ] == ["false"]


def test_only_large_rgbd_writers_receive_the_fastdds_profile():
    environments = node_environments()
    protected = {
        name
        for name, environment in environments.items()
        if PROFILE_VARIABLE in environment
    }
    assert protected == {"rgbd_image_bridge", "rgbd_pointcloud_bridge"}
    assert all(
        environments[name][PROFILE_VARIABLE].endswith(
            "slam_robot_gazebo/config/fastdds_rgbd.xml"
        )
        for name in protected
    )


def test_operator_fastdds_profile_remains_authoritative():
    operator_profile = "/etc/dds/site.xml"
    environments = node_environments(operator_profile)
    assert all(
        environments[name][PROFILE_VARIABLE] == operator_profile
        for name in ("rgbd_image_bridge", "rgbd_pointcloud_bridge")
    )


def test_both_odometry_bridge_profiles_publish_the_camera_info_contract():
    expected = {
        "/camera/color/camera_info",
        "/camera/depth/camera_info",
    }
    for filename in ("bridge.yaml", "bridge_wheel_imu.yaml"):
        entries = yaml.safe_load((PACKAGE / "config" / filename).read_text())
        camera_entries = {
            entry["ros_topic_name"]: entry
            for entry in entries
            if entry["ros_topic_name"] in expected
        }
        assert set(camera_entries) == expected
        assert all(
            entry["gz_topic_name"] == "/camera/camera_info"
            and entry["ros_type_name"] == "sensor_msgs/msg/CameraInfo"
            and entry["direction"] == "GZ_TO_ROS"
            for entry in camera_entries.values()
        )


def test_regression_launch_propagates_a_failed_scorer():
    module = load_launch("rgbd_sensor_regression.launch.py")
    event = type("Event", (), {"returncode": 1})()
    with pytest.raises(RuntimeError, match="exited with code 1"):
        module.shutdown_after_regression(event, None)
    assert module.shutdown_after_regression(
        type("Event", (), {"returncode": 0})(), None
    )
