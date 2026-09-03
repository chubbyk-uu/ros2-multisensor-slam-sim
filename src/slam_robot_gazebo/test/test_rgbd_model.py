"""Keep the optional RGB-D model and its no-interference geometry explicit."""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


XACRO_FILE = (
    Path(__file__).parents[2]
    / "slam_robot_description"
    / "urdf"
    / "slam_robot.urdf.xacro"
)


def expanded_robot(**arguments):
    command = ["xacro", str(XACRO_FILE)]
    command.extend(f"{name}:={value}" for name, value in arguments.items())
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return ET.fromstring(result.stdout)


def named_elements(root, tag):
    return {element.attrib["name"]: element for element in root.findall(tag)}


def xyz(element):
    return tuple(float(value) for value in element.attrib["xyz"].split())


def test_default_model_has_no_camera_cost_or_frames():
    root = expanded_robot(sensor_variant="3d")
    assert not any("camera" in name for name in named_elements(root, "link"))
    assert root.find(".//sensor[@name='rgbd_camera']") is None


def test_rgbd_model_has_standard_frames_and_default_contract():
    root = expanded_robot(sensor_variant="3d", camera_variant="rgbd")
    links = named_elements(root, "link")
    joints = named_elements(root, "joint")
    assert {"camera_mount_link", "camera_link", "camera_optical_frame"} <= links.keys()
    assert joints["camera_mount_joint"].find("parent").attrib["link"] == "base_link"
    assert joints["camera_joint"].find("parent").attrib["link"] == "camera_mount_link"
    assert joints["camera_optical_joint"].find("parent").attrib["link"] == "camera_link"

    sensor = root.find(".//sensor[@name='rgbd_camera']")
    assert sensor is not None
    assert sensor.attrib["type"] == "rgbd_camera"
    assert sensor.findtext("update_rate") == "30.0"
    assert sensor.findtext("topic") == "/camera"
    assert sensor.findtext("camera/image/width") == "640"
    assert sensor.findtext("camera/image/height") == "480"
    assert sensor.findtext("camera/clip/near") == "0.20"
    assert sensor.findtext("camera/clip/far") == "6.0"
    assert sensor.findtext("camera/optical_frame_id") == "camera_optical_frame"
    assert xyz(joints["camera_optical_joint"].find("origin")) == (0.0, 0.0, 0.0)
    optical_rpy = tuple(
        float(value)
        for value in joints["camera_optical_joint"].find("origin").attrib["rpy"].split()
    )
    assert math.isclose(optical_rpy[0], -math.pi / 2.0)
    assert math.isclose(optical_rpy[1], 0.0)
    assert math.isclose(optical_rpy[2], -math.pi / 2.0)


def test_camera_housing_stays_inside_footprint_and_below_lidar_rays():
    root = expanded_robot(sensor_variant="3d", camera_variant="rgbd")
    links = named_elements(root, "link")
    joints = named_elements(root, "joint")

    mount_position = xyz(joints["camera_mount_joint"].find("origin"))
    camera_offset = xyz(joints["camera_joint"].find("origin"))
    camera_position = tuple(
        mount + offset for mount, offset in zip(mount_position, camera_offset)
    )
    body_collision = links["camera_link"].find("collision")
    body_origin = xyz(body_collision.find("origin"))
    body_size = tuple(
        float(value) for value in body_collision.find("geometry/box").attrib["size"].split()
    )

    body_min_x = camera_position[0] + body_origin[0] - body_size[0] / 2.0
    body_max_x = camera_position[0] + body_origin[0] + body_size[0] / 2.0
    body_half_y = body_size[1] / 2.0
    body_top_z = camera_position[2] + body_size[2] / 2.0
    assert body_min_x >= 0.125 - 1.0e-9
    assert body_max_x <= 0.155 + 1.0e-9
    assert body_half_y <= 0.16

    lidar_x = -0.07
    lidar_z = 0.18
    lowest_ray_z = lidar_z - math.tan(math.radians(15.0)) * (
        body_max_x - lidar_x
    )
    assert lowest_ray_z - body_top_z >= 0.009
    assert body_top_z < lidar_z - 0.035


def test_camera_parameters_are_overridable_without_changing_lidar_variant():
    root = expanded_robot(
        sensor_variant="2d",
        camera_variant="rgbd",
        rgbd_width="1280",
        rgbd_height="720",
        rgbd_update_rate="15.0",
        rgbd_minimum_depth="0.30",
        rgbd_maximum_depth="4.0",
    )
    sensor = root.find(".//sensor[@name='rgbd_camera']")
    assert sensor.findtext("update_rate") == "15.0"
    assert sensor.findtext("camera/image/width") == "1280"
    assert sensor.findtext("camera/image/height") == "720"
    assert sensor.findtext("camera/clip/near") == "0.30"
    assert sensor.findtext("camera/clip/far") == "4.0"
    assert root.find(".//sensor[@name='lidar_2d']") is not None
    assert root.find(".//sensor[@name='lidar_3d']") is None
