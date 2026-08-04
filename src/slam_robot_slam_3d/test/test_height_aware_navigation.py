import ast
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
NAVIGATION_LAUNCH = (
    WORKSPACE_SRC
    / "slam_robot_navigation"
    / "launch"
    / "online_slam_navigation.launch.py"
)
ROBOT_XACRO = (
    WORKSPACE_SRC
    / "slam_robot_description"
    / "urdf"
    / "slam_robot.urdf.xacro"
)
RTABMAP_CONFIG = Path(__file__).parents[1] / "config" / "rtabmap_3d.yaml"
STRUCTURED_WORLD = (
    WORKSPACE_SRC
    / "slam_robot_gazebo"
    / "worlds"
    / "structured_loop_3d.sdf"
)


def module_constant(path, name):
    syntax = ast.parse(path.read_text())
    for statement in syntax.body:
        if isinstance(statement, ast.Assign):
            if any(
                isinstance(target, ast.Name) and target.id == name
                for target in statement.targets
            ):
                return ast.literal_eval(statement.value)
    raise AssertionError(f"missing constant {name} in {path}")


def xacro_number(name, kind="property"):
    source = ROBOT_XACRO.read_text()
    match = re.search(
        rf'<xacro:{kind} name="{name}" (?:value|default)="([0-9.\-]+)"',
        source,
    )
    assert match is not None, f"missing {kind} {name}"
    return float(match.group(1))


def collision(root, name):
    element = root.find(f".//collision[@name='{name}']")
    assert element is not None, f"missing collision {name}"
    return element


def pose_xyz(element):
    return tuple(float(value) for value in element.findtext("pose").split()[:3])


def box_size(element):
    return tuple(
        float(value)
        for value in element.findtext("geometry/box/size").split()
    )


def cylinder_size(element):
    return (
        float(element.findtext("geometry/cylinder/radius")),
        float(element.findtext("geometry/cylinder/length")),
    )


def test_navigation_height_band_covers_complete_robot_with_margin():
    wheel_radius = xacro_number("wheel_radius")
    base_height = xacro_number("base_height")
    lidar_center = xacro_number("lidar_3d_z", kind="arg")
    lidar_height = xacro_number("lidar_3d_height")
    robot_height = (
        wheel_radius
        + base_height / 2.0
        + lidar_center
        + lidar_height / 2.0
    )
    navigation_ceiling = float(
        module_constant(NAVIGATION_LAUNCH, "MAX_OBSTACLE_HEIGHT")
    )
    parameters = yaml.safe_load(RTABMAP_CONFIG.read_text())[
        "/**"
    ]["ros__parameters"]

    assert math.isclose(robot_height, 0.35)
    assert math.isclose(navigation_ceiling - robot_height, 0.10)
    assert math.isclose(
        float(parameters["Grid/MaxObstacleHeight"]),
        navigation_ceiling,
    )

    voxel_origin = float(module_constant(NAVIGATION_LAUNCH, "VOXEL_ORIGIN_Z"))
    voxel_resolution = float(
        module_constant(NAVIGATION_LAUNCH, "VOXEL_RESOLUTION_Z")
    )
    voxel_count = int(module_constant(NAVIGATION_LAUNCH, "VOXEL_COUNT_Z"))
    assert voxel_count <= 16
    assert math.isclose(
        voxel_origin + voxel_resolution * voxel_count,
        navigation_ceiling,
    )


def test_structured_world_exercises_both_height_semantics():
    root = ET.parse(STRUCTURED_WORLD).getroot()
    navigation_ceiling = float(
        module_constant(NAVIGATION_LAUNCH, "MAX_OBSTACLE_HEIGHT")
    )

    header = collision(root, "east_clearance_portal_header_collision")
    header_z = pose_xyz(header)[2]
    header_height = box_size(header)[2]
    portal_clearance = header_z - header_height / 2.0
    assert math.isclose(portal_clearance, 0.55)
    assert portal_clearance >= navigation_ceiling + 0.10

    stem = collision(root, "north_mushroom_stem_collision")
    cap = collision(root, "north_mushroom_cap_collision")
    stem_radius, _ = cylinder_size(stem)
    cap_radius, cap_height = cylinder_size(cap)
    cap_x, cap_y, cap_z = pose_xyz(cap)
    assert cap_radius >= 5.0 * stem_radius
    assert cap_z - cap_height / 2.0 < 0.35
    assert cap_z + cap_height / 2.0 <= navigation_ceiling

    # The fixed mapping route at y=12 remains clear, while the narrow strip
    # between the cap and the outer wall cannot fit the 0.376 m-wide robot.
    robot_half_width = 0.188
    assert abs(cap_y - 12.0) - cap_radius > robot_half_width
    outer_wall_inner_face = 13.4
    assert outer_wall_inner_face - (cap_y + cap_radius) < robot_half_width
    assert math.isclose(cap_x, 8.5)


def test_structured_world_has_interactive_camera_controls():
    root = ET.parse(STRUCTURED_WORLD).getroot()
    plugins = {
        plugin.attrib.get("filename")
        for plugin in root.findall(".//gui/plugin")
    }

    assert "MinimalScene" in plugins
    assert "GzSceneManager" in plugins
    assert "InteractiveViewControl" in plugins
    assert "CameraTracking" in plugins
