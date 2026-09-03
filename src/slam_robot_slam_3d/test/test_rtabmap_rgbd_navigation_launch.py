from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, GroupAction


LAUNCH_PATH = (
    Path(__file__).parents[1]
    / "launch"
    / "rtabmap_rgbd_navigation_simulation.launch.py"
)


def load_launch():
    loader = SourceFileLoader("rtabmap_rgbd_navigation_launch", str(LAUNCH_PATH))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_combined_entry_exposes_user_facing_runtime_arguments():
    description = load_launch().generate_launch_description()
    names = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert {
        "world",
        "gui",
        "rviz",
        "database_path",
        "reset_database",
        "rgbd_dds_profiles_file",
    } <= names


def test_combined_entry_scopes_both_includes_and_owns_no_duplicate_nodes():
    description = load_launch().generate_launch_description()
    groups = [
        entity for entity in description.entities if isinstance(entity, GroupAction)
    ]

    assert len(groups) == 2
    content = LAUNCH_PATH.read_text()
    assert '"rtabmap_rgbd_simulation.launch.py"' in content
    assert '"online_slam_navigation.launch.py"' in content
    assert '"rviz": "false"' in content
    assert '"map_topic": "/rtabmap/map"' in content
    assert '"lidar_topic": "/lidar_3d/points"' in content
    assert "Node(" not in content


def test_navigation_rviz_has_goal_depth_map_and_live_lidar_displays():
    rviz = (
        Path(__file__).parents[1]
        / "rviz"
        / "rtabmap_rgbd_navigation.rviz"
    ).read_text()

    assert "nav2_rviz_plugins/GoalTool" in rviz
    assert "RGB-D Accumulated 3D Map" in rviz
    assert "Cloud from scan: false" in rviz
    assert "Color Transformer: RGB8" in rviz
    assert "Value: /lidar_3d/points" in rviz
    assert "Name: Depth Image" in rviz
    assert "Normalize Range: false" in rviz
    assert "QMainWindow State:" in rviz
    assert "RGB Image:\n    collapsed: false" in rviz
    assert "Depth Image:\n    collapsed: false" in rviz


def test_all_rgbd_rviz_configs_disable_depth_auto_normalization():
    rviz_directory = Path(__file__).parents[1] / "rviz"
    for filename in ("rtabmap_rgbd.rviz", "rtabmap_rgbd_navigation.rviz"):
        content = (rviz_directory / filename).read_text()
        depth_section = content.split("Name: Depth Image", maxsplit=1)[1]
        assert "Normalize Range: false" in depth_section, filename
