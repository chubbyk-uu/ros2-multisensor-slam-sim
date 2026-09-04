from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
import sys

from launch.actions import DeclareLaunchArgument


SCRIPT = Path(__file__).parents[1] / "scripts" / "launch_smoke_check"
LOADER = SourceFileLoader("launch_smoke_check", str(SCRIPT))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[LOADER.name] = MODULE
LOADER.exec_module(MODULE)


def test_all_composite_launches_have_a_distinct_smoke_profile():
    profiles = {profile.name: profile for profile in MODULE.PROFILES}

    assert set(profiles) == {
        "corridor",
        "motion",
        "custom_loop",
        "structured_loop",
        "structured_navigation",
        "custom_navigation_safety",
        "rtabmap_rgbd_navigation_safety",
        "structured_dataset",
        "custom_exploration",
        "rtabmap_exploration",
    }
    assert all(profile.required_nodes for profile in profiles.values())
    assert "/rtabmap/rtabmap" in profiles["structured_loop"].required_nodes
    assert "/bt_navigator" in profiles["structured_navigation"].required_nodes


def test_launch_command_is_headless_and_non_driving(tmp_path):
    profile = MODULE.profile_by_name("structured_dataset")
    command = MODULE.launch_command(profile, tmp_path / "recording")

    assert command[:4] == [
        "ros2",
        "launch",
        "slam_robot_slam_3d",
        profile.launch_file,
    ]
    assert "gui:=false" in command
    assert "rviz:=false" in command
    assert "smoke:=true" in command
    assert any(argument.startswith("output:=") for argument in command)


def test_default_smoke_duration_matches_the_launch_lifetime_requirement():
    arguments = MODULE.parse_arguments([])

    assert arguments.startup_timeout == 60.0
    assert arguments.hold_time == 60.0


def test_composite_launches_declare_a_real_smoke_argument():
    launch_directory = Path(__file__).parents[1] / "launch"
    launch_files = [
        "corridor_3d_regression.launch.py",
        "front_end_motion_regression.launch.py",
        "structured_loop_regression.launch.py",
        "structured_navigation_regression.launch.py",
        "custom_3d_navigation_safety_regression.launch.py",
        "rtabmap_rgbd_navigation_safety_regression.launch.py",
        "structured_dataset_recording.launch.py",
        "custom_3d_loop_regression.launch.py",
        "frontier_exploration_regression.launch.py",
        "rtabmap_frontier_exploration_regression.launch.py",
    ]
    for index, filename in enumerate(launch_files):
        loader = SourceFileLoader(
            f"smoke_launch_{index}", str(launch_directory / filename)
        )
        spec = importlib.util.spec_from_loader(loader.name, loader)
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)
        description = module.generate_launch_description()
        smoke_arguments = [
            entity
            for entity in description.entities
            if isinstance(entity, DeclareLaunchArgument)
            and entity.name == "smoke"
        ]
        assert len(smoke_arguments) == 1, filename
