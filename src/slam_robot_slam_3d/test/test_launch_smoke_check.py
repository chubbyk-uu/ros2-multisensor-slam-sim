from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
import sys


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
        "structured_loop",
        "structured_navigation",
        "structured_dataset",
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
