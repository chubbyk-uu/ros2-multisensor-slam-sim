"""Contract tests for the custom 3D navigation safety entry point."""

from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, OpaqueFunction


def load_launch(filename):
    path = Path(__file__).parents[1] / "launch" / filename
    loader = SourceFileLoader(filename.replace(".", "_"), str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_safety_launch_exposes_both_scenarios_and_negative_controls():
    module = load_launch("custom_3d_navigation_safety_regression.launch.py")
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert {
        "scenario",
        "smoke",
        "seal_offset_x",
        "obstacle_offset_y",
        "give_up_budget",
        "wall_watchdog_timeout",
    } <= set(arguments)
    assert any(
        isinstance(entity, OpaqueFunction) for entity in description.entities
    )


def test_custom_navigation_exposes_the_gazebo_world_name_contract():
    module = load_launch("custom_3d_navigation_simulation.launch.py")
    description = module.generate_launch_description()
    names = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert "world" in names
    assert "world_name" in names
