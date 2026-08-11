"""
Keep the exploration seed connected from the regression down to the node.

Goal selection is randomised among near-best candidates, so a regression is
only reproducible while its fixed seed actually reaches the explorer. The value
crosses three launch files and changes name once on the way: the regression
passes exploration_seed, the simulation entry forwards it as
selection_random_seed, and the exploration launch hands that to the node.

A break anywhere along that chain fails silently and in the worst possible
direction. Zero is not an error value, it means "draw a fresh seed", so the
regression would keep passing while quietly measuring a different trajectory
every run, and nothing would report it.

The launch files are read as source rather than executed: the arguments live
inside nested GroupActions and IncludeLaunchDescriptions as substitution
objects, and comparing the text that was written is both simpler and closer to
what a reader has to keep consistent.
"""

import ast
from pathlib import Path

import pytest

LAUNCH_DIRECTORY = Path(__file__).resolve().parents[1] / "launch"
NAVIGATION_LAUNCH = (
    Path(__file__).resolve().parents[2]
    / "slam_robot_navigation" / "launch" / "frontier_exploration.launch.py"
)
SEED_ARGUMENT = "exploration_seed"
SEED_PARAMETER = "selection_random_seed"
REGRESSIONS = (
    "frontier_exploration_regression.launch.py",
    "rtabmap_frontier_exploration_regression.launch.py",
)
SIMULATIONS = (
    "custom_3d_exploration_simulation.launch.py",
    "rtabmap_3d_exploration_simulation.launch.py",
)


def syntax(path):
    return ast.parse(path.read_text())


def call_name(node):
    if not isinstance(node, ast.Call):
        return None
    function = node.func
    if isinstance(function, ast.Name):
        return function.id
    return function.attr if isinstance(function, ast.Attribute) else None


def string_of(node):
    """Return the text a launch argument was written as, if it is a literal."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def launch_configuration_of(node):
    """Return the configuration name a value forwards, if it forwards one."""
    if call_name(node) == "LaunchConfiguration" and node.args:
        return string_of(node.args[0])
    if isinstance(node, ast.Call) and node.args:
        # ParameterValue(LaunchConfiguration("x"), value_type=int) and friends.
        return launch_configuration_of(node.args[0])
    return None


def dictionary_entries(path):
    entries = []
    for node in ast.walk(syntax(path)):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            name = string_of(key) if key is not None else None
            if name is not None:
                entries.append((name, value))
    return entries


def declared_defaults(path):
    defaults = {}
    for node in ast.walk(syntax(path)):
        if call_name(node) != "DeclareLaunchArgument" or not node.args:
            continue
        name = string_of(node.args[0])
        if name is None:
            continue
        default = None
        for keyword in node.keywords:
            if keyword.arg == "default_value":
                default = string_of(keyword.value)
        defaults[name] = default
    return defaults


@pytest.mark.parametrize("regression", REGRESSIONS)
def test_regressions_pin_a_non_zero_seed(regression):
    values = [
        string_of(value)
        for name, value in dictionary_entries(LAUNCH_DIRECTORY / regression)
        if name == SEED_ARGUMENT
    ]

    assert values, f"{regression} forwards no {SEED_ARGUMENT}"
    for value in values:
        assert value is not None and value.strip() not in ("", "0"), (
            f"{regression} must pin a non-zero seed; zero draws a new one every run"
        )


@pytest.mark.parametrize("simulation", SIMULATIONS)
def test_simulation_entries_forward_the_seed_under_its_node_name(simulation):
    path = LAUNCH_DIRECTORY / simulation
    forwarded = [
        launch_configuration_of(value)
        for name, value in dictionary_entries(path)
        if name == SEED_PARAMETER
    ]

    assert SEED_ARGUMENT in declared_defaults(path)
    assert SEED_ARGUMENT in forwarded, (
        f"{simulation} must forward {SEED_ARGUMENT} as {SEED_PARAMETER}"
    )


def test_the_exploration_launch_hands_the_seed_to_the_node():
    forwarded = [
        launch_configuration_of(value)
        for name, value in dictionary_entries(NAVIGATION_LAUNCH)
        if name == SEED_PARAMETER
    ]

    assert SEED_PARAMETER in declared_defaults(NAVIGATION_LAUNCH)
    assert SEED_PARAMETER in forwarded, (
        "the exploration launch must pass selection_random_seed to the node"
    )


@pytest.mark.parametrize("simulation", SIMULATIONS)
def test_a_fresh_seed_is_the_default_for_interactive_runs(simulation):
    # Only the regressions pin it. Interactive runs are meant to vary, which is
    # the reason the selection is randomised at all.
    assert declared_defaults(LAUNCH_DIRECTORY / simulation)[SEED_ARGUMENT] == "0"
