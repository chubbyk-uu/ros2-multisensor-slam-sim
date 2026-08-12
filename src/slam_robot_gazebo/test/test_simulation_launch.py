"""Pin Gazebo as the lifecycle root of the simulation launch."""

import ast
from pathlib import Path


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "simulation.launch.py"


def test_both_gazebo_modes_shutdown_the_launch_when_they_exit():
    tree = ast.parse(LAUNCH_FILE.read_text())
    protected_actions = set()
    shutdown_handlers = 0

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "OnProcessExit":
            continue
        arguments = {keyword.arg: keyword.value for keyword in node.keywords}
        target = arguments.get("target_action")
        if isinstance(target, ast.Name):
            protected_actions.add(target.id)
        on_exit = arguments.get("on_exit")
        if on_exit and any(
            isinstance(child, ast.Call)
            and isinstance(child.func, ast.Name)
            and child.func.id == "Shutdown"
            for child in ast.walk(on_exit)
        ):
            shutdown_handlers += 1

    assert protected_actions == {"gazebo_with_gui", "gazebo_headless"}
    assert shutdown_handlers == 2
