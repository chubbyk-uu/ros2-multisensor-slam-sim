from importlib.machinery import SourceFileLoader
import importlib.util
import math
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "structured_loop_regression"
)
LOADER = SourceFileLoader("structured_loop_regression", str(SCRIPT_PATH))
SPEC = importlib.util.spec_from_loader("structured_loop_regression", LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_pose_composition_and_relative_motion_are_consistent():
    map_to_odom = (1.0, -2.0, math.pi / 2.0)
    odom_pose = (3.0, 0.5, -math.pi / 4.0)

    map_pose = MODULE.compose_pose(map_to_odom, odom_pose)
    recovered = MODULE.relative_pose(map_to_odom, map_pose)

    assert math.isclose(recovered[0], odom_pose[0], abs_tol=1.0e-9)
    assert math.isclose(recovered[1], odom_pose[1], abs_tol=1.0e-9)
    assert math.isclose(recovered[2], odom_pose[2], abs_tol=1.0e-9)


def test_default_regression_requires_two_laps_and_real_loop_evidence():
    arguments = MODULE.parse_arguments([])

    assert arguments.laps == 2
    assert arguments.minimum_route_distance >= 138.0
    assert arguments.minimum_proximity_events >= 1
    assert arguments.minimum_map_correction > 0.0
