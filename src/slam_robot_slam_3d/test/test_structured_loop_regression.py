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
    assert arguments.maximum_map_to_odom_height <= 0.02
    assert arguments.maximum_map_to_odom_tilt_degrees <= 0.5
    assert not arguments.navigation_acceptance
    assert not arguments.drive_only


def test_drive_only_mode_is_explicit_and_keeps_route_defaults():
    arguments = MODULE.parse_arguments(["--drive-only", "--laps", "1"])

    assert arguments.drive_only
    assert arguments.laps == 1


def test_drive_only_start_does_not_require_map_to_odom():
    starts = MODULE.starting_poses((1.0, 2.0, 0.1), (0.5, 0.2, -0.1), None, True)

    assert starts == {"truth": (1.0, 2.0, 0.1), "odom": (0.5, 0.2, -0.1)}


def test_height_aware_navigation_thresholds_exceed_robot_outline():
    arguments = MODULE.parse_arguments(["--navigation-acceptance"])

    assert arguments.navigation_acceptance
    assert arguments.minimum_mushroom_clearance > 0.40 + 0.07
    assert arguments.minimum_mushroom_detour >= 0.20
