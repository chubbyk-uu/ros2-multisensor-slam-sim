from importlib.machinery import SourceFileLoader
import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "front_end_regression"
LOADER = SourceFileLoader("front_end_regression", str(SCRIPT))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(MODULE)


def test_normalize_angle_wraps_both_directions():
    assert math.isclose(MODULE.normalize_angle(3.0 * math.pi), math.pi)
    assert math.isclose(MODULE.normalize_angle(-3.0 * math.pi), -math.pi)


def test_relative_pose_is_expressed_in_origin_frame():
    origin = (2.0, 3.0, math.pi / 2.0)
    pose = (2.0, 4.0, math.pi)

    relative = MODULE.relative_pose(origin, pose)

    assert math.isclose(relative[0], 1.0, abs_tol=1.0e-12)
    assert math.isclose(relative[1], 0.0, abs_tol=1.0e-12)
    assert math.isclose(relative[2], math.pi / 2.0, abs_tol=1.0e-12)
