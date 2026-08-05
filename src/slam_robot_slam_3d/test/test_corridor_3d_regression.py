from importlib.machinery import SourceFileLoader
import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "corridor_3d_regression"
LOADER = SourceFileLoader("corridor_3d_regression", str(SCRIPT))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(MODULE)


def test_relative_pose_uses_origin_heading():
    origin = (1.0, 2.0, math.pi / 2.0)
    pose = (1.0, 5.0, math.pi)

    relative = MODULE.relative_pose(origin, pose)

    assert math.isclose(relative[0], 3.0, abs_tol=1.0e-12)
    assert math.isclose(relative[1], 0.0, abs_tol=1.0e-12)
    assert math.isclose(relative[2], math.pi / 2.0, abs_tol=1.0e-12)


def test_safe_ratio_handles_empty_segment():
    assert MODULE.safe_ratio(3, 4) == 0.75
    assert MODULE.safe_ratio(0, 0) == 0.0


def test_default_runtime_budget_preserves_ten_hertz_margin():
    arguments = MODULE.parse_arguments([])

    assert arguments.maximum_front_end_gap == 0.12
    assert arguments.maximum_processing_p95 == 90.0
