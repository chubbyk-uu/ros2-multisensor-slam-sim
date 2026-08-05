from importlib.machinery import SourceFileLoader
import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "front_end_motion_regression"
LOADER = SourceFileLoader("front_end_motion_regression", str(SCRIPT))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(MODULE)


def test_relative_pose_and_error_use_origin_heading():
    origin = MODULE.Pose2D(1.0, 2.0, math.pi / 2.0)
    pose = MODULE.Pose2D(1.0, 5.0, math.pi)
    relative = MODULE.relative_pose(origin, pose)

    assert math.isclose(relative.x, 3.0, abs_tol=1.0e-12)
    assert math.isclose(relative.y, 0.0, abs_tol=1.0e-12)
    assert math.isclose(relative.yaw, math.pi / 2.0, abs_tol=1.0e-12)
    assert MODULE.pose_error(relative, relative) == (0.0, 0.0)


def test_parse_arguments_rejects_invalid_motion():
    arguments = MODULE.parse_arguments(["--profile", "rotation"])
    assert arguments.angular_speeds == (0.30, 0.60, 0.90)
    assert arguments.maximum_front_end_gap == 0.12
    assert arguments.maximum_processing_p95 == 60.0

    try:
        MODULE.parse_arguments(
            ["--profile", "rotation", "--rotation-duration", "0"]
        )
    except SystemExit as error:
        assert error.code != 0
    else:
        raise AssertionError("zero duration should be rejected")
