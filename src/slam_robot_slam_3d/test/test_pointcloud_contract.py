from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "pointcloud_contract_check"
)
LOADER = SourceFileLoader("pointcloud_contract_check", str(SCRIPT_PATH))
SPEC = importlib.util.spec_from_loader(
    "pointcloud_contract_check", LOADER
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def validate(**overrides):
    arguments = {
        "field_names": ["x", "y", "z", "intensity", "ring"],
        "frame_id": "lidar_3d_link",
        "width": 720,
        "height": 16,
        "point_step": 32,
        "row_step": 720 * 32,
        "data_size": 720 * 16 * 32,
        "expected_frame": "lidar_3d_link",
        "minimum_points": 100,
        "require_point_time": False,
    }
    arguments.update(overrides)
    return MODULE.validate_cloud_metadata(**arguments)


def test_gazebo_cloud_without_point_time_is_valid_for_lo():
    errors, point_time_fields = validate()

    assert errors == []
    assert point_time_fields == []


def test_point_time_can_be_required_for_future_lio_regression():
    errors, _ = validate(require_point_time=True)

    assert any("no per-point time field" in error for error in errors)


def test_timestamp_alias_is_detected():
    errors, point_time_fields = validate(
        field_names=["x", "y", "z", "ring", "timestamp_0", "timestamp_1"]
    )

    assert errors == []
    assert point_time_fields == ["timestamp_0"]


def test_invalid_frame_and_layout_are_rejected():
    errors, _ = validate(
        frame_id="wrong_frame",
        row_step=1,
        data_size=1,
    )

    assert any("frame_id" in error for error in errors)
    assert any("row_step" in error for error in errors)
    assert any("data buffer" in error for error in errors)
