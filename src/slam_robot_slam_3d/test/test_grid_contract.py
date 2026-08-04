from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "grid_contract_check"
LOADER = SourceFileLoader("grid_contract_check", str(SCRIPT_PATH))
SPEC = importlib.util.spec_from_loader("grid_contract_check", LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def validate(**overrides):
    arguments = {
        "frame_id": "map",
        "resolution": 0.05,
        "width": 40,
        "height": 30,
        "data": [-1] * 1200,
        "expected_frame": "map",
        "minimum_width": 20,
        "minimum_height": 20,
        "minimum_known_cells": 20,
        "minimum_free_cells": 1,
        "minimum_occupied_cells": 1,
        "expected_resolution": 0.05,
        "resolution_tolerance": 0.001,
        "minimum_metric_width": 0.0,
        "maximum_metric_width": 0.0,
        "minimum_metric_height": 0.0,
        "maximum_metric_height": 0.0,
    }
    arguments["data"][0:30] = [0] * 30
    arguments["data"][30:35] = [100] * 5
    arguments.update(overrides)
    return MODULE.validate_grid_metadata(**arguments)


def test_navigation_candidate_grid_is_accepted():
    errors, counts = validate()

    assert errors == []
    assert counts == {"known_cells": 35, "free_cells": 30, "occupied_cells": 5}


def test_invalid_frame_resolution_and_layout_are_rejected():
    errors, _ = validate(
        frame_id="odom",
        resolution=0.0,
        data=[0],
    )

    assert any("frame_id" in error for error in errors)
    assert any("resolution" in error for error in errors)
    assert any("data length" in error for error in errors)


def test_unknown_or_one_class_grid_is_rejected():
    errors, _ = validate(data=[-1] * 1200)

    assert any("known cells" in error for error in errors)
    assert any("free cells" in error for error in errors)
    assert any("occupied cells" in error for error in errors)


def test_resolution_class_counts_and_metric_extent_are_checked():
    errors, _ = validate(
        resolution=0.10,
        minimum_free_cells=40,
        minimum_occupied_cells=10,
        minimum_metric_width=5.0,
        maximum_metric_height=2.0,
    )

    assert any("resolution" in error for error in errors)
    assert any("free cells" in error for error in errors)
    assert any("occupied cells" in error for error in errors)
    assert any("metric width" in error for error in errors)
    assert any("metric height" in error for error in errors)
