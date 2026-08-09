from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
CENSUS = SourceFileLoader(
    "map_projection_census", str(SCRIPTS / "map_projection_census")
).load_module()
COMPARE = SourceFileLoader(
    "map_projection_compare", str(SCRIPTS / "map_projection_compare")
).load_module()


def make_grid(data, width, height, resolution=0.5):
    info = type("Info", (), {"width": width, "height": height, "resolution": resolution})()
    stamp = type("Stamp", (), {"sec": 3, "nanosec": 500_000_000})()
    header = type("Header", (), {"stamp": stamp})()
    return type("Grid", (), {"info": info, "data": data, "header": header})()


def test_census_classifies_cells_and_converts_to_area():
    # 2x2 grid at 0.5 m: one free, one occupied, two unknown.
    report = CENSUS.census(make_grid([0, 100, -1, -1], 2, 2), 20, 65)

    assert report["free_cells"] == 1
    assert report["occupied_cells"] == 1
    assert report["unknown_cells"] == 2
    assert report["free_area_m2"] == pytest.approx(0.25)
    assert report["free_ratio"] == pytest.approx(0.5)
    assert report["stamp_sec"] == pytest.approx(3.5)


def test_census_reports_the_known_bounding_box_not_the_grid_extent():
    # Known cells occupy a single 1x1 corner of a 4x4 grid.
    data = [-1] * 16
    data[5] = 0
    report = CENSUS.census(make_grid(data, 4, 4), 20, 65)

    assert report["known_bounding_box_m2"] == pytest.approx(0.25)
    assert report["grid_width"] * report["grid_height"] == 16


def test_census_rejects_a_grid_whose_data_does_not_match_its_size():
    with pytest.raises(ValueError):
        CENSUS.census(make_grid([0, 0, 0], 2, 2), 20, 65)


def test_intermediate_costs_count_as_neither_free_nor_occupied():
    report = CENSUS.census(make_grid([0, 40, 90, -1], 2, 2), 20, 65)

    assert report["free_cells"] == 1
    assert report["occupied_cells"] == 1
    assert report["unknown_cells"] == 1


def test_comparison_reports_the_candidate_to_reference_ratio():
    reference = CENSUS.census(make_grid([0, 0, 100, -1], 2, 2), 20, 65)
    reference["label"] = "rtabmap"
    reference["updates"] = 10
    reference["mean_publish_interval_sec"] = 0.5
    candidate = CENSUS.census(make_grid([0, -1, 100, -1], 2, 2), 20, 65)
    candidate["label"] = "custom"
    candidate["updates"] = 2
    candidate["mean_publish_interval_sec"] = 5.0

    rendered = COMPARE.render(reference, candidate)

    assert "rtabmap" in rendered and "custom" in rendered
    assert "0.50x" in rendered
