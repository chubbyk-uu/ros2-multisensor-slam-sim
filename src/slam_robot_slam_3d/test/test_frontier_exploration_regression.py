from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest

MODULE = SourceFileLoader(
    "frontier_exploration_regression",
    str(Path(__file__).parents[1] / "scripts" / "frontier_exploration_regression"),
).load_module()


def test_known_free_cells_uses_navigation_free_threshold():
    message = type("Grid", (), {"data": [-1, 0, 10, 20, 21, 65, 100]})()
    assert MODULE.known_free_cells(message) == 3


def test_map_bounds_tracks_only_selected_cells_in_world_coordinates():
    origin = type("Origin", (), {"position": type("Point", (), {"x": 1.0, "y": -2.0})()})()
    message = type(
        "Grid",
        (),
        {
            "info": type(
                "Info",
                (),
                {"width": 3, "height": 2, "resolution": 0.5, "origin": origin},
            )(),
            "data": [-1, 0, 100, -1, 20, 20],
        },
    )()
    assert MODULE.map_bounds(message, lambda value: 0 <= value <= 20) == (
        1.5, -2.0, 2.5, -1.0
    )
    assert MODULE.bounds_area((1.5, -2.0, 2.5, -1.0)) == 1.0


def test_default_acceptance_requires_motion_growth_and_success():
    arguments = MODULE.parse_arguments([])
    assert arguments.minimum_successful_goals == 1
    assert arguments.maximum_recovery_events > 0
    assert arguments.minimum_free_cell_growth > 0
    assert arguments.minimum_final_free_cells >= 38000
    assert arguments.minimum_travel_distance >= 35.0
    assert "snapshot service is unavailable" in MODULE.CRITICAL_LOG_FRAGMENTS


def test_elapsed_formatting_distinguishes_unmeasured_from_zero():
    assert MODULE.format_elapsed(None) == "n/a"
    assert MODULE.format_elapsed(0.0) == "0.0 s"


def test_completion_drain_must_not_be_negative():
    with pytest.raises(SystemExit):
        MODULE.parse_arguments(["--completion-drain", "-0.1"])


def test_mean_map_interval_needs_two_updates():
    regression = object.__new__(MODULE.ExplorationRegression)
    regression.map_updates = 1
    regression.first_map_wall = 10.0
    regression.last_map_wall = 10.0
    assert regression.mean_map_interval() is None

    regression.map_updates = 3
    regression.last_map_wall = 22.0
    assert regression.mean_map_interval() == 6.0


def make_regression():
    regression = object.__new__(MODULE.ExplorationRegression)
    regression.complete = False
    regression.seen_active_exploration = False
    regression.initial_free_cells = None
    regression.start_wall = 0.0
    regression.start_sim = None
    regression.latest_sim = None
    regression.completion_wall = None
    regression.completion_sim = None
    return regression


def test_stale_completion_does_not_finish_a_new_regression():
    regression = make_regression()
    MODULE.ExplorationRegression.completion_callback(
        regression, type("Completion", (), {"data": True})()
    )
    assert not regression.complete

    MODULE.ExplorationRegression.completion_callback(
        regression, type("Completion", (), {"data": False})()
    )
    regression.initial_free_cells = 1
    MODULE.ExplorationRegression.completion_callback(
        regression, type("Completion", (), {"data": True})()
    )
    assert regression.complete


def test_completion_records_both_wall_and_simulated_elapsed():
    regression = make_regression()
    regression.initial_free_cells = 1
    MODULE.ExplorationRegression.note_sim_time(regression, 0.0)
    assert regression.start_sim is None

    MODULE.ExplorationRegression.note_sim_time(regression, 100.0)
    MODULE.ExplorationRegression.note_sim_time(regression, 285.0)
    MODULE.ExplorationRegression.completion_callback(
        regression, type("Completion", (), {"data": False})()
    )
    MODULE.ExplorationRegression.completion_callback(
        regression, type("Completion", (), {"data": True})()
    )

    assert regression.completion_sim == 185.0
    assert regression.completion_wall is not None


def value(key, text):
    return type("Value", (), {"key": key, "value": text})()


def test_front_end_diagnostics_are_recorded_separately_from_explorer_state():
    regression = object.__new__(MODULE.ExplorationRegression)
    regression.probability_unknown_cells = 0
    regression.probability_free_cells = 0
    regression.probability_partial_cells = 0
    regression.probability_occupied_cells = 0
    regression.pose_graph_commits = 0
    regression.pose_graph_discards = 0
    regression.pose_graph_failures = 0
    status = type(
        "Status",
        (),
        {
            "name": "custom_slam_3d/scan_to_map_front_end",
            "values": [
                value("occupancy_probability_unknown_cells", "12"),
                value("occupancy_probability_free_cells", "34"),
                value("occupancy_probability_partial_cells", "56"),
                value("occupancy_probability_occupied_cells", "78"),
                value("pose_graph_commits", "9"),
                value("pose_graph_discards", "2"),
                value("pose_graph_failures", "1"),
            ],
        },
    )()
    MODULE.ExplorationRegression.diagnostics_callback(
        regression, type("Diagnostics", (), {"status": [status]})()
    )
    assert (
        regression.probability_unknown_cells,
        regression.probability_free_cells,
        regression.probability_partial_cells,
        regression.probability_occupied_cells,
    ) == (12, 34, 56, 78)
    assert (
        regression.pose_graph_commits,
        regression.pose_graph_discards,
        regression.pose_graph_failures,
    ) == (9, 2, 1)
