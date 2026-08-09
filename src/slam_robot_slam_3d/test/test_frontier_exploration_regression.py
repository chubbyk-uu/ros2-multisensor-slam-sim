from importlib.machinery import SourceFileLoader
from pathlib import Path


MODULE = SourceFileLoader(
    "frontier_exploration_regression",
    str(Path(__file__).parents[1] / "scripts" / "frontier_exploration_regression"),
).load_module()


def test_known_free_cells_uses_navigation_free_threshold():
    message = type("Grid", (), {"data": [-1, 0, 10, 20, 21, 65, 100]})()
    assert MODULE.known_free_cells(message) == 3


def test_default_acceptance_requires_motion_growth_and_success():
    arguments = MODULE.parse_arguments([])
    assert arguments.minimum_successful_goals == 1
    assert arguments.maximum_recovery_events > 0
    assert arguments.minimum_free_cell_growth > 0
    assert arguments.minimum_final_free_cells >= 38000
    assert arguments.minimum_travel_distance >= 35.0
    assert "snapshot service is unavailable" in MODULE.CRITICAL_LOG_FRAGMENTS


def test_stale_completion_does_not_finish_a_new_regression():
    regression = object.__new__(MODULE.ExplorationRegression)
    regression.complete = False
    regression.seen_active_exploration = False
    regression.initial_free_cells = None
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
