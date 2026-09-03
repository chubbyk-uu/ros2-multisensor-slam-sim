"""Unit tests for the shared dynamic-obstacle acceptance contract."""

import math

import pytest

from slam_robot_navigation.dynamic_obstacle import (
    DynamicObstacleCriteria,
    DynamicObstacleObservation,
    evaluate,
    failed_checks,
)


def observation(**overrides):
    fields = {
        "goal_reached": True,
        "global_costmap_observed": True,
        "local_costmap_observed": True,
        "global_costmap_latency": 1.0,
        "local_costmap_latency": 0.4,
        "minimum_obstacle_clearance": 0.30,
        "maximum_path_deviation": 0.80,
        "goal_error": 0.10,
        "recoveries": 0,
        "collision_actions": 0,
    }
    fields.update(overrides)
    return DynamicObstacleObservation(**fields)


def test_a_safe_observed_detour_passes():
    verdict, checks = evaluate(observation(), DynamicObstacleCriteria())

    assert verdict == "PASS"
    assert failed_checks(checks) == []


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    (
        ("goal_reached", False, "goal_reached"),
        ("global_costmap_observed", False, "global_costmap_observed"),
        ("local_costmap_observed", False, "local_costmap_observed"),
        ("global_costmap_latency", 3.0, "global_costmap_latency"),
        ("local_costmap_latency", 1.0, "local_costmap_latency"),
        ("minimum_obstacle_clearance", 0.05, "kept_clearance"),
        ("maximum_path_deviation", 0.10, "detour_observed"),
        ("goal_error", 0.80, "goal_error"),
        ("recoveries", 4, "recovery_budget"),
        ("collision_actions", 1, "collision_free"),
    ),
)
def test_each_failure_is_visible(field, value, expected):
    verdict, checks = evaluate(
        observation(**{field: value}), DynamicObstacleCriteria()
    )

    assert verdict == "FAIL"
    assert expected in failed_checks(checks)


def test_missing_numeric_measurements_fail_closed():
    verdict, checks = evaluate(
        observation(
            global_costmap_latency=math.inf,
            minimum_obstacle_clearance=-math.inf,
            goal_error=math.nan,
        ),
        DynamicObstacleCriteria(),
    )

    assert verdict == "FAIL"
    assert set(failed_checks(checks)) >= {
        "global_costmap_latency",
        "kept_clearance",
        "goal_error",
    }


def test_invalid_thresholds_are_rejected():
    with pytest.raises(ValueError, match="minimum_clearance"):
        DynamicObstacleCriteria(minimum_clearance=0.0)
    with pytest.raises(ValueError, match="maximum_goal_error"):
        DynamicObstacleCriteria(maximum_goal_error=math.nan)
    with pytest.raises(ValueError, match="maximum_recoveries"):
        DynamicObstacleCriteria(maximum_recoveries=-1)
