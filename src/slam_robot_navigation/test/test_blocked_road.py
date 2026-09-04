"""
The blocked-road criteria, made executable.

Each test names the way a run could look correct while being wrong, because
that is what these criteria exist to catch.
"""

import importlib.util
from pathlib import Path

import pytest

from slam_robot_navigation.blocked_road import (
    BlockedRoadCriteria,
    BlockedRoadObservation,
    core_failures,
    evaluate,
    failed_checks,
)


def observation(**overrides):
    """Build a run that recognised the blockage and stopped safely."""
    fields = {
        "goal_reached": False,
        "blockage_closed_in_costmap": True,
        "planner_reported_no_path": True,
        "final_speed": 0.0,
        "recoveries": 2,
        "minimum_blockage_clearance": 0.45,
        "collision_actions": 0,
        "ended_within_budget": True,
    }
    fields.update(overrides)
    return BlockedRoadObservation(**fields)


def test_stopping_safely_short_of_a_blocked_goal_passes():
    verdict, checks = evaluate(observation(), BlockedRoadCriteria())

    assert verdict == "PASS"
    assert failed_checks(checks) == []


def test_reaching_a_goal_behind_a_full_blockage_fails():
    # The blockage was not blocking, so the run measured nothing -- however
    # safely it was driven.
    verdict, checks = evaluate(
        observation(goal_reached=True), BlockedRoadCriteria()
    )

    assert verdict == "FAIL"
    assert "goal_not_reached" in failed_checks(checks)


def test_not_colliding_is_not_enough_on_its_own():
    # A robot that never saw the wall and stopped for an unrelated reason
    # satisfies "did not reach the goal and did not crash". All three of
    # perception, planning and motion must show positive evidence.
    # "Perceived" means the costmap shows no traversable gap, not that some
    # lethal cell appeared: a partly observed barrier leaves a slit the
    # planner will rightly keep trying.
    blind = observation(blockage_closed_in_costmap=False)
    unplanned = observation(planner_reported_no_path=False)
    still_moving = observation(final_speed=0.4)

    for run, expected in (
        (blind, "blockage_perceived"),
        (unplanned, "planner_reported_no_path"),
        (still_moving, "robot_at_rest"),
    ):
        verdict, checks = evaluate(run, BlockedRoadCriteria())
        assert verdict == "FAIL"
        assert expected in failed_checks(checks)


def test_stopping_by_touching_the_wall_is_not_stopping_safely():
    verdict, checks = evaluate(
        observation(minimum_blockage_clearance=0.05),
        BlockedRoadCriteria(minimum_clearance=0.20),
    )

    assert verdict == "FAIL"
    assert "kept_clearance" in failed_checks(checks)


def test_giving_up_without_attempting_recovery_fails():
    # Zero recoveries means the planning failure never reached the recovery
    # chain, so what looks like a clean stop is an untested path.
    verdict, checks = evaluate(
        observation(recoveries=0), BlockedRoadCriteria(minimum_recoveries=1)
    )

    assert verdict == "FAIL"
    assert "recovery_floor" in failed_checks(checks)


def test_collision_monitor_intervention_is_a_core_failure():
    verdict, checks = evaluate(
        observation(collision_actions=1), BlockedRoadCriteria()
    )

    assert verdict == "FAIL"
    assert "collision_free" in core_failures(checks)


def test_recovering_without_end_fails():
    verdict, checks = evaluate(
        observation(recoveries=99), BlockedRoadCriteria(maximum_recoveries=6)
    )

    assert verdict == "FAIL"
    assert "recovery_ceiling" in failed_checks(checks)


def test_ending_only_because_the_budget_expired_is_the_robots_failure():
    # exploration_campaign charges a launch timeout to the environment because
    # a hung launch says nothing about exploration. Here unbounded recovery is
    # the defect under test, so the timeout must not be excusable: it has to
    # land among the core failures, which a busy host cannot downgrade.
    verdict, checks = evaluate(
        observation(ended_within_budget=False), BlockedRoadCriteria()
    )

    assert verdict == "FAIL"
    assert "ended_within_budget" in core_failures(checks)


def test_only_the_recovery_ceiling_may_be_blamed_on_a_busy_host():
    # A slow host can plausibly drive extra recovery attempts. It cannot make
    # the robot fail to perceive a wall, or drive it into one.
    for field, value in (
        ("blockage_closed_in_costmap", False),
        ("planner_reported_no_path", False),
        ("final_speed", 0.4),
        ("minimum_blockage_clearance", 0.05),
        ("collision_actions", 1),
        ("recoveries", 0),
        ("goal_reached", True),
        ("ended_within_budget", False),
    ):
        _, checks = evaluate(observation(**{field: value}), BlockedRoadCriteria())
        assert core_failures(checks), f"{field} must not be host-excusable"

    _, checks = evaluate(
        observation(recoveries=99), BlockedRoadCriteria(maximum_recoveries=6)
    )
    assert failed_checks(checks) == ["recovery_ceiling"]
    assert core_failures(checks) == []


def test_a_recovery_floor_below_one_is_rejected_outright():
    # Allowing zero would silently turn the floor off rather than relax it.
    with pytest.raises(ValueError, match="at least 1"):
        BlockedRoadCriteria(minimum_recoveries=0)


def test_a_ceiling_below_the_floor_is_rejected():
    with pytest.raises(ValueError, match="must not be below"):
        BlockedRoadCriteria(minimum_recoveries=3, maximum_recoveries=2)


def test_clearance_and_rest_thresholds_must_be_positive():
    with pytest.raises(ValueError, match="minimum_clearance"):
        BlockedRoadCriteria(minimum_clearance=0.0)
    with pytest.raises(ValueError, match="rest_speed"):
        BlockedRoadCriteria(rest_speed=0.0)


def test_every_criterion_is_reported_not_just_the_first_failure():
    # A run that failed several ways must show all of them, so a fix aimed at
    # one does not look like a fix for the run.
    verdict, checks = evaluate(
        observation(
            blockage_closed_in_costmap=False,
            planner_reported_no_path=False,
            recoveries=0,
        ),
        BlockedRoadCriteria(),
    )

    assert verdict == "FAIL"
    assert set(failed_checks(checks)) == {
        "blockage_perceived",
        "planner_reported_no_path",
        "recovery_floor",
    }


def test_navigation_regression_accepts_ros_node_arguments():
    script = (
        Path(__file__).parents[1] / "scripts" / "navigation_regression.py"
    )
    spec = importlib.util.spec_from_file_location(
        "navigation_regression_under_test", script
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    arguments = module.parse_arguments(
        [
            "--scenario",
            "blocked-road",
            "--ros-args",
            "-r",
            "__node:=navigation_regression",
        ]
    )

    assert arguments.scenario == "blocked-road"


def test_navigation_regression_can_skip_amcl_for_online_slam():
    script = (
        Path(__file__).parents[1] / "scripts" / "navigation_regression.py"
    )
    spec = importlib.util.spec_from_file_location(
        "navigation_regression_online_slam_test", script
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    arguments = module.parse_arguments(
        [
            "--scenario",
            "dynamic-obstacle",
            "--localization-mode",
            "online-slam",
            "--obstacle-offset-y",
            "3.0",
            "--pre-map-dynamic-route",
        ]
    )

    assert arguments.localization_mode == "online-slam"
    assert arguments.obstacle_offset_y == 3.0
    assert arguments.pre_map_dynamic_route
