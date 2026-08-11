from importlib.machinery import SourceFileLoader
import math
from pathlib import Path
import time

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
    assert MODULE.bounds_width((1.5, -2.0, 2.5, -1.0)) == 1.0
    assert MODULE.bounds_height((1.5, -2.0, 2.5, -1.0)) == 1.0


def test_default_acceptance_requires_motion_growth_and_success():
    arguments = MODULE.parse_arguments([])
    assert arguments.minimum_successful_goals == 1
    assert arguments.maximum_recovery_events > 0
    assert arguments.minimum_free_cell_growth > 0
    assert arguments.minimum_final_free_cells >= 38000
    assert arguments.minimum_travel_distance >= 35.0
    assert arguments.maximum_successful_goals < 156
    assert arguments.maximum_known_bbox_width_m > 0.0
    assert arguments.maximum_known_bbox_height_m > 0.0
    assert arguments.minimum_loop_accepted_candidates == 0
    assert "snapshot service is unavailable" in MODULE.CRITICAL_LOG_FRAGMENTS


def test_map_frame_error_limits_separate_normal_runs_from_a_drifted_map():
    arguments = MODULE.parse_arguments([])

    # Ten runs measured 0.012-0.091 m and 0.103-0.263 deg once the map was
    # rebuilt from scan-matched poses.
    assert arguments.maximum_map_frame_position_error_m > 0.091
    assert arguments.maximum_map_frame_yaw_error_deg > 0.263
    # The run that exposed the defect reached 2.717 m and 13.285 deg, so the
    # limits must still catch it by a wide margin.
    assert arguments.maximum_map_frame_position_error_m < 2.717 / 5.0
    assert arguments.maximum_map_frame_yaw_error_deg < 13.285 / 5.0


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


def test_nav2_starvation_fragments_match_the_messages_nav2_actually_emits():
    # Pinned verbatim. If Nav2 rewords these the parser would report a
    # perfectly healthy host for every run, and that error biases towards
    # calling a starved run PASS, so this test must fail loudly instead.
    control = (
        "Control loop missed its desired rate of 20.0000 Hz. "
        "Current loop rate is 4.9020 Hz."
    )
    planner = (
        "Planner loop missed its desired rate of 20.0000 Hz. "
        "Current loop rate is 1.1062 Hz"
    )
    assert MODULE.CONTROL_STARVATION_FRAGMENT in control
    assert MODULE.PLANNER_STARVATION_FRAGMENT in planner
    assert MODULE.CONTROL_STARVATION_FRAGMENT not in planner
    assert MODULE.PLANNER_STARVATION_FRAGMENT not in control


def test_starvation_is_counted_per_minute_not_per_run():
    regression = object.__new__(MODULE.ExplorationRegression)
    regression.recovery_events = 0
    regression.critical_logs = []
    regression.control_starvation_warnings = 0
    regression.planner_starvation_warnings = 0
    for _ in range(6):
        MODULE.ExplorationRegression.log_callback(
            regression,
            type("Log", (), {"msg": "Control loop missed its desired rate of 20 Hz"})(),
        )
    assert regression.control_starvation_warnings == 6

    # A ten-minute run with six warnings is calmer than a five-minute one.
    regression.start_wall = time.monotonic() - 600.0
    control_rate, _ = MODULE.ExplorationRegression.starvation_rates(regression)
    assert control_rate == pytest.approx(0.6, abs=0.05)


def test_wall_overshoot_is_reported_per_side():
    # A box that pushes past the south wall only.
    overshoot = MODULE.wall_overshoot((-1.6, -2.35, 25.6, 13.6))

    assert overshoot["south"] == pytest.approx(0.75)
    assert overshoot["north"] == pytest.approx(0.0)
    assert overshoot["west"] == pytest.approx(0.0)
    assert overshoot["east"] == pytest.approx(0.0)
    assert MODULE.wall_overshoot(None) is None


def core_checks(**overrides):
    checks = {"exploration_completed": True, "known_map_height_ok": True}
    checks.update({name: True for name in MODULE.NAVIGATION_BEHAVIOUR_CHECKS})
    checks.update(overrides)
    return checks


def test_classification_covers_every_combination_of_core_behaviour_and_host():
    # A core failure is an algorithm regression whatever the host was doing.
    assert MODULE.classify(core_checks(exploration_completed=False), True) == MODULE.FAIL
    assert MODULE.classify(core_checks(exploration_completed=False), False) == MODULE.FAIL

    # Nothing failed: a merely slow host must not spend the campaign's
    # environment budget, so this stays a pass.
    assert MODULE.classify(core_checks(), True) == MODULE.PASS
    assert MODULE.classify(core_checks(), False) == MODULE.PASS

    # Collisions on a healthy host are a real navigation problem.
    assert MODULE.classify(
        core_checks(collision_monitor_budget=False), True) == MODULE.FAIL
    # The same collisions on a starved host are attributed to the environment.
    assert MODULE.classify(
        core_checks(collision_monitor_budget=False), False) == MODULE.INFRA_UNSTABLE
    # Recovery budget is navigation behaviour too, not a core criterion.
    assert MODULE.classify(
        core_checks(navigation_recovery_budget=False), False) == MODULE.INFRA_UNSTABLE


def test_verdict_codes_are_distinct():
    assert len({MODULE.PASS, MODULE.FAIL, MODULE.INFRA_UNSTABLE}) == 3
    assert MODULE.PASS == 0


def test_map_shape_limits_leave_more_margin_than_the_observed_spread():
    arguments = MODULE.parse_arguments([])

    # Ten calibration runs measured 26.95-27.60 m wide and 14.95-16.80 m tall.
    assert arguments.maximum_known_bbox_width_m >= 27.6 + 0.4
    assert arguments.maximum_known_bbox_height_m >= 16.8 + 0.4
    assert arguments.maximum_known_bbox_area_m2 >= 451.0 + 20.0
    # Still far below the 20.10 m height a warped map produced.
    assert arguments.maximum_known_bbox_height_m < 20.0


def test_relative_pose_removes_a_fixed_frame_offset():
    # The estimate and ground truth start from different, fixed offsets. What
    # matters is drift accumulated since the start, not the offset itself.
    origin = (10.0, -5.0, math.pi / 2)
    moved = (10.0, -3.0, math.pi / 2)

    forward = MODULE.relative_pose(origin, moved)

    # Two metres along the frame's own heading, none sideways, no rotation.
    assert forward[0] == pytest.approx(2.0)
    assert forward[1] == pytest.approx(0.0, abs=1e-9)
    assert forward[2] == pytest.approx(0.0)


def test_trajectory_error_is_zero_when_the_estimate_tracks_truth():
    error = MODULE.TrajectoryError()
    truth = [(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0))]
    # A different frame origin, same motion.
    error.observe(0.0, (100.0, 100.0, 0.0), truth)
    error.observe(1.0, (101.0, 100.0, 0.0), truth)

    assert error.samples == 1
    assert error.latest_position == pytest.approx(0.0, abs=1e-9)
    assert error.maximum_yaw == pytest.approx(0.0, abs=1e-9)


def test_trajectory_error_reports_drift_and_keeps_the_peak():
    error = MODULE.TrajectoryError()
    truth = [(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0)), (2.0, (2.0, 0.0, 0.0))]
    error.observe(0.0, (0.0, 0.0, 0.0), truth)
    error.observe(1.0, (1.0, 0.4, 0.0), truth)
    error.observe(2.0, (2.0, 0.1, 0.0), truth)

    assert error.latest_position == pytest.approx(0.1)
    # The peak survives a later recovery, which is what a transient distortion
    # looks like once a loop closure pulls the estimate back.
    assert error.maximum_position == pytest.approx(0.4)


def test_trajectory_error_charges_a_mispaired_stamp_with_real_motion():
    # Why the map-frame pose is looked up at the truth timestamp rather than
    # taken as the newest transform: the front end future-dates map -> odom by
    # its transform tolerance, so pairing on that stamp compares a pose with
    # ground truth from a tenth of a second later. During a turn that alone
    # reads as several degrees of error that the estimate never made.
    error = MODULE.TrajectoryError(maximum_pair_age=0.2)
    turning = [
        (0.0, (0.0, 0.0, 0.0)),
        (0.1, (0.0, 0.0, 0.06)),
    ]
    error.observe(0.0, (0.0, 0.0, 0.0), turning)
    error.observe(0.0, (0.0, 0.0, 0.0), [turning[1]])

    assert math.degrees(error.maximum_yaw) == pytest.approx(3.44, abs=0.1)


def test_trajectory_error_refuses_badly_paired_samples():
    error = MODULE.TrajectoryError(maximum_pair_age=0.05)
    truth = [(0.0, (0.0, 0.0, 0.0))]
    error.observe(10.0, (0.0, 0.0, 0.0), truth)

    # Nothing anchored, because no truth sample was close enough in time.
    assert error.estimate_origin is None
    assert error.samples == 0


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
    regression.submap_reinitializations = 0
    regression.loop_retrieval_eligible = 0
    regression.loop_retrieval_shortlisted = 0
    regression.loop_retrieval_descriptor_rejections = 0
    regression.loop_retrieval_distance_at_most_0_05 = 0
    regression.loop_retrieval_distance_at_most_0_10 = 0
    regression.loop_retrieval_distance_at_most_0_15 = 0
    regression.loop_retrieval_candidates = 0
    regression.loop_verified_candidates = 0
    regression.loop_accepted_candidates = 0
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
                value("submap_reinitializations", "2"),
                value("loop_retrieval_eligible_total", "100"),
                value("loop_retrieval_shortlisted_total", "80"),
                value("loop_retrieval_descriptor_rejections_total", "50"),
                value("loop_retrieval_distance_at_most_0_05_total", "10"),
                value("loop_retrieval_distance_at_most_0_10_total", "20"),
                value("loop_retrieval_distance_at_most_0_15_total", "30"),
                value("loop_retrieval_candidates_total", "30"),
                value("loop_verified_candidates_total", "20"),
                value("loop_accepted_candidates_total", "7"),
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
    assert regression.submap_reinitializations == 2
    assert (
        regression.loop_retrieval_distance_at_most_0_05,
        regression.loop_retrieval_distance_at_most_0_10,
        regression.loop_retrieval_distance_at_most_0_15,
    ) == (10, 20, 30)
    assert (
        regression.loop_retrieval_eligible,
        regression.loop_retrieval_shortlisted,
        regression.loop_retrieval_descriptor_rejections,
        regression.loop_retrieval_candidates,
        regression.loop_verified_candidates,
        regression.loop_accepted_candidates,
    ) == (100, 80, 50, 30, 20, 7)
