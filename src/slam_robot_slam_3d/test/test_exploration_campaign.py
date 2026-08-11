from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest

MODULE = SourceFileLoader(
    "exploration_campaign",
    str(Path(__file__).parents[1] / "scripts" / "exploration_campaign"),
).load_module()


# Verbatim shape of captured output: ros2 launch prefixes every line with the
# node name, so patterns anchored at the line start silently match nothing and
# every run is recorded as a failure with no failing check named.
PREFIX = "[frontier_exploration_regression-21]"
PASSING_RUN = f"""\
{PREFIX} RESULT frontier exploration
{PREFIX}   host health: control starvation=2 (0.22/min) planner starvation=1 \
(0.11/min) healthy=yes
{PREFIX}   PASS exploration_completed
{PREFIX}   PASS collision_monitor_budget
{PREFIX}   VERDICT PASS
"""
UNSTABLE_RUN = f"""\
{PREFIX}   PASS exploration_completed
{PREFIX}   FAIL collision_monitor_budget
{PREFIX}   VERDICT INFRA_UNSTABLE
[ERROR] [gazebo-1]: process has died [pid 1, exit code -11, cmd 'gz sim'].
"""


def test_verdict_is_parsed_from_launch_prefixed_output():
    record = MODULE.parse_run(PASSING_RUN, 0)

    assert record["verdict"] == MODULE.PASS
    assert record["verdict_reported"] == "PASS"
    assert not record["verdict_missing"]
    assert record["failed_checks"] == []


def test_failed_checks_and_teardown_crash_are_parsed_separately():
    record = MODULE.parse_run(UNSTABLE_RUN, 1)

    assert record["verdict"] == MODULE.INFRA_UNSTABLE
    assert record["failed_checks"] == ["collision_monitor_budget"]
    # A simulator that dies while shutting down must not colour the verdict.
    assert record["gazebo_teardown_crash"]


def test_missing_verdict_is_treated_as_a_failure_not_a_pass():
    # ros2 launch can report success while the scored node failed, so a run
    # whose verdict never appeared must never be counted as a pass.
    record = MODULE.parse_run("nothing useful here", 0)

    assert record["verdict"] == MODULE.FAIL
    assert record["verdict_missing"]


def runs(passes, fails, infra):
    return (
        [{"verdict": MODULE.PASS, "gazebo_teardown_crash": False}] * passes +
        [{"verdict": MODULE.FAIL, "gazebo_teardown_crash": False}] * fails +
        [{"verdict": MODULE.INFRA_UNSTABLE, "gazebo_teardown_crash": False}] * infra
    )


def test_batch_is_void_rather_than_rejected_when_the_host_dominates():
    arguments = MODULE.parse_arguments([])
    outcome, _ = MODULE.judge(runs(7, 0, 3), arguments)

    # Three environment-attributed runs leave too few evaluable ones to say
    # anything, so the batch is void instead of condemning the algorithm.
    assert outcome == "BATCH_INVALID"


def test_a_single_core_failure_rejects_the_batch():
    arguments = MODULE.parse_arguments([])
    assert MODULE.judge(runs(9, 1, 0), arguments)[0] == "REJECTED"
    # A core failure is a core failure however many siblings the host claimed.
    assert MODULE.judge(runs(7, 1, 2), arguments)[0] == "REJECTED"


def test_host_attributed_runs_leave_the_denominator():
    arguments = MODULE.parse_arguments([])
    assert MODULE.judge(runs(10, 0, 0), arguments)[0] == "ACCEPTED"
    assert MODULE.judge(runs(9, 0, 1), arguments)[0] == "ACCEPTED"
    # Judging passes out of the whole batch used to reject this case: with no
    # core failure the pass count is only the batch size minus the runs already
    # blamed on the machine, so counting them twice condemned the algorithm for
    # the host's behaviour.
    assert MODULE.judge(runs(8, 0, 2), arguments)[0] == "ACCEPTED"


def test_default_criteria_are_written_down_before_the_numbers_arrive():
    arguments = MODULE.parse_arguments([])

    assert arguments.runs == 10
    assert arguments.minimum_evaluable_runs == 8
    assert arguments.run_timeout_seconds > 900.0


def test_runner_refuses_impossible_criteria():
    with pytest.raises(SystemExit):
        MODULE.parse_arguments(["--runs", "3", "--minimum-evaluable-runs", "5"])


def test_measurements_are_captured_for_later_review():
    text = f"""{PREFIX}   free cells: initial=8268 maximum=76077 growth=67809
{PREFIX}   maximum known width=27.100 m height=15.450 m
{PREFIX}   known map extent vs outer wall (x -1.60..25.60, y -1.60..13.60): \
west=0.000 m south=0.750 m east=0.400 m north=0.850 m
{PREFIX}   goals: succeeded=7 failed=3 unreachable=3
{PREFIX}   truth distance=315.978 m collision transitions=0 recovery events=0
{PREFIX}   completion: reason='none' wall=352.8 s sim=350.1 s budget=900 s wall
{PREFIX}   pose graph: commits=27 discards=0 failures=0
{PREFIX}   front-end recovery: submap reinitializations=0
{PREFIX}   loop retrieval funnel: proposals=79 verified=9 accepted=1
{PREFIX}   host health: control starvation=0 (0.00/min) planner starvation=1 \
(0.07/min) healthy=yes
{PREFIX}   VERDICT FAIL
"""
    record = MODULE.parse_run(text, 1)

    assert record["free_cells"] == 76077
    assert record["known_bbox_height_m"] == 15.450
    assert record["truth_distance_m"] == 315.978
    assert record["pose_graph_commits"] == 27
    assert record["loop_accepted_candidates"] == 1
    assert record["host_healthy"] is True
    # Per side, because a stretch that pushes north and south out together is
    # not the same failure as one wall smearing.
    assert record["wall_overshoot_m"]["north"] == 0.850
    assert record["wall_overshoot_m"]["west"] == 0.000


def test_missing_measurements_are_recorded_as_absent_not_zero():
    record = MODULE.parse_run(f"{PREFIX}   VERDICT PASS\n", 0)

    assert record["free_cells"] is None
    assert record["host_healthy"] is None
    assert record["wall_overshoot_m"] is None
