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

    # Three environment-attributed runs say more about the machine than the
    # code, so the batch is void instead of condemning the algorithm.
    assert outcome == "BATCH_INVALID"


def test_a_single_core_failure_rejects_the_batch():
    arguments = MODULE.parse_arguments([])
    outcome, _ = MODULE.judge(runs(9, 1, 0), arguments)

    assert outcome == "REJECTED"


def test_acceptance_needs_the_preregistered_pass_count():
    arguments = MODULE.parse_arguments([])
    assert MODULE.judge(runs(9, 0, 1), arguments)[0] == "ACCEPTED"
    assert MODULE.judge(runs(10, 0, 0), arguments)[0] == "ACCEPTED"
    # Eight passes with two host-attributed runs is a valid batch that still
    # falls short of the pre-registered bar.
    assert MODULE.judge(runs(8, 0, 2), arguments)[0] == "REJECTED"


def test_default_criteria_are_written_down_before_the_numbers_arrive():
    arguments = MODULE.parse_arguments([])

    assert arguments.runs == 10
    assert arguments.minimum_passes == 9
    assert arguments.maximum_infra_unstable_for_valid_batch == 2


def test_runner_refuses_impossible_criteria():
    with pytest.raises(SystemExit):
        MODULE.parse_arguments(["--runs", "3", "--minimum-passes", "5"])
