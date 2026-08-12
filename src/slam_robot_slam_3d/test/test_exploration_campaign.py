from importlib.machinery import SourceFileLoader
import json
import os
from pathlib import Path
import subprocess
import time

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


def test_a_hung_launch_is_the_environment_not_an_algorithm_failure():
    # A run killed on its timeout never reaches its own scoring, so it has no
    # verdict to report. Calling that a core failure would blame exploration
    # for the machine hanging, the same error as counting a starved run against
    # it.
    record = MODULE.parse_run("no verdict here", None, timed_out=True)

    assert record["verdict"] == MODULE.INFRA_UNSTABLE
    assert record["launch_timed_out"]


def test_a_timeout_does_not_excuse_a_verdict_that_was_reported():
    # The run scored itself before the clock ran out; that verdict stands.
    record = MODULE.parse_run(f"{PREFIX}   VERDICT FAIL\n", None, timed_out=True)

    assert record["verdict"] == MODULE.FAIL


def test_gazebo_dying_before_a_verdict_is_infrastructure_unstable():
    text = "[ERROR] [gazebo-1]: process has died [pid 1, exit code 1, cmd 'gz sim']."
    record = MODULE.parse_run(text, None, simulator_died=True)

    assert record["verdict"] == MODULE.INFRA_UNSTABLE
    assert record["simulator_died_before_verdict"]


def test_launch_overrides_reach_every_run():
    arguments = MODULE.parse_arguments(
        ["-a", "exploration_seed:=0", "-a", "laps:=3"])

    command = MODULE.launch_command(arguments)

    assert command[:4] == [
        "ros2", "launch", "slam_robot_slam_3d", arguments.launch_file]
    assert "gui:=false" in command and "rviz:=false" in command
    assert command[-2:] == ["exploration_seed:=0", "laps:=3"]


def test_a_malformed_override_is_refused_rather_than_passed_on():
    # ros2 launch would reject it anyway, one run at a time, after the
    # simulator had already started.
    arguments = MODULE.parse_arguments(["-a", "exploration_seed=0"])

    with pytest.raises(ValueError):
        MODULE.launch_command(arguments)


def test_no_overrides_leaves_the_command_as_it_was():
    assert MODULE.launch_command(MODULE.parse_arguments([]))[-2:] == [
        "gui:=false", "rviz:=false"]


def runs(passes, fails, infra):
    return (
        [{"verdict": MODULE.PASS, "gazebo_teardown_crash": False}] * passes +
        [{"verdict": MODULE.FAIL, "gazebo_teardown_crash": False}] * fails +
        [{"verdict": MODULE.INFRA_UNSTABLE, "gazebo_teardown_crash": False}] * infra
    )


def test_batch_is_void_rather_than_rejected_when_the_host_dominates():
    arguments = MODULE.parse_arguments([])
    outcome, _ = MODULE.judge(runs(3, 0, 2), arguments)

    # Three environment-attributed runs leave too few evaluable ones to say
    # anything, so the batch is void instead of condemning the algorithm.
    assert outcome == "BATCH_INVALID"


def test_a_single_core_failure_rejects_the_batch():
    arguments = MODULE.parse_arguments([])
    assert MODULE.judge(runs(4, 1, 0), arguments)[0] == "REJECTED"
    # A core failure is a core failure however many siblings the host claimed.
    assert MODULE.judge(runs(3, 1, 1), arguments)[0] == "REJECTED"


def test_host_attributed_runs_leave_the_denominator():
    arguments = MODULE.parse_arguments([])
    assert MODULE.judge(runs(5, 0, 0), arguments)[0] == "ACCEPTED"
    assert MODULE.judge(runs(4, 0, 1), arguments)[0] == "ACCEPTED"
    # Judging passes out of the whole batch used to reject this case: with no
    # core failure the pass count is only the batch size minus the runs already
    # blamed on the machine, so counting them twice condemned the algorithm for
    # the host's behaviour.
    relaxed = MODULE.parse_arguments(["--minimum-evaluable-runs", "3"])
    assert MODULE.judge(runs(3, 0, 2), relaxed)[0] == "ACCEPTED"


def test_default_criteria_are_written_down_before_the_numbers_arrive():
    arguments = MODULE.parse_arguments([])

    assert arguments.runs == 5
    assert arguments.minimum_evaluable_runs == 4
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


def group_is_alive(group):
    try:
        os.killpg(group, 0)
    except ProcessLookupError:
        return False
    return True


def test_a_run_whose_parent_left_first_is_still_cleaned_up():
    # The shape this is really about: ros2 launch exits, and the simulator it
    # started keeps running in the group left behind. Asking the departed pid
    # for its group finds nothing, so the cleanup used to return having killed
    # nobody, and the check that follows would then abort the batch over
    # processes it could have stopped.
    launch = subprocess.Popen(["sh", "-c", "sleep 300 & exit 0"],
                              start_new_session=True)
    group = launch.pid
    launch.wait()
    assert group_is_alive(group)

    MODULE.end_process_group(launch, group, 2.0)

    assert not group_is_alive(group)


def test_cleanup_waits_for_the_group_rather_than_the_parent():
    # A run killed on its timeout still has its parent, unreaped. Judging
    # progress by waiting on that parent returns the moment it is reaped while
    # its children are still up, and a parent left as a zombie is itself still
    # a member of the group answering signals.
    launch = subprocess.Popen(["sh", "-c", "sleep 300 & wait"],
                              start_new_session=True)
    group = launch.pid
    while not group_is_alive(group):
        time.sleep(0.05)

    started = time.monotonic()
    MODULE.end_process_group(launch, group, 2.0)

    assert not group_is_alive(group)
    # Returned on the group actually being gone, not after exhausting every
    # signal's settle window.
    assert time.monotonic() - started < 5.0
    assert launch.poll() is not None


def test_cleanup_of_a_run_that_left_nothing_behind_returns_quietly():
    launch = subprocess.Popen(["true"], start_new_session=True)
    group = launch.pid
    launch.wait()

    started = time.monotonic()
    MODULE.end_process_group(launch, group, 2.0)

    # An empty group cannot be signalled, which is the answer, not an error to
    # sit through three settle windows for.
    assert not group_is_alive(group)
    assert time.monotonic() - started < 1.0


def test_missing_measurements_are_recorded_as_absent_not_zero():
    record = MODULE.parse_run(f"{PREFIX}   VERDICT PASS\n", 0)

    assert record["free_cells"] is None
    assert record["host_healthy"] is None
    assert record["wall_overshoot_m"] is None


def test_random_spawn_requires_a_world():
    with pytest.raises(SystemExit):
        MODULE.parse_arguments(["--random-spawn"])


def test_spawn_pose_and_world_reach_the_launch_command():
    arguments = MODULE.parse_arguments([
        "--world", "/tmp/structured_loop_3d.sdf",
        "--world-profile", "structured_loop_3d",
    ])
    command = MODULE.launch_command(
        arguments, {"x": 1.25, "y": -2.5, "yaw": 0.75}
    )

    assert "world:=/tmp/structured_loop_3d.sdf" in command
    assert "world_profile:=structured_loop_3d" in command
    assert "spawn_x:=1.25" in command
    assert "spawn_y:=-2.5" in command
    assert "spawn_yaw:=0.75" in command


def random_spawn_arguments():
    return MODULE.parse_arguments([
        "--random-spawn", "--world", "/tmp/structured_loop_3d.sdf",
        "--runs", "2", "--minimum-evaluable-runs", "2",
    ])


def sampler_returning(monkeypatch, document):
    def fake_run(command, **kwargs):
        assert "safe_spawn_sampler" in command
        return subprocess.CompletedProcess(
            command, 0, stdout=json.dumps(document), stderr="")

    monkeypatch.setattr(MODULE.subprocess, "run", fake_run)


TWO_POSES = [{"x": 0.0, "y": 0.0, "z": 0.03, "yaw": 0.0},
             {"x": 3.0, "y": 0.0, "z": 0.03, "yaw": 1.0}]


def test_a_spawn_record_without_replay_parameters_is_refused(monkeypatch):
    # The pre-schema sampler emitted exactly this: enough to launch, not enough
    # to ever rerun. Recording it would produce a batch whose poses cannot be
    # reproduced and whose difference from the next batch cannot be explained.
    sampler_returning(monkeypatch, {"seed": 5, "poses": TWO_POSES})

    with pytest.raises(RuntimeError, match="schema_version"):
        MODULE.sample_spawns(random_spawn_arguments())


def test_a_spawn_record_that_drops_the_parameters_is_refused(monkeypatch):
    sampler_returning(monkeypatch, {
        "schema_version": 2, "seed": 5, "poses": TWO_POSES,
        "sampling_bounds": {"minimum_x": 0.0},
    })

    with pytest.raises(RuntimeError, match="could not be replayed"):
        MODULE.sample_spawns(random_spawn_arguments())


def test_a_complete_spawn_record_is_kept_whole(monkeypatch):
    document = {
        "schema_version": 2, "seed": 5, "poses": TWO_POSES,
        "parameters": {"safety_margin": 0.15}, "sampling_bounds": {"minimum_x": 0.0},
    }
    sampler_returning(monkeypatch, document)

    poses, record = MODULE.sample_spawns(random_spawn_arguments())

    assert poses == TWO_POSES
    # Whole, not summarised: the summary is what the next batch gets compared
    # against, and a field dropped here cannot be recovered from the logs.
    assert record == document


def test_source_revision_locates_the_tree_without_claiming_to_prove_it():
    revision = MODULE.source_revision()

    assert set(revision) == {"commit", "dirty"}
    # Absent rather than fabricated when git cannot answer: a made-up commit is
    # worse than a missing one, because it reads as provenance.
    if revision["commit"] is not None:
        assert len(revision["commit"]) == 40
        assert isinstance(revision["dirty"], bool)
