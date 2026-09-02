from importlib.machinery import SourceFileLoader
from pathlib import Path

MODULE = SourceFileLoader(
    "nav2_fault_injection_regression",
    str(Path(__file__).parents[1] / "scripts" / "nav2_fault_injection_regression"),
).load_module()


def arguments(scenario, wall_timeout=300.0):
    # Built through the real parser, so every default under test is the one the
    # run would use. A hand-written Namespace is a second copy that drifts.
    return MODULE.parse_arguments([
        "--scenario", scenario,
        "--world", "/tmp/slam_world.sdf",
        "--world-profile", "slam_world",
        "--wall-timeout", str(wall_timeout),
        "--log", "x.log",
    ])


STARTUP_LOG = """\
[frontier_exploration_regression-9]   exploration fault: class=dependency_lost \
code=nav2_startup_timeout error_level=yes reason=Nav2 never accepted a goal
[frontier_exploration_regression-9]   VERDICT INFRA_UNSTABLE
"""
RUNTIME_LOG = STARTUP_LOG.replace("nav2_startup_timeout", "nav2_runtime_lost")


def test_the_startup_scenario_accepts_only_a_startup_timeout():
    checks = MODULE.judge(MODULE.STARTUP, STARTUP_LOG, 40.0, arguments(MODULE.STARTUP))

    assert all(checks.values()), checks


def test_the_startup_scenario_rejects_a_runtime_loss():
    # The two are not interchangeable. A run that reports runtime loss when
    # nothing ever accepted a goal means the "ever operational" fact is wrong,
    # which is exactly the discriminator the two codes exist to carry.
    checks = MODULE.judge(MODULE.STARTUP, RUNTIME_LOG, 40.0, arguments(MODULE.STARTUP))

    assert not checks["fault_code_names_the_right_failure"]
    assert checks["fault_is_a_dependency_loss"]


def test_the_runtime_scenario_accepts_only_a_runtime_loss():
    checks = MODULE.judge(MODULE.RUNTIME, RUNTIME_LOG, 40.0, arguments(MODULE.RUNTIME))

    assert all(checks.values()), checks


def test_a_run_that_used_its_whole_budget_fails_even_with_the_right_code():
    # The fault path exists to end the run. Reporting the right classification
    # after sitting out the full timeout means the early exit never happened,
    # and a campaign would still pay the whole wall clock for it.
    checks = MODULE.judge(
        MODULE.RUNTIME, RUNTIME_LOG, 290.0, arguments(MODULE.RUNTIME))

    assert checks["fault_code_names_the_right_failure"]
    assert not checks["run_ended_well_before_its_budget"]


def test_a_silent_run_fails_every_criterion_that_needs_evidence():
    checks = MODULE.judge(MODULE.RUNTIME, "nothing happened", 10.0,
                          arguments(MODULE.RUNTIME))

    assert not checks["explorer_declared_a_fault"]
    assert not checks["regression_reported_a_verdict"]
    assert not checks["regression_classified_it_as_infrastructure"]


def test_a_core_failure_verdict_is_not_accepted_as_infrastructure():
    # The inverted failure this whole scenario exists to catch: the fault is
    # declared but the regression still records it against the algorithm.
    text = STARTUP_LOG.replace("VERDICT INFRA_UNSTABLE", "VERDICT FAIL")
    checks = MODULE.judge(MODULE.STARTUP, text, 40.0, arguments(MODULE.STARTUP))

    assert checks["explorer_declared_a_fault"]
    assert not checks["regression_classified_it_as_infrastructure"]


def test_only_the_startup_scenario_holds_nav2_inactive():
    startup = MODULE.scenario_command(arguments(MODULE.STARTUP))
    runtime = MODULE.scenario_command(arguments(MODULE.RUNTIME))

    assert "nav2_autostart:=false" in startup
    # The runtime scenario needs a working Nav2 first; holding it inactive
    # would collapse it into the startup scenario.
    assert "nav2_autostart:=false" not in runtime
    assert "world_profile:=slam_world" in runtime


def test_the_scenario_outcomes_are_distinct():
    # A run that could not inject its fault has to be told apart from one that
    # injected it and saw the wrong answer: the first asks for a retry, the
    # second asks for an investigation.
    assert len({MODULE.PASS, MODULE.FAIL, MODULE.VOID}) == 3


def test_leftover_processes_are_the_same_ones_the_campaign_refuses():
    campaign = SourceFileLoader(
        "exploration_campaign",
        str(Path(__file__).parents[1] / "scripts" / "exploration_campaign"),
    ).load_module()

    # Two simulators on one host is what produced the hung Nav2 bringup that
    # made this check necessary, and it is the same hazard either entry point
    # is exposed to.
    assert set(MODULE.LEFTOVER_PROCESS_PATTERNS) == set(
        campaign.LEFTOVER_PROCESS_PATTERNS)


FROZEN_CLOCK_LOG = RUNTIME_LOG + (
    "[frontier_explorer_node-20] [ERROR] [1786511359.829575756] "
    "[frontier_explorer]: EXPLORATION ABORTED\n"
)


def test_the_frozen_clock_criterion_needs_the_abort_to_come_after_the_pause():
    early = MODULE.judge(
        MODULE.RUNTIME, FROZEN_CLOCK_LOG, 40.0, arguments(MODULE.RUNTIME),
        paused_at=1786511350.0)
    late = MODULE.judge(
        MODULE.RUNTIME, FROZEN_CLOCK_LOG, 40.0, arguments(MODULE.RUNTIME),
        paused_at=1786511400.0)

    # Fault well after the pause: simulation time had stopped, so the deadline
    # that expired can only have been the steady one.
    assert early["fault_fired_while_simulation_time_was_frozen"]
    # Fault before the pause proves nothing about which clock was used.
    assert not late["fault_fired_while_simulation_time_was_frozen"]


def test_an_abort_too_close_to_the_pause_decides_nothing():
    # Both instants are system-clock readings, and this host steps its system
    # clock back by 1.2-1.7 s when WSL recalibrates. An abort landing inside
    # that window would have its ordering decided by the recalibration rather
    # than by which clock the deadline used, so it must not count as proof.
    # nav2_runtime_grace is 5 s, so a healthy run is nowhere near this edge.
    checks = MODULE.judge(
        MODULE.RUNTIME, FROZEN_CLOCK_LOG, 40.0, arguments(MODULE.RUNTIME),
        paused_at=1786511359.0)

    assert not checks["fault_fired_while_simulation_time_was_frozen"]


def test_the_clock_jump_allowance_covers_the_step_this_host_makes():
    assert MODULE.parse_arguments(
        ["--scenario", MODULE.RUNTIME]).clock_jump_allowance >= 1.7


def test_the_startup_scenario_does_not_claim_the_frozen_clock_property():
    checks = MODULE.judge(
        MODULE.STARTUP, STARTUP_LOG, 40.0, arguments(MODULE.STARTUP))

    # It never pauses Gazebo, so it must not appear to have tested this.
    assert "fault_fired_while_simulation_time_was_frozen" not in checks
