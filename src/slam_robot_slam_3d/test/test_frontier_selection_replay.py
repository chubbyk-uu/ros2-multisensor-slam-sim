from importlib.machinery import SourceFileLoader
import math
from pathlib import Path

import pytest

MODULE = SourceFileLoader(
    "frontier_selection_replay",
    str(Path(__file__).parents[1] / "scripts" / "frontier_selection_replay"),
).load_module()

# Verbatim shape of captured output, prefix included: ros2 launch stamps every
# line with the node name, and a pattern anchored at the line start silently
# matches nothing.
PREFIX = "[frontier_explorer_node-20] [INFO] [1786452284.719993713] [frontier_explorer]:"
RUN = f"""\
{PREFIX} FRONTIER SELECTION decision=1 candidates=2 pool=1 rank=1 seed=20260811 \
chosen_score=35.140
{PREFIX} FRONTIER CANDIDATE decision=1 index=0 x=4.025 y=0.775 score=35.140 \
gain=17.750 clearance=0.621 distance=4.099 path=4.338 cells=355 chosen=1
{PREFIX} FRONTIER CANDIDATE decision=1 index=1 x=-0.375 y=3.775 score=11.912 \
gain=6.100 clearance=1.021 distance=3.794 path=4.013 cells=122 chosen=0
{PREFIX} FRONTIER SELECTION decision=2 candidates=3 pool=2 rank=2 seed=20260811 \
chosen_score=6.559
{PREFIX} FRONTIER CANDIDATE decision=2 index=0 x=10.125 y=-0.075 score=6.906 \
gain=3.700 clearance=1.300 distance=6.165 path=6.310 cells=74 chosen=0
{PREFIX} FRONTIER CANDIDATE decision=2 index=1 x=0.475 y=5.675 score=6.559 \
gain=3.600 clearance=0.941 distance=6.136 path=8.576 cells=72 chosen=1
{PREFIX} FRONTIER CANDIDATE decision=2 index=2 x=1.375 y=6.125 score=0.559 \
gain=0.650 clearance=0.550 distance=6.069 path=9.853 cells=13 chosen=0
"""


def test_records_are_parsed_from_launch_prefixed_output():
    decisions = MODULE.parse_decisions(RUN, "run.log")

    assert [decision["decision"] for decision in decisions] == [1, 2]
    assert decisions[1]["candidates"][1]["chosen"] is True
    assert decisions[1]["candidates"][2]["path"] == pytest.approx(9.853)
    assert decisions[0]["candidates"][0]["cells"] == 355


def test_a_decision_missing_candidates_is_dropped_rather_than_half_read():
    # A run killed mid-decision writes the header and some of its candidates.
    # Scoring a rule against a truncated candidate list would silently compare
    # it against a choice that never existed.
    truncated = RUN.rsplit("\n", 2)[0] + "\n"

    decisions = MODULE.parse_decisions(truncated, "run.log")

    assert [decision["decision"] for decision in decisions] == [1]


def test_pooled_runs_do_not_collapse_onto_each_other():
    # Every run numbers its decisions from one, so keying on that number folds
    # concatenated runs together. Measured: three pooled runs parsed as a
    # single decision, which reads as a working tool producing a tiny sample.
    pooled = RUN + RUN + RUN

    decisions = MODULE.parse_decisions(pooled, "pooled.log")

    assert len(decisions) == 6
    assert [decision["decision"] for decision in decisions] == [1, 2, 1, 2, 1, 2]
    assert all(
        len(decision["candidates"]) == decision["expected_candidates"]
        for decision in decisions
    )


def test_a_candidate_from_another_decision_is_not_absorbed():
    stray = (
        f"{PREFIX} FRONTIER SELECTION decision=1 candidates=1 pool=1 rank=1 "
        "seed=1 chosen_score=5.000\n"
        f"{PREFIX} FRONTIER CANDIDATE decision=9 index=0 x=0.000 y=0.000 "
        "score=5.000 gain=1.000 clearance=1.000 distance=1.000 path=1.000 "
        "cells=9 chosen=1\n"
    )

    assert MODULE.parse_decisions(stray, "run.log") == []


def test_negative_scores_survive_the_round_trip():
    text = (
        f"{PREFIX} FRONTIER SELECTION decision=1 candidates=1 pool=1 rank=1 "
        "seed=1 chosen_score=-2.500\n"
        f"{PREFIX} FRONTIER CANDIDATE decision=1 index=0 x=-1.500 y=-2.000 "
        "score=-2.500 gain=0.100 clearance=0.200 distance=8.000 path=9.000 "
        "cells=8 chosen=1\n"
    )

    decisions = MODULE.parse_decisions(text, "run.log")

    assert decisions[0]["candidates"][0]["score"] == pytest.approx(-2.5)
    assert decisions[0]["candidates"][0]["x"] == pytest.approx(-1.5)


def test_the_band_admits_only_what_is_within_the_fraction_of_the_best():
    assert MODULE.band_probabilities([10.0, 9.0, 8.0], 0.05) == [1.0, 0.0, 0.0]
    assert MODULE.band_probabilities([10.0, 9.0, 8.0], 0.25) == [
        pytest.approx(1 / 3)] * 3
    # Independent of the worst candidate, matching the deployed rule.
    assert MODULE.band_probabilities([10.0, 9.0, 8.0, 0.0], 0.05)[:3] == [
        1.0, 0.0, 0.0]


def test_top_k_ignores_how_far_apart_the_candidates_are():
    # The reason to consider it: it engages at every decision with enough
    # candidates, however dominant the best one is.
    assert MODULE.top_k_probabilities([100.0, 1.0, 0.5], 2) == [0.5, 0.5, 0.0]
    assert MODULE.top_k_probabilities([1.0], 3) == [1.0]


def test_softmax_leaves_the_best_most_likely_and_never_excludes_a_candidate():
    probabilities = MODULE.softmax_probabilities([10.0, 9.0, 1.0], 1.0)

    assert sum(probabilities) == pytest.approx(1.0)
    assert probabilities[0] > probabilities[1] > probabilities[2]
    assert probabilities[2] > 0.0


def test_a_cold_softmax_becomes_greedy_and_a_hot_one_becomes_uniform():
    cold = MODULE.softmax_probabilities([10.0, 9.0, 1.0], 0.01)
    hot = MODULE.softmax_probabilities([10.0, 9.0, 1.0], 1.0e6)

    assert cold[0] == pytest.approx(1.0, abs=1e-6)
    assert hot == [pytest.approx(1 / 3, abs=1e-3)] * 3


def test_softmax_is_invariant_to_shifting_every_score():
    # Scores subtract a travel cost and can sit anywhere on the axis, so the
    # rule must depend on the gaps between candidates, not their offset.
    base = MODULE.softmax_probabilities([10.0, 9.0, 1.0], 2.0)
    shifted = MODULE.softmax_probabilities([-90.0, -91.0, -99.0], 2.0)

    for first, second in zip(base, shifted):
        assert first == pytest.approx(second)


def test_perplexity_counts_choices_that_are_actually_being_made():
    # Three equal candidates really are three choices; a distribution that
    # almost always takes the best is close to one, even though three
    # candidates carry non-zero probability.
    assert MODULE.perplexity([1 / 3, 1 / 3, 1 / 3]) == pytest.approx(3.0)
    assert MODULE.perplexity([1.0, 0.0, 0.0]) == pytest.approx(1.0)
    assert MODULE.perplexity([0.98, 0.01, 0.01]) < 1.2


def test_the_cost_of_a_rule_is_measured_against_the_greedy_choice():
    decisions = MODULE.parse_decisions(RUN, "run.log")

    greedy = MODULE.evaluate(decisions, lambda scores: MODULE.top_k_probabilities(
        scores, 1))
    both = MODULE.evaluate(decisions, lambda scores: MODULE.top_k_probabilities(
        scores, 2))

    assert greedy["engagement"] == 0.0
    assert greedy["extra_path_m"] == pytest.approx(0.0)
    assert greedy["score_deficit"] == pytest.approx(0.0)
    # Decision 2 trades 6.310 m of path for 8.576 m half the time, and decision
    # 1 trades 4.338 m for 4.013 m, so the average moves by their mean halved.
    assert both["extra_path_m"] == pytest.approx(
        ((4.013 - 4.338) / 2 + (8.576 - 6.310) / 2) / 2)
    assert both["engagement"] == pytest.approx(0.5)


def test_perplexity_separates_a_wide_band_from_a_warm_softmax():
    decisions = MODULE.parse_decisions(RUN, "run.log")

    band = MODULE.evaluate(
        decisions, lambda scores: MODULE.band_probabilities(scores, 0.05))
    softmax = MODULE.evaluate(
        decisions, lambda scores: MODULE.softmax_probabilities(scores, 1.0))

    # The band never engages on decision 1 because the best leads by threefold;
    # softmax always carries some probability, so its perplexity is higher even
    # when it nearly always takes the best.
    assert band["perplexity"] < softmax["perplexity"]


def test_defaults_sweep_more_than_one_value_of_each_rule():
    arguments = MODULE.parse_arguments(["a.log"])

    assert len(arguments.band) > 1
    assert len(arguments.top_k) > 1
    assert len(arguments.temperature) > 1
    assert math.isclose(arguments.band[0], 0.20)
