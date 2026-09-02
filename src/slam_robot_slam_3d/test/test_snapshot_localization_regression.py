from importlib.machinery import SourceFileLoader
import os
from pathlib import Path

import pytest

MODULE = SourceFileLoader(
    "snapshot_localization_regression",
    str(Path(__file__).parents[1] / "scripts" / "snapshot_localization_regression"),
).load_module()


def record_report(**overrides):
    report = {
        "map_frame_samples": 600,
        "peak_position_error_m": 0.030,
        "peak_yaw_error_deg": 0.20,
        "last_global_keyframes": 84,
        "final_known_cells": 60000,
        "local_map_points": 41000,
        "maximum_local_keyframes": 12,
        "snapshot_save": {"success": True, "message": "saved"},
    }
    report.update(overrides)
    return report


def localize_report(**overrides):
    report = {
        "map_frame_samples": 600,
        "peak_position_error_m": 0.045,
        "peak_yaw_error_deg": 0.30,
        "join_peak_position_error_m": 0.020,
        "join_peak_yaw_error_deg": 0.10,
        "operation_mode": "localization",
        "first_snapshot_loaded": True,
        "first_global_keyframes": 84,
        "last_global_keyframes": 84,
        "pose_graph_commits": 0,
        "submap_reinitializations": 0,
        "distinct_local_map_sizes": 1,
        "local_map_points": 380000,
        "maximum_local_keyframes": 0,
        "match_attempts": 900,
        "match_accepted": 890,
        "match_acceptance_rate": 0.9889,
        "restored_known_cells": 59000,
    }
    report.update(overrides)
    return report


# A sentinel, so a test can say "the file was missing" with None and still be
# told apart from a test that did not care about the fingerprint at all.
UNCHANGED = object()


def verdict(record=None, localize=None, before=UNCHANGED, after=UNCHANGED, argv=None):
    checks = MODULE.evaluate(
        record or record_report(),
        localize or localize_report(),
        [1024, 111] if before is UNCHANGED else before,
        [1024, 111] if after is UNCHANGED else after,
        MODULE.parse_arguments(argv or []),
    )
    return {entry["name"]: entry["passed"] for entry in checks}


def test_a_healthy_localization_run_passes_every_criterion():
    assert all(verdict().values())


def test_mapping_mode_masquerading_as_localization_is_rejected():
    # The launch plumbing is what selects the mode, and a phase that quietly
    # ran as mapping would satisfy the accuracy checks while testing the branch
    # this regression exists for not at all.
    assert not verdict(localize=localize_report(operation_mode="mapping"))[
        "ran_in_localization_mode"]


def test_a_rolling_submap_instead_of_the_prior_map_is_rejected():
    # Localization replaces the local map with the whole restored cloud, so it
    # has no local keyframes behind it. Any means the restore took the mapping
    # branch.
    checks = verdict(localize=localize_report(
        maximum_local_keyframes=12, local_map_points=40000))

    assert not checks["match_target_is_the_restored_map"]


def test_dead_reckoning_is_rejected_even_when_the_pose_looks_right():
    # The failure this criterion exists for: every match fails, the front end
    # falls back to the odometry prediction, and over one lap of a fixed bag
    # the trajectory still looks close enough to pass an accuracy limit.
    checks = verdict(localize=localize_report(
        match_attempts=900, match_accepted=90, match_acceptance_rate=0.10))

    assert not checks["localized_by_matching_not_dead_reckoning"]
    assert checks["localized_accuracy_holds"]


def test_any_write_to_the_prior_map_is_rejected():
    for overrides in (
        {"distinct_local_map_sizes": 2},
        {"last_global_keyframes": 85},
        {"pose_graph_commits": 1},
        {"submap_reinitializations": 1},
    ):
        checks = verdict(localize=localize_report(**overrides))
        assert not checks["the_prior_map_was_never_written_to"], overrides


def test_rewriting_the_snapshot_file_is_rejected():
    # The guard being wrong would overwrite the very map the robot was
    # localizing against, so this is measured on the file rather than on a log
    # line that says a save was skipped.
    assert not verdict(before=[1024, 111], after=[1024, 222])[
        "the_snapshot_file_was_not_rewritten"]
    assert not verdict(before=[1024, 111], after=[2048, 111])[
        "the_snapshot_file_was_not_rewritten"]
    # A snapshot that vanished is not "unchanged" either.
    assert not verdict(before=None, after=None)[
        "the_snapshot_file_was_not_rewritten"]


def test_a_localization_run_that_drifts_is_rejected():
    checks = verdict(localize=localize_report(peak_position_error_m=0.400))

    assert not checks["localized_accuracy_holds"]
    # The jump check is about the join alone, so it is untouched by late drift.
    assert checks["localized_without_a_jump"]


def test_a_jump_at_the_join_is_rejected_separately_from_drift():
    checks = verdict(localize=localize_report(join_peak_position_error_m=0.400))

    assert not checks["localized_without_a_jump"]


def test_a_run_that_never_restored_is_rejected():
    assert not verdict(localize=localize_report(first_snapshot_loaded=False))[
        "snapshot_restored"]
    assert not verdict(localize=localize_report(first_global_keyframes=10))[
        "snapshot_restored"]


def test_a_phase_with_too_few_pose_samples_cannot_report_accuracy():
    assert not verdict(localize=localize_report(map_frame_samples=5))[
        "phases_measured_enough_poses"]


def test_the_criteria_are_written_down_before_the_numbers_arrive():
    arguments = MODULE.parse_arguments([])

    assert arguments.minimum_match_acceptance_rate == 0.80
    assert arguments.split == 240.0
    assert arguments.maximum_join_position_error_m == 0.10


def test_impossible_criteria_are_refused():
    for argv in (
        ["--split", "0"],
        ["--rate", "0"],
        ["--window", "-1"],
        ["--minimum-match-acceptance-rate", "0"],
        ["--minimum-match-acceptance-rate", "1.5"],
    ):
        with pytest.raises(SystemExit):
            MODULE.parse_arguments(argv)


def test_the_localize_phase_restores_the_snapshot_it_was_given():
    arguments = MODULE.parse_arguments([])

    command = MODULE.phase_command(
        arguments, "localize", "/tmp/l.json", "/tmp/r.json", "/tmp/snap")

    assert "phase:=localize" in command
    assert "snapshot_path:=/tmp/snap" in command
    assert "previous_report:=/tmp/r.json" in command


def test_a_missing_previous_report_is_omitted_rather_than_passed_empty():
    # ros2 launch rejects an empty value instead of falling back to the
    # argument's default.
    command = MODULE.phase_command(
        MODULE.parse_arguments([]), "record", "/tmp/r.json", "", "/tmp/snap")

    assert not any(part.startswith("previous_report:=") for part in command)


def test_the_fingerprint_sees_a_rewrite_that_kept_the_length(tmp_path):
    # The whole reason this is a content hash. A rewrite that lands on the same
    # length inside one filesystem timestamp tick is indistinguishable by size
    # and mtime, and it is the case that matters: the file would still be a
    # snapshot, just not the one the robot was localizing against.
    original = tmp_path / "snapshot"
    original.write_bytes(b"\x01" * 4096)
    before = MODULE.snapshot_fingerprint(original)

    stat = original.stat()
    original.write_bytes(b"\x01" * 4095 + b"\x02")
    os.utime(original, ns=(stat.st_atime_ns, stat.st_mtime_ns))

    assert original.stat().st_size == before[0]
    assert original.stat().st_mtime_ns == stat.st_mtime_ns
    assert MODULE.snapshot_fingerprint(original) != before


def test_a_missing_snapshot_has_no_fingerprint(tmp_path):
    assert MODULE.snapshot_fingerprint(tmp_path / "absent") is None
