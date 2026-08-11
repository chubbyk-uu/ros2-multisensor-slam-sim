from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest

MODULE = SourceFileLoader(
    "snapshot_resume_regression",
    str(Path(__file__).parents[1] / "scripts" / "snapshot_resume_regression"),
).load_module()

PROBE = SourceFileLoader(
    "snapshot_resume_probe",
    str(Path(__file__).parents[1] / "scripts" / "snapshot_resume_probe"),
).load_module()

STATIC_TF = SourceFileLoader(
    "recorded_static_tf_publisher",
    str(Path(__file__).parents[1] / "scripts" / "recorded_static_tf_publisher"),
).load_module()

REFERENCE_BAG = (
    Path(__file__).resolve().parents[3] / "bags" / "structured_3d_reference"
)


def record_report(**overrides):
    report = {
        "phase": "record",
        "map_frame_samples": 1800,
        "peak_position_error_m": 0.11,
        "peak_yaw_error_deg": 0.80,
        "last_global_keyframes": 60,
        "final_known_cells": 120000,
        "pose_graph_commits": 2,
        "pose_graph_discards": 0,
        "pose_graph_failures": 0,
        "snapshot_save": {"success": True, "message": "/tmp/snapshot"},
        "anchor": {"estimate": [0.0, 0.0, 0.0], "truth": [0.0, 0.0, 0.0]},
    }
    report.update(overrides)
    return report


def resume_report(**overrides):
    report = {
        "phase": "resume",
        "map_frame_samples": 1700,
        "peak_position_error_m": 0.14,
        "peak_yaw_error_deg": 0.90,
        "join_peak_position_error_m": 0.07,
        "join_peak_yaw_error_deg": 0.40,
        "first_snapshot_loaded": True,
        "first_global_keyframes": 60,
        "last_global_keyframes": 95,
        "restored_known_cells": 118000,
        "final_known_cells": 119000,
        "map_messages": 240,
        "pose_graph_commits": 1,
        "pose_graph_discards": 0,
        "pose_graph_failures": 0,
        "snapshot_save": None,
    }
    report.update(overrides)
    return report


def failures(record, resume, arguments=None):
    arguments = arguments or MODULE.parse_arguments([])
    return {
        entry["name"]
        for entry in MODULE.evaluate(record, resume, arguments)
        if not entry["passed"]
    }


def test_a_clean_save_and_resume_passes_every_check():
    assert failures(record_report(), resume_report()) == set()


def test_a_resumed_session_that_lands_in_the_wrong_frame_fails():
    # The defect the whole regression exists for: restored poses left in the
    # front end's own frame put the resumed session an accumulated correction
    # away from where the saved one stopped.
    assert "resumed_without_a_jump" in failures(
        record_report(),
        resume_report(join_peak_position_error_m=1.60, peak_position_error_m=1.80),
    )


def test_a_snapshot_that_did_not_load_fails_even_with_a_good_trajectory():
    # Mapping from scratch would trace ground truth just as well; what makes it
    # a resume is starting with the keyframes that were saved.
    assert "snapshot_restored" in failures(
        record_report(),
        resume_report(first_snapshot_loaded=False, first_global_keyframes=0),
    )


def test_a_resumed_session_that_forgot_the_saved_map_fails():
    assert "restored_map_rebuilt" in failures(
        record_report(), resume_report(restored_known_cells=4000)
    )


def test_a_session_that_only_replays_the_saved_map_fails():
    # Restoring and then standing still satisfies every continuity check, so
    # the keyframe count has to grow for the run to count as mapping.
    assert "mapping_continued" in failures(
        record_report(), resume_report(last_global_keyframes=60)
    )


def test_a_resumed_session_is_not_asked_to_cover_new_ground():
    # The reference dataset is two laps of one loop, so a resumed phase
    # re-traverses mapped ground and adds almost no coverage. Known cells are
    # not monotonic either: a cell in the band between free and occupied
    # publishes as unknown again. Requiring growth there would fail a healthy
    # run for a property the dataset cannot supply.
    assert "mapping_continued" not in failures(
        record_report(),
        resume_report(restored_known_cells=79260, final_known_cells=79227),
    )


def test_a_map_that_collapses_after_the_restart_still_fails():
    assert "mapping_continued" in failures(
        record_report(),
        resume_report(restored_known_cells=79260, final_known_cells=10000),
    )


def test_a_resumed_session_that_stopped_publishing_maps_fails():
    assert "mapping_continued" in failures(
        record_report(), resume_report(map_messages=1)
    )


def test_drift_introduced_after_the_restart_fails_against_the_saved_phase():
    # Judged against the same run's first phase rather than an absolute bound:
    # the two phases see the same dataset, so the recorded phase is the fairest
    # control for what this front end achieves on it.
    assert "resumed_accuracy_holds" in failures(
        record_report(peak_position_error_m=0.11),
        resume_report(peak_position_error_m=0.40),
    )


def test_a_run_that_never_optimised_is_reported_as_untested_not_passed():
    # Both directions matter. Without a commit before the save the frame
    # correction is identity and rebasing does nothing; without one after the
    # restart the join edge never reaches the optimiser.
    assert "recorded_phase_committed_an_optimisation" in failures(
        record_report(pose_graph_commits=0), resume_report()
    )
    assert "resumed_phase_committed_an_optimisation" in failures(
        record_report(), resume_report(pose_graph_commits=0)
    )


def test_a_phase_that_measured_almost_nothing_cannot_pass():
    assert "phases_measured_enough_poses" in failures(
        record_report(), resume_report(map_frame_samples=12)
    )


def test_a_failed_save_is_reported_before_anything_else():
    checks = MODULE.evaluate(
        record_report(snapshot_save={"success": False, "message": "no keyframes"}),
        resume_report(),
        MODULE.parse_arguments([]),
    )

    assert checks[0]["name"] == "snapshot_written"
    assert not checks[0]["passed"]


def test_missing_measurements_fail_rather_than_being_read_as_zero():
    # A phase that crashed early writes nulls. Treating those as zero would
    # score a run that measured nothing as a perfect one.
    assert failures(
        record_report(peak_position_error_m=None),
        resume_report(join_peak_position_error_m=None, restored_known_cells=None),
    ) >= {"resumed_without_a_jump", "restored_map_rebuilt", "resumed_accuracy_holds"}


def test_the_verdict_line_reports_the_worst_check():
    passing = MODULE.report_lines(MODULE.evaluate(
        record_report(), resume_report(), MODULE.parse_arguments([])))
    failing = MODULE.report_lines(MODULE.evaluate(
        record_report(), resume_report(pose_graph_failures=1),
        MODULE.parse_arguments([])))

    assert passing[-1] == "  VERDICT PASS"
    assert failing[-1] == "  VERDICT FAIL"


def test_the_recorded_phase_stops_at_the_split_and_the_resumed_phase_starts_there():
    arguments = MODULE.parse_arguments(["--split", "150.0"])

    recorded = MODULE.phase_command(arguments, "record", "a.json", "", "snap")
    resumed = MODULE.phase_command(arguments, "resume", "b.json", "a.json", "snap")

    assert "phase:=record" in recorded and "split:=150.0" in recorded
    assert "phase:=resume" in resumed and "previous_report:=a.json" in resumed
    # ros2 launch rejects an empty argument value, so the first phase must not
    # carry the one it has no report to point at.
    assert not any(item.startswith("previous_report:=") for item in recorded)


def test_a_rerun_cannot_be_judged_on_the_previous_run_s_reports(tmp_path, monkeypatch):
    # Reusing the directory used to leave the reports in place. A phase that
    # crashed before writing one would then be scored on the last run's
    # numbers, which is how a broken run reports a pass.
    directory = tmp_path / "reports"
    directory.mkdir()
    (directory / "record.json").write_text("{}")
    (directory / "resume.json").write_text("{}")
    monkeypatch.setattr(MODULE, "run_phase", lambda *arguments, **keywords: 0)

    code = MODULE.main([
        "--bag", str(tmp_path), "--report-directory", str(directory),
        "--overwrite-report-directory",
    ])

    assert code == 1
    assert not (directory / "record.json").exists()
    assert not (directory / "resume.json").exists()


def test_impossible_run_parameters_are_refused():
    for argv in (["--split", "0"], ["--rate", "0"], ["--window", "-1"]):
        with pytest.raises(SystemExit):
            MODULE.parse_arguments(argv)


def test_defaults_are_written_down_before_the_numbers_arrive():
    arguments = MODULE.parse_arguments([])

    # Past the first lap of the reference dataset on purpose: an earlier split
    # leaves the recorded phase with no loop closure, and the run then reports
    # that it tested nothing rather than passing.
    assert arguments.split == 240.0
    assert arguments.maximum_join_position_error_m == 0.10
    assert arguments.maximum_join_yaw_error_deg == 0.50
    assert arguments.minimum_restored_known_cell_ratio == 0.90


def test_the_resumed_phase_is_measured_in_the_frame_the_saved_one_left():
    # Anchoring each phase to its own first sample is what every other
    # trajectory check here does, and it is exactly wrong for this one: a
    # restart that resumed a metre away would have that metre absorbed into the
    # new origin and measure as no error at all.
    saved_anchor = (0.0, 0.0, 0.0)
    truth_anchor = (0.0, 0.0, 0.0)
    displaced_estimate = (1.0, 0.0, 0.0)
    truth = (0.0, 0.0, 0.0)

    position, yaw = PROBE.pose_error(
        displaced_estimate, truth, saved_anchor, truth_anchor
    )

    assert position == pytest.approx(1.0)
    assert yaw == pytest.approx(0.0)


@pytest.mark.skipif(
    not REFERENCE_BAG.exists(), reason="the fixed 3D dataset is not present"
)
def test_the_sensor_extrinsic_a_resumed_phase_seeks_past_is_read_from_the_bag():
    # Measured on the dataset the regression runs: the resumed phase starts
    # past the single /tf_static message, and without this the front end waits
    # for lidar_3d_link forever while reporting a successful restore.
    transforms = STATIC_TF.recorded_transforms(str(REFERENCE_BAG), "/tf_static")

    assert {item.child_frame_id for item in transforms} >= {"lidar_3d_link"}


def test_the_join_window_is_measured_from_the_first_resumed_sample():
    samples = [(100.0, 0.05, 0.1), (105.0, 0.06, 0.2), (140.0, 0.90, 3.0)]

    position, yaw, count = PROBE.peak(samples, until=115.0)

    # The late sample belongs to the rest of the phase, not to the join.
    assert position == pytest.approx(0.06)
    assert yaw == pytest.approx(0.2)
    assert count == 2
