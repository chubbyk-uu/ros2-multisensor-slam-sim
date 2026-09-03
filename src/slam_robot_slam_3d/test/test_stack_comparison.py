from importlib.machinery import SourceFileLoader
import json
import math
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
TRAJECTORY = SourceFileLoader(
    "stack_trajectory_census", str(SCRIPTS / "stack_trajectory_census")
).load_module()
REPORT = SourceFileLoader(
    "stack_comparison_report", str(SCRIPTS / "stack_comparison_report")
).load_module()


def trajectory_report(label, **overrides):
    report = {
        "label": label,
        "trajectory_samples": 1800,
        "rmse_position_error_m": 0.04,
        "peak_position_error_m": 0.088,
        "final_position_error_m": 0.031,
        "rmse_yaw_error_deg": 0.33,
        "peak_yaw_error_deg": 1.1,
        "final_yaw_error_deg": 0.42,
        "pose_graph_commits": 3,
        "pose_graph_discards": 0,
        "pose_graph_failures": 0,
        "rtabmap_loop_closure_events": 0,
        "rtabmap_proximity_events": 0,
        "database_bytes": None,
        "process": {
            "pid_found": True,
            "samples": 300,
            "average_cpu_percent": 48.2,
            "maximum_cpu_percent": 91.0,
            "initial_rss_mb": 120.0,
            "final_rss_mb": 410.0,
            "peak_rss_mb": 430.0,
            "cpu_seconds": 170.0,
        },
    }
    report.update(overrides)
    return report


def test_anchoring_reports_drift_rather_than_the_frame_offset():
    # The estimate frame is translated by 10 m and rotated a quarter turn from
    # truth. An unanchored comparison would call that a 10 m error; anchoring
    # both trajectories to their own first sample must report no drift at all.
    truth_origin = (0.0, 0.0, 0.0)
    estimate_origin = (10.0, 10.0, math.pi / 2.0)
    truth = (2.0, 0.0, 0.0)
    # The same 2 m forward step expressed in the rotated estimate frame.
    estimate = (10.0, 12.0, math.pi / 2.0)

    position_error, yaw_error = TRAJECTORY.pose_error(
        estimate, truth, estimate_origin, truth_origin
    )

    assert position_error == pytest.approx(0.0, abs=1.0e-9)
    assert yaw_error == pytest.approx(0.0, abs=1.0e-9)


def test_anchoring_still_reports_a_real_divergence():
    truth_origin = estimate_origin = (0.0, 0.0, 0.0)

    position_error, yaw_error = TRAJECTORY.pose_error(
        (2.0, 0.3, 0.1), (2.0, 0.0, 0.0), estimate_origin, truth_origin
    )

    assert position_error == pytest.approx(0.3)
    assert yaw_error == pytest.approx(0.1)


def test_root_mean_square_is_not_the_mean():
    # A stack that is mostly accurate but occasionally far off must not be
    # scored as if the excursion were averaged away.
    assert TRAJECTORY.root_mean_square([0.0, 0.0, 0.0, 2.0]) == pytest.approx(1.0)
    assert TRAJECTORY.root_mean_square([]) is None


def test_cpu_percent_is_absent_rather_than_zero_before_two_samples():
    sampler = TRAJECTORY.ProcessSampler("/nothing_matches_this")

    assert sampler.average_cpu_percent() is None
    assert sampler.report()["pid_found"] is False
    assert sampler.report()["average_cpu_percent"] is None


def test_loop_events_are_rendered_without_a_ratio_column():
    # Committed pose-graph constraints and proximity detections count different
    # things, so a ratio between them would assert a comparison the numbers do
    # not support.
    rendered = REPORT.render(
        trajectory_report("custom"),
        trajectory_report("rtabmap", pose_graph_commits=0, rtabmap_proximity_events=7),
    )

    loop_section = rendered.split("loop events")[1].split("cost (")[0]
    assert "candidate/ref" not in loop_section
    assert "pose graph commits" in loop_section
    assert "RTAB-Map proximity" in loop_section
    # The trajectory section above it does carry ratios.
    assert "candidate/ref" in rendered.split("loop events")[0]


def test_an_unmeasured_stack_is_flagged_rather_than_scored_as_zero():
    absent = trajectory_report("rtabmap")
    absent["process"] = dict(absent["process"], pid_found=False)

    rendered = REPORT.render(trajectory_report("custom"), absent)

    assert "never located in /proc" in rendered
    assert "absent, not zero" in rendered


def test_a_stack_scored_over_less_of_the_replay_is_flagged():
    rendered = REPORT.render(
        trajectory_report("custom"),
        trajectory_report("rtabmap", trajectory_samples=900),
    )

    assert "sample counts differ by more than 10%" in rendered


def test_comparable_sample_counts_raise_no_warning():
    rendered = REPORT.render(
        trajectory_report("custom"),
        trajectory_report("rtabmap", trajectory_samples=1750),
    )

    assert "WARNING" not in rendered


def test_on_disk_map_size_is_reported_beside_resident_memory():
    # RTAB-Map keeps its map in sqlite, which never appears in RSS. Comparing
    # RSS alone would credit it for memory it moved to disk.
    rendered = REPORT.render(
        trajectory_report("custom"),
        trajectory_report("rtabmap", database_bytes=88 * 1024 * 1024),
    )

    assert "rtabmap keeps 88.0 MiB of map on disk" in rendered
    assert "RSS above does not include" in rendered
    # The custom stack keeps none, so no line is emitted for it.
    assert "custom keeps" not in rendered


def test_a_missing_measurement_prints_as_not_available():
    missing = trajectory_report("rtabmap")
    missing["process"] = dict(missing["process"], average_cpu_percent=None)

    rendered = REPORT.render(trajectory_report("custom"), missing)

    cost_section = rendered.split("cost (")[1]
    mean_cpu_row = [
        line for line in cost_section.splitlines() if line.startswith("mean CPU")
    ][0]
    assert "n/a" in mean_cpu_row


def test_the_grid_section_comes_from_map_projection_compare():
    # Loaded rather than reimplemented, so the two renderings cannot drift.
    module = REPORT.grid_renderer()

    assert hasattr(module, "render")
    assert module.ROWS[0][1] == "free_cells"


def test_the_merged_json_keeps_both_stacks_whole(tmp_path):
    reference = tmp_path / "custom.json"
    candidate = tmp_path / "rtabmap.json"
    output = tmp_path / "merged.json"
    reference.write_text(json.dumps(trajectory_report("custom")), encoding="utf-8")
    candidate.write_text(json.dumps(trajectory_report("rtabmap")), encoding="utf-8")

    arguments = REPORT.parse_arguments(
        [str(reference), str(candidate), "--output", str(output)]
    )
    merged = {
        "reference": {"trajectory": REPORT.load(arguments.reference), "grid": None},
        "candidate": {"trajectory": REPORT.load(arguments.candidate), "grid": None},
    }
    output.write_text(json.dumps(merged), encoding="utf-8")

    restored = json.loads(output.read_text(encoding="utf-8"))
    assert restored["reference"]["trajectory"]["label"] == "custom"
    assert restored["candidate"]["trajectory"]["pose_graph_commits"] == 3


def test_the_report_is_written_during_the_run_not_only_at_teardown(tmp_path):
    # A report produced only at teardown is lost whenever the launch is killed
    # rather than shut down, which is exactly when the numbers are wanted.
    census = TRAJECTORY.StackTrajectoryCensus.__new__(
        TRAJECTORY.StackTrajectoryCensus
    )
    output = tmp_path / "trajectory.json"
    census.arguments = TRAJECTORY.parse_arguments(
        ["--label", "custom", "--output", str(output)]
    )
    census.errors = [(1.0, 0.02, 0.01), (2.0, 0.03, 0.02)]
    census.truth_messages = 2
    census.truth_distance = 1.0
    census.lookup_failures = 0
    census.pose_graph_commits = 1
    census.pose_graph_discards = 0
    census.pose_graph_failures = 0
    census.info_messages = 0
    census.loop_closure_events = 0
    census.proximity_events = 0
    census.sampler = TRAJECTORY.ProcessSampler("/nothing")

    census.write()

    written = json.loads(output.read_text(encoding="utf-8"))
    assert written["trajectory_samples"] == 2
    assert written["peak_position_error_m"] == pytest.approx(0.03)
