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
DISTRIBUTION = SourceFileLoader(
    "stack_comparison_distribution", str(SCRIPTS / "stack_comparison_distribution")
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
            "complete_process_set": True,
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


def test_process_matching_ignores_command_line_options():
    # The census itself receives --process-match=/target. Searching the whole
    # cmdline would therefore select the census process instead of /target.
    assert TRAJECTORY.executable_matches("/opt/ros/lib/target", "/target")
    assert not TRAJECTORY.executable_matches(
        "/opt/ros/lib/stack_trajectory_census", "/target"
    )


def test_multi_process_sampling_sums_cpu_and_memory():
    sampler = TRAJECTORY.ProcessSampler(["/preprocessor", "/matcher"])
    sampler.pids = {"/preprocessor": 101, "/matcher": 202}
    readings = {
        101: (2.0, 100.0, 110.0),
        202: (3.5, 220.0, 240.0),
    }
    sampler.read_process = readings.__getitem__

    assert sampler.read() == pytest.approx((5.5, 320.0, 350.0))


def test_repeated_process_match_options_define_the_whole_stack():
    arguments = TRAJECTORY.parse_arguments(
        [
            "--output",
            "/tmp/report.json",
            "--process-match",
            "/preprocessor",
            "--process-match",
            "/matcher",
        ]
    )

    assert arguments.process_match == ["/preprocessor", "/matcher"]


def test_duplicate_process_match_tokens_are_rejected():
    with pytest.raises(ValueError, match="must be unique"):
        TRAJECTORY.ProcessSampler(["/matcher", "/matcher"])


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


def test_a_partial_process_set_is_flagged():
    partial = trajectory_report("custom")
    partial["process"] = dict(
        partial["process"], complete_process_set=False
    )

    rendered = REPORT.render(partial, trajectory_report("rtabmap"))

    assert "complete process set" in rendered
    assert "cost row is partial" in rendered


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


def test_a_single_run_is_not_dressed_up_as_a_distribution():
    assert DISTRIBUTION.spread([0.0113], "{:.4f}") == "0.0113"
    assert "[" not in DISTRIBUTION.spread([0.0113], "{:.4f}")


def test_repeated_runs_report_the_range_beside_the_centre():
    rendered = DISTRIBUTION.spread([0.0325, 0.0476, 0.0769], "{:.4f}")

    assert rendered == "0.0476 [0.0325, 0.0769]"


def test_the_centre_is_the_median_so_one_bad_replay_cannot_move_it():
    # A host hiccup on one of three runs must not invent a difference. The mean
    # here is 0.36; the median is the value the other two runs agree on.
    values = [0.03, 0.03, 1.02]

    assert DISTRIBUTION.spread(values, "{:.2f}") == "0.03 [0.03, 1.02]"


def test_a_run_that_measured_nothing_is_excluded_and_said_so():
    runs = [
        trajectory_report("custom"),
        trajectory_report("custom", trajectory_samples=0),
    ]

    rendered = DISTRIBUTION.render({"custom": runs})

    assert "1 of 2 runs usable" in rendered
    assert "narrower than what was observed" in rendered


def test_all_runs_usable_raises_no_exclusion_warning():
    runs = [trajectory_report("custom"), trajectory_report("custom")]

    rendered = DISTRIBUTION.render({"custom": runs})

    assert "2 of 2 runs usable" in rendered
    assert "WARNING" not in rendered


def test_the_headline_ratio_is_a_range_over_runs_not_one_number():
    # Reported best- and worst-case pairing, so a reader cannot quote a single
    # ratio that only one pairing of runs supports.
    profiles = {
        "custom": [
            trajectory_report("custom", rmse_position_error_m=0.010),
            trajectory_report("custom", rmse_position_error_m=0.012),
        ],
        "rtabmap": [
            trajectory_report("rtabmap", rmse_position_error_m=0.030),
            trajectory_report("rtabmap", rmse_position_error_m=0.048),
        ],
    }

    note = "\n".join(DISTRIBUTION.ratio_note(profiles))

    assert "2.50x to 4.80x" in note


def test_a_profile_specification_must_name_its_files():
    with pytest.raises(ValueError, match="expected LABEL=PATH"):
        DISTRIBUTION.parse_profiles(["custom"])
    with pytest.raises(ValueError, match="lists no files"):
        DISTRIBUTION.parse_profiles(["custom="])


def test_the_summary_json_keeps_every_run_not_just_the_centre(tmp_path):
    profiles = {
        "custom": [
            trajectory_report("custom", rmse_position_error_m=0.010),
            trajectory_report("custom", rmse_position_error_m=0.014),
            trajectory_report("custom", rmse_position_error_m=0.012),
        ]
    }

    summary = DISTRIBUTION.summarise(profiles)

    entry = summary["custom"]["metrics"]["rmse_position_error_m"]
    assert entry["median"] == pytest.approx(0.012)
    assert entry["minimum"] == pytest.approx(0.010)
    assert entry["maximum"] == pytest.approx(0.014)
    assert sorted(entry["values"]) == pytest.approx([0.010, 0.012, 0.014])
