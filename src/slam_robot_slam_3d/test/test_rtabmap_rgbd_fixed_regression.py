from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "rtabmap_rgbd_fixed_regression_check"
LOADER = SourceFileLoader("rtabmap_rgbd_fixed_regression_check", str(SCRIPT))
SPEC = importlib.util.spec_from_loader("rtabmap_rgbd_fixed_regression_check", LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid_reports(input_rmse=0.60):
    trajectory = {
        "truth_distance_m": 143.0,
        "trajectory_samples": 18000,
        "input_odom_samples": 18000,
        "rtabmap_visual_frames": 720,
        "rtabmap_frames_with_visual_words": 620,
        "rtabmap_info_messages": 720,
        "rtabmap_maximum_visual_inliers": 80,
        "rtabmap_loop_closure_events": 100,
        "rmse_position_error_m": 0.20,
        "peak_position_error_m": 0.70,
        "final_position_error_m": 0.04,
        "rmse_yaw_error_deg": 1.0,
        "peak_yaw_error_deg": 3.0,
        "rtabmap_processing_p95_ms": 90.0,
        "database_bytes": 250_000_000,
        "input_odom_rmse_position_error_m": input_rmse,
        "input_odom_final_position_error_m": 1.20,
        "process": {
            "complete_process_set": True,
            "average_cpu_percent": 40.0,
            "peak_rss_mb": 1250.0,
        },
    }
    grid = {
        "updates": 677,
        "free_cells": 73000,
        "occupied_cells": 3500,
        "known_bounding_box_m2": 405.0,
    }
    return trajectory, grid


def test_complete_textured_replay_passes_and_requires_drift_improvement():
    trajectory, grid = valid_reports()

    checks, coverage = MODULE.evaluate(trajectory, grid)

    assert coverage > 0.85
    assert all(checks.values())
    assert "improves_drifted_input_odometry" in checks


def test_zero_visual_loops_cannot_be_reported_as_a_pass():
    trajectory, grid = valid_reports()
    trajectory["rtabmap_loop_closure_events"] = 0

    checks, _ = MODULE.evaluate(trajectory, grid)

    assert checks["visual_loop_closures_accepted"] is False


def test_good_input_odom_does_not_require_slam_to_improve_it():
    trajectory, grid = valid_reports(input_rmse=0.05)

    checks, _ = MODULE.evaluate(trajectory, grid)

    assert "improves_drifted_input_odometry" not in checks
    assert all(checks.values())


def test_missing_drift_reference_fails_the_conditional_improvement_check():
    trajectory, grid = valid_reports()
    trajectory["input_odom_final_position_error_m"] = None

    checks, _ = MODULE.evaluate(trajectory, grid)

    assert checks["improves_drifted_input_odometry"] is False
