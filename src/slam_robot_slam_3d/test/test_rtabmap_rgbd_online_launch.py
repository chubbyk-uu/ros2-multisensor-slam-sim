from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "rtabmap_rgbd_online_regression.launch.py"
).read_text()


def test_online_regression_uses_live_rgbd_simulation_and_two_lap_driver():
    assert '"rtabmap_rgbd_simulation.launch.py"' in LAUNCH
    assert '"structured_loop_3d.sdf"' in LAUNCH
    assert 'arguments=["--laps", "2"]' in LAUNCH
    assert '"reset_database": "true"' in LAUNCH


def test_online_regression_measures_both_processes_trajectory_and_map():
    assert '"--process-match", "/rtabmap_sync/rgbd_sync"' in LAUNCH
    assert '"--process-match", "/rtabmap_slam/rtabmap"' in LAUNCH
    assert '"--topic", "/rtabmap/map"' in LAUNCH
    assert 'default_value="900.0"' in LAUNCH


def test_online_regression_checks_only_after_successful_route():
    assert "if event.returncode != 0:" in LAUNCH
    assert "TimerAction(period=5.0, actions=[checker])" in LAUNCH
    assert '"rtabmap_rgbd_fixed_regression_check"' in LAUNCH
    assert 'default_value="500"' in LAUNCH
    assert 'default_value="450"' in LAUNCH
    assert 'default_value="9000"' in LAUNCH
    assert 'default_value="27.0"' in LAUNCH
    assert '"--camera-info-topic", "/camera/color/camera_info"' in LAUNCH
    assert "target_action=checker" in LAUNCH


def test_online_regression_removes_stale_reports_before_starting():
    assert "OpaqueFunction(function=prepare_output_files)" in LAUNCH
    assert "output_path.unlink()" in LAUNCH
