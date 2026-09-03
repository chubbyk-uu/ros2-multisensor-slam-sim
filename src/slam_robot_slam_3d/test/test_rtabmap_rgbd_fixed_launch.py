from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "rtabmap_rgbd_fixed_regression.launch.py"
).read_text()


def test_fixed_replay_uses_rgbd_stack_and_delayed_large_message_playback():
    assert '"rtabmap_rgbd.launch.py"' in LAUNCH
    assert "TimerAction(period=6.0, actions=[playback])" in LAUNCH
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" in LAUNCH
    assert '"reset_database": "true"' in LAUNCH


def test_fixed_replay_measures_complete_rgbd_process_set_and_map():
    assert '"--process-match", "/rtabmap_sync/rgbd_sync"' in LAUNCH
    assert '"--process-match", "/rtabmap_slam/rtabmap"' in LAUNCH
    assert '"--topic", "/rtabmap/map"' in LAUNCH
    assert 'executable="recorded_odom_tf_publisher"' in LAUNCH


def test_fixed_replay_runs_a_bounded_positive_loop_verdict_before_shutdown():
    assert '"rtabmap_rgbd_fixed_regression_check"' in LAUNCH
    assert "checker = ExecuteProcess(" in LAUNCH
    assert 'DeclareLaunchArgument("minimum_loop_closures", default_value="20")' in LAUNCH
    assert "period=20.0" in LAUNCH
    assert "actions=[checker]" in LAUNCH
    assert "target_action=checker" in LAUNCH
