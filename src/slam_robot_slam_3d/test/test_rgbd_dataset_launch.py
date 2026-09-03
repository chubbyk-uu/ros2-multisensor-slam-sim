from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_recording_contract_is_input_only_and_uses_fast_compression():
    text = (
        PACKAGE_ROOT / "launch" / "record_rgbd_slam_data.launch.py"
    ).read_text()

    assert '"zstd_fast"' in text
    assert '"storage_profile=zstd_fast"' in text
    assert 'f"route_laps={route_laps}"' in text
    assert '"/camera/color/image_raw"' in text
    assert '"/camera/depth/image_raw"' in text
    assert '"/lidar_3d/points"' in text
    assert '"/ground_truth/odom"' in text
    assert '"/rtabmap/map"' not in text
    assert '"/tf"' not in text
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" in text


def test_structured_recording_is_one_lap_and_slam_independent():
    text = (
        PACKAGE_ROOT / "launch" / "structured_rgbd_dataset_recording.launch.py"
    ).read_text()

    assert 'DeclareLaunchArgument("laps", default_value="1")' in text
    assert 'arguments=["--drive-only", "--laps"' in text
    assert '"route_laps": LaunchConfiguration("laps")' in text
    assert '"rgbd_update_rate": "10.0"' in text
    assert "rtabmap_rgbd_simulation.launch.py" not in text


def test_playback_protects_large_message_writer_and_publishes_clock():
    text = (
        PACKAGE_ROOT / "launch" / "play_rgbd_slam_data.launch.py"
    ).read_text()

    assert '"bag",' in text
    assert '"play",' in text
    assert '"--clock",' in text
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" in text
