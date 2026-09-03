from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def load_parameters():
    return yaml.safe_load(
        (PACKAGE_ROOT / "config" / "rtabmap_rgbd.yaml").read_text()
    )["/**"]["ros__parameters"]


def test_rgbd_mode_uses_external_odometry_and_visual_registration():
    parameters = load_parameters()

    assert parameters["subscribe_rgbd"] is True
    assert parameters["subscribe_depth"] is False
    assert parameters["subscribe_scan_cloud"] is False
    assert parameters["odom_frame_id"] == ""
    assert parameters["publish_tf"] is True
    assert parameters["Reg/Strategy"] == "0"
    assert parameters["Reg/Force3DoF"] == "true"
    assert parameters["RGBD/ForceOdom3DoF"] == "true"


def test_navigation_grid_is_built_from_depth_with_verified_height_band():
    parameters = load_parameters()

    assert parameters["Grid/Sensor"] == "1"
    assert parameters["Grid/3D"] == "false"
    assert parameters["Grid/RangeMin"] == "0.20"
    assert parameters["Grid/RangeMax"] == "6.0"
    assert parameters["Grid/MaxGroundHeight"] == "0.05"
    assert parameters["Grid/MaxObstacleHeight"] == "0.45"


def test_launch_uses_official_sync_and_large_message_profile():
    launch_text = (PACKAGE_ROOT / "launch" / "rtabmap_rgbd.launch.py").read_text()
    simulation_text = (
        PACKAGE_ROOT / "launch" / "rtabmap_rgbd_simulation.launch.py"
    ).read_text()

    assert 'package="rtabmap_sync"' in launch_text
    assert 'executable="rgbd_sync"' in launch_text
    assert '"approx_sync": False' in launch_text
    assert launch_text.count("additional_env=large_message_environment") == 2
    assert 'arguments=["-d"] if reset_database else []' in launch_text
    assert '"camera_variant": "rgbd"' in simulation_text
    assert '"rgbd_pointcloud": "false"' in simulation_text
    assert 'name="rtabmap_rgbd_rviz"' in simulation_text
