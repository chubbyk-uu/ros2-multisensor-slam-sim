from pathlib import Path

import yaml


def load_rtabmap_parameters():
    config_path = Path(__file__).parents[1] / "config" / "rtabmap_3d.yaml"
    return yaml.safe_load(config_path.read_text())["/**"]["ros__parameters"]


def test_external_odometry_neighbor_links_are_refined_with_icp():
    parameters = load_rtabmap_parameters()

    assert parameters["Reg/Strategy"] == "1"
    assert parameters["RGBD/NeighborLinkRefining"] == "true"
    assert parameters["Icp/PointToPlane"] == "true"


def test_planar_robot_cannot_tilt_the_navigation_map_frame():
    parameters = load_rtabmap_parameters()

    assert parameters["Reg/Force3DoF"] == "true"
    assert parameters["RGBD/ForceOdom3DoF"] == "true"
