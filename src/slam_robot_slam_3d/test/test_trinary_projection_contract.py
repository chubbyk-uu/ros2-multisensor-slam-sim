"""
Keep the projection and its consumers on one set of thresholds.

The occupancy projection decides what /map publishes; the frontier detector
decides what /map means. Both carry their own copy of the free and occupied
thresholds, and both configs say they must agree. They stopped agreeing once:
occupied_minimum was lowered from 65 to 50 on the projection side, so that
cells leaning occupied were published as obstacles rather than as unexplored
space, and the detector's copy stayed at 65 for weeks without anyone noticing.

Nothing broke, because a trinary /map only ever carries -1, 0 and 100 and both
values classify 100 as occupied. That is exactly why it went unnoticed, and
exactly why it needs a test rather than a comment: the drift is invisible until
one side publishes an intermediate value, at which point the two disagree about
which cells are free.
"""

from pathlib import Path

import yaml

WORKSPACE_SRC = Path(__file__).resolve().parents[2]
PROJECTION_CONFIG = Path(__file__).parents[1] / "config" / "custom_3d_slam.yaml"
DETECTOR_CONFIG = (
    WORKSPACE_SRC / "slam_robot_navigation" / "config" / "frontier_exploration.yaml"
)


def projection_thresholds():
    # The projection config applies to every node under "/**", which is how the
    # preprocessor and the front end share one file.
    document = yaml.safe_load(PROJECTION_CONFIG.read_text())
    grid = document["/**"]["ros__parameters"]["occupancy_grid"]
    return grid["free_maximum"], grid["occupied_minimum"]


def cloud_resolutions():
    document = yaml.safe_load(PROJECTION_CONFIG.read_text())
    parameters = document["/**"]["ros__parameters"]
    return (
        parameters["voxel_leaf_size"],
        parameters["front_end"]["registration_voxel_leaf_size"],
        parameters["occupancy_grid"]["resolution"],
    )


def detector_thresholds():
    document = yaml.safe_load(DETECTOR_CONFIG.read_text())
    frontier = document["frontier_explorer"]["ros__parameters"]["frontier"]
    return frontier["free_maximum"], frontier["occupied_minimum"]


def test_the_detector_reads_the_grid_the_projection_writes():
    assert detector_thresholds() == projection_thresholds()


def test_the_thresholds_leave_no_third_category_unaccounted_for():
    # Anything between the two is neither free nor occupied and is published as
    # unknown. That band is deliberate, so it must stay non-empty and ordered:
    # a free_maximum at or above occupied_minimum would make a cell both.
    free_maximum, occupied_minimum = projection_thresholds()

    assert 0 <= free_maximum < occupied_minimum <= 100


def test_mapping_and_registration_clouds_keep_separate_resolutions():
    mapping_leaf, registration_leaf, grid_resolution = cloud_resolutions()

    assert mapping_leaf == grid_resolution == 0.05
    assert registration_leaf == 0.10
    assert registration_leaf > mapping_leaf
