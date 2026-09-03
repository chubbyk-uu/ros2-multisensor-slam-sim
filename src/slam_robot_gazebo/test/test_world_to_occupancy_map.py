"""
Tests for rasterising a world into the map Nav2 loads.

The map decides where the planner believes it may go, so the cases here are
the ways a generated map could quietly claim more freedom than the world has.
"""

from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
MODULE = SourceFileLoader(
    "world_to_occupancy_map", str(SCRIPTS / "world_to_occupancy_map")
).load_module()


def world_file(tmp_path, boxes, static="true"):
    collisions = "\n".join(
        f"""<collision name="c{index}">
              <pose>{x} {y} 0.6 0 0 {yaw}</pose>
              <geometry><box><size>{sx} {sy} 1.2</size></box></geometry>
            </collision>"""
        for index, (x, y, sx, sy, yaw) in enumerate(boxes)
    )
    path = tmp_path / "world.sdf"
    path.write_text(
        f"""<sdf version="1.10"><world name="test">
          <model name="m"><static>{static}</static><link name="l">
          {collisions}
          </link></model></world></sdf>""",
        encoding="utf-8",
    )
    return path


def cell_of(x, y, minimum=-2.0, resolution=0.1):
    return int((y - minimum) / resolution), int((x - minimum) / resolution)


def test_a_rotated_box_is_filled_along_its_own_axis():
    # A 2.0 x 0.1 wall turned 45 degrees covers the diagonal and nothing else.
    # Rasterising it as though it were unrotated would leave the diagonal open
    # and seal the axis instead -- both wrong, and in opposite directions.
    turned, _, _ = MODULE.rasterise(
        [(0.0, 0.0, 2.0, 0.1, 0.7853981633974483)],
        (-2.0, -2.0, 2.0, 2.0),
        0.1,
    )
    straight, _, _ = MODULE.rasterise(
        [(0.0, 0.0, 2.0, 0.1, 0.0)], (-2.0, -2.0, 2.0, 2.0), 0.1
    )
    on_diagonal = cell_of(0.5, 0.5)
    on_axis = cell_of(0.8, 0.0)

    assert turned[on_diagonal[0]][on_diagonal[1]] == MODULE.OCCUPIED
    assert turned[on_axis[0]][on_axis[1]] != MODULE.OCCUPIED
    assert straight[on_diagonal[0]][on_diagonal[1]] != MODULE.OCCUPIED
    assert straight[on_axis[0]][on_axis[1]] == MODULE.OCCUPIED


def test_space_the_robot_cannot_reach_stays_unknown():
    # A map that marked unreachable space free would let the planner route
    # through walls it has never seen.
    grid, width, height = MODULE.rasterise(
        [
            (0.0, 1.0, 4.0, 0.2, 0.0),
            (0.0, -1.0, 4.0, 0.2, 0.0),
            (-2.0, 0.0, 0.2, 2.0, 0.0),
            (2.0, 0.0, 0.2, 2.0, 0.0),
        ],
        (-3.0, -3.0, 3.0, 3.0),
        0.1,
    )
    seed_column = int((0.0 - (-3.0)) / 0.1)
    seed_row = int((0.0 - (-3.0)) / 0.1)
    MODULE.flood_free(grid, width, height, seed_column, seed_row)

    assert grid[seed_row][seed_column] == MODULE.FREE
    # Outside the sealed box, and therefore never visited.
    assert grid[2][2] == MODULE.UNKNOWN


def test_a_seed_inside_a_wall_is_rejected_rather_than_flooding_nothing():
    grid, width, height = MODULE.rasterise(
        [(0.0, 0.0, 1.0, 1.0, 0.0)], (-2.0, -2.0, 2.0, 2.0), 0.1
    )
    column = int((0.0 - (-2.0)) / 0.1)
    row = int((0.0 - (-2.0)) / 0.1)

    with pytest.raises(ValueError, match="inside an obstacle"):
        MODULE.flood_free(grid, width, height, column, row)


def test_only_static_models_are_read():
    # A movable model is not part of the map; treating it as a wall would bake
    # a transient object into the static layer.
    assert list(MODULE.box_collisions(
        world_file(Path("/tmp"), [(0.0, 0.0, 1.0, 1.0, 0.0)], static="false")
    )) == []


def test_poses_compose_from_model_link_and_collision():
    root = world_file(Path("/tmp"), [(1.0, 2.0, 0.5, 0.5, 0.0)])
    boxes = list(MODULE.box_collisions(root))

    assert len(boxes) == 1
    assert boxes[0][0] == pytest.approx(1.0)
    assert boxes[0][1] == pytest.approx(2.0)


def test_a_pose_with_no_rotation_fields_defaults_to_the_identity():
    assert MODULE.parse_pose(None) == (0.0, 0.0, 0.0)
    assert MODULE.parse_pose("1 2 3") == (1.0, 2.0, 0.0)
    assert MODULE.parse_pose("1 2 3 0 0 1.5")[2] == pytest.approx(1.5)
