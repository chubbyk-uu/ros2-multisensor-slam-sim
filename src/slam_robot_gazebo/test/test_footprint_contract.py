"""
Keep the spawn sampler's robot envelope in step with the Nav2 footprint.

Nothing in the build connects these two. The sampler decides where the robot
may be created; Nav2's costmaps decide what counts as a collision once it is
there. If someone widens the footprint polygon and the sampler keeps its old
radius, spawns quietly start landing closer to walls than the planner believes
is possible -- and that failure appears as an unexplained navigation problem
several files away from the edit that caused it.
"""

import ast
import math
import re
from pathlib import Path

NAVIGATION_LAUNCH = (
    Path(__file__).parents[2] / "slam_robot_navigation" / "launch"
)
SAMPLER_HEADER = (
    Path(__file__).parents[1] / "include" / "slam_robot_gazebo"
    / "safe_spawn_sampler.hpp"
)

# How much larger than the robot the sampler's own radius may be. Some slack is
# fine -- rounding up is the safe direction -- but a value far above the
# polygon stops being robot geometry and becomes an undeclared safety margin
# hiding in the wrong field, where the recorded parameters would misdescribe it.
MAXIMUM_ROUNDING_UP_M = 0.005


def footprint_vertices(path):
    text = path.read_text()
    found = re.search(r"ROBOT_FOOTPRINT = \((.*?)\)\n", text, re.S)
    assert found is not None, f"no ROBOT_FOOTPRINT in {path}"
    joined = "".join(re.findall(r'"([^"]*)"', found.group(1)))
    return ast.literal_eval(joined)


def sampler_circumscribed_radius():
    text = SAMPLER_HEADER.read_text()
    found = re.search(r"double robot_circumscribed_radius\{([0-9.]+)\}", text)
    assert found is not None, "no robot_circumscribed_radius in the sampler header"
    return float(found.group(1))


def test_the_sampler_never_reports_the_robot_smaller_than_it_is():
    vertices = footprint_vertices(NAVIGATION_LAUNCH / "online_slam_navigation.launch.py")
    circumscribed = max(math.hypot(x, y) for x, y in vertices)
    radius = sampler_circumscribed_radius()

    # Not an equality against a hand-written constant: the point is the
    # direction of the inequality. Under-reporting puts a spawn inside the
    # robot's own outline.
    assert radius >= circumscribed, (
        f"sampler radius {radius} is inside the footprint's {circumscribed}")
    assert radius - circumscribed <= MAXIMUM_ROUNDING_UP_M


def test_both_navigation_entry_points_describe_the_same_robot():
    # The exploration chain goes through online_slam_navigation; the mapped
    # chain goes through navigation. A robot that changes shape depending on
    # which launch file started it would make the contract above meaningless.
    assert footprint_vertices(NAVIGATION_LAUNCH / "navigation.launch.py") == (
        footprint_vertices(NAVIGATION_LAUNCH / "online_slam_navigation.launch.py"))


def test_the_footprint_is_the_asymmetric_polygon_and_not_a_circle():
    vertices = footprint_vertices(NAVIGATION_LAUNCH / "online_slam_navigation.launch.py")
    radii = [math.hypot(x, y) for x, y in vertices]

    # Nav2's official robot_radius stays in the parameter file and is inert
    # while a polygon is set. If this ever collapsed to a circle, the sampler's
    # radius would need to come from that value instead, and the test above
    # would be reading the wrong source.
    assert len(vertices) > 4
    assert max(radii) - min(radii) > 0.05
