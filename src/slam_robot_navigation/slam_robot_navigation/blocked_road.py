"""
Decide whether a robot facing an impassable road gave up correctly.

Two navigation scenarios have opposite success criteria. When an obstacle
leaves a way through, the goal must be reached. When the road is completely
blocked, the goal must *not* be reached and the robot must come to a safe
stop. This module owns the second one so both the 2D and the 3D regression
reach it through the same code; two copies of a safety criterion would be free
to drift apart, and the drift would be invisible until one of them passed a
run the other would have failed.

No verdict class is added for "correctly blocked". PASS, FAIL and
INFRA_UNSTABLE already partition every outcome, and stopping short of the goal
is this scenario's *passing* criterion rather than a third kind of result. A
dependency that genuinely dies during the run is still classified upstream by
its (class, code) pair, untouched by anything here.

Not colliding is not evidence of having given up correctly. A robot that never
perceived the blockage and stopped for an unrelated reason would satisfy "did
not reach the goal and did not crash". A pass therefore requires positive
evidence from all three of perception, planning and motion: the costmap showed
the way closed, the planner reported it had no path, and the robot ended at
rest.

The perception evidence is that the costmap shows *no traversable gap*, not
that some lethal cell appeared. A partly observed barrier does contain lethal
cells while still leaving a slit the planner will thread, and a robot that goes
on trying such a route is behaving correctly rather than failing to perceive.
Measuring the widest remaining gap keeps the criterion tied to the thing that
actually decides whether a route exists.

The scenario also owns its own giving-up budget. `exploration_campaign` charges
a launch timeout to the environment, on the reasoning that a hung launch says
nothing about the algorithm. That reasoning does not carry over here, because
unbounded recovery is the defect this scenario exists to catch: a run that only
ended because the launch timer fired is a failure of the robot, not of the
host, and is reported as such.
"""

# Checks a slow or overloaded host can plausibly cause. Only these may be
# downgraded to INFRA_UNSTABLE by the caller; everything else is the robot's.
BEHAVIOUR_CHECKS = ("recovery_ceiling",)

PASS = "PASS"
FAIL = "FAIL"


class BlockedRoadCriteria:
    """
    Thresholds for one blocked-road run.

    `minimum_clearance` is a distance to the blockage, not a contact test:
    waiting for an actual collision to be reported would accept a robot that
    stopped by touching the wall.
    """

    def __init__(
        self,
        minimum_clearance=0.20,
        minimum_recoveries=1,
        maximum_recoveries=6,
        rest_speed=0.05,
    ):
        if minimum_clearance <= 0.0:
            raise ValueError("minimum_clearance must be positive")
        if minimum_recoveries < 1:
            raise ValueError(
                "minimum_recoveries must be at least 1: giving up without "
                "attempting recovery means the planning failure never reached "
                "the recovery chain"
            )
        if maximum_recoveries < minimum_recoveries:
            raise ValueError(
                "maximum_recoveries must not be below minimum_recoveries"
            )
        if rest_speed <= 0.0:
            raise ValueError("rest_speed must be positive")
        self.minimum_clearance = minimum_clearance
        self.minimum_recoveries = minimum_recoveries
        self.maximum_recoveries = maximum_recoveries
        self.rest_speed = rest_speed


class BlockedRoadObservation:
    """What one blocked-road run actually did."""

    def __init__(
        self,
        goal_reached,
        blockage_closed_in_costmap,
        planner_reported_no_path,
        final_speed,
        recoveries,
        minimum_blockage_distance,
        ended_within_budget,
    ):
        self.goal_reached = goal_reached
        self.blockage_closed_in_costmap = blockage_closed_in_costmap
        self.planner_reported_no_path = planner_reported_no_path
        self.final_speed = final_speed
        self.recoveries = recoveries
        self.minimum_blockage_distance = minimum_blockage_distance
        self.ended_within_budget = ended_within_budget


def evaluate(observation, criteria):
    """
    Return (verdict, checks) for one blocked-road run.

    `checks` maps every criterion to whether it held, so a caller can report
    which one failed rather than only that something did.
    """
    checks = {
        # Reaching a goal behind a full blockage means the blockage was not
        # blocking, which makes the run measure nothing regardless of how
        # safely it was driven.
        "goal_not_reached": not observation.goal_reached,
        "blockage_perceived": bool(observation.blockage_closed_in_costmap),
        "planner_reported_no_path": bool(observation.planner_reported_no_path),
        "robot_at_rest": observation.final_speed <= criteria.rest_speed,
        "kept_clearance": (
            observation.minimum_blockage_distance >= criteria.minimum_clearance
        ),
        "recovery_floor": observation.recoveries >= criteria.minimum_recoveries,
        "recovery_ceiling": (
            observation.recoveries <= criteria.maximum_recoveries
        ),
        # Not an infrastructure question: see the module docstring.
        "ended_within_budget": bool(observation.ended_within_budget),
    }
    failed = [name for name, held in checks.items() if not held]
    return (FAIL if failed else PASS), checks


def failed_checks(checks):
    return [name for name, held in checks.items() if not held]


def core_failures(checks):
    """Failed checks that a busy host cannot excuse."""
    return [
        name for name in failed_checks(checks) if name not in BEHAVIOUR_CHECKS
    ]
