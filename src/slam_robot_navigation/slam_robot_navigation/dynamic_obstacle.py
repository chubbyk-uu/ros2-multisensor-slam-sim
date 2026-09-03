"""Executable acceptance criteria for a traversable dynamic obstruction."""

import math


PASS = "PASS"
FAIL = "FAIL"


class DynamicObstacleCriteria:
    """Thresholds shared by the 2D and 3D dynamic-obstacle regressions."""

    def __init__(
        self,
        maximum_global_costmap_latency=2.2,
        maximum_local_costmap_latency=0.8,
        minimum_clearance=0.15,
        minimum_path_deviation=0.35,
        maximum_goal_error=0.35,
        maximum_recoveries=3,
    ):
        positive = {
            "maximum_global_costmap_latency": maximum_global_costmap_latency,
            "maximum_local_costmap_latency": maximum_local_costmap_latency,
            "minimum_clearance": minimum_clearance,
            "minimum_path_deviation": minimum_path_deviation,
            "maximum_goal_error": maximum_goal_error,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if maximum_recoveries < 0:
            raise ValueError("maximum_recoveries must be non-negative")
        self.maximum_global_costmap_latency = maximum_global_costmap_latency
        self.maximum_local_costmap_latency = maximum_local_costmap_latency
        self.minimum_clearance = minimum_clearance
        self.minimum_path_deviation = minimum_path_deviation
        self.maximum_goal_error = maximum_goal_error
        self.maximum_recoveries = maximum_recoveries


class DynamicObstacleObservation:
    """Measurements from one dynamic-obstacle run."""

    def __init__(
        self,
        goal_reached,
        global_costmap_observed,
        local_costmap_observed,
        global_costmap_latency,
        local_costmap_latency,
        minimum_obstacle_clearance,
        maximum_path_deviation,
        goal_error,
        recoveries,
        collision_actions,
    ):
        self.goal_reached = goal_reached
        self.global_costmap_observed = global_costmap_observed
        self.local_costmap_observed = local_costmap_observed
        self.global_costmap_latency = global_costmap_latency
        self.local_costmap_latency = local_costmap_latency
        self.minimum_obstacle_clearance = minimum_obstacle_clearance
        self.maximum_path_deviation = maximum_path_deviation
        self.goal_error = goal_error
        self.recoveries = recoveries
        self.collision_actions = collision_actions


def evaluate(observation, criteria):
    """Return a verdict and every independently useful safety check."""
    checks = {
        "goal_reached": bool(observation.goal_reached),
        "global_costmap_observed": bool(
            observation.global_costmap_observed
        ),
        "local_costmap_observed": bool(observation.local_costmap_observed),
        "global_costmap_latency": (
            observation.global_costmap_latency
            <= criteria.maximum_global_costmap_latency
        ),
        "local_costmap_latency": (
            observation.local_costmap_latency
            <= criteria.maximum_local_costmap_latency
        ),
        "kept_clearance": (
            observation.minimum_obstacle_clearance
            >= criteria.minimum_clearance
        ),
        "detour_observed": (
            observation.maximum_path_deviation
            >= criteria.minimum_path_deviation
        ),
        "goal_error": observation.goal_error <= criteria.maximum_goal_error,
        "recovery_budget": observation.recoveries <= criteria.maximum_recoveries,
        "collision_free": observation.collision_actions == 0,
    }
    return (PASS if all(checks.values()) else FAIL), checks


def failed_checks(checks):
    return [name for name, held in checks.items() if not held]
