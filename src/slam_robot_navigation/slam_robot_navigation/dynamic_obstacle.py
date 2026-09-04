"""Executable acceptance criteria for a traversable dynamic obstruction."""

import math


PASS = "PASS"
FAIL = "FAIL"


class NominalRouteDetour:
    """Measure a detour only where a nominal route crosses an obstacle."""

    def __init__(
        self,
        start,
        goal,
        obstacle,
        route_half_width,
        observation_half_window=None,
    ):
        values = (*start, *goal, *obstacle, route_half_width)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("detour geometry must be finite")
        if route_half_width <= 0.0:
            raise ValueError("route_half_width must be positive")
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            raise ValueError("nominal route must have non-zero length")
        if observation_half_window is None:
            observation_half_window = route_half_width
        if (
            not math.isfinite(observation_half_window)
            or observation_half_window <= 0.0
        ):
            raise ValueError("observation_half_window must be positive")

        self.start = start
        self.unit_x = dx / length
        self.unit_y = dy / length
        self.normal_x = -self.unit_y
        self.normal_y = self.unit_x
        self.obstacle_along, self.obstacle_cross = self._coordinates(obstacle)
        self.route_half_width = route_half_width
        self.observation_half_window = observation_half_window
        self.maximum_local_deviation = 0.0
        self.saw_before_obstacle = False
        self.saw_after_obstacle = False

    def _coordinates(self, point):
        relative_x = point[0] - self.start[0]
        relative_y = point[1] - self.start[1]
        along = relative_x * self.unit_x + relative_y * self.unit_y
        cross = relative_x * self.normal_x + relative_y * self.normal_y
        return along, cross

    @property
    def obstacle_on_nominal_route(self):
        return abs(self.obstacle_cross) <= self.route_half_width

    @property
    def crossed_obstacle_station(self):
        return self.saw_before_obstacle and self.saw_after_obstacle

    def observe(self, point):
        if not all(math.isfinite(value) for value in point):
            return
        along, cross = self._coordinates(point)
        self.saw_before_obstacle |= along <= self.obstacle_along
        self.saw_after_obstacle |= along >= self.obstacle_along
        if abs(along - self.obstacle_along) <= self.observation_half_window:
            self.maximum_local_deviation = max(
                self.maximum_local_deviation, abs(cross)
            )


class DynamicObstacleCriteria:
    """Thresholds shared by the 2D and 3D dynamic-obstacle regressions."""

    def __init__(
        self,
        maximum_global_costmap_latency=2.2,
        maximum_local_costmap_latency=0.8,
        minimum_clearance=0.15,
        minimum_path_deviation=0.336 + math.hypot(0.30, 0.30),
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
        obstacle_on_nominal_route,
        crossed_obstacle_station,
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
        self.obstacle_on_nominal_route = obstacle_on_nominal_route
        self.crossed_obstacle_station = crossed_obstacle_station
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
            observation.obstacle_on_nominal_route
            and observation.crossed_obstacle_station
            and observation.maximum_path_deviation
            >= criteria.minimum_path_deviation
        ),
        "goal_error": observation.goal_error <= criteria.maximum_goal_error,
        "recovery_budget": observation.recoveries <= criteria.maximum_recoveries,
        "collision_free": observation.collision_actions == 0,
    }
    return (PASS if all(checks.values()) else FAIL), checks


def failed_checks(checks):
    return [name for name, held in checks.items() if not held]
