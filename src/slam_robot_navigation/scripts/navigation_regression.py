#!/usr/bin/env python3

import argparse
import math
import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from slam_robot_navigation.blocked_road import (
    BlockedRoadCriteria,
    BlockedRoadObservation,
    core_failures,
    evaluate as evaluate_blocked_road,
    failed_checks,
)
from slam_robot_navigation.dynamic_obstacle import (
    DynamicObstacleCriteria,
    DynamicObstacleObservation,
    evaluate as evaluate_dynamic_obstacle,
    failed_checks as failed_dynamic_checks,
    NominalRouteDetour,
)


DEFAULT_ROUTE = (
    (1.0, 1.0, 0.0),
    (4.0, 1.0, -math.pi / 2.0),
    (4.0, -2.0, math.pi),
    (0.5, 0.8, math.pi),
    (-1.0, -1.0, math.pi),
    (-4.0, -1.0, math.pi / 2.0),
    (-4.0, 1.0, -math.pi / 2.0),
    (-1.0, -1.0, 0.0),
    (0.0, 0.0, 0.0),
)

DYNAMIC_GOAL = (-4.0, -1.0, math.pi)

# The blocked-road scenario runs in blocked_road_world, whose north-east
# alcove has exactly one 1.1 m doorway; its other three sides are walls the
# pre-built map already contains. One seal across that doorway therefore closes
# the alcove in the costmap while the robot is looking straight at it.
#
# Three designs in slam_world failed first, and the reasons are why a dedicated
# world exists.
#
# A single 9.8 m wall dividing the room fails because the static layer holds
# only the pre-built map, so a spawned barrier exists only where the laser has
# seen it. The planner threads the unobserved middle and the robot concludes
# nothing.
#
# Sealing partition_a's two doorways fails differently: it leaves the goal
# reachable round the south of the room and up through the 0.30 m gap at
# partition_d's west end. That gap is narrower than the robot, but the
# discretised costmap and planner still admit a route through it. An earlier
# investigation incorrectly blamed a 253/254 threshold: Jazzy NavFn treats
# 253 as an obstacle too. The exact grid/inflation interaction was not isolated.
#
# Sealing the north-east pocket's two openings fails for the reason recorded in
# docs/incidents.md: they lie about 3.5 m apart, beyond obstacle_max_range
# 2.5 m, so the obstacle layer clears whichever one the robot is not looking
# at. Measured directly as a narrowest gap of 0.00 m during the run against
# 0.30 m at the moment the planner was asked.
#
# A single doorway needs no memory at all, which is the property slam_world's
# open plan could not provide.
BLOCKED_ROAD_WORLD = "blocked_road_world"
BLOCKED_ROAD_GOAL = (4.1, 3.6, math.pi / 2.0)
# (name, centre x, centre y, size x, size y)
BLOCKED_ROAD_SEALS = (
    ("navigation_test_seal", 4.15, 2.0, 1.1, 0.2),
)
BLOCKED_ROAD_SEAL_HEIGHT = 1.2
# Distance from the start pose after which the robot counts as committed to
# the route, and the seal appears.
BLOCKED_ROAD_COMMIT_DISTANCE = 1.0
DEPENDENCY_FAILURES = {
    "GAZEBO_SERVICES_UNAVAILABLE",
    "NAV2_GOAL_RESPONSE_TIMEOUT",
    "NAV2_RUNTIME_LOST",
    "PLANNER_UNAVAILABLE",
    "SPAWN_FAILED",
    "WALL_WATCHDOG_TIMEOUT",
}


def blocked_road_seal_sdf(name, size_x, size_y):
    size = f"{size_x} {size_y} {BLOCKED_ROAD_SEAL_HEIGHT}"
    return f"""
<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <collision name="collision">
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>0.9 0.1 0.1 1</ambient>
          <diffuse>0.9 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


def distance_to_seal(x, y, seal):
    """Return the shortest distance from a point to a seal's surface."""
    _, centre_x, centre_y, size_x, size_y = seal
    dx = max(abs(x - centre_x) - size_x / 2.0, 0.0)
    dy = max(abs(y - centre_y) - size_y / 2.0, 0.0)
    return math.hypot(dx, dy)


DYNAMIC_OBSTACLE_NAME = "navigation_test_obstacle"
DYNAMIC_OBSTACLE_X = -1.35
DYNAMIC_OBSTACLE_Y = -0.34
DYNAMIC_OBSTACLE_RADIUS = math.hypot(0.30, 0.30)
DYNAMIC_OBSTACLE_SDF = """
<sdf version="1.10">
  <model name="navigation_test_obstacle">
    <static>true</static>
    <link name="body">
      <collision name="collision">
        <geometry><box><size>0.60 0.60 0.80</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.60 0.60 0.80</size></box></geometry>
        <material>
          <ambient>1.0 0.25 0.05 1</ambient>
          <diffuse>1.0 0.25 0.05 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


class NavigationRegression:
    def __init__(
        self,
        navigator,
        world_name,
        localization_mode="amcl",
        seal_offset_x=0.0,
        obstacle_offset_y=0.0,
    ):
        self.navigator = navigator
        # The Gazebo entity services are namespaced by world, and the
        # blocked-road scenario runs in a different world from the others.
        self.world_name = world_name
        self.localization_mode = localization_mode
        self.blocked_road_seals = tuple(
            (name, x + seal_offset_x, y, size_x, size_y)
            for name, x, y, size_x, size_y in BLOCKED_ROAD_SEALS
        )
        self.latest_amcl_pose = None
        self.latest_navigation_pose = None
        self.dynamic_obstacle_x = DYNAMIC_OBSTACLE_X
        self.dynamic_obstacle_y = DYNAMIC_OBSTACLE_Y + obstacle_offset_y
        self.collision_actions = 0
        self.last_collision_action = CollisionMonitorState.DO_NOTHING
        self.obstacle_spawned = False
        self.global_costmap_observed = False
        self.local_costmap_observed = False
        self.obstacle_spawn_simulation_time = None
        self.global_costmap_latency = math.nan
        self.local_costmap_latency = math.nan
        self.minimum_obstacle_distance = math.inf
        self.maximum_path_deviation = 0.0
        self.detour_monitor = None
        self.blockage_spawned = False
        # The widest gap the costmap still shows across either doorway, and
        # the best (narrowest) value seen: the obstacle layer can clear cells
        # again once the robot looks away, and the criterion is whether the
        # robot ever had the evidence, not whether it still holds it.
        self.widest_doorway_gap = math.inf
        self.narrowest_doorway_gap = math.inf
        self.minimum_blockage_distance = math.inf
        self.latest_speed = 0.0
        self.path_client = ActionClient(
            self.navigator, ComputePathToPose, "compute_path_to_pose"
        )
        self.spawn_client = self.navigator.create_client(
            SpawnEntity,
            f"/world/{world_name}/create",
        )
        self.delete_client = self.navigator.create_client(
            DeleteEntity,
            f"/world/{world_name}/remove",
        )
        if localization_mode == "amcl":
            self.navigator.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self._amcl_pose_callback,
                10,
            )
        self.navigator.create_subscription(
            CollisionMonitorState,
            "/collision_monitor_state",
            self._collision_callback,
            10,
        )
        self.navigator.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._global_costmap_callback,
            10,
        )
        self.navigator.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._local_costmap_callback,
            10,
        )
        self.navigator.create_subscription(
            Odometry,
            "/odom",
            self._odometry_callback,
            10,
        )

    def _amcl_pose_callback(self, message):
        self.latest_amcl_pose = message.pose.pose

    def _collision_callback(self, message):
        if (
            message.action_type != CollisionMonitorState.DO_NOTHING
            and message.action_type != self.last_collision_action
        ):
            self.collision_actions += 1
        self.last_collision_action = message.action_type

    def _costmap_contains_obstacle(self, message):
        resolution = message.info.resolution
        origin = message.info.origin.position
        cell_x = round((self.dynamic_obstacle_x - origin.x) / resolution)
        cell_y = round((self.dynamic_obstacle_y - origin.y) / resolution)
        radius_cells = max(1, round(0.20 / resolution))
        values = []
        for y in range(cell_y - radius_cells, cell_y + radius_cells + 1):
            if y < 0 or y >= message.info.height:
                continue
            for x in range(cell_x - radius_cells, cell_x + radius_cells + 1):
                if x < 0 or x >= message.info.width:
                    continue
                values.append(message.data[y * message.info.width + x])

        return bool(values and max(values) >= 99)

    @staticmethod
    def _message_time_seconds(message):
        stamp = message.header.stamp
        return stamp.sec + stamp.nanosec * 1.0e-9

    def _global_costmap_callback(self, message):
        if self.blockage_spawned:
            self.widest_doorway_gap = self._widest_doorway_gap(message)
            self.narrowest_doorway_gap = min(
                self.narrowest_doorway_gap, self.widest_doorway_gap
            )
        if (
            self.obstacle_spawned
            and not self.global_costmap_observed
            and self._costmap_contains_obstacle(message)
        ):
            self.global_costmap_observed = True
            self.global_costmap_latency = (
                self._message_time_seconds(message)
                - self.obstacle_spawn_simulation_time
            )
            print(
                "  global costmap marked the spawned obstacle as lethal "
                f"after {self.global_costmap_latency:.3f} sim s",
                flush=True,
            )

    def _local_costmap_callback(self, message):
        if (
            self.obstacle_spawned
            and not self.local_costmap_observed
            and self._costmap_contains_obstacle(message)
        ):
            self.local_costmap_observed = True
            self.local_costmap_latency = (
                self._message_time_seconds(message)
                - self.obstacle_spawn_simulation_time
            )
            print(
                "  local costmap marked the spawned obstacle as lethal "
                f"after {self.local_costmap_latency:.3f} sim s",
                flush=True,
            )

    def _odometry_callback(self, message):
        linear = message.twist.twist.linear
        self.latest_speed = math.hypot(linear.x, linear.y)

    def _widest_doorway_gap(self, message):
        """
        Return the widest traversable gap left in any sealed opening.

        A lethal cell somewhere in an opening does not close it. What decides
        whether a route survives is the longest run of non-lethal cells across
        the opening, so that is what is measured; anything narrower than the
        robot's inscribed diameter cannot be planned through.
        """
        resolution = message.info.resolution
        origin = message.info.origin.position
        width = message.info.width
        height = message.info.height
        widest = 0.0
        # Perception is always scored at the real doorway. The negative
        # control moves the spawned seal, not the place whose closure the
        # robot is supposed to perceive.
        for seal in BLOCKED_ROAD_SEALS:
            _, centre_x, centre_y, size_x, size_y = seal
            along_y = size_y >= size_x
            length = size_y if along_y else size_x
            steps = max(1, int(round(length / resolution)))
            longest = run = 0
            for step in range(steps + 1):
                offset = -length / 2.0 + step * resolution
                x = centre_x if along_y else centre_x + offset
                y = centre_y + offset if along_y else centre_y
                cell_x = round((x - origin.x) / resolution)
                cell_y = round((y - origin.y) / resolution)
                lethal = False
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        px, py = cell_x + dx, cell_y + dy
                        if 0 <= px < width and 0 <= py < height:
                            if message.data[py * width + px] >= 99:
                                lethal = True
                                break
                    if lethal:
                        break
                if lethal:
                    run = 0
                else:
                    run += 1
                    longest = max(longest, run)
            widest = max(widest, longest * resolution)
        return widest

    def measure_rest_speed(self, seconds):
        """
        Return the fastest speed seen over a settling window.

        A single instantaneous reading can catch a moving robot at a zero
        crossing, which would report a robot still pushing at the wall as
        stopped.
        """
        deadline = time.monotonic() + seconds
        peak = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
            peak = max(peak, self.latest_speed)
        return peak

    def planner_reports_no_path(self, goal_pose, timeout=20.0):
        """
        Ask the planner directly whether a path to the goal exists.

        The navigation task failing is not the same statement: it also fails
        when the controller gives up on a path that the planner did produce.
        The criterion is about the planner, so the planner is asked.
        """
        if not self.path_client.wait_for_server(timeout_sec=timeout):
            return None, "planner action server unavailable"
        request = ComputePathToPose.Goal()
        request.goal = goal_pose
        request.use_start = False
        send = self.path_client.send_goal_async(request)
        rclpy.spin_until_future_complete(self.navigator, send, timeout_sec=timeout)
        if not send.done() or send.result() is None:
            return None, "planner did not accept the request"
        handle = send.result()
        if not handle.accepted:
            return None, "planner rejected the request"
        finish = handle.get_result_async()
        rclpy.spin_until_future_complete(self.navigator, finish, timeout_sec=timeout)
        if not finish.done() or finish.result() is None:
            return None, "planner did not return a result"
        result = finish.result().result
        code = result.error_code
        if code == ComputePathToPose.Result.NO_VALID_PATH:
            return True, f"error_code={code}"
        # A claimed path is the thing to look at when this criterion fails:
        # it says which way round the seals the planner believes it can go.
        poses = list(result.path.poses)
        if poses:
            step = max(1, len(poses) // 8)
            route = " ".join(
                f"({p.pose.position.x:.1f},{p.pose.position.y:.1f})"
                for p in poses[::step]
            )
            detail = f"error_code={code} path[{len(poses)}]: {route}"
        else:
            detail = f"error_code={code} path is empty"
        return False, detail

    def start_navigation_goal(self, pose, response_timeout):
        """Start NavigateToPose without BasicNavigator's unbounded waits."""
        client = self.navigator.nav_to_pose_client
        if not client.wait_for_server(timeout_sec=response_timeout):
            return False, "NAV2_GOAL_RESPONSE_TIMEOUT"
        request = NavigateToPose.Goal()
        request.pose = pose
        send = client.send_goal_async(
            request, self.navigator._feedbackCallback
        )
        rclpy.spin_until_future_complete(
            self.navigator, send, timeout_sec=response_timeout
        )
        if not send.done() or send.result() is None:
            return False, "NAV2_GOAL_RESPONSE_TIMEOUT"
        self.navigator.goal_handle = send.result()
        if not self.navigator.goal_handle.accepted:
            return False, "NAV2_GOAL_RESPONSE_TIMEOUT"
        self.navigator.result_future = (
            self.navigator.goal_handle.get_result_async()
        )
        return True, None

    def cancel_navigation(self, timeout=5.0):
        """Best-effort bounded cancellation for dependency-failure paths."""
        handle = self.navigator.goal_handle
        if handle is None:
            return
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self.navigator, future, timeout_sec=timeout
        )

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def simulation_time(self):
        return self.navigator.get_clock().now().nanoseconds * 1.0e-9

    def wait_until_nav2_active(self, timeout):
        deadline = time.monotonic() + timeout
        lifecycle_nodes = ["bt_navigator"]
        if self.localization_mode == "amcl":
            lifecycle_nodes.insert(0, "amcl")
        for node_name in lifecycle_nodes:
            client = self.navigator.create_client(
                GetState,
                f"/{node_name}/get_state",
            )
            state = "unknown"
            while state != "active":
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                if not client.wait_for_service(
                    timeout_sec=min(0.5, remaining)
                ):
                    continue
                future = client.call_async(GetState.Request())
                while not future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self.navigator, timeout_sec=0.1)
                if future.done() and future.result() is not None:
                    state = future.result().current_state.label

            if node_name == "amcl":
                while (
                    not self.navigator.initial_pose_received
                    and time.monotonic() < deadline
                ):
                    self.navigator._setInitialPose()
                    rclpy.spin_once(self.navigator, timeout_sec=0.5)
                if not self.navigator.initial_pose_received:
                    return False
        return True

    def run(self, route, timeout):
        poses = [self.make_pose(*waypoint) for waypoint in route]
        goal = poses[-1].pose
        start_simulation_time = self.simulation_time()
        start_wall_time = time.monotonic()
        max_recoveries = 0
        last_distance = math.nan

        for index, pose in enumerate(poses, start=1):
            segment_start = self.simulation_time()
            last_report_time = time.monotonic() - 5.0
            print(
                f"waypoint {index}/{len(poses)}: "
                f"goal=({pose.pose.position.x:.2f}, "
                f"{pose.pose.position.y:.2f})",
                flush=True,
            )
            if not self.navigator.goToPose(pose):
                return {
                    "result": "REJECTED",
                    "elapsed": self.simulation_time()
                    - start_simulation_time,
                    "remaining": last_distance,
                    "recoveries": max_recoveries,
                    "goal_error": math.nan,
                    "failed_waypoint": index,
                }

            while not self.navigator.isTaskComplete():
                now = time.monotonic()
                simulation_elapsed = (
                    self.simulation_time() - start_simulation_time
                )
                feedback = self.navigator.getFeedback()
                if feedback is not None:
                    self.latest_navigation_pose = feedback.current_pose.pose
                    max_recoveries = max(
                        max_recoveries,
                        feedback.number_of_recoveries,
                    )
                    last_distance = feedback.distance_remaining
                    if now - last_report_time >= 5.0:
                        current = feedback.current_pose.pose.position
                        print(
                            "  progress: "
                            f"position=({current.x:.2f}, {current.y:.2f}) "
                            f"remaining={feedback.distance_remaining:.2f} m "
                            f"recoveries={feedback.number_of_recoveries}",
                            flush=True,
                        )
                        last_report_time = now

                if simulation_elapsed > timeout:
                    self.navigator.cancelTask()
                    return {
                        "result": "TIMEOUT",
                        "elapsed": simulation_elapsed,
                        "remaining": last_distance,
                        "recoveries": max_recoveries,
                        "goal_error": math.nan,
                        "failed_waypoint": index,
                    }
                if now - start_wall_time > max(timeout * 5.0, timeout + 120.0):
                    self.navigator.cancelTask()
                    return {
                        "result": "WALL_WATCHDOG_TIMEOUT",
                        "elapsed": simulation_elapsed,
                        "remaining": last_distance,
                        "recoveries": max_recoveries,
                        "goal_error": math.nan,
                        "failed_waypoint": index,
                    }

            segment_result = self.navigator.getResult()
            if segment_result != TaskResult.SUCCEEDED:
                result_name = {
                    TaskResult.CANCELED: "CANCELED",
                    TaskResult.FAILED: "FAILED",
                }.get(segment_result, "UNKNOWN")
                return {
                    "result": result_name,
                    "elapsed": self.simulation_time()
                    - start_simulation_time,
                    "remaining": last_distance,
                    "recoveries": max_recoveries,
                    "goal_error": math.nan,
                    "failed_waypoint": index,
                }

            print(
                f"  reached in "
                f"{self.simulation_time() - segment_start:.1f} sim s",
                flush=True,
            )

        elapsed = self.simulation_time() - start_simulation_time
        rclpy.spin_once(self.navigator, timeout_sec=0.2)
        goal_error = math.nan
        final_pose = self.latest_amcl_pose or self.latest_navigation_pose
        if final_pose is not None:
            dx = final_pose.position.x - goal.position.x
            dy = final_pose.position.y - goal.position.y
            goal_error = math.hypot(dx, dy)

        return {
            "result": "SUCCEEDED",
            "elapsed": elapsed,
            "remaining": last_distance,
            "recoveries": max_recoveries,
            "goal_error": goal_error,
            "failed_waypoint": 0,
        }

    def wait_for_gazebo_services(self, timeout=10.0):
        spawn_ready = self.spawn_client.wait_for_service(timeout_sec=timeout)
        delete_ready = self.delete_client.wait_for_service(timeout_sec=timeout)
        return spawn_ready and delete_ready

    def spawn_obstacle(self):
        request = SpawnEntity.Request()
        request.entity_factory.name = DYNAMIC_OBSTACLE_NAME
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf = DYNAMIC_OBSTACLE_SDF
        request.entity_factory.pose.position.x = self.dynamic_obstacle_x
        request.entity_factory.pose.position.y = self.dynamic_obstacle_y
        request.entity_factory.pose.position.z = 0.40
        request.entity_factory.pose.orientation.w = 1.0
        request.entity_factory.relative_to = "world"
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.navigator,
            future,
            timeout_sec=5.0,
        )
        if not future.done() or future.result() is None:
            return False
        self.obstacle_spawned = future.result().success
        if self.obstacle_spawned:
            self.obstacle_spawn_simulation_time = self.simulation_time()
        return self.obstacle_spawned

    def delete_obstacle(self):
        if not self.delete_client.service_is_ready():
            return False
        request = DeleteEntity.Request()
        request.entity.name = DYNAMIC_OBSTACLE_NAME
        request.entity.type = Entity.MODEL
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.navigator,
            future,
            timeout_sec=5.0,
        )
        if not future.done() or future.result() is None:
            return False
        deleted = future.result().success
        if deleted:
            self.obstacle_spawned = False
            self.obstacle_spawn_simulation_time = None
        return deleted

    def spawn_blockage(self):
        """Seal the dedicated world's only doorway."""
        for seal in self.blocked_road_seals:
            name, centre_x, centre_y, size_x, size_y = seal
            request = SpawnEntity.Request()
            request.entity_factory.name = name
            request.entity_factory.allow_renaming = False
            request.entity_factory.sdf = blocked_road_seal_sdf(
                name, size_x, size_y
            )
            request.entity_factory.pose.position.x = centre_x
            request.entity_factory.pose.position.y = centre_y
            request.entity_factory.pose.position.z = 0.60
            request.entity_factory.pose.orientation.w = 1.0
            request.entity_factory.relative_to = "world"
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.navigator, future, timeout_sec=5.0
            )
            if not future.done() or future.result() is None:
                return False
            if not future.result().success:
                return False
        self.blockage_spawned = True
        return True

    def delete_blockage(self):
        if not self.delete_client.service_is_ready():
            return False
        removed = True
        for name, _, _, _, _ in self.blocked_road_seals:
            request = DeleteEntity.Request()
            request.entity.name = name
            request.entity.type = Entity.MODEL
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.navigator, future, timeout_sec=5.0
            )
            if not future.done() or future.result() is None:
                removed = False
            elif not future.result().success:
                removed = False
        if removed:
            self.blockage_spawned = False
        return removed

    def run_blocked_road(
        self,
        give_up_budget,
        criteria,
        inscribed_diameter,
        robot_circumscribed_radius,
        wall_watchdog_timeout,
        dependency_grace,
    ):
        """Seal the only route to the goal and score how the robot gives up."""
        if not self.wait_for_gazebo_services():
            return None, "GAZEBO_SERVICES_UNAVAILABLE"
        self.delete_blockage()

        goal_pose = self.make_pose(*BLOCKED_ROAD_GOAL)
        start_simulation_time = self.simulation_time()
        start_wall_time = time.monotonic()
        last_report_time = start_wall_time - 5.0
        dependency_missing_since = None
        recoveries = 0
        ended_within_budget = True
        start_position = (0.0, 0.0)
        if self.latest_amcl_pose is not None:
            start_position = (
                self.latest_amcl_pose.position.x,
                self.latest_amcl_pose.position.y,
            )

        print(
            f"blocked-road goal: ({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f}); the opening is sealed "
            f"once the robot has committed to the route",
            flush=True,
        )
        started, failure = self.start_navigation_goal(
            goal_pose, dependency_grace
        )
        if not started:
            return None, failure

        while not self.navigator.isTaskComplete():
            now = time.monotonic()
            elapsed = self.simulation_time() - start_simulation_time
            if now - start_wall_time > wall_watchdog_timeout:
                self.cancel_navigation()
                return None, "WALL_WATCHDOG_TIMEOUT"

            if self.navigator.nav_to_pose_client.server_is_ready():
                dependency_missing_since = None
            elif dependency_missing_since is None:
                dependency_missing_since = now
            elif now - dependency_missing_since > dependency_grace:
                self.cancel_navigation()
                return None, "NAV2_RUNTIME_LOST"
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                self.latest_navigation_pose = feedback.current_pose.pose
                position = feedback.current_pose.pose.position
                recoveries = max(recoveries, feedback.number_of_recoveries)

                travelled = math.hypot(
                    position.x - start_position[0],
                    position.y - start_position[1],
                )
                if (
                    not self.blockage_spawned
                    and travelled >= BLOCKED_ROAD_COMMIT_DISTANCE
                ):
                    if not self.spawn_blockage():
                        self.cancel_navigation()
                        return None, "SPAWN_FAILED"
                    print(
                        f"  sealed the opening into the pocket while the "
                        f"robot was at ({position.x:.2f}, {position.y:.2f})",
                        flush=True,
                    )

                if self.blockage_spawned:
                    self.minimum_blockage_distance = min(
                        [self.minimum_blockage_distance]
                        + [
                            distance_to_seal(position.x, position.y, seal)
                            for seal in self.blocked_road_seals
                        ]
                    )

                if now - last_report_time >= 5.0:
                    print(
                        f"  progress: position=({position.x:.2f}, "
                        f"{position.y:.2f}) recoveries={recoveries} "
                        f"widest_gap={self.narrowest_doorway_gap:.2f} m "
                        "centre_distance="
                        f"{self.minimum_blockage_distance:.2f} m "
                        f"elapsed={elapsed:.0f}/{give_up_budget:.0f} sim s",
                        flush=True,
                    )
                    last_report_time = now

            # The scenario's own budget, deliberately shorter than the launch
            # timeout: a run that only ended because the launch timer fired
            # would be charged to the environment, and unbounded recovery is
            # exactly the defect this scenario exists to catch.
            if elapsed > give_up_budget:
                ended_within_budget = False
                self.cancel_navigation()
                break

        goal_reached = (
            self.navigator.getResult() == TaskResult.SUCCEEDED
            if ended_within_budget
            else False
        )
        no_path, planner_detail = self.planner_reports_no_path(goal_pose)
        if no_path is None:
            return None, "PLANNER_UNAVAILABLE"
        peak_speed = self.measure_rest_speed(2.0)
        minimum_clearance = (
            self.minimum_blockage_distance - robot_circumscribed_radius
        )

        observation = BlockedRoadObservation(
            goal_reached=goal_reached,
            blockage_closed_in_costmap=(
                self.narrowest_doorway_gap < inscribed_diameter
            ),
            planner_reported_no_path=no_path,
            final_speed=peak_speed,
            recoveries=recoveries,
            minimum_blockage_clearance=minimum_clearance,
            collision_actions=self.collision_actions,
            ended_within_budget=ended_within_budget,
        )
        verdict, checks = evaluate_blocked_road(observation, criteria)
        return {
            "verdict": verdict,
            "checks": checks,
            "observation": observation,
            "planner_detail": planner_detail,
            "elapsed": self.simulation_time() - start_simulation_time,
        }, None

    def run_dynamic_obstacle(self, timeout, minimum_path_deviation):
        if not self.wait_for_gazebo_services():
            return {
                "result": "GAZEBO_SERVICES_UNAVAILABLE",
                "elapsed": 0.0,
                "remaining": math.nan,
                "recoveries": 0,
                "goal_error": math.nan,
            }

        # Remove an entity left behind by an interrupted previous test.
        self.delete_obstacle()
        goal_pose = self.make_pose(*DYNAMIC_GOAL)
        start_simulation_time = self.simulation_time()
        start_wall_time = time.monotonic()
        last_report_time = start_wall_time - 5.0
        max_recoveries = 0
        last_distance = math.nan

        print(
            "dynamic goal: "
            f"({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f})",
            flush=True,
        )
        if not self.navigator.goToPose(goal_pose):
            return {
                "result": "REJECTED",
                "elapsed": 0.0,
                "remaining": last_distance,
                "recoveries": max_recoveries,
                "goal_error": math.nan,
            }

        while not self.navigator.isTaskComplete():
            now = time.monotonic()
            simulation_elapsed = (
                self.simulation_time() - start_simulation_time
            )
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                self.latest_navigation_pose = feedback.current_pose.pose
                position = feedback.current_pose.pose.position
                if self.detour_monitor is None:
                    self.detour_monitor = NominalRouteDetour(
                        (position.x, position.y),
                        DYNAMIC_GOAL[:2],
                        (self.dynamic_obstacle_x, self.dynamic_obstacle_y),
                        minimum_path_deviation,
                    )
                max_recoveries = max(
                    max_recoveries,
                    feedback.number_of_recoveries,
                )
                last_distance = feedback.distance_remaining

                if not self.obstacle_spawned and position.x <= -0.25:
                    if not self.spawn_obstacle():
                        self.navigator.cancelTask()
                        return {
                            "result": "SPAWN_FAILED",
                            "elapsed": simulation_elapsed,
                            "remaining": last_distance,
                            "recoveries": max_recoveries,
                            "goal_error": math.nan,
                        }
                    print(
                        "  spawned 0.60 m obstacle at "
                        f"({self.dynamic_obstacle_x:.2f}, "
                        f"{self.dynamic_obstacle_y:.2f})",
                        flush=True,
                    )

                if self.obstacle_spawned:
                    obstacle_distance = math.hypot(
                        position.x - self.dynamic_obstacle_x,
                        position.y - self.dynamic_obstacle_y,
                    )
                    self.minimum_obstacle_distance = min(
                        self.minimum_obstacle_distance,
                        obstacle_distance,
                    )
                    self.detour_monitor.observe((position.x, position.y))
                    self.maximum_path_deviation = (
                        self.detour_monitor.maximum_local_deviation
                    )

                if now - last_report_time >= 5.0:
                    print(
                        "  progress: "
                        f"position=({position.x:.2f}, {position.y:.2f}) "
                        f"remaining={last_distance:.2f} m "
                        f"global_observed={self.global_costmap_observed} "
                        f"local_observed={self.local_costmap_observed}",
                        flush=True,
                    )
                    last_report_time = now

            if simulation_elapsed > timeout:
                self.navigator.cancelTask()
                return {
                    "result": "TIMEOUT",
                    "elapsed": simulation_elapsed,
                    "remaining": last_distance,
                    "recoveries": max_recoveries,
                    "goal_error": math.nan,
                }
            if now - start_wall_time > max(timeout * 5.0, timeout + 120.0):
                self.navigator.cancelTask()
                return {
                    "result": "WALL_WATCHDOG_TIMEOUT",
                    "elapsed": simulation_elapsed,
                    "remaining": last_distance,
                    "recoveries": max_recoveries,
                    "goal_error": math.nan,
                }

        elapsed = self.simulation_time() - start_simulation_time
        task_result = self.navigator.getResult()
        result_name = {
            TaskResult.SUCCEEDED: "SUCCEEDED",
            TaskResult.CANCELED: "CANCELED",
            TaskResult.FAILED: "FAILED",
        }.get(task_result, "UNKNOWN")
        rclpy.spin_once(self.navigator, timeout_sec=0.2)
        goal_error = math.nan
        final_pose = self.latest_amcl_pose or self.latest_navigation_pose
        if final_pose is not None:
            dx = final_pose.position.x - goal_pose.pose.position.x
            dy = final_pose.position.y - goal_pose.pose.position.y
            goal_error = math.hypot(dx, dy)

        if (
            result_name == "SUCCEEDED"
            and not (
                self.global_costmap_observed
                and self.local_costmap_observed
            )
        ):
            result_name = "OBSTACLE_NOT_OBSERVED"

        return {
            "result": result_name,
            "goal_reached": task_result == TaskResult.SUCCEEDED,
            "elapsed": elapsed,
            "remaining": last_distance,
            "recoveries": max_recoveries,
            "goal_error": goal_error,
            "global_costmap_latency": self.global_costmap_latency,
            "local_costmap_latency": self.local_costmap_latency,
            "obstacle_on_nominal_route": (
                self.detour_monitor is not None
                and self.detour_monitor.obstacle_on_nominal_route
            ),
            "crossed_obstacle_station": (
                self.detour_monitor is not None
                and self.detour_monitor.crossed_obstacle_station
            ),
        }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the project's Nav2 navigation regression scenarios."
    )
    parser.add_argument(
        "--scenario",
        choices=("multi-goal", "dynamic-obstacle", "blocked-road"),
        default="multi-goal",
        help="Regression scenario to execute (default: multi-goal).",
    )
    parser.add_argument(
        "--localization-mode",
        choices=("amcl", "online-slam"),
        default="amcl",
        help=(
            "Wait for AMCL and its initial pose, or only for Nav2 when an "
            "online SLAM node owns map -> odom (default: amcl)."
        ),
    )
    parser.add_argument(
        "--obstacle-offset-y",
        type=float,
        default=0.0,
        help=(
            "Move the dynamic test obstacle perpendicular to the nominal "
            "route. Intended for the executable negative control."
        ),
    )
    parser.add_argument(
        "--pre-map-dynamic-route",
        action="store_true",
        help=(
            "Map the dynamic route incrementally and return to the origin "
            "before injecting the obstacle. Intended for directional online "
            "mapping sensors (default: disabled)."
        ),
    )
    parser.add_argument(
        "--minimum-dynamic-clearance",
        type=float,
        default=0.15,
        help="Minimum body-to-obstacle clearance in metres (default: 0.15).",
    )
    parser.add_argument(
        "--minimum-path-deviation",
        type=float,
        default=None,
        help=(
            "Minimum obstacle-local detour from the nominal route. Defaults "
            "to the robot plus obstacle circumscribed radii."
        ),
    )
    parser.add_argument(
        "--maximum-goal-error",
        type=float,
        default=0.35,
        help="Maximum final map-frame goal error in metres (default: 0.35).",
    )
    parser.add_argument(
        "--maximum-dynamic-recoveries",
        type=int,
        default=3,
        help="Maximum recoveries during a traversable obstruction (default: 3).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="Simulation-time timeout in seconds (default: 300).",
    )
    parser.add_argument(
        "--activation-timeout",
        type=float,
        default=120.0,
        help="Wall-clock timeout while waiting for Nav2 (default: 120).",
    )
    parser.add_argument(
        "--maximum-global-costmap-latency",
        type=float,
        default=2.2,
        help=(
            "Maximum simulated seconds from obstacle spawn to global costmap "
            "publication (default: 2.2)."
        ),
    )
    parser.add_argument(
        "--maximum-local-costmap-latency",
        type=float,
        default=0.8,
        help=(
            "Maximum simulated seconds from obstacle spawn to local costmap "
            "publication (default: 0.8)."
        ),
    )
    parser.add_argument(
        "--world-name",
        default=None,
        help=(
            "Gazebo world whose entity services are used. Defaults to "
            "blocked_road_world for the blocked-road scenario and slam_world "
            "otherwise."
        ),
    )
    parser.add_argument(
        "--give-up-budget",
        type=float,
        default=180.0,
        help=(
            "Simulated seconds the robot may spend before it must have given "
            "up on a blocked goal. Sized from the route it must actually "
            "drive -- roughly 6.5 m between the two doorways at the planner's "
            "nominal speed -- with margin, and deliberately far shorter than "
            "--timeout: exceeding it is the robot's failure, not the "
            "environment's (default: 180)."
        ),
    )
    parser.add_argument(
        "--minimum-clearance",
        type=float,
        default=0.20,
        help="Metres the robot must keep from the blockage (default: 0.20).",
    )
    parser.add_argument(
        "--robot-circumscribed-radius",
        type=float,
        default=0.336,
        help=(
            "Conservative circular body envelope subtracted from the "
            "base-centre-to-seal distance (default: 0.336)."
        ),
    )
    parser.add_argument(
        "--wall-watchdog-timeout",
        type=float,
        default=300.0,
        help=(
            "Steady-clock timeout for a blocked-road run, so a frozen /clock "
            "cannot hang the regression (default: 300)."
        ),
    )
    parser.add_argument(
        "--dependency-grace",
        type=float,
        default=15.0,
        help=(
            "Steady-clock grace for action-server responses or loss "
            "(default: 15)."
        ),
    )
    parser.add_argument(
        "--seal-offset-x",
        type=float,
        default=0.0,
        help=(
            "Move the test seal in x. Intended for the executable negative "
            "control; normal acceptance uses zero (default: 0)."
        ),
    )
    parser.add_argument(
        "--minimum-recoveries",
        type=int,
        default=1,
        help=(
            "Recovery attempts the robot must make before giving up; zero "
            "would turn the check off rather than relax it (default: 1)."
        ),
    )
    parser.add_argument(
        "--maximum-recoveries",
        type=int,
        default=18,
        help=(
            "Recovery attempts the robot may make. Derived from the shipped "
            "navigate_to_pose_w_replanning_and_recovery.xml: six outer "
            "retries, each able to drive one planner retry, one controller "
            "retry and one RoundRobin recovery action, so 6 x 3 = 18. "
            "Exceeding it means the behaviour tree is not the one this "
            "threshold was read from (default: 18)."
        ),
    )
    parser.add_argument(
        "--inscribed-diameter",
        type=float,
        default=0.31,
        help=(
            "Widest costmap gap the robot could still be planned through. "
            "Twice the footprint's inscribed radius, which is what the "
            "inflation layer marks as untraversable (default: 0.31)."
        ),
    )
    parser.add_argument(
        "--rest-speed",
        type=float,
        default=0.05,
        help="Speed below which the robot counts as stopped (default: 0.05).",
    )
    raw_arguments = sys.argv if argv is None else [sys.argv[0], *argv]
    return parser.parse_args(remove_ros_args(raw_arguments)[1:])


def main():
    args = parse_arguments()
    minimum_path_deviation = args.minimum_path_deviation
    if minimum_path_deviation is None:
        minimum_path_deviation = (
            args.robot_circumscribed_radius + DYNAMIC_OBSTACLE_RADIUS
        )
    for name, value in (
        ("robot_circumscribed_radius", args.robot_circumscribed_radius),
        ("minimum_path_deviation", minimum_path_deviation),
        ("wall_watchdog_timeout", args.wall_watchdog_timeout),
        ("dependency_grace", args.dependency_grace),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    for name, value in (
        ("seal_offset_x", args.seal_offset_x),
        ("obstacle_offset_y", args.obstacle_offset_y),
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    rclpy.init()
    navigator = BasicNavigator("navigation_regression")
    navigator.set_parameters(
        [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    world_name = args.world_name
    if world_name is None:
        world_name = (
            BLOCKED_ROAD_WORLD
            if args.scenario == "blocked-road"
            else "slam_world"
        )
    regression = NavigationRegression(
        navigator,
        world_name,
        localization_mode=args.localization_mode,
        seal_offset_x=args.seal_offset_x,
        obstacle_offset_y=args.obstacle_offset_y,
    )

    try:
        dependency = (
            "Nav2 and the AMCL initial pose"
            if args.localization_mode == "amcl"
            else "Nav2 with online SLAM localization"
        )
        print(f"waiting for {dependency}...", flush=True)
        if not regression.wait_until_nav2_active(args.activation_timeout):
            print(
                "summary: result=NAV2_ACTIVATION_TIMEOUT "
                "failure_class=dependency_lost",
                flush=True,
            )
            print("VERDICT INFRA_UNSTABLE", flush=True)
            raise SystemExit(2)
        if args.scenario == "multi-goal":
            print(f"sending {len(DEFAULT_ROUTE)} navigation poses", flush=True)
            result = regression.run(DEFAULT_ROUTE, args.timeout)
            print(
                "summary: "
                f"result={result['result']} "
                f"failed_waypoint={result['failed_waypoint']} "
                f"elapsed={result['elapsed']:.1f} s "
                f"remaining={result['remaining']:.2f} m "
                f"recoveries={result['recoveries']} "
                f"collision_actions={regression.collision_actions} "
                f"goal_error={result['goal_error']:.3f} m",
                flush=True,
            )
        elif args.scenario == "blocked-road":
            print("starting blocked-road scenario", flush=True)
            criteria = BlockedRoadCriteria(
                minimum_clearance=args.minimum_clearance,
                minimum_recoveries=args.minimum_recoveries,
                maximum_recoveries=args.maximum_recoveries,
                rest_speed=args.rest_speed,
            )
            report, aborted = regression.run_blocked_road(
                args.give_up_budget,
                criteria,
                args.inscribed_diameter,
                args.robot_circumscribed_radius,
                args.wall_watchdog_timeout,
                args.dependency_grace,
            )
            if report is None:
                failure_class = (
                    "dependency_lost"
                    if aborted in DEPENDENCY_FAILURES
                    else "internal"
                )
                print(
                    "summary: "
                    f"result={aborted} "
                    f"failure_class={failure_class}",
                    flush=True,
                )
                verdict = (
                    "INFRA_UNSTABLE"
                    if aborted in DEPENDENCY_FAILURES
                    else "FAIL"
                )
                print(f"VERDICT {verdict}", flush=True)
                raise SystemExit(2 if verdict == "INFRA_UNSTABLE" else 1)
            observation = report["observation"]
            print(
                "summary: "
                f"verdict={report['verdict']} "
                f"elapsed={report['elapsed']:.1f} s "
                f"goal_reached={observation.goal_reached} "
                f"doorway_closed={observation.blockage_closed_in_costmap} "
                f"(narrowest gap seen {regression.narrowest_doorway_gap:.2f} m,"
                f" gap now {regression.widest_doorway_gap:.2f} m) "
                f"planner_no_path={observation.planner_reported_no_path} "
                f"({report['planner_detail']}) "
                f"final_speed={observation.final_speed:.3f} m/s "
                f"recoveries={observation.recoveries} "
                f"centre_distance={regression.minimum_blockage_distance:.3f} m "
                f"clearance={observation.minimum_blockage_clearance:.3f} m "
                f"ended_within_budget={observation.ended_within_budget} "
                f"collision_actions={regression.collision_actions}",
                flush=True,
            )
            failed = failed_checks(report["checks"])
            if failed:
                print(f"  failed checks: {', '.join(failed)}", flush=True)
                print(
                    f"  core failures: "
                    f"{', '.join(core_failures(report['checks'])) or 'none'}",
                    flush=True,
                )
            # Printed in the form exploration_campaign already greps for, so a
            # blocked-road run can be repeated by the same campaign driver.
            print(f"VERDICT {report['verdict']}", flush=True)
            if report["verdict"] != "PASS":
                raise SystemExit(1)
            result = {"result": "SUCCEEDED"}
        else:
            if args.pre_map_dynamic_route:
                print(
                    "mapping the directional sensor route before navigation...",
                    flush=True,
                )
                mapping_turn = regression.run(
                    (
                        (0.0, 0.0, math.pi),
                        (-2.0, -0.5, math.pi),
                        (0.0, 0.0, 0.0),
                    ),
                    min(args.timeout, 120.0),
                )
                if mapping_turn["result"] != "SUCCEEDED":
                    print(
                        "summary: result=INITIAL_MAPPING_ROUTE_"
                        f"{mapping_turn['result']}",
                        flush=True,
                    )
                    print("VERDICT FAIL", flush=True)
                    raise SystemExit(1)
            print("starting dynamic-obstacle scenario", flush=True)
            result = regression.run_dynamic_obstacle(
                args.timeout, minimum_path_deviation
            )
            minimum_clearance = (
                regression.minimum_obstacle_distance
                - args.robot_circumscribed_radius
                - DYNAMIC_OBSTACLE_RADIUS
            )
            observation = DynamicObstacleObservation(
                goal_reached=result.get("goal_reached", False),
                global_costmap_observed=regression.global_costmap_observed,
                local_costmap_observed=regression.local_costmap_observed,
                global_costmap_latency=result.get(
                    "global_costmap_latency", math.inf
                ),
                local_costmap_latency=result.get(
                    "local_costmap_latency", math.inf
                ),
                minimum_obstacle_clearance=minimum_clearance,
                maximum_path_deviation=regression.maximum_path_deviation,
                obstacle_on_nominal_route=result.get(
                    "obstacle_on_nominal_route", False
                ),
                crossed_obstacle_station=result.get(
                    "crossed_obstacle_station", False
                ),
                goal_error=result["goal_error"],
                recoveries=result["recoveries"],
                collision_actions=regression.collision_actions,
            )
            criteria = DynamicObstacleCriteria(
                maximum_global_costmap_latency=(
                    args.maximum_global_costmap_latency
                ),
                maximum_local_costmap_latency=args.maximum_local_costmap_latency,
                minimum_clearance=args.minimum_dynamic_clearance,
                minimum_path_deviation=minimum_path_deviation,
                maximum_goal_error=args.maximum_goal_error,
                maximum_recoveries=args.maximum_dynamic_recoveries,
            )
            verdict, checks = evaluate_dynamic_obstacle(observation, criteria)
            print(
                "summary: "
                f"verdict={verdict} result={result['result']} "
                f"elapsed={result['elapsed']:.1f} s "
                f"remaining={result['remaining']:.2f} m "
                f"recoveries={result['recoveries']} "
                f"collision_actions={regression.collision_actions} "
                f"global_costmap_observed="
                f"{regression.global_costmap_observed} "
                f"local_costmap_observed="
                f"{regression.local_costmap_observed} "
                f"global_costmap_latency="
                f"{result.get('global_costmap_latency', math.nan):.3f} s "
                f"local_costmap_latency="
                f"{result.get('local_costmap_latency', math.nan):.3f} s "
                f"minimum_obstacle_distance="
                f"{regression.minimum_obstacle_distance:.3f} m "
                f"clearance={minimum_clearance:.3f} m "
                f"maximum_path_deviation="
                f"{regression.maximum_path_deviation:.3f} m "
                f"obstacle_on_nominal_route="
                f"{result.get('obstacle_on_nominal_route', False)} "
                f"crossed_obstacle_station="
                f"{result.get('crossed_obstacle_station', False)} "
                f"goal_error={result['goal_error']:.3f} m",
                flush=True,
            )
            failed = failed_dynamic_checks(checks)
            if failed:
                print(f"  failed checks: {', '.join(failed)}", flush=True)
            print(f"VERDICT {verdict}", flush=True)
            if verdict != "PASS":
                raise SystemExit(1)
        if result["result"] != "SUCCEEDED":
            raise SystemExit(1)
    finally:
        if regression.obstacle_spawned:
            deleted = regression.delete_obstacle()
            print(f"dynamic obstacle removed={deleted}", flush=True)
        if regression.blockage_spawned:
            deleted = regression.delete_blockage()
            print(f"blockage removed={deleted}", flush=True)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
