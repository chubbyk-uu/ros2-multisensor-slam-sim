#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.msg import CollisionMonitorState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity


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
DYNAMIC_OBSTACLE_NAME = "navigation_test_obstacle"
DYNAMIC_OBSTACLE_X = -1.35
DYNAMIC_OBSTACLE_Y = -0.34
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
    def __init__(self, navigator):
        self.navigator = navigator
        self.latest_amcl_pose = None
        self.collision_actions = 0
        self.last_collision_action = CollisionMonitorState.DO_NOTHING
        self.obstacle_spawned = False
        self.global_costmap_observed = False
        self.local_costmap_observed = False
        self.minimum_obstacle_distance = math.inf
        self.maximum_path_deviation = 0.0
        self.spawn_client = self.navigator.create_client(
            SpawnEntity,
            "/world/slam_world/create",
        )
        self.delete_client = self.navigator.create_client(
            DeleteEntity,
            "/world/slam_world/remove",
        )
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
        cell_x = round((DYNAMIC_OBSTACLE_X - origin.x) / resolution)
        cell_y = round((DYNAMIC_OBSTACLE_Y - origin.y) / resolution)
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

    def _global_costmap_callback(self, message):
        if (
            self.obstacle_spawned
            and not self.global_costmap_observed
            and self._costmap_contains_obstacle(message)
        ):
            self.global_costmap_observed = True
            print(
                "  global costmap marked the spawned obstacle as lethal",
                flush=True,
            )

    def _local_costmap_callback(self, message):
        if (
            self.obstacle_spawned
            and not self.local_costmap_observed
            and self._costmap_contains_obstacle(message)
        ):
            self.local_costmap_observed = True
            print(
                "  local costmap marked the spawned obstacle as lethal",
                flush=True,
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

    def run(self, route, timeout):
        poses = [self.make_pose(*waypoint) for waypoint in route]
        goal = poses[-1].pose
        start_time = time.monotonic()
        max_recoveries = 0
        last_distance = math.nan

        for index, pose in enumerate(poses, start=1):
            segment_start = time.monotonic()
            last_report_time = segment_start - 5.0
            print(
                f"waypoint {index}/{len(poses)}: "
                f"goal=({pose.pose.position.x:.2f}, "
                f"{pose.pose.position.y:.2f})",
                flush=True,
            )
            if not self.navigator.goToPose(pose):
                return {
                    "result": "REJECTED",
                    "elapsed": time.monotonic() - start_time,
                    "remaining": last_distance,
                    "recoveries": max_recoveries,
                    "goal_error": math.nan,
                    "failed_waypoint": index,
                }

            while not self.navigator.isTaskComplete():
                now = time.monotonic()
                feedback = self.navigator.getFeedback()
                if feedback is not None:
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

                if now - start_time > timeout:
                    self.navigator.cancelTask()
                    return {
                        "result": "TIMEOUT",
                        "elapsed": now - start_time,
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
                    "elapsed": time.monotonic() - start_time,
                    "remaining": last_distance,
                    "recoveries": max_recoveries,
                    "goal_error": math.nan,
                    "failed_waypoint": index,
                }

            print(
                f"  reached in {time.monotonic() - segment_start:.1f} s",
                flush=True,
            )

        elapsed = time.monotonic() - start_time
        rclpy.spin_once(self.navigator, timeout_sec=0.2)
        goal_error = math.nan
        if self.latest_amcl_pose is not None:
            dx = self.latest_amcl_pose.position.x - goal.position.x
            dy = self.latest_amcl_pose.position.y - goal.position.y
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
        request.entity_factory.pose.position.x = DYNAMIC_OBSTACLE_X
        request.entity_factory.pose.position.y = DYNAMIC_OBSTACLE_Y
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
        return deleted

    def run_dynamic_obstacle(self, timeout):
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
        start_time = time.monotonic()
        last_report_time = start_time - 5.0
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
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                position = feedback.current_pose.pose.position
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
                            "elapsed": now - start_time,
                            "remaining": last_distance,
                            "recoveries": max_recoveries,
                            "goal_error": math.nan,
                        }
                    print(
                        "  spawned 0.60 m obstacle at "
                        f"({DYNAMIC_OBSTACLE_X:.2f}, "
                        f"{DYNAMIC_OBSTACLE_Y:.2f})",
                        flush=True,
                    )

                if self.obstacle_spawned:
                    obstacle_distance = math.hypot(
                        position.x - DYNAMIC_OBSTACLE_X,
                        position.y - DYNAMIC_OBSTACLE_Y,
                    )
                    self.minimum_obstacle_distance = min(
                        self.minimum_obstacle_distance,
                        obstacle_distance,
                    )
                    # Perpendicular distance from the original straight path
                    # y = 0.25 x between the start and the dynamic goal.
                    path_deviation = abs(position.y - 0.25 * position.x)
                    path_deviation /= math.sqrt(1.0 + 0.25**2)
                    self.maximum_path_deviation = max(
                        self.maximum_path_deviation,
                        path_deviation,
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

            if now - start_time > timeout:
                self.navigator.cancelTask()
                return {
                    "result": "TIMEOUT",
                    "elapsed": now - start_time,
                    "remaining": last_distance,
                    "recoveries": max_recoveries,
                    "goal_error": math.nan,
                }

        elapsed = time.monotonic() - start_time
        task_result = self.navigator.getResult()
        result_name = {
            TaskResult.SUCCEEDED: "SUCCEEDED",
            TaskResult.CANCELED: "CANCELED",
            TaskResult.FAILED: "FAILED",
        }.get(task_result, "UNKNOWN")
        rclpy.spin_once(self.navigator, timeout_sec=0.2)
        goal_error = math.nan
        if self.latest_amcl_pose is not None:
            dx = self.latest_amcl_pose.position.x - goal_pose.pose.position.x
            dy = self.latest_amcl_pose.position.y - goal_pose.pose.position.y
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
            "elapsed": elapsed,
            "remaining": last_distance,
            "recoveries": max_recoveries,
            "goal_error": goal_error,
        }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Run the project's Nav2 navigation regression scenarios."
    )
    parser.add_argument(
        "--scenario",
        choices=("multi-goal", "dynamic-obstacle"),
        default="multi-goal",
        help="Regression scenario to execute (default: multi-goal).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="Wall-clock timeout in seconds (default: 300).",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    rclpy.init()
    navigator = BasicNavigator("navigation_regression")
    regression = NavigationRegression(navigator)

    try:
        print("waiting for Nav2 and the AMCL initial pose...", flush=True)
        navigator.waitUntilNav2Active()
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
        else:
            print("starting dynamic-obstacle scenario", flush=True)
            result = regression.run_dynamic_obstacle(args.timeout)
            print(
                "summary: "
                f"result={result['result']} "
                f"elapsed={result['elapsed']:.1f} s "
                f"remaining={result['remaining']:.2f} m "
                f"recoveries={result['recoveries']} "
                f"collision_actions={regression.collision_actions} "
                f"global_costmap_observed="
                f"{regression.global_costmap_observed} "
                f"local_costmap_observed="
                f"{regression.local_costmap_observed} "
                f"minimum_obstacle_distance="
                f"{regression.minimum_obstacle_distance:.3f} m "
                f"maximum_path_deviation="
                f"{regression.maximum_path_deviation:.3f} m "
                f"goal_error={result['goal_error']:.3f} m",
                flush=True,
            )
        if result["result"] != "SUCCEEDED":
            raise SystemExit(1)
    finally:
        if regression.obstacle_spawned:
            deleted = regression.delete_obstacle()
            print(f"dynamic obstacle removed={deleted}", flush=True)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
