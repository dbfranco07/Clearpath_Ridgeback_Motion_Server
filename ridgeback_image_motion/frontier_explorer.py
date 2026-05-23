#!/usr/bin/env python3
"""Frontier-based exploration on the SLAM occupancy grid.

Detects frontier cells (free cells adjacent to unknown cells), groups
them into clusters, picks the closest viable cluster centroid, and sends
a NavigateToPose goal. Replans periodically and on goal completion. The
mission orchestrator can suspend/resume exploration via /frontier/cancel.

Status JSON published on /frontier/status:
    {state, current_goal:[x,y], attempts, last_error, frontier_count}
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, yaw_to_quaternion
except ImportError:
    from autonomy_common import json_dumps, yaw_to_quaternion  # type: ignore[no-redef]


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("status_topic", "/frontier/status")
        self.declare_parameter("cancel_topic", "/frontier/cancel")
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("min_frontier_size_cells", 10)
        self.declare_parameter("goal_blacklist_radius_m", 0.5)
        self.declare_parameter("replan_period_s", 3.0)
        self.declare_parameter("max_attempts_per_goal", 2)
        self.declare_parameter("goal_padding_m", 0.3)

        self._min_size = int(self.get_parameter("min_frontier_size_cells").value)
        self._blacklist_radius = float(self.get_parameter("goal_blacklist_radius_m").value)
        self._replan_period = float(self.get_parameter("replan_period_s").value)
        self._max_attempts = int(self.get_parameter("max_attempts_per_goal").value)
        self._goal_padding = float(self.get_parameter("goal_padding_m").value)

        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        self._cancelled = False
        self._active = False
        self._current_goal: tuple[float, float] | None = None
        self._goal_handle = None
        self._attempts: dict[tuple[float, float], int] = {}
        self._blacklist: list[tuple[float, float]] = []
        self._last_error = ""
        self._frontier_count = 0
        self._last_replan = 0.0

        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, _LATCHED_QOS
        )
        self.create_subscription(
            Bool, self.get_parameter("cancel_topic").value, self._on_cancel, 10
        )
        self._pub_status = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self._action = ActionClient(self, NavigateToPose, self.get_parameter("nav_action").value)
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"frontier_explorer ready (min_size={self._min_size}, "
            f"blacklist_r={self._blacklist_radius}m, replan={self._replan_period}s)"
        )

    # --- callbacks ----------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._map = msg

    def _on_cancel(self, msg: Bool) -> None:
        with self._lock:
            self._cancelled = bool(msg.data)
            if self._cancelled and self._goal_handle is not None:
                self.get_logger().info("frontier exploration cancelled by mission")
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
                self._current_goal = None

    # --- main loop ----------------------------------------------------------
    def _tick(self) -> None:
        with self._lock:
            cancelled = self._cancelled
            active = self._active
            need_replan = (time.time() - self._last_replan) > self._replan_period

        if cancelled:
            self._publish_status("cancelled")
            return
        if active and not need_replan:
            self._publish_status("navigating")
            return

        # Compute next frontier goal.
        self._last_replan = time.time()
        with self._lock:
            grid = self._map
        if grid is None:
            self._publish_status("waiting_for_map")
            return

        clusters = self._find_frontier_clusters(grid)
        with self._lock:
            self._frontier_count = len(clusters)

        if not clusters:
            self._last_error = "no_frontiers"
            self._publish_status("no_frontiers")
            return

        goal = self._pick_best_cluster(grid, clusters)
        if goal is None:
            self._last_error = "all_blacklisted"
            self._publish_status("blocked")
            return

        if active and self._current_goal is not None and self._dist(goal, self._current_goal) < 0.4:
            # Already heading there — let it finish.
            self._publish_status("navigating")
            return

        self._send_goal(goal)

    # --- frontier detection -------------------------------------------------
    def _find_frontier_clusters(self, grid: OccupancyGrid) -> list[tuple[int, int, int]]:
        info = grid.info
        w, h = int(info.width), int(info.height)
        if w == 0 or h == 0:
            return []
        data = np.asarray(grid.data, dtype=np.int8).reshape((h, w))
        free = data == 0
        unknown = data < 0
        obstacle = data > 50
        # Frontier = free cell adjacent to unknown.
        shifted = np.zeros_like(unknown)
        shifted[1:, :] |= unknown[:-1, :]
        shifted[:-1, :] |= unknown[1:, :]
        shifted[:, 1:] |= unknown[:, :-1]
        shifted[:, :-1] |= unknown[:, 1:]
        frontier = free & shifted

        # Erode near obstacles slightly so we don't try to drive into walls.
        for _ in range(1):
            obstacle_neighbor = np.zeros_like(obstacle)
            obstacle_neighbor[1:, :] |= obstacle[:-1, :]
            obstacle_neighbor[:-1, :] |= obstacle[1:, :]
            obstacle_neighbor[:, 1:] |= obstacle[:, :-1]
            obstacle_neighbor[:, :-1] |= obstacle[:, 1:]
            frontier &= ~obstacle_neighbor

        # Connected-components via flood fill (BFS).
        visited = np.zeros_like(frontier)
        clusters: list[tuple[int, int, int]] = []
        ys, xs = np.where(frontier)
        for y, x in zip(ys, xs):
            if visited[y, x]:
                continue
            stack = [(int(y), int(x))]
            sum_y = sum_x = count = 0
            while stack:
                cy, cx = stack.pop()
                if cy < 0 or cy >= h or cx < 0 or cx >= w:
                    continue
                if visited[cy, cx] or not frontier[cy, cx]:
                    continue
                visited[cy, cx] = True
                sum_y += cy
                sum_x += cx
                count += 1
                stack.extend(((cy + 1, cx), (cy - 1, cx), (cy, cx + 1), (cy, cx - 1)))
            if count >= self._min_size:
                clusters.append((sum_x // count, sum_y // count, count))
        return clusters

    def _pick_best_cluster(
        self,
        grid: OccupancyGrid,
        clusters: list[tuple[int, int, int]],
    ) -> tuple[float, float] | None:
        info = grid.info
        ox, oy = float(info.origin.position.x), float(info.origin.position.y)
        res = float(info.resolution)
        # Robot pose is (0,0) in map until we have TF; rank by cluster size as tiebreaker.
        candidates: list[tuple[float, float, int]] = []
        for cx, cy, size in clusters:
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            world = (round(wx, 2), round(wy, 2))
            if any(self._dist(world, b) < self._blacklist_radius for b in self._blacklist):
                continue
            attempts = self._attempts.get(world, 0)
            if attempts >= self._max_attempts:
                self._blacklist.append(world)
                continue
            candidates.append((wx, wy, size))
        if not candidates:
            return None
        # Prefer larger cluster (more unknown to reveal). Ties broken by recency.
        candidates.sort(key=lambda t: -t[2])
        return (candidates[0][0], candidates[0][1])

    # --- nav2 plumbing ------------------------------------------------------
    def _send_goal(self, goal: tuple[float, float]) -> None:
        if not self._action.wait_for_server(timeout_sec=10.0):
            self._last_error = "nav2_unavailable"
            self._publish_status("nav2_unavailable")
            return
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = goal[0]
        ps.pose.position.y = goal[1]
        ps.pose.orientation = yaw_to_quaternion(0.0)

        msg = NavigateToPose.Goal()
        msg.pose = ps

        with self._lock:
            self._current_goal = goal
            self._active = True
            self._attempts[goal] = self._attempts.get(goal, 0) + 1

        self.get_logger().info(f"frontier goal -> ({goal[0]:.2f}, {goal[1]:.2f})")
        future = self._action.send_goal_async(msg)
        future.add_done_callback(self._on_goal_response)
        self._publish_status("navigating")

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._last_error = f"send_goal_failed:{exc}"
            with self._lock:
                self._active = False
                self._current_goal = None
            return
        if not handle.accepted:
            self._last_error = "goal_rejected"
            with self._lock:
                if self._current_goal is not None:
                    self._blacklist.append(self._current_goal)
                self._active = False
                self._current_goal = None
            return
        with self._lock:
            self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            res = future.result()
            status = res.status
        except Exception as exc:
            status = GoalStatus.STATUS_ABORTED
            self._last_error = f"result_failed:{exc}"
        with self._lock:
            goal = self._current_goal
            self._active = False
            self._current_goal = None
            self._goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._last_error = ""
        elif status == GoalStatus.STATUS_CANCELED:
            self._last_error = "cancelled"
        else:
            if goal is not None and self._attempts.get(goal, 0) >= self._max_attempts:
                self._blacklist.append(goal)
            self._last_error = f"nav_status_{status}"

    # --- helpers ------------------------------------------------------------
    def _publish_status(self, state: str) -> None:
        with self._lock:
            payload: dict[str, Any] = {
                "state": state,
                "current_goal": list(self._current_goal) if self._current_goal else None,
                "attempts": self._attempts.get(self._current_goal, 0) if self._current_goal else 0,
                "last_error": self._last_error,
                "frontier_count": self._frontier_count,
                "blacklist_size": len(self._blacklist),
                "ts": time.time(),
            }
        msg = String()
        msg.data = json_dumps(payload)
        self._pub_status.publish(msg)

    @staticmethod
    def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
