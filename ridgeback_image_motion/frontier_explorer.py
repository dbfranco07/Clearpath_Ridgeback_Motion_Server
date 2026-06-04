#!/usr/bin/env python3
"""Frontier-based exploration on the SLAM occupancy grid.

Detects frontier cells (free cells adjacent to unknown cells), groups
them into clusters, scores each candidate with a weighted utility
(information gain − distance − recency − blacklist proximity + optional
direction hint), and sends a NavigateToPose goal to the best one. When
no candidate scores above the threshold, publishes state="no_frontiers"
so the mission orchestrator can declare the map complete.

Status JSON published on /frontier/status (TRANSIENT_LOCAL):
    {state, current_goal:[x,y], attempts, last_error, frontier_count,
     blacklist_size, score, ts}
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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, json_loads, yaw_to_quaternion
except ImportError:
    from autonomy_common import json_dumps, json_loads, yaw_to_quaternion  # type: ignore[no-redef]


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
        self.declare_parameter("hint_topic", "/frontier/hint")
        # External "this opening is too narrow / impassable" hints (e.g. from
        # passage_monitor). A point published here is blacklisted so the
        # explorer reroutes away from it instead of repeatedly trying to thread
        # a gap the robot doesn't fit through.
        self.declare_parameter("blacklist_point_topic", "/frontier/blacklist_point")
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("base_frame", "")
        self.declare_parameter("home_frame", "map")
        # 25 cells (~1.25 m at 0.05 res) is comfortably above clutter strips
        # while a real doorway frontier is typically 30-60 cells.
        self.declare_parameter("min_frontier_size_cells", 25)
        # 1.0 m so a single failed cluster blanks out the whole local patch,
        # preventing the explorer from rediscovering the same trap by
        # shifting the centroid 50-60 cm.
        self.declare_parameter("goal_blacklist_radius_m", 1.0)
        self.declare_parameter("replan_period_s", 3.0)
        self.declare_parameter("max_attempts_per_goal", 2)
        self.declare_parameter("goal_padding_m", 0.3)
        # Reject frontier candidates whose centroid is within this many cells
        # of the SLAM map edge. Must be ≥ ceil(planner tolerance / resolution).
        # NavFn's tolerance search sweeps cells in an expanding square around
        # the goal; if those cells overflow the costmap the planner emits a
        # worldToMap-failed log line per overflow cell. With tolerance 0.6 m
        # and resolution 0.05 m, the search reaches 12 cells; 16 gives a
        # 4-cell safety buffer for static_layer/SLAM-map sync lag.
        self.declare_parameter("edge_margin_cells", 16)
        # Weighted scoring (see _score_cluster). Higher = preferred.
        self.declare_parameter("score_w_info_gain", 1.0)
        self.declare_parameter("score_w_distance", 0.6)
        self.declare_parameter("score_w_recency", 0.4)
        self.declare_parameter("score_w_blacklist_prox", 0.8)
        self.declare_parameter("score_w_hint_align", 0.5)
        # Directional persistence: reward frontiers in the same direction as the
        # goal we last committed to. Without this, at a hallway T-junction the
        # two arms score almost equally and the robot dithers ("not sure left or
        # right"), re-aiming every goal cycle and making no progress. With it,
        # the robot commits to one arm, explores it, then takes the other.
        self.declare_parameter("score_w_persist", 0.6)
        # Penalty for clutter-induced "leaky" frontiers. Real openings
        # (doorways) have very few obstacle cells within 1 m of the
        # centroid; clutter forests (chair legs, fences) are dense with
        # them. This term is the difference between this honest signal
        # and the honeypot trap.
        self.declare_parameter("score_w_clutter", 0.7)
        self.declare_parameter("clutter_radius_m", 1.0)
        self.declare_parameter("recency_window_s", 60.0)
        self.declare_parameter("info_gain_radius_m", 2.0)
        self.declare_parameter("max_distance_cap_m", 8.0)
        # 0.15 keeps low-value candidates from being attempted at all.
        # Scores at 0.10 in logs were marginal frontiers that the planner
        # repeatedly failed to reach, triggering the recovery cascade.
        # Safe to raise because of the no_frontiers debounce in the
        # orchestrator (3 consecutive reports required).
        self.declare_parameter("min_score_threshold", 0.15)

        self._min_size = int(self.get_parameter("min_frontier_size_cells").value)
        self._blacklist_radius = float(self.get_parameter("goal_blacklist_radius_m").value)
        self._replan_period = float(self.get_parameter("replan_period_s").value)
        self._max_attempts = int(self.get_parameter("max_attempts_per_goal").value)
        self._goal_padding = float(self.get_parameter("goal_padding_m").value)
        self._edge_margin = int(self.get_parameter("edge_margin_cells").value)
        self._base_frame = self.get_parameter("base_frame").value or "base_link"
        self._home_frame = self.get_parameter("home_frame").value
        self._w_info = float(self.get_parameter("score_w_info_gain").value)
        self._w_dist = float(self.get_parameter("score_w_distance").value)
        self._w_recency = float(self.get_parameter("score_w_recency").value)
        self._w_blacklist = float(self.get_parameter("score_w_blacklist_prox").value)
        self._w_hint = float(self.get_parameter("score_w_hint_align").value)
        self._w_persist = float(self.get_parameter("score_w_persist").value)
        self._w_clutter = float(self.get_parameter("score_w_clutter").value)
        self._clutter_radius = max(float(self.get_parameter("clutter_radius_m").value), 0.1)
        self._recency_window = max(float(self.get_parameter("recency_window_s").value), 1.0)
        self._info_radius = max(float(self.get_parameter("info_gain_radius_m").value), 0.1)
        self._max_dist_cap = max(float(self.get_parameter("max_distance_cap_m").value), 0.1)
        self._min_score = float(self.get_parameter("min_score_threshold").value)

        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        # Default to cancelled: never explore on boot unless the mission
        # orchestrator explicitly enables us (cancel=False on the latched
        # topic). This makes `Ctrl+C goridge` restartable safely — the
        # robot won't auto-resume exploration the moment nodes come back.
        self._cancelled = True
        self._active = False
        self._current_goal: tuple[float, float] | None = None
        self._goal_handle = None
        self._attempts: dict[tuple[float, float], int] = {}
        self._blacklist: list[tuple[float, float]] = []
        self._visited: list[tuple[float, float, float]] = []  # (x, y, ts)
        self._hint: dict[str, Any] = {}  # {dx, dy, target_room}
        # World-frame goal the explorer last committed to; anchors the
        # directional-persistence term so it keeps pursuing one hallway arm.
        self._committed_goal: tuple[float, float] | None = None
        self._last_error = ""
        self._frontier_count = 0
        self._last_replan = 0.0
        self._last_score = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, _LATCHED_QOS
        )
        # Latched so a late subscriber (explorer started before orchestrator,
        # or restarted independently) always receives the most recent cancel
        # state. Without this, the boot-time cancel(True) from the
        # orchestrator is lost and the explorer auto-explores.
        self.create_subscription(
            Bool, self.get_parameter("cancel_topic").value, self._on_cancel, _LATCHED_QOS
        )
        self.create_subscription(
            String, self.get_parameter("hint_topic").value, self._on_hint, 10
        )
        self.create_subscription(
            String,
            self.get_parameter("blacklist_point_topic").value,
            self._on_blacklist_point,
            10,
        )
        self._pub_status = self.create_publisher(
            String, self.get_parameter("status_topic").value, _LATCHED_QOS
        )
        self._action = ActionClient(self, NavigateToPose, self.get_parameter("nav_action").value)
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"frontier_explorer ready (min_size={self._min_size}, "
            f"blacklist_r={self._blacklist_radius}m, replan={self._replan_period}s, "
            f"score_w[info={self._w_info},dist={self._w_dist},rec={self._w_recency},"
            f"black={self._w_blacklist},clutter={self._w_clutter},hint={self._w_hint}], "
            f"min_score={self._min_score})"
        )

    # --- callbacks ----------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._map = msg

    def _on_cancel(self, msg: Bool) -> None:
        with self._lock:
            self._cancelled = bool(msg.data)
            if self._cancelled:
                # Drop directional commitment so a fresh mission starts unbiased.
                self._committed_goal = None
                if self._goal_handle is not None:
                    self.get_logger().info("frontier exploration cancelled by mission")
                    self._goal_handle.cancel_goal_async()
                    self._goal_handle = None
                    self._current_goal = None

    def _on_hint(self, msg: String) -> None:
        payload = json_loads(msg.data, default={})
        dx = float(payload.get("dx", 0.0) or 0.0)
        dy = float(payload.get("dy", 0.0) or 0.0)
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            dx, dy = dx / norm, dy / norm
        else:
            dx = dy = 0.0
        with self._lock:
            self._hint = {
                "dx": dx,
                "dy": dy,
                "target_room": str(payload.get("target_room") or "").upper(),
                "active": norm > 1e-6,
            }

    def _on_blacklist_point(self, msg: String) -> None:
        """Blacklist an externally-flagged impassable point (e.g. narrow gap).

        Adds the point to the blacklist so future cluster scoring avoids it,
        and cancels the in-flight goal if it sits near the flagged point so
        Nav2 stops trying to thread it and the next tick reroutes.
        """
        payload = json_loads(msg.data, default={})
        try:
            px = float(payload["x"])
            py = float(payload["y"])
        except (KeyError, TypeError, ValueError):
            return
        point = (round(px, 2), round(py, 2))
        with self._lock:
            if point not in self._blacklist:
                self._blacklist.append(point)
            near_current = (
                self._current_goal is not None
                and self._dist(point, self._current_goal) < self._blacklist_radius
            )
            handle = self._goal_handle if near_current else None
            if near_current:
                self._current_goal = None
                self._goal_handle = None
        if handle is not None:
            self.get_logger().info(
                f"blacklisting impassable point {point}; cancelling current goal to reroute"
            )
            handle.cancel_goal_async()

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

        result = self._pick_best_cluster(grid, clusters)
        if result is None:
            self._last_error = "all_blacklisted"
            self._publish_status("blocked")
            return
        goal, score = result
        with self._lock:
            self._last_score = score
        if score < self._min_score:
            # All remaining candidates score below threshold — treat as map
            # complete. The orchestrator picks this up to enter IDLE_MAP_COMPLETE.
            self._last_error = "no_frontiers"
            self._publish_status("no_frontiers")
            return

        if active:
            # Never preempt our own in-flight goal. Preemption races with
            # Nav2's recovery dance (clear_costmap → spin → backup): the
            # planner is still working on the old goal when the explorer
            # yanks the target away, producing the
            # "Planning algorithm failed → goal aborted" cascade. Let the
            # current goal succeed, fail, or get blacklisted naturally —
            # the next tick will pick a fresh winner with up-to-date map.
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

        # Erode 2 cells (~10 cm) near obstacles to kill thin LiDAR-stripe
        # frontiers between clutter (chair/table legs, fences). Those
        # generate scoring honeypots that look like high-info goals but
        # are unreachable. Real openings (doorways) survive easily.
        for _ in range(2):
            obstacle_neighbor = np.zeros_like(obstacle)
            obstacle_neighbor[1:, :] |= obstacle[:-1, :]
            obstacle_neighbor[:-1, :] |= obstacle[1:, :]
            obstacle_neighbor[:, 1:] |= obstacle[:, :-1]
            obstacle_neighbor[:, :-1] |= obstacle[:, 1:]
            frontier &= ~obstacle_neighbor
            # Grow obstacle for the next iteration so erosion compounds.
            obstacle = obstacle | obstacle_neighbor

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
                cx_centroid = sum_x // count
                cy_centroid = sum_y // count
                # Drop centroids near the SLAM map boundary — the global
                # costmap may not have grown to match yet, and sending a
                # Nav2 goal at an off-costmap pose produces worldToMap-fail
                # spam until the BT recovery cascade aborts the goal.
                if (
                    cx_centroid < self._edge_margin
                    or cx_centroid >= w - self._edge_margin
                    or cy_centroid < self._edge_margin
                    or cy_centroid >= h - self._edge_margin
                ):
                    continue
                clusters.append((cx_centroid, cy_centroid, count))
        return clusters

    def _pick_best_cluster(
        self,
        grid: OccupancyGrid,
        clusters: list[tuple[int, int, int]],
    ) -> tuple[tuple[float, float], float] | None:
        """Weighted-score selection of the best frontier cluster.

        Returns (world_xy, score) for the highest-scoring candidate, or None
        if every cluster was blacklisted. Score may be negative; the caller
        decides whether to treat low scores as "map complete".
        """
        info = grid.info
        ox, oy = float(info.origin.position.x), float(info.origin.position.y)
        res = float(info.resolution)
        robot = self._lookup_robot_pose()  # (x, y) or None
        now = time.time()
        with self._lock:
            hint = dict(self._hint) if self._hint.get("active") else {}
            visited = list(self._visited)
            blacklist = list(self._blacklist)
            committed = self._committed_goal

        # Drop visit history older than 2× the recency window — it can never
        # contribute meaningfully and grows unbounded otherwise.
        cutoff = now - 2.0 * self._recency_window
        if visited:
            kept = [v for v in visited if v[2] >= cutoff]
            if len(kept) != len(visited):
                with self._lock:
                    self._visited = kept

        best: tuple[tuple[float, float], float, int] | None = None
        for cx, cy, size in clusters:
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            world = (round(wx, 2), round(wy, 2))
            if any(self._dist(world, b) < self._blacklist_radius for b in blacklist):
                continue
            attempts = self._attempts.get(world, 0)
            if attempts >= self._max_attempts:
                with self._lock:
                    if world not in self._blacklist:
                        self._blacklist.append(world)
                continue
            score = self._score_cluster(
                world, size, grid, robot, visited, blacklist, hint, committed, now,
            )
            if best is None or score > best[1] or (
                math.isclose(score, best[1]) and size > best[2]
            ):
                best = (world, score, size)
        if best is None:
            return None
        return best[0], best[1]

    def _score_cluster(
        self,
        world: tuple[float, float],
        size: int,
        grid: OccupancyGrid,
        robot: tuple[float, float] | None,
        visited: list[tuple[float, float, float]],
        blacklist: list[tuple[float, float]],
        hint: dict[str, Any],
        committed: tuple[float, float] | None,
        now: float,
    ) -> float:
        info_gain = self._info_gain(grid, world)
        # Distance penalty: normalised by cap so the term stays in [0, 1].
        if robot is None:
            distance_term = 0.0
        else:
            d = self._dist(world, robot)
            distance_term = min(d, self._max_dist_cap) / self._max_dist_cap
        # Recency: visits closer in time and space hurt more.
        recency_term = 0.0
        for vx, vy, vts in visited:
            age = max(0.0, now - vts)
            if age > 2.0 * self._recency_window:
                continue
            time_decay = math.exp(-age / self._recency_window)
            spatial = math.exp(-((world[0] - vx) ** 2 + (world[1] - vy) ** 2) / 2.0)
            recency_term += time_decay * spatial
        # Blacklist proximity: smooth penalty in addition to the hard filter
        # above, since a cluster *near* a blacklist point is also suspect.
        blacklist_term = 0.0
        if blacklist:
            sigma2 = max(self._blacklist_radius, 0.1) ** 2
            for bx, by in blacklist:
                blacklist_term += math.exp(-((world[0] - bx) ** 2 + (world[1] - by) ** 2) / (2.0 * sigma2))
        # Hint alignment: only when an orchestrator hint is active *and* we
        # have a robot pose to anchor the direction.
        hint_term = 0.0
        if hint and robot is not None:
            cx, cy = world[0] - robot[0], world[1] - robot[1]
            cnorm = math.hypot(cx, cy)
            if cnorm > 1e-6:
                hint_term = max(0.0, (cx * hint.get("dx", 0.0) + cy * hint.get("dy", 0.0)) / cnorm)
        # Directional persistence: cosine between (robot->candidate) and
        # (robot->last committed goal). Only when there's no explicit operator
        # hint, so an operator direction always wins. This is what breaks the
        # symmetric-junction tie: once committed to one arm, candidates down
        # that arm get a bonus and the robot stops flip-flopping.
        persist_term = 0.0
        if not hint and committed is not None and robot is not None:
            gx, gy = committed[0] - robot[0], committed[1] - robot[1]
            cx2, cy2 = world[0] - robot[0], world[1] - robot[1]
            gnorm = math.hypot(gx, gy)
            cnorm2 = math.hypot(cx2, cy2)
            if gnorm > 1e-6 and cnorm2 > 1e-6:
                persist_term = max(0.0, (cx2 * gx + cy2 * gy) / (cnorm2 * gnorm))
        # Clutter penalty: candidates surrounded by obstacle cells are
        # almost certainly LiDAR-strip honeypots, not real openings.
        clutter_term = self._clutter_fraction(grid, world)
        # Tiny cluster-size boost so genuine ties prefer the bigger frontier.
        size_bonus = 0.01 * min(size, 100) / 100.0
        return (
            self._w_info * info_gain
            - self._w_dist * distance_term
            - self._w_recency * recency_term
            - self._w_blacklist * blacklist_term
            - self._w_clutter * clutter_term
            + self._w_hint * hint_term
            + self._w_persist * persist_term
            + size_bonus
        )

    def _info_gain(self, grid: OccupancyGrid, world: tuple[float, float]) -> float:
        """Fraction of unknown cells inside info_gain_radius around the goal.

        Cheap O(r²/res²) array slice on the existing OccupancyGrid — no extra
        subscriptions or raycasts. Returns a value in [0, 1].
        """
        info = grid.info
        res = float(info.resolution)
        if res <= 0.0:
            return 0.0
        w, h = int(info.width), int(info.height)
        ox, oy = float(info.origin.position.x), float(info.origin.position.y)
        gx = int((world[0] - ox) / res)
        gy = int((world[1] - oy) / res)
        r_cells = max(int(self._info_radius / res), 1)
        x0 = max(0, gx - r_cells)
        x1 = min(w, gx + r_cells + 1)
        y0 = max(0, gy - r_cells)
        y1 = min(h, gy + r_cells + 1)
        if x1 <= x0 or y1 <= y0:
            return 0.0
        data = np.asarray(grid.data, dtype=np.int8).reshape((h, w))
        window = data[y0:y1, x0:x1]
        total = window.size
        if total == 0:
            return 0.0
        unknown = int(np.count_nonzero(window < 0))
        return unknown / float(total)

    def _clutter_fraction(self, grid: OccupancyGrid, world: tuple[float, float]) -> float:
        """Fraction of obstacle cells inside clutter_radius around the goal.

        A real doorway / open-room frontier has very few obstacle cells
        within ~1 m of its centroid. A LiDAR-strip honeypot is dense with
        them. Returns a value in [0, 1].
        """
        info = grid.info
        res = float(info.resolution)
        if res <= 0.0:
            return 0.0
        w, h = int(info.width), int(info.height)
        ox, oy = float(info.origin.position.x), float(info.origin.position.y)
        gx = int((world[0] - ox) / res)
        gy = int((world[1] - oy) / res)
        r_cells = max(int(self._clutter_radius / res), 1)
        x0 = max(0, gx - r_cells)
        x1 = min(w, gx + r_cells + 1)
        y0 = max(0, gy - r_cells)
        y1 = min(h, gy + r_cells + 1)
        if x1 <= x0 or y1 <= y0:
            return 0.0
        data = np.asarray(grid.data, dtype=np.int8).reshape((h, w))
        window = data[y0:y1, x0:x1]
        total = window.size
        if total == 0:
            return 0.0
        obstacle = int(np.count_nonzero(window > 50))
        return obstacle / float(total)

    def _lookup_robot_pose(self) -> tuple[float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._home_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return None
        return (float(tf.transform.translation.x), float(tf.transform.translation.y))

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
        # Face the frontier from the robot's current pose. Sending yaw=0
        # forced RotateToGoal to spin the robot to world-east at every
        # waypoint regardless of travel direction, which thrashed the
        # drives. atan2(dy,dx) lets the goal yaw match the natural arrival
        # heading, so DWB barely needs to rotate at the end.
        robot_xy = self._lookup_robot_pose()
        if robot_xy is not None:
            heading = math.atan2(goal[1] - robot_xy[1], goal[0] - robot_xy[0])
        else:
            heading = 0.0
        ps.pose.orientation = yaw_to_quaternion(heading)

        msg = NavigateToPose.Goal()
        msg.pose = ps

        with self._lock:
            self._current_goal = goal
            self._committed_goal = goal
            self._active = True
            self._attempts[goal] = self._attempts.get(goal, 0) + 1
            self._visited.append((goal[0], goal[1], time.time()))

        self.get_logger().info(
            f"frontier goal -> ({goal[0]:.2f}, {goal[1]:.2f}) score={self._last_score:.3f}"
        )
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
                "score": self._last_score,
                "hint_active": bool(self._hint.get("active", False)),
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
