#!/usr/bin/env python3
"""Frontier exploration node for blank-map Ridgeback missions."""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, json_loads, quaternion_to_yaw_rad, yaw_to_quaternion
    from ridgeback_image_motion.vlm_client import build_vlm_client, rank_frontiers
except ImportError:
    from autonomy_common import json_dumps, json_loads, quaternion_to_yaw_rad, yaw_to_quaternion
    from vlm_client import build_vlm_client, rank_frontiers


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/r100_0140/platform/odom/filtered")
        self.declare_parameter("command_topic", "/ridgeback/exploration/command")
        self.declare_parameter("status_topic", "/ridgeback/exploration/status")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("min_frontier_size_cells", 8)
        self.declare_parameter("occupied_inflation_cells", 8)
        self.declare_parameter("goal_check_period_s", 2.0)
        self.declare_parameter("goal_reached_pause_s", 1.0)
        self.declare_parameter("selection_mode", "distance")
        self.declare_parameter("vlm_top_n", 4)
        self.declare_parameter("vlm_call_timeout_s", 2.0)
        self.declare_parameter("vlm_min_replan_interval_s", 4.0)
        self.declare_parameter("vlm_max_image_age_s", 2.0)
        self.declare_parameter("image_topic", "/r100_0140/image/compressed")
        self.declare_parameter("fallback_image_topic", "/r100_0140/sensors/camera_0/color/compressed")

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self._map_cb, map_qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, sensor_qos)
        self.create_subscription(String, self.get_parameter("command_topic").value, self._command_cb, reliable_qos)
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, reliable_qos)
        self.nav_client = ActionClient(self, NavigateToPose, self.get_parameter("navigate_action").value)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        seen_topics = set()
        for topic in (
            str(self.get_parameter("image_topic").value).strip(),
            str(self.get_parameter("fallback_image_topic").value).strip(),
        ):
            if not topic or topic in seen_topics:
                continue
            seen_topics.add(topic)
            self.create_subscription(CompressedImage, topic, self._image_cb, image_qos)

        self.map_msg: OccupancyGrid | None = None
        self.map_data: np.ndarray | None = None
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.active = False
        self.target_room = ""
        self.state = "IDLE"
        self.goal_handle: Any = None
        self.result_future: Any = None
        self.last_goal_time = 0.0
        self.last_goal: dict[str, float] | None = None
        self.last_error = ""
        self.latest_image_jpeg: bytes | None = None
        self.latest_image_time = 0.0
        self.last_vlm_call_time = 0.0
        self.last_vlm_reasoning = ""
        self.last_selection_method = "distance"

        self._vlm_client: Any = None
        self._vlm_config: Any = None
        if str(self.get_parameter("selection_mode").value).lower() == "vlm":
            try:
                self._vlm_client, self._vlm_config = build_vlm_client()
                self.get_logger().info(
                    f"VLM ranking enabled: {self._vlm_config.base_url} model={self._vlm_config.model_name}"
                )
            except Exception as exc:
                self.get_logger().warn(f"VLM client init failed; falling back to distance: {exc}")
                self._vlm_client = None

        self.create_timer(0.5, self._tick)
        self.get_logger().info("frontier_explorer ready")

    def _map_cb(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            return
        self.map_msg = msg
        self.map_data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))

    def _odom_cb(self, msg: Odometry) -> None:
        self.pose["x"] = float(msg.pose.pose.position.x)
        self.pose["y"] = float(msg.pose.pose.position.y)
        self.pose["yaw"] = float(quaternion_to_yaw_rad(msg.pose.pose.orientation))

    def _image_cb(self, msg: CompressedImage) -> None:
        self.latest_image_jpeg = bytes(msg.data)
        self.latest_image_time = time.time()

    def _refresh_map_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(),
            )
            self.pose["x"] = float(transform.transform.translation.x)
            self.pose["y"] = float(transform.transform.translation.y)
            self.pose["yaw"] = float(quaternion_to_yaw_rad(transform.transform.rotation))
        except Exception:
            pass

    def _command_cb(self, msg: String) -> None:
        payload = json_loads(msg.data)
        action = str(payload.get("action", "")).lower()
        if action == "start":
            self.active = True
            self.target_room = str(payload.get("target_room", ""))
            self.state = "EXPLORING"
            self.last_error = ""
            self.get_logger().info(f"frontier exploration started target={self.target_room}")
        elif action == "stop":
            self.active = False
            self.state = "IDLE"
            self._cancel_goal()
            self.get_logger().info("frontier exploration stopped")

    def _tick(self) -> None:
        self._refresh_map_pose()
        if not self.active:
            self._publish_status()
            return

        if self.result_future is not None and self.result_future.done():
            result = self.result_future.result()
            status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
            self.goal_handle = None
            self.result_future = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.state = "EXPLORING"
                self.last_goal_time = time.time() - float(self.get_parameter("goal_reached_pause_s").value)
            else:
                self.state = "EXPLORING"
                self.last_error = f"nav_status_{status}"

        if self.goal_handle is not None:
            self.state = "NAVIGATING"
            self._publish_status()
            return

        if time.time() - self.last_goal_time < float(self.get_parameter("goal_check_period_s").value):
            self._publish_status()
            return

        goal = self._choose_frontier_goal()
        if goal is None:
            self.state = "FAILED"
            self.last_error = "no_safe_frontier"
            self._publish_status()
            return

        self._send_goal(goal)
        self._publish_status()

    def _choose_frontier_goal(self) -> dict[str, float] | None:
        candidates = self._collect_frontier_candidates()
        if not candidates:
            return None

        mode = str(self.get_parameter("selection_mode").value).lower()
        if mode == "vlm" and self._vlm_client is not None:
            chosen = self._rank_with_vlm(candidates)
            if chosen is not None:
                return chosen

        self.last_selection_method = "distance"
        self.last_vlm_reasoning = ""
        return candidates[0]

    def _collect_frontier_candidates(self) -> list[dict[str, float]]:
        if self.map_msg is None or self.map_data is None:
            self.last_error = "no_map"
            return []

        data = self.map_data
        free = data == 0
        unknown = data < 0
        occupied = data >= 65
        frontier = np.zeros_like(unknown, dtype=bool)
        frontier[1:-1, 1:-1] = free[1:-1, 1:-1] & (
            unknown[:-2, 1:-1] | unknown[2:, 1:-1] | unknown[1:-1, :-2] | unknown[1:-1, 2:]
        )

        inflated_occupied = self._inflate_bool_grid(occupied, int(self.get_parameter("occupied_inflation_cells").value))
        frontier &= ~inflated_occupied

        if self._world_to_cell(self.pose["x"], self.pose["y"]) is None:
            self.last_error = "robot_outside_map"
            return []

        visited = np.zeros_like(frontier, dtype=bool)
        min_size = int(self.get_parameter("min_frontier_size_cells").value)
        candidates: list[dict[str, float]] = []

        for cy, cx in zip(*np.where(frontier)):
            if visited[cy, cx]:
                continue
            cells = self._collect_component(frontier, visited, int(cx), int(cy))
            if len(cells) < min_size:
                continue
            mx = sum(cell[0] for cell in cells) / len(cells)
            my = sum(cell[1] for cell in cells) / len(cells)
            wx, wy = self._cell_to_world(mx, my)
            distance = math.hypot(wx - self.pose["x"], wy - self.pose["y"])
            if distance < 0.75:
                continue
            score = distance - 0.03 * min(len(cells), 100)
            yaw = math.atan2(wy - self.pose["y"], wx - self.pose["x"])
            candidates.append({
                "x": wx,
                "y": wy,
                "yaw": yaw,
                "score": score,
                "size": float(len(cells)),
                "distance": distance,
            })

        candidates.sort(key=lambda c: c["score"])
        top_n = max(1, int(self.get_parameter("vlm_top_n").value))
        return candidates[:top_n]

    def _rank_with_vlm(self, candidates: list[dict[str, float]]) -> dict[str, float] | None:
        now = time.time()
        max_image_age = float(self.get_parameter("vlm_max_image_age_s").value)
        if self.latest_image_jpeg is None or now - self.latest_image_time > max_image_age:
            self.last_error = "vlm_no_image"
            return None

        replan_interval = float(self.get_parameter("vlm_min_replan_interval_s").value)
        if now - self.last_vlm_call_time < replan_interval:
            return None

        candidate_views = []
        for c in candidates:
            bearing_world = math.atan2(c["y"] - self.pose["y"], c["x"] - self.pose["x"])
            bearing_rel = self._wrap_pi(bearing_world - self.pose["yaw"])
            candidate_views.append({
                "bearing_deg": math.degrees(bearing_rel),
                "distance_m": c["distance"],
                "cluster_size": int(c["size"]),
            })

        timeout_s = float(self.get_parameter("vlm_call_timeout_s").value)
        try:
            self.last_vlm_call_time = now
            result = rank_frontiers(
                self._vlm_client,
                self._vlm_config,
                self.latest_image_jpeg,
                candidate_views,
                timeout_s=timeout_s,
            )
        except Exception as exc:
            self.get_logger().warn(f"VLM rank failed; falling back to distance: {exc}")
            self.last_error = f"vlm_error:{str(exc)[:80]}"
            self.last_vlm_reasoning = ""
            self.last_selection_method = "distance_fallback"
            return None

        chosen = candidates[result["chosen_index"]]
        self.last_vlm_reasoning = result["reasoning"]
        self.last_selection_method = "vlm"
        self.get_logger().info(
            f"VLM picked frontier {result['chosen_index']} "
            f"(bearing={math.degrees(self._wrap_pi(math.atan2(chosen['y'] - self.pose['y'], chosen['x'] - self.pose['x']) - self.pose['yaw'])):+.1f}deg, "
            f"dist={chosen['distance']:.2f}m): {result['reasoning']}"
        )
        return chosen

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _inflate_bool_grid(self, grid: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return grid.copy()
        inflated = grid.copy()
        occupied_cells = np.argwhere(grid)
        height, width = grid.shape
        for y, x in occupied_cells:
            y0 = max(0, y - radius)
            y1 = min(height, y + radius + 1)
            x0 = max(0, x - radius)
            x1 = min(width, x + radius + 1)
            inflated[y0:y1, x0:x1] = True
        return inflated

    def _collect_component(self, grid: np.ndarray, visited: np.ndarray, start_x: int, start_y: int) -> list[tuple[int, int]]:
        queue: deque[tuple[int, int]] = deque([(start_x, start_y)])
        visited[start_y, start_x] = True
        cells: list[tuple[int, int]] = []
        height, width = grid.shape
        while queue:
            x, y = queue.popleft()
            cells.append((x, y))
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if 0 <= nx < width and 0 <= ny < height and grid[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True
                    queue.append((nx, ny))
        return cells

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        assert self.map_msg is not None
        res = float(self.map_msg.info.resolution)
        ox = float(self.map_msg.info.origin.position.x)
        oy = float(self.map_msg.info.origin.position.y)
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if 0 <= cx < self.map_msg.info.width and 0 <= cy < self.map_msg.info.height:
            return cx, cy
        return None

    def _cell_to_world(self, cx: float, cy: float) -> tuple[float, float]:
        assert self.map_msg is not None
        res = float(self.map_msg.info.resolution)
        ox = float(self.map_msg.info.origin.position.x)
        oy = float(self.map_msg.info.origin.position.y)
        return ox + (cx + 0.5) * res, oy + (cy + 0.5) * res

    def _send_goal(self, goal: dict[str, float]) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.last_error = "nav2_unavailable"
            return
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal["x"])
        msg.pose.pose.position.y = float(goal["y"])
        msg.pose.pose.orientation = yaw_to_quaternion(float(goal["yaw"]))
        send_future = self.nav_client.send_goal_async(msg)
        send_future.add_done_callback(lambda future: self._goal_response_cb(future, goal))
        self.last_goal_time = time.time()
        self.last_goal = goal
        self.state = "NAVIGATING"

    def _goal_response_cb(self, future: Any, goal: dict[str, float]) -> None:
        handle = future.result()
        if not handle.accepted:
            self.last_error = "frontier_goal_rejected"
            self.state = "EXPLORING"
            return
        self.goal_handle = handle
        self.result_future = handle.get_result_async()
        self.last_goal = goal

    def _cancel_goal(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.result_future = None

    def _publish_status(self) -> None:
        payload = {
            "stamp": time.time(),
            "active": self.active,
            "state": self.state,
            "target_room": self.target_room,
            "last_goal": self.last_goal or {},
            "last_error": self.last_error,
            "selection_mode": str(self.get_parameter("selection_mode").value),
            "last_selection_method": self.last_selection_method,
            "last_vlm_reasoning": self.last_vlm_reasoning,
        }
        self.status_pub.publish(String(data=json_dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
