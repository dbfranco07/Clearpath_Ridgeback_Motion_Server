#!/usr/bin/env python3
"""Mission state machine for natural-language room finding and return."""

from __future__ import annotations

import math
import time
from typing import Any

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from ridgeback_image_motion.autonomy_common import (
        json_dumps,
        json_loads,
        parse_intent_and_room,
        quaternion_to_yaw_rad,
        yaw_to_quaternion,
    )
    from ridgeback_image_motion.spatial_memory import SpatialMemory
except ImportError:
    from autonomy_common import json_dumps, json_loads, parse_intent_and_room, quaternion_to_yaw_rad, yaw_to_quaternion
    from spatial_memory import SpatialMemory


class MissionOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("mission_orchestrator")

        self.declare_parameter("command_topic", "/ridgeback/mission/command")
        self.declare_parameter("status_topic", "/ridgeback/mission/status")
        self.declare_parameter("detections_topic", "/ridgeback/semantic/room_detections")
        self.declare_parameter("exploration_command_topic", "/ridgeback/exploration/command")
        self.declare_parameter("safety_status_topic", "/ridgeback/safety/status")
        self.declare_parameter("odom_topic", "/r100_0140/platform/odom/filtered")
        self.declare_parameter("stop_topic", "/cmd_vel_nav")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("memory_db_path", "")
        self.declare_parameter("room_min_confidence", 0.55)
        self.declare_parameter("start_tolerance_m", 0.45)

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(String, self.get_parameter("command_topic").value, self._command_cb, reliable_qos)
        self.create_subscription(String, self.get_parameter("detections_topic").value, self._detection_cb, reliable_qos)
        self.create_subscription(String, self.get_parameter("safety_status_topic").value, self._safety_status_cb, reliable_qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, sensor_qos)
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, reliable_qos)
        self.explore_pub = self.create_publisher(String, self.get_parameter("exploration_command_topic").value, reliable_qos)
        self.stop_pub = self.create_publisher(Twist, self.get_parameter("stop_topic").value, sensor_qos)
        self.nav_client = ActionClient(self, NavigateToPose, self.get_parameter("navigate_action").value)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        db_path = str(self.get_parameter("memory_db_path").value).strip() or None
        self.memory = SpatialMemory(db_path)
        self.session_id = self.memory.create_session({"source": "mission_orchestrator", "map": "blank_slam"})

        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.have_pose = False
        self.start_saved = False
        self.state = "STARTING"
        self.phase = "waiting_for_pose"
        self.command = ""
        self.intent = ""
        self.target_room = ""
        self.last_detection: dict[str, Any] = {}
        self.last_error = ""
        self.safety_safe = True
        self.safety_reasons: list[str] = []
        self.goal_handle: Any = None
        self.result_future: Any = None
        self.pending_goal_kind = ""
        self.mission_started_at = 0.0

        self.create_timer(0.2, self._tick)
        self.get_logger().info(f"mission_orchestrator ready session={self.session_id}")

    def _odom_cb(self, msg: Odometry) -> None:
        self.pose["x"] = float(msg.pose.pose.position.x)
        self.pose["y"] = float(msg.pose.pose.position.y)
        self.pose["yaw"] = float(quaternion_to_yaw_rad(msg.pose.pose.orientation))
        self.have_pose = True

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
            self.have_pose = True
        except Exception:
            pass

    def _safety_status_cb(self, msg: String) -> None:
        payload = json_loads(msg.data)
        self.safety_safe = bool(payload.get("is_safe", True))
        reasons = payload.get("reasons", [])
        self.safety_reasons = reasons if isinstance(reasons, list) else []

    def _command_cb(self, msg: String) -> None:
        payload = json_loads(msg.data, {"command": msg.data})
        command = str(payload.get("command", "")).strip()
        parsed = parse_intent_and_room(command)
        intent = parsed["intent"]
        room = parsed["room"]

        if intent == "STOP":
            self._fail_or_stop("STOPPED", "operator_stop")
            return
        if intent == "RETURN_TO_START":
            self._start_return(command)
            return
        if intent != "GO_TO_ROOM" or not room:
            self.last_error = "unsupported_command"
            self.command = command
            self.intent = intent
            self.target_room = room
            self.state = "FAILED"
            self.phase = "unsupported_command"
            self.memory.record_mission(command, "failed", {"reason": self.last_error, **parsed}, self.session_id)
            return

        self.command = command
        self.intent = intent
        self.target_room = room
        self.last_error = ""
        self.last_detection = {}
        self.mission_started_at = time.time()
        self.memory.record_mission(command, "started", parsed, self.session_id)

        if not self.start_saved:
            self.state = "STARTING"
            self.phase = "waiting_for_start_position"
            return

        known = self.memory.find_room(self.session_id, room, float(self.get_parameter("room_min_confidence").value))
        if known:
            self._stop_exploration()
            self._navigate_to_location(known, "room")
            self.state = "NAVIGATING_TO_ROOM"
            self.phase = "using_memory"
        else:
            self._start_exploration()
            self.state = "EXPLORING"
            self.phase = "frontier_search"

    def _detection_cb(self, msg: String) -> None:
        payload = json_loads(msg.data)
        detections = payload.get("detections", [])
        pose = payload.get("pose", {})
        if not isinstance(detections, list) or not isinstance(pose, dict):
            return

        for detection in detections:
            room = str(detection.get("room_number", "")).strip().upper()
            confidence = float(detection.get("confidence", 0.0) or 0.0)
            if not room:
                continue
            self.memory.store_room_detection(
                self.session_id,
                room,
                float(pose.get("x", self.pose["x"])),
                float(pose.get("y", self.pose["y"])),
                float(pose.get("yaw", self.pose["yaw"])),
                confidence,
                {"source": "vlm", "raw": detection, "frame_stamp": payload.get("frame_stamp", "")},
            )
            self.last_detection = {
                "room_number": room,
                "confidence": confidence,
                "pose": {"x": float(pose.get("x", self.pose["x"])), "y": float(pose.get("y", self.pose["y"])), "yaw": float(pose.get("yaw", self.pose["yaw"]))},
            }

        if self.state == "EXPLORING" and self.target_room:
            found = self.memory.find_room(self.session_id, self.target_room, float(self.get_parameter("room_min_confidence").value))
            if found:
                self._stop_exploration()
                self._navigate_to_location(found, "room")
                self.state = "NAVIGATING_TO_ROOM"
                self.phase = "target_detected"

    def _tick(self) -> None:
        self._refresh_map_pose()
        if self.have_pose and not self.start_saved:
            self.memory.store_start_position(self.session_id, self.pose["x"], self.pose["y"], self.pose["yaw"])
            self.start_saved = True
            if self.state == "STARTING":
                if self.target_room:
                    known = self.memory.find_room(
                        self.session_id,
                        self.target_room,
                        float(self.get_parameter("room_min_confidence").value),
                    )
                    if known:
                        self._navigate_to_location(known, "room")
                        self.state = "NAVIGATING_TO_ROOM"
                        self.phase = "using_memory"
                    else:
                        self._start_exploration()
                        self.state = "EXPLORING"
                        self.phase = "frontier_search"
                else:
                    self.state = "IDLE"
                    self.phase = "ready"
            self.get_logger().info("saved start_position for current SLAM session")

        if self.state in {"EXPLORING", "NAVIGATING_TO_ROOM", "RETURNING_TO_START"} and not self.safety_safe:
            self._fail_or_stop("FAILED", "safety_stop:" + ",".join(str(item) for item in self.safety_reasons))

        if self.result_future is not None and self.result_future.done():
            result = self.result_future.result()
            status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
            goal_kind = self.pending_goal_kind
            self.goal_handle = None
            self.result_future = None
            self.pending_goal_kind = ""
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._fail_or_stop("FAILED", f"nav2_status_{status}")
            elif goal_kind == "room":
                self._start_return(self.command)
            elif goal_kind == "start":
                if self._distance_to_start() <= float(self.get_parameter("start_tolerance_m").value):
                    self.state = "DONE"
                    self.phase = "returned_to_start"
                    self._publish_stop()
                    self.memory.record_mission(self.command or "return_to_start", "done", {"target_room": self.target_room}, self.session_id)
                else:
                    self._fail_or_stop("FAILED", "return_tolerance_not_met")

        self._publish_status()

    def _start_exploration(self) -> None:
        self._cancel_nav_goal()
        self.explore_pub.publish(String(data=json_dumps({"action": "start", "target_room": self.target_room})))

    def _stop_exploration(self) -> None:
        self.explore_pub.publish(String(data=json_dumps({"action": "stop"})))

    def _start_return(self, command: str) -> None:
        self.command = command or self.command
        if not self.start_saved:
            self.state = "FAILED"
            self.phase = "no_start_position"
            self.last_error = "start_position_not_saved"
            return
        start = self.memory.get_start_position(self.session_id)
        if not start:
            self.state = "FAILED"
            self.phase = "no_start_position"
            self.last_error = "start_position_missing"
            return
        self._stop_exploration()
        self._navigate_to_location(start, "start")
        self.state = "RETURNING_TO_START"
        self.phase = "returning"

    def _navigate_to_location(self, location: dict[str, Any], kind: str) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.25):
            self.state = "FAILED"
            self.phase = "nav2_unavailable"
            self.last_error = "nav2_unavailable"
            return
        self._cancel_nav_goal()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(location["x"])
        goal.pose.pose.position.y = float(location["y"])
        goal.pose.pose.orientation = yaw_to_quaternion(float(location.get("yaw", 0.0)))
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._goal_response_cb(done, kind))
        self.pending_goal_kind = kind

    def _goal_response_cb(self, future: Any, kind: str) -> None:
        handle = future.result()
        if not handle.accepted:
            self._fail_or_stop("FAILED", f"{kind}_goal_rejected")
            return
        self.goal_handle = handle
        self.pending_goal_kind = kind
        self.result_future = handle.get_result_async()

    def _distance_to_start(self) -> float:
        start = self.memory.get_start_position(self.session_id)
        if not start:
            return float("inf")
        return math.hypot(self.pose["x"] - float(start["x"]), self.pose["y"] - float(start["y"]))

    def _cancel_nav_goal(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.result_future = None
        self.pending_goal_kind = ""

    def _fail_or_stop(self, state: str, reason: str) -> None:
        self._cancel_nav_goal()
        self._stop_exploration()
        self._publish_stop()
        self.state = state
        self.phase = reason
        self.last_error = reason
        self.memory.record_mission(self.command or "mission", state.lower(), {"reason": reason, "target_room": self.target_room}, self.session_id)

    def _publish_stop(self) -> None:
        for _ in range(3):
            self.stop_pub.publish(Twist())

    def _publish_status(self) -> None:
        payload = {
            "stamp": time.time(),
            "session_id": self.session_id,
            "state": self.state,
            "phase": self.phase,
            "command": self.command,
            "intent": self.intent,
            "target_room": self.target_room,
            "start_saved": self.start_saved,
            "pose": dict(self.pose),
            "last_detection": self.last_detection,
            "last_error": self.last_error,
            "safety_safe": self.safety_safe,
            "safety_reasons": self.safety_reasons,
            "elapsed_s": time.time() - self.mission_started_at if self.mission_started_at else 0.0,
        }
        self.status_pub.publish(String(data=json_dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._fail_or_stop("STOPPED", "shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
