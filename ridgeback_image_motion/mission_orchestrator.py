#!/usr/bin/env python3
"""Top-level mission FSM for find-room / explore / return-home.

States:
    IDLE -> CAPTURE_HOME -> EXPLORING -> APPROACHING_ROOM
        -> CONFIRMING -> RETURNING -> DONE
    -> ABORTED (from any state on safety latch / error)

Inputs:
    /mission/goal         String JSON {intent, room, command}
    /vlm/observation      String JSON {room, confidence, pose}
    /frontier/status      String JSON {state, current_goal, ...}
    /safety/latched       Bool (latched)
    TF map -> base_link

Outputs:
    /mission/state        String JSON (latched)
    /mission/home_pose    PoseStamped (latched)
    /frontier/cancel      Bool
    /vlm/trigger          String (room hint)
    Action client /navigate_to_pose
    Pub /memory/query / sub /memory/result
"""

from __future__ import annotations

import threading
import time
import uuid
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ridgeback_image_motion.autonomy_common import (
        ACTIVE_MISSION_STATES,
        json_dumps,
        json_loads,
        yaw_to_quaternion,
    )
except ImportError:
    from autonomy_common import (  # type: ignore[no-redef]
        ACTIVE_MISSION_STATES,
        json_dumps,
        json_loads,
        yaw_to_quaternion,
    )


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class MissionOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("mission_orchestrator")

        # Topics & frames
        self.declare_parameter("namespace", "r100_0140")
        self.declare_parameter("home_frame", "map")
        self.declare_parameter("base_frame", "")
        self.declare_parameter("goal_topic", "/mission/goal")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("home_topic", "/mission/home_pose")
        self.declare_parameter("frontier_cancel_topic", "/frontier/cancel")
        self.declare_parameter("frontier_status_topic", "/frontier/status")
        self.declare_parameter("vlm_trigger_topic", "/vlm/trigger")
        self.declare_parameter("vlm_observation_topic", "/vlm/observation")
        self.declare_parameter("memory_query_topic", "/memory/query")
        self.declare_parameter("memory_result_topic", "/memory/result")
        self.declare_parameter("safety_latched_topic", "/safety/latched")
        self.declare_parameter("nav_action", "navigate_to_pose")

        # Behaviour params
        self.declare_parameter("periodic_vlm_period_s", 3.0)
        self.declare_parameter("target_match_confidence", 0.7)
        self.declare_parameter("max_explore_time_s", 600.0)
        self.declare_parameter("approach_offset_m", 0.8)
        self.declare_parameter("memory_lookup_timeout_s", 1.0)
        self.declare_parameter("tick_period_s", 1.0)

        ns = self.get_parameter("namespace").value
        self._home_frame = self.get_parameter("home_frame").value
        # Platform's robot_state_publisher emits URDF frame names verbatim
        # (no ROS namespace prefix), so default to bare "base_link".
        self._base_frame = self.get_parameter("base_frame").value or "base_link"
        self._periodic_vlm_period = float(self.get_parameter("periodic_vlm_period_s").value)
        self._match_conf = float(self.get_parameter("target_match_confidence").value)
        self._max_explore_time = float(self.get_parameter("max_explore_time_s").value)
        self._approach_offset = float(self.get_parameter("approach_offset_m").value)
        self._memory_timeout = float(self.get_parameter("memory_lookup_timeout_s").value)

        # State
        self._lock = threading.Lock()
        self._state = "IDLE"
        self._intent = ""
        self._target_room = ""
        self._command = ""
        self._home_pose: PoseStamped | None = None
        self._target_pose: tuple[float, float, float] | None = None
        self._mission_start_ts = 0.0
        self._last_periodic_vlm = 0.0
        self._safety_latched = True
        self._goal_handle = None
        self._memory_pending: dict[str, dict[str, Any]] = {}

        # ROS plumbing
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            String, self.get_parameter("goal_topic").value, self._on_goal, 10
        )
        self.create_subscription(
            String, self.get_parameter("vlm_observation_topic").value, self._on_observation, 10
        )
        self.create_subscription(
            String, self.get_parameter("frontier_status_topic").value, self._on_frontier, 10
        )
        self.create_subscription(
            String, self.get_parameter("memory_result_topic").value, self._on_memory_result, 10
        )
        self.create_subscription(
            Bool, self.get_parameter("safety_latched_topic").value, self._on_safety, _LATCHED_QOS
        )

        self._pub_state = self.create_publisher(
            String, self.get_parameter("state_topic").value, _LATCHED_QOS
        )
        self._pub_home = self.create_publisher(
            PoseStamped, self.get_parameter("home_topic").value, _LATCHED_QOS
        )
        self._pub_cancel = self.create_publisher(
            Bool, self.get_parameter("frontier_cancel_topic").value, 10
        )
        self._pub_vlm_trigger = self.create_publisher(
            String, self.get_parameter("vlm_trigger_topic").value, 10
        )
        self._pub_memory_query = self.create_publisher(
            String, self.get_parameter("memory_query_topic").value, 10
        )

        self._action = ActionClient(self, NavigateToPose, self.get_parameter("nav_action").value)

        self._publish_state("ready")
        self._cancel_frontier(True)
        self._timer = self.create_timer(
            float(self.get_parameter("tick_period_s").value), self._tick
        )

        self.get_logger().info(
            f"mission_orchestrator ready (home={self._home_frame}, base={self._base_frame}, "
            f"match_conf>={self._match_conf})"
        )

    # --- callbacks ----------------------------------------------------------
    def _on_goal(self, msg: String) -> None:
        payload = json_loads(msg.data)
        intent = (payload.get("intent") or "").upper()
        room = (payload.get("room") or "").strip().upper()
        command = payload.get("command") or ""
        self.get_logger().info(f"mission goal received: intent={intent} room={room}")
        with self._lock:
            self._intent = intent
            self._target_room = room
            self._command = command

        if intent in ("STOP",):
            self._abort("operator_stop")
            return
        if intent == "RETURN_TO_START":
            self._begin_return_home()
            return
        if intent == "EXPLORE":
            self._begin_explore()
            return
        if intent == "GO_TO_ROOM":
            if not room:
                self._publish_state("error", note="no_room")
                return
            self._begin_find_room(room)
            return
        if intent == "QUERY":
            # Free-form "what do you see"-style question. Forward the operator's
            # text to vlm_client as a one-shot Q&A trigger; the answer comes back
            # on /vlm/observation with kind="answer" and surfaces in the chat.
            trigger = String()
            trigger.data = json_dumps({"prompt": command, "intent": "QUERY"})
            self._pub_vlm_trigger.publish(trigger)
            self._publish_state("idle", note="query_to_vlm")
            return
        # Unknown / ROOM_QUERY — no motion.
        self._publish_state("idle", note=f"unhandled_intent:{intent}")

    def _on_observation(self, msg: String) -> None:
        evt = json_loads(msg.data)
        with self._lock:
            target = self._target_room
            state = self._state
        if not target or state not in ("EXPLORING", "CONFIRMING", "APPROACHING_ROOM"):
            return
        room = (evt.get("room") or "").strip().upper()
        if room != target:
            return
        confidence = float(evt.get("confidence") or 0.0)
        if confidence < self._match_conf:
            return
        pose = evt.get("pose") or {}
        try:
            x = float(pose["x"])
            y = float(pose["y"])
            yaw = float(pose.get("yaw") or 0.0)
        except (KeyError, TypeError, ValueError):
            return
        with self._lock:
            self._target_pose = (x, y, yaw)
        if state == "EXPLORING":
            self.get_logger().info(
                f"observed target room {target} (conf={confidence:.2f}) — approaching"
            )
            self._begin_approach(x, y, yaw)
        elif state == "CONFIRMING":
            self.get_logger().info(f"confirmed target room {target}")
            self._begin_return_home()

    def _on_frontier(self, msg: String) -> None:
        info = json_loads(msg.data)
        if info.get("state") == "no_frontiers":
            with self._lock:
                state = self._state
            if state in ("EXPLORING",):
                self._abort("no_frontiers")

    def _on_memory_result(self, msg: String) -> None:
        result = json_loads(msg.data)
        request_id = result.get("request_id", "")
        if request_id not in self._memory_pending:
            return
        ctx = self._memory_pending.pop(request_id)
        if not result.get("found"):
            ctx["on_miss"]()
            return
        x, y = float(result.get("x", 0.0)), float(result.get("y", 0.0))
        yaw = float(result.get("yaw", 0.0))
        ctx["on_hit"](x, y, yaw)

    def _on_safety(self, msg: Bool) -> None:
        with self._lock:
            self._safety_latched = bool(msg.data)
            state = self._state
        if msg.data and state in ACTIVE_MISSION_STATES:
            self._abort("safety_latched")

    # --- mission flow -------------------------------------------------------
    def _begin_explore(self) -> None:
        if not self._capture_home():
            return
        with self._lock:
            self._state = "EXPLORING"
            self._mission_start_ts = time.time()
        self._cancel_frontier(False)
        self._publish_state("exploring")

    def _begin_find_room(self, room: str) -> None:
        if not self._capture_home():
            return
        # Try memory first.
        request_id = uuid.uuid4().hex
        self._memory_pending[request_id] = {
            "on_hit": lambda x, y, yaw: self._begin_approach(x, y, yaw, source="memory"),
            "on_miss": self._begin_explore_for_target,
        }
        msg = String()
        msg.data = json_dumps({"request_id": request_id, "room": room})
        self._pub_memory_query.publish(msg)
        # If memory doesn't reply within timeout, treat as miss.
        threading.Timer(self._memory_timeout, self._memory_timeout_check, args=[request_id]).start()
        self._publish_state("checking_memory", note=f"room={room}")

    def _memory_timeout_check(self, request_id: str) -> None:
        if request_id in self._memory_pending:
            ctx = self._memory_pending.pop(request_id)
            ctx["on_miss"]()

    def _begin_explore_for_target(self) -> None:
        with self._lock:
            self._state = "EXPLORING"
            self._mission_start_ts = time.time()
        self._cancel_frontier(False)
        self._publish_state("exploring")

    def _begin_approach(self, x: float, y: float, yaw: float, source: str = "vlm") -> None:
        # Apply small offset *toward* the robot so we stop in front of the marker.
        with self._lock:
            self._state = "APPROACHING_ROOM"
            self._target_pose = (x, y, yaw)
        self._cancel_frontier(True)
        ps = self._make_pose(x, y, yaw)
        self._send_nav_goal(ps, on_done=self._on_approach_done)
        self._publish_state("approaching", note=f"source={source}")

    def _on_approach_done(self, status: int) -> None:
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._abort(f"approach_failed_status_{status}")
            return
        with self._lock:
            self._state = "CONFIRMING"
            target = self._target_room
        # Trigger one focused VLM read.
        msg = String()
        msg.data = target
        self._pub_vlm_trigger.publish(msg)
        self._publish_state("confirming", note=f"room={target}")
        # Give the VLM ~6 s to confirm; if no observation, return-home anyway.
        threading.Timer(6.0, self._confirm_timeout).start()

    def _confirm_timeout(self) -> None:
        with self._lock:
            if self._state == "CONFIRMING":
                self.get_logger().info("confirm timeout — proceeding to return home")
                self._begin_return_home()

    def _begin_return_home(self) -> None:
        with self._lock:
            home = self._home_pose
        if home is None:
            self._publish_state("error", note="no_home_pose")
            return
        with self._lock:
            self._state = "RETURNING_TO_START"
        self._cancel_frontier(True)
        self._send_nav_goal(home, on_done=self._on_return_done)
        self._publish_state("returning")

    def _on_return_done(self, status: int) -> None:
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._abort(f"return_failed_status_{status}")
            return
        with self._lock:
            self._state = "DONE"
        self._publish_state("done")

    # --- helpers ------------------------------------------------------------
    def _capture_home(self) -> bool:
        with self._lock:
            if self._home_pose is not None:
                return True
        try:
            tf = self._tf_buffer.lookup_transform(
                self._home_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.5),
            )
        except TransformException as exc:
            self.get_logger().warn(f"could not capture home pose: {exc}")
            self._publish_state("error", note="tf_unavailable")
            return False
        ps = PoseStamped()
        ps.header = tf.header
        ps.header.frame_id = self._home_frame
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        with self._lock:
            self._home_pose = ps
        self._pub_home.publish(ps)
        self.get_logger().info(
            f"home pose captured @ ({ps.pose.position.x:.2f}, {ps.pose.position.y:.2f})"
        )
        return True

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self._home_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = yaw_to_quaternion(yaw)
        return ps

    def _send_nav_goal(self, pose: PoseStamped, on_done) -> None:
        if not self._action.wait_for_server(timeout_sec=10.0):
            self._abort("nav2_unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self._action.send_goal_async(goal)

        def _on_response(f):
            try:
                handle = f.result()
            except Exception as exc:
                self._abort(f"send_goal_failed:{exc}")
                return
            if not handle.accepted:
                self._abort("goal_rejected")
                return
            self._goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda r: on_done(r.result().status if r.result() else GoalStatus.STATUS_ABORTED)
            )

        future.add_done_callback(_on_response)

    def _cancel_frontier(self, cancel: bool) -> None:
        msg = Bool()
        msg.data = bool(cancel)
        self._pub_cancel.publish(msg)

    def _abort(self, reason: str) -> None:
        self.get_logger().warn(f"mission aborted: {reason}")
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self._cancel_frontier(True)
        with self._lock:
            self._state = "ABORTED"
        self._publish_state("aborted", note=reason)

    def _publish_state(self, state: str, note: str = "") -> None:
        with self._lock:
            payload: dict[str, Any] = {
                "state": state,
                "intent": self._intent,
                "room": self._target_room,
                "command": self._command,
                "note": note,
                "fsm": self._state,
                "ts": time.time(),
            }
            if self._home_pose is not None:
                payload["home"] = {
                    "x": self._home_pose.pose.position.x,
                    "y": self._home_pose.pose.position.y,
                }
            if self._target_pose is not None:
                payload["target"] = {
                    "x": self._target_pose[0],
                    "y": self._target_pose[1],
                    "yaw": self._target_pose[2],
                }
        msg = String()
        msg.data = json_dumps(payload)
        self._pub_state.publish(msg)

    def _tick(self) -> None:
        with self._lock:
            state = self._state
            elapsed = time.time() - self._mission_start_ts if self._mission_start_ts else 0.0
            target = self._target_room
        if state == "EXPLORING":
            now = time.time()
            if (now - self._last_periodic_vlm) >= self._periodic_vlm_period and target:
                self._last_periodic_vlm = now
                msg = String()
                msg.data = target
                self._pub_vlm_trigger.publish(msg)
            if elapsed > self._max_explore_time:
                self._abort("max_explore_time")


def main() -> None:
    rclpy.init()
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
