#!/usr/bin/env python3
"""
Mission Orchestrator — Runs on Jetson Orin
==========================================
High-level state machine that accepts natural language commands from
the web dashboard and coordinates Nav2, VLM perception, and spatial memory
to complete autonomous room-finding missions.

State machine:
  IDLE → CHECKING_MEMORY → (NAVIGATING | EXPLORING) → ARRIVED → RETURNING → RETURNED → IDLE
                                         ↑ VLM finds target while exploring

Natural language parsing (v1 — keyword based):
  "go to room 305"       → GO_TO_ROOM, room=305
  "find room 305"        → GO_TO_ROOM, room=305
  "navigate to 305"      → GO_TO_ROOM, room=305
  "come back"            → RETURN_TO_START
  "return to start"      → RETURN_TO_START
  "go home"              → RETURN_TO_START
  "stop" / "cancel"      → STOP
  "where am i"           → QUERY_STATUS
  "what do you see"      → QUERY_PERCEPTION
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

import re
import math
import time
import threading

from ridgeback_autonomy.msg import Perception, SafetyStatus, MissionStatus
from ridgeback_autonomy.srv import QueryLocation, GetAllLocations, MissionCommand
from ridgeback_autonomy.exploration import find_frontiers, is_goal_reached


# Mission states
class State:
    IDLE = 'IDLE'
    CHECKING_MEMORY = 'CHECKING_MEMORY'
    NAVIGATING = 'NAVIGATING'
    EXPLORING = 'EXPLORING'
    ARRIVED = 'ARRIVED'
    RETURNING = 'RETURNING'
    RETURNED = 'RETURNED'
    FAILED = 'FAILED'
    CANCELLED = 'CANCELLED'


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('odom_topic', '/r100_0140/platform/odom/filtered')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('explore_goal_tolerance_m', 0.8)
        self.declare_parameter('arrive_tolerance_m', 1.0)
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('max_exploration_time_s', 600.0)  # 10 min timeout

        odom_topic = self.get_parameter('odom_topic').value
        map_topic = self.get_parameter('map_topic').value
        self.explore_tol = self.get_parameter('explore_goal_tolerance_m').value
        self.arrive_tol = self.get_parameter('arrive_tolerance_m').value
        self.max_explore_time = self.get_parameter('max_exploration_time_s').value

        # QoS
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, sensor_qos
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_cb, latch_qos
        )
        self.perception_sub = self.create_subscription(
            Perception, '/vlm/perception', self._perception_cb, reliable_qos
        )
        self.safety_sub = self.create_subscription(
            SafetyStatus, '/safety/status', self._safety_cb, reliable_qos
        )

        # Publishers
        self.status_pub = self.create_publisher(MissionStatus, '/mission/status', reliable_qos)

        # Nav2 action client
        self.nav2_client = ActionClient(
            self, NavigateToPose, self.get_parameter('nav2_action').value,
            callback_group=self.cb_group
        )

        # Memory service clients
        self.query_loc_client = self.create_client(
            QueryLocation, 'memory/query_location', callback_group=self.cb_group
        )
        self.get_all_client = self.create_client(
            GetAllLocations, 'memory/get_all_locations', callback_group=self.cb_group
        )

        # Mission command service (called by web dashboard)
        self.command_srv = self.create_service(
            MissionCommand, 'mission/command', self._command_cb,
            callback_group=self.cb_group
        )

        # State
        self.state = State.IDLE
        self.target_room = ''
        self.status_text = 'Ready'
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_recorded = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_initialized = False

        self.current_map = None
        self.map_lock = threading.Lock()

        self.frontiers_explored = 0
        self.rooms_discovered = []
        self.safety_ok = True

        self.mission_thread = None
        self.cancel_flag = threading.Event()
        self.exploration_start_time = None

        self.nav2_goal_handle = None

        # Status timer
        self.create_timer(
            1.0 / self.get_parameter('status_rate_hz').value,
            self._status_timer_cb
        )

        self.get_logger().info('Mission Orchestrator started')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose_initialized = True

        if not self.start_recorded and self.pose_initialized:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.start_recorded = True

    def _map_cb(self, msg: OccupancyGrid):
        with self.map_lock:
            self.current_map = msg

    def _perception_cb(self, msg: Perception):
        for room in msg.detected_rooms:
            if room not in self.rooms_discovered:
                self.rooms_discovered.append(room)

        # If we're exploring and the target room was just detected → transition
        if self.state == State.EXPLORING and self.target_room:
            if self.target_room in msg.detected_rooms:
                self.get_logger().info(
                    f'Mission: VLM detected target room {self.target_room}! '
                    f'Transitioning EXPLORING → NAVIGATING'
                )
                # Cancel current nav2 goal and start navigating to the room
                self._cancel_current_nav2_goal()

    def _safety_cb(self, msg: SafetyStatus):
        self.safety_ok = msg.is_safe

    def _status_timer_cb(self):
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.target_room = self.target_room
        msg.status_text = self.status_text
        msg.progress = self._estimate_progress()
        msg.goal_x = self.goal_x
        msg.goal_y = self.goal_y
        msg.start_x = self.start_x
        msg.start_y = self.start_y
        msg.frontiers_explored = self.frontiers_explored
        msg.rooms_discovered = len(self.rooms_discovered)
        msg.known_rooms = list(self.rooms_discovered)
        self.status_pub.publish(msg)

    # ── Command Handler ────────────────────────────────────────────────────

    def _command_cb(
        self, request: MissionCommand.Request, response: MissionCommand.Response
    ):
        intent, room = self._parse_command(request.command)
        response.parsed_intent = intent
        response.parsed_room = room or ''

        self.get_logger().info(
            f'Command received: "{request.command}" → {intent} room={room}'
        )

        if intent == 'GO_TO_ROOM':
            if not room:
                response.accepted = False
                response.message = 'Could not extract a room number from your command'
                return response
            if self.state not in (State.IDLE, State.ARRIVED, State.RETURNED, State.FAILED, State.CANCELLED):
                response.accepted = False
                response.message = f'Robot is busy ({self.state}). Say "stop" first.'
                return response
            response.accepted = True
            response.message = f'Understood. Going to room {room}.'
            self._start_mission(room)

        elif intent == 'RETURN_TO_START':
            if self.state not in (State.ARRIVED, State.IDLE, State.FAILED):
                response.accepted = False
                response.message = f'Cannot return from state {self.state}'
                return response
            response.accepted = True
            response.message = 'Returning to start position.'
            self._start_return()

        elif intent == 'STOP':
            response.accepted = True
            response.message = 'Mission cancelled. Robot stopping.'
            self._cancel_mission()

        elif intent == 'QUERY_STATUS':
            response.accepted = True
            response.message = (
                f'State: {self.state}. '
                f'Position: ({self.robot_x:.1f}, {self.robot_y:.1f}). '
                f'Rooms found: {self.rooms_discovered}.'
            )

        else:
            response.accepted = False
            response.message = (
                'I didn\'t understand that. Try: "go to room 305", '
                '"come back", or "stop".'
            )

        return response

    # ── Natural Language Parser ────────────────────────────────────────────

    def _parse_command(self, text: str) -> tuple[str, str | None]:
        """Extract intent and optional room number from natural language."""
        t = text.lower().strip()

        # Stop/cancel
        if any(w in t for w in ['stop', 'cancel', 'abort', 'halt']):
            return 'STOP', None

        # Return to start
        if any(p in t for p in ['come back', 'go back', 'return', 'go home', 'home']):
            return 'RETURN_TO_START', None

        # Status query
        if any(p in t for p in ['where am i', 'what do you see', 'status', 'where are you']):
            return 'QUERY_STATUS', None

        # Go to room — extract room number
        if any(w in t for w in ['go to', 'find', 'navigate to', 'go', 'take me to', 'look for']):
            rooms = re.findall(r'\b\d{2,4}\b', t)
            if rooms:
                return 'GO_TO_ROOM', rooms[0]

        # Fallback: just a number?
        rooms = re.findall(r'\b\d{2,4}\b', t)
        if rooms:
            return 'GO_TO_ROOM', rooms[0]

        return 'UNKNOWN', None

    # ── Mission Logic ──────────────────────────────────────────────────────

    def _start_mission(self, room: str):
        self.target_room = room
        self.cancel_flag.clear()
        self.state = State.CHECKING_MEMORY
        self.status_text = f'Checking memory for room {room}...'

        self.mission_thread = threading.Thread(
            target=self._mission_runner, daemon=True
        )
        self.mission_thread.start()

    def _mission_runner(self):
        """Main mission execution loop. Runs in background thread."""
        try:
            # Step 1: Check memory
            self.state = State.CHECKING_MEMORY
            self.status_text = f'Checking memory for room {self.target_room}...'

            known_x, known_y = self._query_room_from_memory(self.target_room)

            if known_x is not None and not self.cancel_flag.is_set():
                # Room is known — navigate directly
                self.get_logger().info(
                    f'Mission: room {self.target_room} in memory at '
                    f'({known_x:.2f}, {known_y:.2f}). Navigating directly.'
                )
                self.status_text = f'Room {self.target_room} found in memory. Navigating...'
                success = self._nav2_goto(known_x, known_y)
                if success and not self.cancel_flag.is_set():
                    self.state = State.ARRIVED
                    self.status_text = f'Arrived at room {self.target_room}!'
                    self.get_logger().info(f'Mission: ARRIVED at room {self.target_room}')
                else:
                    self.state = State.FAILED
                    self.status_text = f'Navigation to room {self.target_room} failed'
            else:
                # Room unknown — explore
                self.get_logger().info(
                    f'Mission: room {self.target_room} not in memory. Starting exploration.'
                )
                self._run_exploration()

        except Exception as e:
            self.get_logger().error(f'Mission runner error: {e}')
            self.state = State.FAILED
            self.status_text = f'Mission error: {e}'

    def _run_exploration(self):
        """Frontier-based exploration until target room is found or all frontiers exhausted."""
        self.state = State.EXPLORING
        self.exploration_start_time = time.time()
        self.frontiers_explored = 0

        while not self.cancel_flag.is_set():
            # Timeout check
            elapsed = time.time() - self.exploration_start_time
            if elapsed > self.max_explore_time:
                self.state = State.FAILED
                self.status_text = f'Exploration timeout after {elapsed:.0f}s. Room {self.target_room} not found.'
                self.get_logger().warn(self.status_text)
                return

            # Check if target room was found (by VLM perception callback)
            known_x, known_y = self._query_room_from_memory(self.target_room)
            if known_x is not None:
                self.get_logger().info(
                    f'Mission: room {self.target_room} found at ({known_x:.2f}, {known_y:.2f})!'
                )
                self.state = State.NAVIGATING
                self.status_text = f'Room {self.target_room} found! Navigating...'
                success = self._nav2_goto(known_x, known_y)
                if success:
                    self.state = State.ARRIVED
                    self.status_text = f'Arrived at room {self.target_room}!'
                else:
                    self.state = State.FAILED
                    self.status_text = 'Navigation to found room failed'
                return

            # Get next frontier
            with self.map_lock:
                current_map = self.current_map

            if current_map is None:
                self.status_text = 'Waiting for map...'
                time.sleep(1.0)
                continue

            frontiers = find_frontiers(
                grid_data=list(current_map.data),
                width=current_map.info.width,
                height=current_map.info.height,
                resolution=current_map.info.resolution,
                origin_x=current_map.info.origin.position.x,
                origin_y=current_map.info.origin.position.y,
                robot_x=self.robot_x,
                robot_y=self.robot_y
            )

            if not frontiers:
                self.state = State.FAILED
                self.status_text = f'All frontiers exhausted. Room {self.target_room} not found on this floor.'
                self.get_logger().warn(self.status_text)
                return

            # Navigate to the nearest frontier
            fx, fy = frontiers[0]
            self.goal_x = fx
            self.goal_y = fy
            self.status_text = (
                f'Exploring... frontier {self.frontiers_explored + 1} '
                f'at ({fx:.1f}, {fy:.1f}). '
                f'Looking for room {self.target_room}.'
            )
            self.get_logger().info(
                f'Exploration: heading to frontier ({fx:.2f}, {fy:.2f})'
            )

            success = self._nav2_goto(fx, fy)
            self.frontiers_explored += 1

            if not success and not self.cancel_flag.is_set():
                # Nav2 failed (obstacle, timeout) — try next frontier
                self.get_logger().debug(
                    f'Frontier {self.frontiers_explored} unreachable — trying next'
                )
                continue

            # Small pause to let VLM scan the new area
            time.sleep(0.5)

    def _start_return(self):
        """Return to the start position."""
        self.cancel_flag.clear()
        self.state = State.RETURNING
        self.status_text = 'Returning to start position...'

        def _return_runner():
            # Query memory for start position
            x, y = self._query_room_from_memory('start_position')
            if x is None:
                x, y = self.start_x, self.start_y

            self.goal_x = x
            self.goal_y = y
            self.get_logger().info(f'Mission: returning to start ({x:.2f}, {y:.2f})')
            success = self._nav2_goto(x, y)
            if success:
                self.state = State.RETURNED
                self.status_text = 'Returned to start position.'
            else:
                self.state = State.FAILED
                self.status_text = 'Failed to return to start position.'

        self.mission_thread = threading.Thread(target=_return_runner, daemon=True)
        self.mission_thread.start()

    def _cancel_mission(self):
        self.cancel_flag.set()
        self._cancel_current_nav2_goal()
        self.state = State.CANCELLED
        self.status_text = 'Mission cancelled.'
        self.target_room = ''

    def _cancel_current_nav2_goal(self):
        if self.nav2_goal_handle is not None:
            try:
                self.nav2_goal_handle.cancel_goal()
            except Exception:
                pass

    # ── Nav2 Interface ─────────────────────────────────────────────────────

    def _nav2_goto(self, x: float, y: float, timeout_s: float = 120.0) -> bool:
        """
        Send a NavigateToPose goal to Nav2 and wait for completion.
        Returns True if goal succeeded, False if failed/cancelled.
        Blocking — call from a non-spin thread.
        """
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available')
            return False

        if self.cancel_flag.is_set():
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0  # No specific heading required

        self.goal_x = x
        self.goal_y = y

        future = self.nav2_client.send_goal_async(goal)

        # Wait for goal acceptance (up to 10s)
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            if self.cancel_flag.is_set():
                return False
            time.sleep(0.05)

        if not future.done():
            self.get_logger().warn('Nav2 goal send timed out')
            return False

        self.nav2_goal_handle = future.result()
        if not self.nav2_goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            return False

        # Wait for result
        result_future = self.nav2_goal_handle.get_result_async()
        deadline = time.time() + timeout_s

        while not result_future.done() and time.time() < deadline:
            if self.cancel_flag.is_set():
                self._cancel_current_nav2_goal()
                return False
            time.sleep(0.1)

        if not result_future.done():
            self.get_logger().warn(f'Nav2 goal timed out after {timeout_s}s')
            self._cancel_current_nav2_goal()
            return False

        result = result_future.result()
        status = result.status
        return status == GoalStatus.STATUS_SUCCEEDED

    # ── Memory Interface ───────────────────────────────────────────────────

    def _query_room_from_memory(self, room_id: str) -> tuple[float | None, float | None]:
        """Query spatial memory for a room. Returns (x, y) or (None, None)."""
        if not self.query_loc_client.wait_for_service(timeout_sec=2.0):
            return None, None
        req = QueryLocation.Request()
        req.room_id = room_id
        future = self.query_loc_client.call_async(req)

        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if future.done():
            result = future.result()
            if result.found:
                return result.map_x, result.map_y
        return None, None

    def _estimate_progress(self) -> float:
        if self.state == State.IDLE:
            return 0.0
        if self.state in (State.ARRIVED, State.RETURNED):
            return 1.0
        if self.state == State.NAVIGATING:
            dist = math.hypot(self.robot_x - self.goal_x, self.robot_y - self.goal_y)
            start_dist = math.hypot(self.start_x - self.goal_x, self.start_y - self.goal_y)
            if start_dist > 0:
                return min(1.0, max(0.0, 1.0 - dist / start_dist))
        return 0.3  # In-progress


def main(args=None):
    print('=' * 50)
    print('Ridgeback Autonomy — Mission Orchestrator')
    print('=' * 50)
    rclpy.init(args=args)
    node = MissionOrchestrator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
