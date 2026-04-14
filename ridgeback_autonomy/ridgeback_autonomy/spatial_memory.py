#!/usr/bin/env python3
"""
Spatial Memory Node — Runs on Jetson Orin
==========================================
Persistent SQLite database that maps room numbers to map-frame coordinates.
The memory is built incrementally as the robot explores — never pre-loaded.

Services provided:
  - memory/store_location   — save a room → (x,y) association
  - memory/query_location   — look up a room's coordinates
  - memory/get_all_locations — list all known rooms
  - memory/reset            — clear all entries (std_srvs/Trigger)

On node startup:
  - Opens (or creates) the SQLite DB at ~/ridgeback_memory.db
  - Records the robot's current pose as 'start_position'
    (or uses the most recent start_position if the robot hasn't moved)

Memory is intentionally session-aware:
  - Room locations persist across restarts (good for multi-session use)
  - Start position is updated each launch
  - Map coordinates are in SLAM map frame (will drift across sessions, but
    Nav2 + VLM confirmation makes this acceptable)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

import sqlite3
import math
import os
import time
from datetime import datetime

from ridgeback_autonomy.msg import Perception
from ridgeback_autonomy.srv import StoreLocation, QueryLocation, GetAllLocations


DB_PATH = os.path.expanduser('~/ridgeback_memory.db')

CREATE_TABLE_SQL = """
CREATE TABLE IF NOT EXISTS room_locations (
    room_id         TEXT NOT NULL,
    map_x           REAL NOT NULL,
    map_y           REAL NOT NULL,
    map_yaw         REAL NOT NULL DEFAULT 0.0,
    confidence      REAL NOT NULL DEFAULT 0.5,
    detection_count INTEGER NOT NULL DEFAULT 1,
    first_seen      TEXT NOT NULL,
    last_seen       TEXT NOT NULL,
    session_id      TEXT,
    PRIMARY KEY (room_id)
);

CREATE TABLE IF NOT EXISTS memory_log (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp       TEXT NOT NULL,
    event_type      TEXT NOT NULL,
    room_id         TEXT,
    map_x           REAL,
    map_y           REAL,
    details         TEXT
);
"""


class SpatialMemoryNode(Node):
    def __init__(self):
        super().__init__('spatial_memory')

        # Parameters
        self.declare_parameter('db_path', DB_PATH)
        self.declare_parameter('odom_topic', '/r100_0140/platform/odom/filtered')
        self.declare_parameter('perception_topic', '/vlm/perception')
        self.declare_parameter('auto_store_detections', True)  # Auto-store VLM detections
        self.declare_parameter('min_confidence_to_store', 0.6)

        db_path = self.get_parameter('db_path').value
        odom_topic = self.get_parameter('odom_topic').value
        perception_topic = self.get_parameter('perception_topic').value
        self.auto_store = self.get_parameter('auto_store_detections').value
        self.min_confidence = self.get_parameter('min_confidence_to_store').value

        # QoS
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, sensor_qos
        )
        self.perception_sub = self.create_subscription(
            Perception, perception_topic, self._perception_cb, reliable_qos
        )

        # Services
        self.store_srv = self.create_service(
            StoreLocation, 'memory/store_location', self._store_cb
        )
        self.query_srv = self.create_service(
            QueryLocation, 'memory/query_location', self._query_cb
        )
        self.get_all_srv = self.create_service(
            GetAllLocations, 'memory/get_all_locations', self._get_all_cb
        )
        self.reset_srv = self.create_service(
            Trigger, 'memory/reset', self._reset_cb
        )

        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_initialized = False

        # Session ID
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Initialize database
        self._init_db(db_path)

        # Wait for first odom message, then record start position
        self.start_recorded = False
        self.create_timer(2.0, self._record_start_position_once)

        self.get_logger().info('Spatial Memory Node started')
        self.get_logger().info(f'  Database: {db_path}')
        self.get_logger().info(f'  Session ID: {self.session_id}')

    def _init_db(self, db_path: str):
        """Initialize SQLite database."""
        self.db_path = db_path
        conn = sqlite3.connect(db_path)
        conn.executescript(CREATE_TABLE_SQL)
        conn.commit()
        conn.close()
        self.get_logger().info(f'Database initialized at {db_path}')

    def _get_conn(self) -> sqlite3.Connection:
        return sqlite3.connect(self.db_path)

    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose_initialized = True

    def _record_start_position_once(self):
        """Record start_position in DB on first run after odom is available."""
        if self.start_recorded:
            return
        if not self.pose_initialized:
            self.get_logger().info('Waiting for odometry before recording start position...')
            return

        self._store_location(
            room_id='start_position',
            map_x=self.current_x,
            map_y=self.current_y,
            map_yaw=self.current_yaw,
            confidence=1.0,
            session_id=self.session_id
        )
        self.start_recorded = True
        self.get_logger().info(
            f'Start position recorded: ({self.current_x:.2f}, {self.current_y:.2f})'
        )
        # Cancel timer — only run once per startup
        self.create_timer(0.0, lambda: None)  # Workaround: can't cancel a timer in ROS2 easily

    def _perception_cb(self, msg: Perception):
        """Auto-store rooms detected by VLM if auto_store is enabled."""
        if not self.auto_store:
            return
        if not self.pose_initialized:
            return

        for i, room_id in enumerate(msg.detected_rooms):
            confidence = msg.room_confidence[i] if i < len(msg.room_confidence) else 0.5
            if confidence >= self.min_confidence:
                stored = self._store_location(
                    room_id=room_id,
                    map_x=self.current_x,
                    map_y=self.current_y,
                    map_yaw=self.current_yaw,
                    confidence=confidence,
                    session_id=self.session_id
                )
                if stored:
                    self.get_logger().info(
                        f'Memory: stored room {room_id} at '
                        f'({self.current_x:.2f}, {self.current_y:.2f}) '
                        f'confidence={confidence:.2f}'
                    )

    def _store_location(
        self,
        room_id: str,
        map_x: float,
        map_y: float,
        map_yaw: float,
        confidence: float,
        session_id: str = ''
    ) -> bool:
        """Store or update a room location. Returns True if stored/updated."""
        now = datetime.now().isoformat()
        try:
            conn = self._get_conn()
            cur = conn.cursor()

            # Check if room already exists
            cur.execute('SELECT confidence, detection_count, map_x, map_y FROM room_locations WHERE room_id=?', (room_id,))
            row = cur.fetchone()

            if row is None:
                # New entry
                cur.execute("""
                    INSERT INTO room_locations
                    (room_id, map_x, map_y, map_yaw, confidence, detection_count, first_seen, last_seen, session_id)
                    VALUES (?, ?, ?, ?, ?, 1, ?, ?, ?)
                """, (room_id, map_x, map_y, map_yaw, confidence, now, now, session_id))
            else:
                existing_conf, count, existing_x, existing_y = row
                # Update: keep best confidence position, increment count
                if confidence > existing_conf:
                    # New detection is better — update position
                    cur.execute("""
                        UPDATE room_locations
                        SET map_x=?, map_y=?, map_yaw=?, confidence=?, detection_count=?, last_seen=?
                        WHERE room_id=?
                    """, (map_x, map_y, map_yaw, confidence, count + 1, now, room_id))
                else:
                    # Just increment count + update last_seen
                    cur.execute("""
                        UPDATE room_locations
                        SET detection_count=?, last_seen=?
                        WHERE room_id=?
                    """, (count + 1, now, room_id))

            # Log the event
            cur.execute("""
                INSERT INTO memory_log (timestamp, event_type, room_id, map_x, map_y, details)
                VALUES (?, 'STORE', ?, ?, ?, ?)
            """, (now, room_id, map_x, map_y, f'confidence={confidence:.2f}'))

            conn.commit()
            conn.close()
            return True

        except Exception as e:
            self.get_logger().error(f'Memory store failed for {room_id}: {e}')
            return False

    # ── ROS2 Service Handlers ──────────────────────────────────────────────

    def _store_cb(self, request: StoreLocation.Request, response: StoreLocation.Response):
        success = self._store_location(
            room_id=request.room_id,
            map_x=request.map_x,
            map_y=request.map_y,
            map_yaw=request.map_yaw,
            confidence=request.confidence,
            session_id=request.session_id or self.session_id
        )
        response.success = success
        response.message = 'OK' if success else 'Store failed — check logs'
        return response

    def _query_cb(self, request: QueryLocation.Request, response: QueryLocation.Response):
        try:
            conn = self._get_conn()
            cur = conn.cursor()
            cur.execute("""
                SELECT map_x, map_y, map_yaw, confidence, last_seen, detection_count
                FROM room_locations WHERE room_id=?
            """, (request.room_id,))
            row = cur.fetchone()
            conn.close()

            if row:
                response.found = True
                response.map_x = row[0]
                response.map_y = row[1]
                response.map_yaw = row[2]
                response.confidence = row[3]
                response.last_seen = row[4]
                response.detection_count = row[5]
            else:
                response.found = False

        except Exception as e:
            self.get_logger().error(f'Memory query failed for {request.room_id}: {e}')
            response.found = False

        return response

    def _get_all_cb(self, request: GetAllLocations.Request, response: GetAllLocations.Response):
        try:
            conn = self._get_conn()
            cur = conn.cursor()
            if request.include_start_position:
                cur.execute(
                    'SELECT room_id, map_x, map_y, confidence, last_seen, detection_count FROM room_locations'
                )
            else:
                cur.execute(
                    "SELECT room_id, map_x, map_y, confidence, last_seen, detection_count FROM room_locations WHERE room_id != 'start_position'"
                )
            rows = cur.fetchall()
            conn.close()

            response.room_ids = [r[0] for r in rows]
            response.map_xs = [r[1] for r in rows]
            response.map_ys = [r[2] for r in rows]
            response.confidences = [r[3] for r in rows]
            response.last_seen_times = [r[4] for r in rows]
            response.detection_counts = [r[5] for r in rows]

        except Exception as e:
            self.get_logger().error(f'Memory get_all failed: {e}')

        return response

    def _reset_cb(self, request, response: Trigger.Response):
        try:
            conn = self._get_conn()
            conn.execute("DELETE FROM room_locations WHERE room_id != 'start_position'")
            conn.execute(
                "INSERT INTO memory_log (timestamp, event_type, details) VALUES (?, 'RESET', 'User reset all room memories')",
                (datetime.now().isoformat(),)
            )
            conn.commit()
            conn.close()
            response.success = True
            response.message = 'All room memories cleared (start_position preserved)'
            self.get_logger().info('Memory reset: all room locations cleared')
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def get_all_rooms_dict(self) -> dict:
        """Utility method for internal use — returns {room_id: {x, y, confidence}} dict."""
        try:
            conn = self._get_conn()
            cur = conn.cursor()
            cur.execute(
                "SELECT room_id, map_x, map_y, confidence, last_seen FROM room_locations WHERE room_id != 'start_position'"
            )
            rows = cur.fetchall()
            conn.close()
            return {r[0]: {'x': r[1], 'y': r[2], 'confidence': r[3], 'last_seen': r[4]} for r in rows}
        except Exception:
            return {}


def main(args=None):
    print('=' * 50)
    print('Ridgeback Autonomy — Spatial Memory Node')
    print('=' * 50)
    rclpy.init(args=args)
    node = SpatialMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
