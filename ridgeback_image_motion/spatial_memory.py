#!/usr/bin/env python3
"""SQLite-backed room+pose store.

Subscribes to /vlm/observation, dedupes nearby observations of the same
room (keeping the highest-confidence one), and persists to a SQLite file
under ~/.ridgeback. Answers room lookups via a String pub/sub RPC:

    orchestrator publishes  /memory/query   {request_id, room}
    spatial_memory replies  /memory/result  {request_id, room, found,
                                              x, y, yaw, confidence}
"""

from __future__ import annotations

import math
import os
import sqlite3
import threading
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from ridgeback_image_motion.autonomy_common import (
        json_dumps,
        json_loads,
        normalize_room_id,
    )
    from ridgeback_image_motion.autonomy_geometry import lidar_agrees
except ImportError:
    from autonomy_common import (  # type: ignore[no-redef]
        json_dumps,
        json_loads,
        normalize_room_id,
    )
    from autonomy_geometry import lidar_agrees  # type: ignore[no-redef]


_DEFAULT_DB = Path.home() / ".ridgeback" / "spatial_memory.sqlite"

# Latched so late subscribers (the dashboard, RViz) always get the current map
# and the current landmark set without waiting for the next change.
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def _grid_agrees(
    grid: OccupancyGrid, x: float, y: float, radius_m: float, min_cells: int
) -> bool:
    """OccupancyGrid wrapper around the pure autonomy_geometry.lidar_agrees."""
    info = grid.info
    return lidar_agrees(
        grid.data,
        int(info.width),
        int(info.height),
        float(info.origin.position.x),
        float(info.origin.position.y),
        float(info.resolution),
        x,
        y,
        radius_m,
        min_cells,
    )


class SpatialMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("spatial_memory")

        self.declare_parameter("db_path", str(_DEFAULT_DB))
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("dedup_radius_m", 1.0)
        self.declare_parameter("observation_topic", "/vlm/observation")
        self.declare_parameter("query_topic", "/memory/query")
        self.declare_parameter("result_topic", "/memory/result")
        self.declare_parameter("forget_topic", "/memory/forget")
        # Landmark promotion: a stored room becomes a *map landmark* only when
        # the VLM read is corroborated (vote_count >= min_votes) AND the LiDAR
        # map shows real structure nearby (a door sits in a wall). This is the
        # "VLM and LiDAR agreement" the operator asked for.
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("landmark_topic", "/room/landmarks")
        self.declare_parameter("landmark_min_votes", 2)
        self.declare_parameter("require_lidar_agreement", True)
        self.declare_parameter("landmark_agree_radius_m", 0.8)
        self.declare_parameter("landmark_min_occupied_cells", 3)

        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._dedup_radius = float(self.get_parameter("dedup_radius_m").value)
        self._landmark_min_votes = int(self.get_parameter("landmark_min_votes").value)
        self._require_agreement = bool(self.get_parameter("require_lidar_agreement").value)
        self._agree_radius = float(self.get_parameter("landmark_agree_radius_m").value)
        self._agree_min_cells = int(self.get_parameter("landmark_min_occupied_cells").value)

        db_path = Path(self.get_parameter("db_path").value).expanduser()
        db_path.parent.mkdir(parents=True, exist_ok=True)
        self._db_lock = threading.Lock()
        self._db = sqlite3.connect(str(db_path), check_same_thread=False)
        self._db.execute(
            """CREATE TABLE IF NOT EXISTS rooms (
                room TEXT PRIMARY KEY,
                x REAL, y REAL, yaw REAL,
                confidence REAL, ts REAL,
                landmark INTEGER DEFAULT 0
            )"""
        )
        # Migrate older DBs that predate the landmark column.
        cols = [r[1] for r in self._db.execute("PRAGMA table_info(rooms)").fetchall()]
        if "landmark" not in cols:
            self._db.execute("ALTER TABLE rooms ADD COLUMN landmark INTEGER DEFAULT 0")
        self._db.commit()

        self._map: OccupancyGrid | None = None

        self.create_subscription(
            String, self.get_parameter("observation_topic").value, self._on_observation, 10
        )
        self.create_subscription(
            String, self.get_parameter("query_topic").value, self._on_query, 10
        )
        self.create_subscription(
            String, self.get_parameter("forget_topic").value, self._on_forget, 10
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, _LATCHED_QOS
        )
        self._pub_result = self.create_publisher(
            String, self.get_parameter("result_topic").value, 10
        )
        self._pub_landmarks = self.create_publisher(
            String, self.get_parameter("landmark_topic").value, _LATCHED_QOS
        )
        # Publish the (possibly already-populated) landmark set once so late
        # subscribers have an initial latched message.
        self._publish_landmarks()

        self.get_logger().info(
            f"spatial_memory ready (db={db_path}, min_conf={self._min_confidence}, "
            f"dedup_r={self._dedup_radius}m, landmark_votes>={self._landmark_min_votes}, "
            f"lidar_agreement={self._require_agreement})"
        )

    # --- callbacks ----------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_observation(self, msg: String) -> None:
        evt = json_loads(msg.data)
        room = normalize_room_id(str(evt.get("room") or ""))
        confidence = float(evt.get("confidence") or 0.0)
        if not room or confidence < self._min_confidence:
            return
        pose = evt.get("pose") or {}
        try:
            x = float(pose.get("x"))
            y = float(pose.get("y"))
            yaw = float(pose.get("yaw") or 0.0)
        except (TypeError, ValueError):
            return
        ts = float(evt.get("ts") or time.time())
        votes = int(evt.get("vote_count") or 0)

        # Landmark gate: corroborated VLM read AND LiDAR map structure nearby.
        is_landmark = votes >= self._landmark_min_votes
        if is_landmark and self._require_agreement:
            grid = self._map
            is_landmark = grid is not None and _grid_agrees(
                grid, x, y, self._agree_radius, self._agree_min_cells
            )

        # Decide the DB write under the lock; do all logging/publishing AFTER
        # releasing it — _publish_landmarks re-acquires _db_lock, which is a
        # plain (non-reentrant) Lock, so calling it while held would deadlock.
        stored = False
        final_landmark = False
        publish_needed = False
        with self._db_lock:
            row = self._db.execute(
                "SELECT x, y, confidence, landmark FROM rooms WHERE room=?", (room,)
            ).fetchone()
            if row is not None:
                ex, ey, ec, elm = row
                elm = int(elm or 0)
                if math.hypot(x - ex, y - ey) < self._dedup_radius and ec >= confidence:
                    # Keep the better-located/-confident pose, but a corroborated
                    # +agreeing sighting can still PROMOTE the room to a landmark.
                    if is_landmark and not elm:
                        self._db.execute(
                            "UPDATE rooms SET landmark=1 WHERE room=?", (room,)
                        )
                        self._db.commit()
                        final_landmark = True
                        publish_needed = True
                else:
                    # Better pose: rewrite. Never demote an existing landmark.
                    landmark_flag = 1 if (is_landmark or elm) else 0
                    self._db.execute(
                        "INSERT OR REPLACE INTO rooms VALUES (?, ?, ?, ?, ?, ?, ?)",
                        (room, x, y, yaw, confidence, ts, landmark_flag),
                    )
                    self._db.commit()
                    stored = True
                    final_landmark = bool(landmark_flag)
                    # Republish if it is (still) a landmark — its pose moved.
                    publish_needed = final_landmark
            else:
                landmark_flag = 1 if is_landmark else 0
                self._db.execute(
                    "INSERT OR REPLACE INTO rooms VALUES (?, ?, ?, ?, ?, ?, ?)",
                    (room, x, y, yaw, confidence, ts, landmark_flag),
                )
                self._db.commit()
                stored = True
                final_landmark = bool(landmark_flag)
                publish_needed = final_landmark

        if stored:
            self.get_logger().info(
                f"memory: stored {room} @ ({x:.2f},{y:.2f}) conf={confidence:.2f} "
                f"votes={votes} landmark={'yes' if final_landmark else 'no'}"
            )
        elif publish_needed:
            self.get_logger().info(f"memory: promoted {room} to landmark")
        if publish_needed:
            self._publish_landmarks()

    def _on_query(self, msg: String) -> None:
        req = json_loads(msg.data)
        room = normalize_room_id(str(req.get("room") or ""))
        request_id = req.get("request_id", "")
        if not room:
            self._publish_result(request_id, room, False)
            return
        with self._db_lock:
            row = self._db.execute(
                "SELECT x, y, yaw, confidence FROM rooms WHERE room=?", (room,)
            ).fetchone()
        if row is None:
            self._publish_result(request_id, room, False)
            return
        x, y, yaw, conf = row
        self._publish_result(request_id, room, True, x=x, y=y, yaw=yaw, confidence=conf)

    def _on_forget(self, msg: String) -> None:
        """Invalidate memory entries. Payloads:
          {"clear_all": true}    -> wipe every row (used at SLAM session start)
          {"room": "203"}        -> drop one room (used when an approach to a
                                    remembered pose fails: the saved coordinate
                                    is no longer valid).
        """
        req = json_loads(msg.data, default={})
        # Publish AFTER releasing _db_lock — _publish_landmarks re-acquires it
        # (non-reentrant Lock), so publishing while held would deadlock.
        publish_needed = False
        with self._db_lock:
            if bool(req.get("clear_all")):
                self._db.execute("DELETE FROM rooms")
                self._db.commit()
                self.get_logger().info("memory: cleared all rooms (session reset)")
                publish_needed = True
            else:
                room = normalize_room_id(str(req.get("room") or ""))
                if room:
                    self._db.execute("DELETE FROM rooms WHERE room=?", (room,))
                    self._db.commit()
                    self.get_logger().info(f"memory: forgot {room}")
                    # A forgotten room may have been a landmark — refresh.
                    publish_needed = True
        if publish_needed:
            self._publish_landmarks()

    def _publish_landmarks(self) -> None:
        """Publish the current landmark set (latched) for the dashboard/RViz."""
        with self._db_lock:
            rows = self._db.execute(
                "SELECT room, x, y, confidence FROM rooms WHERE landmark=1 ORDER BY room"
            ).fetchall()
        landmarks = [
            {"room": r, "x": float(x), "y": float(y), "confidence": float(c)}
            for (r, x, y, c) in rows
        ]
        msg = String()
        msg.data = json_dumps({"landmarks": landmarks, "ts": time.time()})
        self._pub_landmarks.publish(msg)

    def _publish_result(
        self,
        request_id: str,
        room: str,
        found: bool,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        confidence: float = 0.0,
    ) -> None:
        payload = {
            "request_id": request_id,
            "room": room,
            "found": bool(found),
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "confidence": float(confidence),
            "ts": time.time(),
        }
        msg = String()
        msg.data = json_dumps(payload)
        self._pub_result.publish(msg)


def main() -> None:
    rclpy.init()
    node = SpatialMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
