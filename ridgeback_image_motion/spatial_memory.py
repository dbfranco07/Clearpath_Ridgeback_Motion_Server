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
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, json_loads
except ImportError:
    from autonomy_common import json_dumps, json_loads  # type: ignore[no-redef]


_DEFAULT_DB = Path.home() / ".ridgeback" / "spatial_memory.sqlite"


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

        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._dedup_radius = float(self.get_parameter("dedup_radius_m").value)

        db_path = Path(self.get_parameter("db_path").value).expanduser()
        db_path.parent.mkdir(parents=True, exist_ok=True)
        self._db_lock = threading.Lock()
        self._db = sqlite3.connect(str(db_path), check_same_thread=False)
        self._db.execute(
            """CREATE TABLE IF NOT EXISTS rooms (
                room TEXT PRIMARY KEY,
                x REAL, y REAL, yaw REAL,
                confidence REAL, ts REAL
            )"""
        )
        self._db.commit()

        self.create_subscription(
            String, self.get_parameter("observation_topic").value, self._on_observation, 10
        )
        self.create_subscription(
            String, self.get_parameter("query_topic").value, self._on_query, 10
        )
        self.create_subscription(
            String, self.get_parameter("forget_topic").value, self._on_forget, 10
        )
        self._pub_result = self.create_publisher(
            String, self.get_parameter("result_topic").value, 10
        )

        self.get_logger().info(
            f"spatial_memory ready (db={db_path}, min_conf={self._min_confidence}, dedup_r={self._dedup_radius}m)"
        )

    # --- callbacks ----------------------------------------------------------
    def _on_observation(self, msg: String) -> None:
        evt = json_loads(msg.data)
        room = str(evt.get("room") or "").strip().upper()
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

        with self._db_lock:
            row = self._db.execute(
                "SELECT x, y, confidence FROM rooms WHERE room=?", (room,)
            ).fetchone()
            if row is not None:
                ex, ey, ec = row
                if math.hypot(x - ex, y - ey) < self._dedup_radius and ec >= confidence:
                    return
            self._db.execute(
                "INSERT OR REPLACE INTO rooms VALUES (?, ?, ?, ?, ?, ?)",
                (room, x, y, yaw, confidence, ts),
            )
            self._db.commit()
        self.get_logger().info(
            f"memory: stored {room} @ ({x:.2f},{y:.2f}) conf={confidence:.2f}"
        )

    def _on_query(self, msg: String) -> None:
        req = json_loads(msg.data)
        room = str(req.get("room") or "").strip().upper()
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
        with self._db_lock:
            if bool(req.get("clear_all")):
                self._db.execute("DELETE FROM rooms")
                self._db.commit()
                self.get_logger().info("memory: cleared all rooms (session reset)")
                return
            room = str(req.get("room") or "").strip().upper()
            if not room:
                return
            self._db.execute("DELETE FROM rooms WHERE room=?", (room,))
            self._db.commit()
        self.get_logger().info(f"memory: forgot {room}")

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
