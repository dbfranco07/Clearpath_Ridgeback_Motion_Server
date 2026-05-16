#!/usr/bin/env python3
"""SQLite-backed spatial and mission memory for Ridgeback autonomy."""

from __future__ import annotations

import json
import sqlite3
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SpatialLocation:
    label: str
    x: float
    y: float
    yaw: float
    room_number: str = ""
    confidence: float = 0.0
    metadata: dict[str, Any] | None = None


class SpatialMemory:
    def __init__(self, db_path: str | None = None):
        resolved = Path(db_path or (Path.home() / "ridgeback_memory.db")).expanduser()
        resolved.parent.mkdir(parents=True, exist_ok=True)
        self.db_path = resolved
        self._lock = threading.Lock()
        self._ensure_schema()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.db_path)
        connection.row_factory = sqlite3.Row
        return connection

    def _ensure_schema(self) -> None:
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS sessions (
                    id TEXT PRIMARY KEY,
                    active INTEGER NOT NULL DEFAULT 0,
                    metadata TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL
                )
                """
            )
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS locations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT NOT NULL DEFAULT '',
                    label TEXT NOT NULL,
                    room_number TEXT NOT NULL DEFAULT '',
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    yaw REAL NOT NULL,
                    confidence REAL NOT NULL DEFAULT 0,
                    metadata TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL
                )
                """
            )
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS missions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT NOT NULL DEFAULT '',
                    command TEXT NOT NULL,
                    status TEXT NOT NULL,
                    metadata TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL
                )
                """
            )
            self._ensure_column(connection, "locations", "session_id", "TEXT NOT NULL DEFAULT ''")
            self._ensure_column(connection, "missions", "session_id", "TEXT NOT NULL DEFAULT ''")
            connection.commit()

    def _ensure_column(self, connection: sqlite3.Connection, table: str, column: str, spec: str) -> None:
        columns = {row["name"] for row in connection.execute(f"PRAGMA table_info({table})").fetchall()}
        if column not in columns:
            connection.execute(f"ALTER TABLE {table} ADD COLUMN {column} {spec}")

    def create_session(self, metadata: dict[str, Any] | None = None) -> str:
        session_id = f"session-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
        payload = json.dumps(metadata or {}, separators=(",", ":"))
        with self._lock, self._connect() as connection:
            connection.execute("UPDATE sessions SET active = 0")
            connection.execute(
                """
                INSERT INTO sessions (id, active, metadata, created_at)
                VALUES (?, 1, ?, ?)
                """,
                (session_id, payload, time.strftime("%Y-%m-%dT%H:%M:%S")),
            )
            connection.commit()
        return session_id

    def get_active_session(self) -> str:
        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT id FROM sessions WHERE active = 1 ORDER BY created_at DESC LIMIT 1"
            ).fetchone()
        return str(row["id"]) if row else ""

    def store_location(self, location: SpatialLocation, session_id: str = "") -> None:
        metadata = json.dumps(location.metadata or {}, separators=(",", ":"))
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO locations (session_id, label, room_number, x, y, yaw, confidence, metadata, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    session_id,
                    location.label,
                    location.room_number,
                    float(location.x),
                    float(location.y),
                    float(location.yaw),
                    float(location.confidence),
                    metadata,
                    time.strftime("%Y-%m-%dT%H:%M:%S"),
                ),
            )
            connection.commit()

    def store_start_position(self, session_id: str, x: float, y: float, yaw: float) -> None:
        self.store_location(
            SpatialLocation(
                label="start_position",
                room_number="",
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                confidence=1.0,
                metadata={"type": "start_position"},
            ),
            session_id=session_id,
        )

    def get_start_position(self, session_id: str) -> dict[str, Any] | None:
        return self._get_location_by_label(session_id, "start_position")

    def store_room_detection(
        self,
        session_id: str,
        room_number: str,
        x: float,
        y: float,
        yaw: float,
        confidence: float,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        room = str(room_number).strip().upper()
        self.store_location(
            SpatialLocation(
                label=f"room_{room}",
                room_number=room,
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                confidence=float(confidence),
                metadata={"type": "room_detection", **(metadata or {})},
            ),
            session_id=session_id,
        )

    def find_room(self, session_id: str, room_number: str, min_confidence: float = 0.0) -> dict[str, Any] | None:
        room = str(room_number).strip().upper()
        with self._lock, self._connect() as connection:
            row = connection.execute(
                """
                SELECT * FROM locations
                WHERE session_id = ?
                  AND room_number = ?
                  AND confidence >= ?
                ORDER BY confidence DESC, id DESC
                LIMIT 1
                """,
                (session_id, room, float(min_confidence)),
            ).fetchone()
        return self._row_to_location(row) if row else None

    def _get_location_by_label(self, session_id: str, label: str) -> dict[str, Any] | None:
        with self._lock, self._connect() as connection:
            row = connection.execute(
                """
                SELECT * FROM locations
                WHERE session_id = ? AND label = ?
                ORDER BY id DESC
                LIMIT 1
                """,
                (session_id, label),
            ).fetchone()
        return self._row_to_location(row) if row else None

    def record_mission(
        self,
        command: str,
        status: str = "queued",
        metadata: dict[str, Any] | None = None,
        session_id: str = "",
    ) -> int:
        payload = json.dumps(metadata or {}, separators=(",", ":"))
        with self._lock, self._connect() as connection:
            cursor = connection.execute(
                """
                INSERT INTO missions (session_id, command, status, metadata, created_at)
                VALUES (?, ?, ?, ?, ?)
                """,
                (session_id, command, status, payload, time.strftime("%Y-%m-%dT%H:%M:%S")),
            )
            connection.commit()
            return int(cursor.lastrowid)

    def get_locations(self, session_id: str = "") -> list[dict[str, Any]]:
        with self._lock, self._connect() as connection:
            if session_id:
                rows = connection.execute(
                    "SELECT * FROM locations WHERE session_id = ? ORDER BY id DESC",
                    (session_id,),
                ).fetchall()
            else:
                rows = connection.execute("SELECT * FROM locations ORDER BY id DESC").fetchall()
        return [self._row_to_location(row) for row in rows]

    def get_recent_missions(self, limit: int = 20, session_id: str = "") -> list[dict[str, Any]]:
        with self._lock, self._connect() as connection:
            if session_id:
                rows = connection.execute(
                    "SELECT * FROM missions WHERE session_id = ? ORDER BY id DESC LIMIT ?",
                    (session_id, int(limit)),
                ).fetchall()
            else:
                rows = connection.execute(
                    "SELECT * FROM missions ORDER BY id DESC LIMIT ?",
                    (int(limit),),
                ).fetchall()
        missions: list[dict[str, Any]] = []
        for row in rows:
            missions.append(
                {
                    "id": row["id"],
                    "session_id": row["session_id"],
                    "command": row["command"],
                    "status": row["status"],
                    "metadata": json.loads(row["metadata"] or "{}"),
                    "created_at": row["created_at"],
                }
            )
        return missions

    def _row_to_location(self, row: sqlite3.Row) -> dict[str, Any]:
        return {
            "id": row["id"],
            "session_id": row["session_id"],
            "label": row["label"],
            "room_number": row["room_number"],
            "x": row["x"],
            "y": row["y"],
            "yaw": row["yaw"],
            "confidence": row["confidence"],
            "metadata": json.loads(row["metadata"] or "{}"),
            "created_at": row["created_at"],
        }
