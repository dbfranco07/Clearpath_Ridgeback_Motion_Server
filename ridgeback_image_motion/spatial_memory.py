#!/usr/bin/env python3
"""SQLite-backed spatial and mission memory for Ridgeback autonomy."""

from __future__ import annotations

import json
import sqlite3
import threading
import time
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
                CREATE TABLE IF NOT EXISTS locations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
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
                    command TEXT NOT NULL,
                    status TEXT NOT NULL,
                    metadata TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL
                )
                """
            )
            connection.commit()

    def store_location(self, location: SpatialLocation) -> None:
        metadata = json.dumps(location.metadata or {}, separators=(",", ":"))
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO locations (label, room_number, x, y, yaw, confidence, metadata, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
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

    def record_mission(self, command: str, status: str = "queued", metadata: dict[str, Any] | None = None) -> None:
        payload = json.dumps(metadata or {}, separators=(",", ":"))
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO missions (command, status, metadata, created_at)
                VALUES (?, ?, ?, ?)
                """,
                (command, status, payload, time.strftime("%Y-%m-%dT%H:%M:%S")),
            )
            connection.commit()

    def get_locations(self) -> list[dict[str, Any]]:
        with self._lock, self._connect() as connection:
            rows = connection.execute("SELECT * FROM locations ORDER BY id DESC").fetchall()
        locations: list[dict[str, Any]] = []
        for row in rows:
            locations.append(
                {
                    "id": row["id"],
                    "label": row["label"],
                    "room_number": row["room_number"],
                    "x": row["x"],
                    "y": row["y"],
                    "yaw": row["yaw"],
                    "confidence": row["confidence"],
                    "metadata": json.loads(row["metadata"] or "{}"),
                    "created_at": row["created_at"],
                }
            )
        return locations

    def get_recent_missions(self, limit: int = 20) -> list[dict[str, Any]]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                "SELECT * FROM missions ORDER BY id DESC LIMIT ?",
                (int(limit),),
            ).fetchall()
        missions: list[dict[str, Any]] = []
        for row in rows:
            missions.append(
                {
                    "id": row["id"],
                    "command": row["command"],
                    "status": row["status"],
                    "metadata": json.loads(row["metadata"] or "{}"),
                    "created_at": row["created_at"],
                }
            )
        return missions