#!/usr/bin/env python3
"""Shared helpers for Ridgeback room-finding autonomy nodes."""

from __future__ import annotations

import json
import math
import re
from typing import Any

try:
    from geometry_msgs.msg import Quaternion
except ImportError:
    class Quaternion:  # type: ignore[no-redef]
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0


ACTIVE_MISSION_STATES = {"STARTING", "EXPLORING", "NAVIGATING_TO_ROOM", "RETURNING_TO_START"}


def parse_intent_and_room(text: str) -> dict[str, str]:
    lowered = text.lower().strip()
    room = ""
    room_match = re.search(r"(?:room|rm|r)\s*#?\s*([a-z]?\d{2,4}[a-z]?)", lowered)
    if room_match:
        room = room_match.group(1).upper()

    intent = "QUERY"
    if re.search(r"\b(stop|halt|cancel|abort)\b", lowered):
        intent = "STOP"
    elif room and re.search(r"\b(go|navigate|head|move|find|reach|visit)\b", lowered):
        intent = "GO_TO_ROOM"
    elif re.search(r"\b(explore|look around|wander|survey|scan the area|map the area|build (?:a |the )?map)\b", lowered):
        intent = "EXPLORE"
    elif re.search(r"\b(return|go back|come back|home|start)\b", lowered):
        intent = "RETURN_TO_START"
    elif room:
        intent = "ROOM_QUERY"

    return {"intent": intent, "room": room}


def json_dumps(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def json_loads(text: str, default: dict[str, Any] | None = None) -> dict[str, Any]:
    try:
        value = json.loads(text)
    except Exception:
        return default or {}
    return value if isinstance(value, dict) else (default or {})


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def quaternion_to_yaw_rad(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def twist_is_nonzero(msg: Any, epsilon: float = 1e-5) -> bool:
    return (
        abs(float(msg.linear.x)) > epsilon
        or abs(float(msg.linear.y)) > epsilon
        or abs(float(msg.angular.z)) > epsilon
    )
