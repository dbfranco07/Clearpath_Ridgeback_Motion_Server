from __future__ import annotations

from ridgeback_image_motion.autonomy_common import parse_intent_and_room
from ridgeback_image_motion.spatial_memory import SpatialMemory

from conftest import read_repo_file


def test_parser_accepts_supported_room_mission_phrasing() -> None:
    examples = {
        "go to room 301": ("GO_TO_ROOM", "301"),
        "navigate to rm 302": ("GO_TO_ROOM", "302"),
        "find r 303A": ("GO_TO_ROOM", "303A"),
        "please visit room B204": ("GO_TO_ROOM", "B204"),
        "come back to start": ("RETURN_TO_START", ""),
        "cancel the mission": ("STOP", ""),
    }
    for text, expected in examples.items():
        parsed = parse_intent_and_room(text)
        assert (parsed["intent"], parsed["room"]) == expected


def test_parser_fails_closed_for_unsupported_or_ambiguous_commands() -> None:
    assert parse_intent_and_room("drive around for a while")["intent"] == "QUERY"
    assert parse_intent_and_room("room 301")["intent"] == "ROOM_QUERY"
    assert parse_intent_and_room("go somewhere")["intent"] == "QUERY"


def test_dashboard_uses_shared_parser_instead_of_duplicate_parser() -> None:
    dashboard = read_repo_file("ridgeback_image_motion", "web_dashboard.py")
    assert "from ridgeback_image_motion.autonomy_common import json_dumps, json_loads, parse_intent_and_room" in dashboard
    assert "def parse_intent_and_room(text" not in dashboard


def test_memory_lookup_does_not_cross_blank_map_sessions(tmp_path) -> None:
    memory = SpatialMemory(str(tmp_path / "memory.db"))
    old_session = memory.create_session({"run": "old"})
    memory.store_room_detection(old_session, "301", 5.0, 0.0, 0.0, 0.95)

    new_session = memory.create_session({"run": "new"})
    assert memory.find_room(new_session, "301", 0.55) is None


def test_target_room_detection_path_stops_exploration_and_navigates() -> None:
    orchestrator = read_repo_file("ridgeback_image_motion", "mission_orchestrator.py")
    assert 'if self.state == "EXPLORING" and self.target_room:' in orchestrator
    assert "self._stop_exploration()" in orchestrator
    assert 'self._navigate_to_location(found, "room")' in orchestrator
    assert 'self.state = "NAVIGATING_TO_ROOM"' in orchestrator
