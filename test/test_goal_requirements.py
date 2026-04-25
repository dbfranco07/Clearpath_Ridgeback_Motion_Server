from __future__ import annotations

from ridgeback_image_motion.autonomy_common import parse_intent_and_room
from ridgeback_image_motion.spatial_memory import SpatialMemory

from conftest import read_repo_file


def test_requirement_1_natural_language_room_return_and_stop_commands_parse() -> None:
    assert parse_intent_and_room("go to room 301 and come back") == {
        "intent": "GO_TO_ROOM",
        "room": "301",
    }
    assert parse_intent_and_room("please find rm A204") == {
        "intent": "GO_TO_ROOM",
        "room": "A204",
    }
    assert parse_intent_and_room("return home")["intent"] == "RETURN_TO_START"
    assert parse_intent_and_room("stop now")["intent"] == "STOP"


def test_requirement_2_jetson_launch_owns_slam_nav2_exploration_and_mission() -> None:
    launch = read_repo_file("ridgeback_image_motion", "launch", "autonomy.launch.py")
    for token in (
        "async_slam_toolbox_node",
        "navigation_launch.py",
        "mission_orchestrator.py",
        "frontier_explorer.py",
        "room_detector.py",
        "safety_controller.py",
        "cmd_vel_mux.py",
        "web_dashboard.py",
    ):
        assert token in launch


def test_requirement_3_spatial_memory_is_session_scoped(tmp_path) -> None:
    memory = SpatialMemory(str(tmp_path / "memory.db"))
    session_a = memory.create_session({"test": "a"})
    session_b = memory.create_session({"test": "b"})

    memory.store_start_position(session_a, 1.0, 2.0, 0.1)
    memory.store_room_detection(session_a, "301", 3.0, 4.0, 0.2, 0.9)
    memory.store_start_position(session_b, 10.0, 20.0, 1.1)

    assert memory.find_room(session_a, "301", 0.55)["x"] == 3.0
    assert memory.find_room(session_b, "301", 0.55) is None
    assert memory.get_start_position(session_b)["x"] == 10.0


def test_requirement_4_slam_starts_blank_with_no_preloaded_map() -> None:
    slam = read_repo_file("config", "slam_params.yaml")
    assert "mode: mapping" in slam
    active_lines = [
        line.strip()
        for line in slam.splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    assert not any(line.startswith("map_file_name:") for line in active_lines)
    assert not any(line.startswith("serialized_map_file_name:") for line in active_lines)


def test_requirement_5_vlm_room_number_detection_contract_exists() -> None:
    room_detector = read_repo_file("ridgeback_image_motion", "room_detector.py")
    assert "room_detections" in room_detector
    assert "room_number" in room_detector
    assert "strict JSON" in room_detector
    assert "min_confidence" in read_repo_file("config", "autonomy_params.yaml")


def test_requirement_6_jetson_safety_watchdog_mux_and_heartbeat_are_configured() -> None:
    launch = read_repo_file("ridgeback_image_motion", "launch", "autonomy.launch.py")
    params = read_repo_file("config", "autonomy_params.yaml")
    dashboard = read_repo_file("ridgeback_image_motion", "web_dashboard.py")
    assert "jetson_watchdog.py" in launch
    assert '"require_initial_heartbeat": True' in launch
    assert "safety_controller.py" in launch
    assert "cmd_vel_mux.py" in launch
    assert "operator_heartbeat_topic: /pc_heartbeat" in params
    assert "/api/heartbeat" in dashboard


def test_requirement_7_safety_covers_people_sensor_and_network_failure_modes() -> None:
    params = read_repo_file("config", "autonomy_params.yaml")
    safety = read_repo_file("ridgeback_image_motion", "safety_controller.py")
    policy = read_repo_file("ridgeback_image_motion", "safety_policy.py")
    for token in (
        "danger_distance_m",
        "danger_release_distance_m",
        "lidar_timeout_s",
        "odom_timeout_s",
        "emergency_stop_topic",
        "operator_heartbeat_timeout_s",
        "vlm_network_lost",
        "stop_hold_s",
    ):
        assert token in params or token in safety or token in policy
