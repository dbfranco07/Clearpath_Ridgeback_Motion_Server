from __future__ import annotations

from conftest import read_repo_file


def test_room_detector_prompt_requires_strict_room_json() -> None:
    detector = read_repo_file("ridgeback_image_motion", "room_detector.py")
    assert "Return strict JSON only" in detector
    assert "room_detections" in detector
    assert "room_number" in detector
    assert "Use an empty room_detections array" in detector


def test_room_detector_filters_by_confidence_and_room_number() -> None:
    detector = read_repo_file("ridgeback_image_motion", "room_detector.py")
    assert "confidence >= min_conf" in detector
    assert "if room and confidence" in detector
    assert "detections_topic" in detector


def test_room_detector_throttles_vlm_calls_by_time_and_motion() -> None:
    params = read_repo_file("config", "autonomy_params.yaml")
    detector = read_repo_file("ridgeback_image_motion", "room_detector.py")
    assert "detect_period_s: 7.0" in params
    assert "min_vlm_travel_m: 0.75" in params
    assert "min_vlm_rotation_deg: 25.0" in params
    assert "self._movement_gate_passed()" in detector


def test_room_detector_subscribes_to_primary_and_fallback_camera_topics() -> None:
    params = read_repo_file("config", "autonomy_params.yaml")
    detector = read_repo_file("ridgeback_image_motion", "room_detector.py")
    assert "fallback_image_topic: /r100_0140/sensors/camera_0/color/compressed" in params
    assert "self.image_subscriptions" in detector
    assert "fallback_image_topic" in detector
    assert "lambda msg, source=source, topic=topic" in detector
    assert "frame_source" in detector
    assert "frame_topic" in detector


def test_vlm_endpoint_config_defaults_to_remote_and_normalizes_url() -> None:
    vlm = read_repo_file("ridgeback_image_motion", "vlm_client.py")
    env = read_repo_file("ridgeback_image_motion", ".env")
    assert "normalize_base_url" in vlm
    assert "OpenAI(base_url=resolved.openai_base_url" in vlm
    assert "VLM_ENDPOINT=http://" in env
    assert "VLM_PORT=" in env


def test_vlm_loss_is_safety_relevant_during_active_mission() -> None:
    safety = read_repo_file("ridgeback_image_motion", "safety_policy.py")
    assert "mission_active and not inputs.network_healthy" in safety
    assert "vlm_network_lost" in safety
    assert "vlm_unhealthy" in safety
