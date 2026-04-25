from __future__ import annotations

from ridgeback_image_motion.safety_policy import SafetyDecisionFilter, SafetyInputs, SafetyPolicyConfig


def make_filter() -> SafetyDecisionFilter:
    return SafetyDecisionFilter(
        SafetyPolicyConfig(
            danger_distance_m=0.45,
            danger_release_distance_m=0.60,
            warning_distance_m=0.80,
            lidar_timeout_s=1.0,
            odom_timeout_s=1.0,
            vlm_timeout_s=8.0,
            operator_heartbeat_timeout_s=2.0,
            stop_hold_s=1.0,
            require_operator_heartbeat=True,
        )
    )


def safe_inputs(now: float = 10.0, **overrides) -> SafetyInputs:
    values = {
        "now": now,
        "closest_obstacle_m": 2.0,
        "last_lidar_time": now - 0.1,
        "last_odom_time": now - 0.1,
        "estop_active": False,
        "mission_state": "EXPLORING",
        "network_healthy": True,
        "vlm_healthy": True,
        "last_vlm_status_time": now - 0.1,
        "last_operator_heartbeat_time": now - 0.1,
    }
    values.update(overrides)
    return SafetyInputs(**values)


def test_safe_inputs_are_safe() -> None:
    decision = make_filter().evaluate(safe_inputs())
    assert decision.is_safe
    assert decision.risk_level == "OK"


def test_stale_lidar_stale_odom_estop_and_network_are_unsafe() -> None:
    cases = [
        ("stale_lidar", {"last_lidar_time": 0.0}),
        ("stale_odom", {"last_odom_time": 0.0}),
        ("robot_estop", {"estop_active": True}),
        ("vlm_network_lost", {"network_healthy": False}),
        ("vlm_unhealthy", {"vlm_healthy": False}),
        ("operator_heartbeat_lost", {"last_operator_heartbeat_time": 1.0}),
    ]
    for reason, overrides in cases:
        decision = make_filter().evaluate(safe_inputs(**overrides))
        assert not decision.is_safe
        assert reason in decision.reasons


def test_vlm_network_loss_does_not_block_idle_state() -> None:
    decision = make_filter().evaluate(
        safe_inputs(mission_state="IDLE", network_healthy=False, last_operator_heartbeat_time=0.0)
    )
    assert decision.is_safe


def test_obstacle_inside_danger_distance_latches_stop() -> None:
    policy = make_filter()
    decision = policy.evaluate(safe_inputs(now=10.0, closest_obstacle_m=0.30))
    assert not decision.is_safe
    assert decision.stop_latched
    assert any(reason.startswith("obstacle ") for reason in decision.reasons)


def test_obstacle_hysteresis_prevents_near_threshold_jitter() -> None:
    policy = make_filter()
    assert not policy.evaluate(safe_inputs(now=10.0, closest_obstacle_m=0.40)).is_safe
    still_stopped = policy.evaluate(safe_inputs(now=10.2, closest_obstacle_m=0.50))
    assert not still_stopped.is_safe
    assert still_stopped.stop_latched
    assert "obstacle_hold" in still_stopped.reasons


def test_obstacle_release_requires_release_distance_and_hold_expiration() -> None:
    policy = make_filter()
    assert not policy.evaluate(safe_inputs(now=10.0, closest_obstacle_m=0.40)).is_safe
    too_soon = policy.evaluate(safe_inputs(now=10.5, closest_obstacle_m=1.00))
    assert not too_soon.is_safe
    assert too_soon.stop_latched

    released = policy.evaluate(safe_inputs(now=11.2, closest_obstacle_m=1.00))
    assert released.is_safe
    assert not released.stop_latched
