#!/usr/bin/env python3
"""Pure safety decision policy for Ridgeback autonomy.

This module intentionally has no ROS imports so laptop tests can exercise the
same stop/hold/hysteresis logic used by the Jetson safety node.
"""

from __future__ import annotations

from dataclasses import dataclass, field


ACTIVE_MISSION_STATES = {"STARTING", "EXPLORING", "NAVIGATING_TO_ROOM", "RETURNING_TO_START"}


@dataclass(frozen=True)
class SafetyPolicyConfig:
    warning_distance_m: float = 0.80
    danger_distance_m: float = 0.45
    danger_release_distance_m: float = 0.60
    lidar_timeout_s: float = 1.0
    odom_timeout_s: float = 1.0
    vlm_timeout_s: float = 8.0
    operator_heartbeat_timeout_s: float = 2.0
    stop_hold_s: float = 1.0
    require_operator_heartbeat: bool = True


@dataclass(frozen=True)
class SafetyInputs:
    now: float
    closest_obstacle_m: float = 99.0
    last_lidar_time: float = 0.0
    last_odom_time: float = 0.0
    estop_active: bool = False
    mission_state: str = "IDLE"
    network_healthy: bool = True
    vlm_healthy: bool = True
    last_vlm_status_time: float = 0.0
    last_operator_heartbeat_time: float = 0.0


@dataclass(frozen=True)
class SafetyDecision:
    is_safe: bool
    risk_level: str
    reasons: list[str] = field(default_factory=list)
    stop_recommended: bool = False
    stop_latched: bool = False
    hold_remaining_s: float = 0.0
    release_distance_m: float = 0.0
    lidar_age_s: float = 999.0
    odom_age_s: float = 999.0
    vlm_age_s: float = 999.0
    operator_heartbeat_age_s: float = 999.0


class SafetyDecisionFilter:
    """Stateful filter that prevents obstacle-threshold stop/go jitter."""

    def __init__(self, config: SafetyPolicyConfig | None = None) -> None:
        self.config = config or SafetyPolicyConfig()
        self._obstacle_latched = False
        self._stop_hold_until = 0.0

    def reset(self) -> None:
        self._obstacle_latched = False
        self._stop_hold_until = 0.0

    def evaluate(self, inputs: SafetyInputs) -> SafetyDecision:
        cfg = self.config
        now = float(inputs.now)
        lidar_age = now - inputs.last_lidar_time if inputs.last_lidar_time else 999.0
        odom_age = now - inputs.last_odom_time if inputs.last_odom_time else 999.0
        vlm_age = now - inputs.last_vlm_status_time if inputs.last_vlm_status_time else 999.0
        heartbeat_age = (
            now - inputs.last_operator_heartbeat_time if inputs.last_operator_heartbeat_time else 999.0
        )
        mission_active = inputs.mission_state in ACTIVE_MISSION_STATES

        reasons: list[str] = []

        if inputs.closest_obstacle_m < cfg.danger_distance_m:
            self._obstacle_latched = True
            self._stop_hold_until = max(self._stop_hold_until, now + cfg.stop_hold_s)
            reasons.append(f"obstacle {inputs.closest_obstacle_m:.2f}m")

        hold_remaining = max(0.0, self._stop_hold_until - now)
        if self._obstacle_latched:
            can_release = inputs.closest_obstacle_m >= cfg.danger_release_distance_m and hold_remaining <= 0.0
            if can_release:
                self._obstacle_latched = False
            else:
                if not any(reason.startswith("obstacle ") for reason in reasons):
                    if hold_remaining > 0.0:
                        reasons.append("obstacle_hold")
                    else:
                        reasons.append(
                            f"obstacle_clearance_below_release {inputs.closest_obstacle_m:.2f}m"
                        )

        if lidar_age > cfg.lidar_timeout_s:
            reasons.append("stale_lidar")
        if odom_age > cfg.odom_timeout_s:
            reasons.append("stale_odom")
        if inputs.estop_active:
            reasons.append("robot_estop")
        if mission_active and not inputs.network_healthy:
            reasons.append("vlm_network_lost")
        if mission_active and vlm_age < cfg.vlm_timeout_s and not inputs.vlm_healthy:
            reasons.append("vlm_unhealthy")
        if (
            cfg.require_operator_heartbeat
            and mission_active
            and heartbeat_age > cfg.operator_heartbeat_timeout_s
        ):
            reasons.append("operator_heartbeat_lost")

        risk = "OK"
        if reasons:
            risk = "DANGER"
        elif inputs.closest_obstacle_m < cfg.warning_distance_m:
            risk = "WARNING"

        return SafetyDecision(
            is_safe=not reasons,
            risk_level=risk,
            reasons=reasons,
            stop_recommended=bool(reasons),
            stop_latched=self._obstacle_latched,
            hold_remaining_s=hold_remaining,
            release_distance_m=cfg.danger_release_distance_m,
            lidar_age_s=lidar_age,
            odom_age_s=odom_age,
            vlm_age_s=vlm_age,
            operator_heartbeat_age_s=heartbeat_age,
        )
