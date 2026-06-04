"""Tests for the door-fit gate, landmark agreement, and return-home wiring.

The geometry here is pure Python (no ROS/numpy), so it runs anywhere. The
return-home behavior is asserted as static-source contracts to avoid needing a
live ROS graph for what is fundamentally control-flow wiring.
"""

from __future__ import annotations

import math

from ridgeback_image_motion.autonomy_geometry import (
    count_occupied_near,
    lidar_agrees,
    passage_width,
)

from conftest import read_repo_file


# --- passage_width (door-fit gate) -----------------------------------------

_ANGLE_MIN = -math.pi / 2
_ANGLE_INC = math.pi / 180.0  # 1 degree
_N = 181  # -90..+90 deg
_RANGE_MIN = 0.05
_RANGE_MAX = 10.0
_FWD_MIN = 0.15
_FWD_MAX = 1.8
_LAT_WINDOW = 1.5


def _build_scan(points: list[tuple[float, float]]) -> list[float]:
    """Place (forward, lateral) obstacle points into a forward-facing scan."""
    ranges = [float("inf")] * _N
    for fx, ly in points:
        ang = math.atan2(ly, fx)
        r = math.hypot(fx, ly)
        idx = round((ang - _ANGLE_MIN) / _ANGLE_INC)
        if 0 <= idx < _N:
            ranges[idx] = min(ranges[idx], r)
    return ranges


def _width(points: list[tuple[float, float]]) -> float:
    return passage_width(
        _build_scan(points), _ANGLE_MIN, _ANGLE_INC, _RANGE_MIN, _RANGE_MAX,
        0.0, _FWD_MIN, _FWD_MAX, _LAT_WINDOW,
    )


def _walls(half_width: float) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for f in (0.3, 0.6, 0.9, 1.2, 1.5):
        pts.append((f, half_width))
        pts.append((f, -half_width))
    return pts


def test_passage_width_open_corridor_reports_infinite() -> None:
    # Nothing ahead in the window -> treated as fully open.
    assert math.isinf(_width([]))


def test_passage_width_wide_door_fits() -> None:
    # Walls at +/-0.8 m -> ~1.6 m opening, comfortably wider than a 0.8 m robot.
    width = _width(_walls(0.8))
    assert width == math.inf or width >= 1.1
    assert width >= 1.5  # roughly left_clear + right_clear = 0.8 + 0.8


def test_passage_width_narrow_gap_does_not_fit() -> None:
    # Walls at +/-0.45 m -> 0.90 m opening, below the 1.10 m the robot needs.
    width = _width(_walls(0.45))
    assert width < 1.10


def test_passage_width_one_sided_obstacle_uses_open_side() -> None:
    # Wall only on the left at 0.5 m; right side open -> open side caps at the
    # lateral window, so the corridor still reads as passable.
    width = _width([(f, 0.5) for f in (0.3, 0.6, 0.9, 1.2)])
    assert width >= 1.10


# --- lidar_agrees (VLM + LiDAR landmark gate) ------------------------------

def _grid_with_block(cx: int, cy: int, w: int, h: int) -> list[int]:
    """Row-major grid: -1 unknown everywhere, a 2x2 occupied block at (cx,cy)."""
    data = [-1] * (w * h)
    for dy in range(2):
        for dx in range(2):
            x, y = cx + dx, cy + dy
            if 0 <= x < w and 0 <= y < h:
                data[y * w + x] = 100
    return data


def test_lidar_agrees_when_structure_is_near_detection() -> None:
    w = h = 20
    res = 0.1
    data = _grid_with_block(10, 10, w, h)  # cells (10,10)..(11,11) occupied
    # World (1.0, 1.0) maps to cell (10, 10); 0.3 m radius covers the block.
    assert lidar_agrees(data, w, h, 0.0, 0.0, res, 1.0, 1.0, 0.3, 3) is True


def test_lidar_disagrees_in_open_space() -> None:
    w = h = 20
    res = 0.1
    data = _grid_with_block(10, 10, w, h)
    # World (1.8, 0.2) is far from the occupied block -> no agreement.
    assert lidar_agrees(data, w, h, 0.0, 0.0, res, 1.8, 0.2, 0.3, 3) is False


def test_count_occupied_ignores_unknown_and_free() -> None:
    w = h = 10
    data = [-1] * (w * h)  # all unknown
    data[5 * w + 5] = 0  # one free cell
    # Unknown + free must not count as structure.
    assert count_occupied_near(data, w, h, 0.0, 0.0, 0.1, 0.5, 0.5, 0.3) == 0


# --- return-home robustness (source contracts) -----------------------------

def test_return_home_retries_instead_of_stranding() -> None:
    src = read_repo_file("ridgeback_image_motion", "mission_orchestrator.py")
    # Retry path with bounded attempts + costmap clearing, and a terminal that
    # leaves the base steerable rather than the old silent abort.
    for token in (
        "max_return_attempts",
        "_resend_return_home",
        "_clear_costmaps",
        "ClearEntireCostmap",
        "RETURN_FAILED",
        "_return_after",
    ):
        assert token in src, f"missing return-home robustness token: {token}"


def test_terminal_explore_outcomes_route_to_return_home() -> None:
    src = read_repo_file("ridgeback_image_motion", "mission_orchestrator.py")
    # max explore time and map-complete must funnel into return home, not abort.
    assert src.count("_begin_return_home()") >= 3


def test_passage_monitor_is_jetson_launched_not_on_ridgeback() -> None:
    launch = read_repo_file("ridgeback_image_motion", "launch", "autonomy.launch.py")
    assert "passage_monitor.py" in launch
    start = read_repo_file("scripts", "ridgeback_start.sh")
    assert "passage_monitor" not in start  # robot side stays motion_server-only


def test_costmap_padding_zero_so_robot_clears_tight_doors() -> None:
    # Padding MUST stay 0.0: any padding inflates the inscribed radius past a
    # tight door's free center and the robot gets stuck at the threshold.
    nav2 = read_repo_file("config", "nav2_params.yaml")
    assert "footprint_padding: 0.04" not in nav2
    assert nav2.count("footprint_padding: 0.0") >= 2  # local + global


def test_passage_monitor_is_advisory_not_blocking_by_default() -> None:
    # With tight doors, an active reroute would blacklist the doorway and trap
    # the robot in the room. The fit gate must default to report-only.
    params = read_repo_file("config", "autonomy_params.yaml")
    assert "reroute_on_block: false" in params


def test_landmark_topic_wired_through_config() -> None:
    params = read_repo_file("config", "autonomy_params.yaml")
    assert "landmark_topic: /room/landmarks" in params
