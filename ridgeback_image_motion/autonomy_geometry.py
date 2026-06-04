#!/usr/bin/env python3
"""Pure-Python geometry helpers for the autonomy nodes.

Deliberately free of ROS and numpy so the logic can be unit-tested anywhere
(and stays trivially fast at the rates we call it: a 270-deg scan is ~1080
points, the occupancy windows are a few hundred cells). The nodes pass plain
sequences / primitives extracted from their ROS messages.
"""

from __future__ import annotations

import math
from typing import Sequence


def passage_width(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    heading: float,
    forward_min: float,
    forward_max: float,
    lateral_window: float,
) -> float:
    """Free lateral width of the opening ahead, along `heading`.

    Projects each valid scan return into the travel frame (forward along
    heading, left positive), keeps only points inside the forward window and
    lateral band, and returns nearest-left-clear + nearest-right-clear. A side
    with no obstacle inside the window is treated as open (lateral_window).
    Returns +inf when no point bounds the corridor ahead at all.
    """
    c = math.cos(heading)
    s = math.sin(heading)
    left_clear = lateral_window
    right_clear = lateral_window
    any_band = False
    for i, raw in enumerate(ranges):
        try:
            r = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        px = r * math.cos(a)
        py = r * math.sin(a)
        forward = px * c + py * s
        lateral = -px * s + py * c
        if forward < forward_min or forward > forward_max:
            continue
        if abs(lateral) > lateral_window:
            continue
        any_band = True
        if lateral >= 0.0:
            if lateral < left_clear:
                left_clear = lateral
        else:
            d = -lateral
            if d < right_clear:
                right_clear = d
    if not any_band:
        return float("inf")
    return left_clear + right_clear


def decimate_path(
    points: Sequence[tuple[float, float]], min_spacing: float
) -> list[tuple[float, float]]:
    """Thin a polyline so consecutive kept points are >= min_spacing apart.

    The first point is always kept; subsequent points are dropped until one is
    at least min_spacing from the last kept point. Used to turn a dense
    breadcrumb trail into a manageable set of return waypoints. Pure helper.
    """
    out: list[tuple[float, float]] = []
    for p in points:
        if not out:
            out.append((float(p[0]), float(p[1])))
            continue
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_spacing:
            out.append((float(p[0]), float(p[1])))
    return out


def count_occupied_near(
    data: Sequence[int],
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    x: float,
    y: float,
    radius_m: float,
    occupied_threshold: int = 50,
) -> int:
    """Count occupied cells (value > threshold) within radius_m of (x, y).

    `data` is a row-major OccupancyGrid.data of length width*height.
    """
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return 0
    gx = int((x - origin_x) / resolution)
    gy = int((y - origin_y) / resolution)
    r_cells = max(int(radius_m / resolution), 1)
    x0 = max(0, gx - r_cells)
    x1 = min(width, gx + r_cells + 1)
    y0 = max(0, gy - r_cells)
    y1 = min(height, gy + r_cells + 1)
    if x1 <= x0 or y1 <= y0:
        return 0
    count = 0
    for ry in range(y0, y1):
        base = ry * width
        for v in data[base + x0 : base + x1]:
            if v > occupied_threshold:
                count += 1
    return count


def lidar_agrees(
    data: Sequence[int],
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    x: float,
    y: float,
    radius_m: float,
    min_occupied_cells: int,
    occupied_threshold: int = 50,
) -> bool:
    """True if the SLAM map shows real structure (wall/door frame) near (x, y).

    A genuine room sign sits in a wall, so a corroborated VLM read should have
    occupied cells nearby; a number 'seen' across open space won't.
    """
    occupied = count_occupied_near(
        data, width, height, origin_x, origin_y, resolution, x, y, radius_m, occupied_threshold
    )
    return occupied >= min_occupied_cells
