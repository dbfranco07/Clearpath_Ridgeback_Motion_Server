"""
Frontier-Based Exploration Helper
===================================
Finds frontier cells in a ROS2 OccupancyGrid (boundaries between known
free space and unknown space) and returns them as candidate exploration goals.

A frontier cell is a free cell adjacent to at least one unknown cell.
We cluster nearby frontier cells and return the centroid of each cluster
as a waypoint for Nav2.

This module is stateless — it operates purely on the map data passed in.
"""

import math
import numpy as np
from typing import List, Tuple


def find_frontiers(
    grid_data: list,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    robot_x: float,
    robot_y: float,
    min_cluster_size: int = 5,
    max_frontiers: int = 20
) -> List[Tuple[float, float]]:
    """
    Find frontier waypoints in the OccupancyGrid.

    Args:
        grid_data: flat int8 list from OccupancyGrid.data
                   -1 = unknown, 0 = free, 100 = occupied
        width, height: grid dimensions in cells
        resolution: meters per cell
        origin_x, origin_y: map origin in meters
        robot_x, robot_y: current robot position in meters (for sorting)
        min_cluster_size: minimum cells in a frontier cluster to be considered
        max_frontiers: maximum number of frontier waypoints to return

    Returns:
        List of (x, y) tuples in map coordinates, sorted by distance from robot.
    """
    grid = np.array(grid_data, dtype=np.int8).reshape((height, width))

    # Find all frontier cells: free cells (value == 0) adjacent to unknown (-1)
    frontier_mask = _find_frontier_cells(grid, height, width)

    # Cluster the frontier cells using simple BFS grouping
    clusters = _cluster_frontiers(frontier_mask, height, width)

    # Convert cluster centroids to map coordinates
    waypoints = []
    for cluster in clusters:
        if len(cluster) < min_cluster_size:
            continue
        cells = np.array(cluster)
        centroid_row = np.mean(cells[:, 0])
        centroid_col = np.mean(cells[:, 1])
        # Convert grid cell to map coordinates
        map_x = origin_x + (centroid_col + 0.5) * resolution
        map_y = origin_y + (centroid_row + 0.5) * resolution
        waypoints.append((map_x, map_y))

    # Sort by distance from robot (nearest first)
    waypoints.sort(key=lambda p: math.hypot(p[0] - robot_x, p[1] - robot_y))

    return waypoints[:max_frontiers]


def _find_frontier_cells(grid: np.ndarray, height: int, width: int) -> np.ndarray:
    """Return boolean mask of frontier cells."""
    # Free cells
    free = grid == 0
    # Unknown cells
    unknown = grid == -1

    # A free cell is a frontier if it has at least one unknown neighbor (4-connected)
    frontier = np.zeros((height, width), dtype=bool)

    # Check 4 neighbors
    frontier[1:, :] |= free[1:, :] & unknown[:-1, :]    # Above
    frontier[:-1, :] |= free[:-1, :] & unknown[1:, :]   # Below
    frontier[:, 1:] |= free[:, 1:] & unknown[:, :-1]    # Left
    frontier[:, :-1] |= free[:, :-1] & unknown[:, 1:]   # Right

    return frontier


def _cluster_frontiers(
    frontier_mask: np.ndarray, height: int, width: int
) -> List[List[Tuple[int, int]]]:
    """BFS-cluster connected frontier cells. Returns list of clusters."""
    visited = np.zeros((height, width), dtype=bool)
    clusters = []

    rows, cols = np.where(frontier_mask)

    for start_row, start_col in zip(rows, cols):
        if visited[start_row, start_col]:
            continue

        # BFS from this cell
        cluster = []
        queue = [(start_row, start_col)]
        visited[start_row, start_col] = True

        while queue:
            r, c = queue.pop(0)
            cluster.append((r, c))

            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # 8-connected
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width:
                    if frontier_mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        queue.append((nr, nc))

        clusters.append(cluster)

    # Sort by cluster size descending (largest frontiers first before size filter)
    clusters.sort(key=len, reverse=True)
    return clusters


def is_goal_reached(
    robot_x: float, robot_y: float,
    goal_x: float, goal_y: float,
    tolerance_m: float = 0.5
) -> bool:
    """Check if the robot has reached a goal position."""
    return math.hypot(robot_x - goal_x, robot_y - goal_y) < tolerance_m


def get_closest_room_direction(
    robot_x: float, robot_y: float, robot_yaw: float,
    room_x: float, room_y: float
) -> Tuple[float, float]:
    """
    Get the angular difference and distance to a room from the robot.
    Returns (angle_error_rad, distance_m) where angle_error is in [-pi, pi].
    """
    dx = room_x - robot_x
    dy = room_y - robot_y
    distance = math.hypot(dx, dy)
    target_angle = math.atan2(dy, dx)
    angle_error = target_angle - robot_yaw
    # Normalize to [-pi, pi]
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
    return angle_error, distance
