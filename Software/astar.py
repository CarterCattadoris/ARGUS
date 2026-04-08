"""
ARGUS — A* Pathfinding on Occupancy Grid

8-directional A* that operates on a 2D numpy grid where
0 = free and 1 = obstacle.  Returns a list of world-coordinate
waypoints from start to goal.
"""

import heapq
import math
import numpy as np
import logging

log = logging.getLogger("argus.astar")

# 8-directional moves: (drow, dcol, cost)
_DIRECTIONS = [
    (-1,  0, 1.0),   # N
    ( 1,  0, 1.0),   # S
    ( 0, -1, 1.0),   # W
    ( 0,  1, 1.0),   # E
    (-1, -1, 1.414), # NW
    (-1,  1, 1.414), # NE
    ( 1, -1, 1.414), # SW
    ( 1,  1, 1.414), # SE
]


def _heuristic(a, b):
    """Octile distance heuristic (admissible for 8-dir movement)."""
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return max(dr, dc) + (1.414 - 1.0) * min(dr, dc)


def astar_grid(grid, start, goal):
    """
    Run A* on a 2D occupancy grid.

    Args:
        grid:  numpy uint8 array, shape (rows, cols). 0=free, 1=obstacle.
        start: (row, col) tuple — must be a free cell.
        goal:  (row, col) tuple — must be a free cell.

    Returns:
        List of (row, col) cells forming the path from start to goal,
        inclusive.  Returns empty list if no path exists.
    """
    rows, cols = grid.shape

    # Clamp to grid bounds
    start = (max(0, min(rows - 1, start[0])), max(0, min(cols - 1, start[1])))
    goal  = (max(0, min(rows - 1, goal[0])),  max(0, min(cols - 1, goal[1])))

    # If start or goal is blocked, try to find nearest free cell
    if grid[start[0], start[1]] == 1:
        start = _nearest_free(grid, start)
        if start is None:
            return []
    if grid[goal[0], goal[1]] == 1:
        goal = _nearest_free(grid, goal)
        if goal is None:
            return []

    if start == goal:
        return [start]

    # Priority queue: (f_cost, counter, (row, col))
    counter = 0
    open_set = [(0.0, counter, start)]
    came_from = {}
    g_score = {start: 0.0}

    while open_set:
        f, _, current = heapq.heappop(open_set)

        if current == goal:
            return _reconstruct(came_from, current)

        for dr, dc, cost in _DIRECTIONS:
            nr, nc = current[0] + dr, current[1] + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                # For diagonal moves, check that both axis-adjacent cells are free
                # to prevent cutting corners around obstacles
                if dr != 0 and dc != 0:
                    if grid[current[0] + dr, current[1]] == 1 or \
                       grid[current[0], current[1] + dc] == 1:
                        continue

                tentative_g = g_score[current] + cost
                neighbour = (nr, nc)
                if tentative_g < g_score.get(neighbour, float('inf')):
                    g_score[neighbour] = tentative_g
                    f_score = tentative_g + _heuristic(neighbour, goal)
                    came_from[neighbour] = current
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbour))

    log.warning(f"A* found no path from {start} to {goal}")
    return []


def _reconstruct(came_from, current):
    """Trace back the path from goal to start."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _nearest_free(grid, cell, max_radius=10):
    """Spiral search for the nearest free cell around a blocked cell."""
    rows, cols = grid.shape
    for r in range(1, max_radius + 1):
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if abs(dr) != r and abs(dc) != r:
                    continue  # only check the ring perimeter
                nr, nc = cell[0] + dr, cell[1] + dc
                if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                    return (nr, nc)
    return None


def find_path_world(grid, start_world, goal_world, detector):
    """
    Convenience wrapper: takes world coords + ObstacleDetector, returns
    a list of (x_m, y_m) world-coordinate waypoints.

    Args:
        grid:        occupancy grid (numpy uint8).
        start_world: (x_m, y_m) robot position.
        goal_world:  (x_m, y_m) target position.
        detector:    ObstacleDetector instance (for coordinate conversion).

    Returns:
        List of (x_m, y_m) waypoints, or empty list on failure.
    """
    start_cell = detector.world_to_grid(start_world[0], start_world[1])
    goal_cell  = detector.world_to_grid(goal_world[0], goal_world[1])

    cell_path = astar_grid(grid, start_cell, goal_cell)
    if not cell_path:
        return []

    # Convert to world coordinates
    world_path = [detector.grid_to_world(r, c) for r, c in cell_path]

    # Simplify: Ramer-Douglas-Peucker removes redundant points,
    # keeping only key turning points
    simplified = _rdp_simplify(world_path, epsilon=0.08)  # 8cm tolerance for smoother paths

    return simplified


def find_path_along_waypoints(grid, start_world, waypoints, target, detector):
    """
    Route A* through a sequence of user-drawn waypoints, only detouring
    around obstacles on each segment.  This preserves the user's intended
    path shape and only modifies segments that are blocked.

    Args:
        grid:        occupancy grid (numpy uint8).
        start_world: (x_m, y_m) current robot position.
        waypoints:   list of (x_m, y_m) user-drawn waypoints.
        target:      (x_m, y_m) final target.
        detector:    ObstacleDetector instance.

    Returns:
        List of (x_m, y_m) waypoints forming the full route.
    """
    # Build the full sequence of points to route through:
    #   robot → waypoint[0] → waypoint[1] → ... → target
    checkpoints = list(waypoints) + [target]

    full_path = []
    current = start_world

    for checkpoint in checkpoints:
        # Check if the direct segment is blocked
        segment_blocked = _is_segment_blocked(grid, current, checkpoint, detector)

        if segment_blocked:
            # Run A* for this segment
            sub_path = find_path_world(grid, current, checkpoint, detector)
            if sub_path:
                # Skip the first point (it's the current position, already in path)
                if full_path:
                    sub_path = sub_path[1:]
                full_path.extend(sub_path)
            else:
                # No path found — just add the checkpoint directly
                full_path.append(checkpoint)
        else:
            # Segment is clear — go direct
            full_path.append(checkpoint)

        current = checkpoint

    return full_path


def _is_segment_blocked(grid, start_world, end_world, detector):
    """
    Check if a straight line between two world points passes through
    any obstacle cells using Bresenham-like sampling.
    """
    r1, c1 = detector.world_to_grid(start_world[0], start_world[1])
    r2, c2 = detector.world_to_grid(end_world[0], end_world[1])

    rows, cols = grid.shape
    # Sample points along the line
    steps = max(abs(r2 - r1), abs(c2 - c1), 1)
    for i in range(steps + 1):
        t = i / steps
        r = int(r1 + (r2 - r1) * t)
        c = int(c1 + (c2 - c1) * t)
        if 0 <= r < rows and 0 <= c < cols and grid[r, c] == 1:
            return True
    return False


def _rdp_simplify(points, epsilon=0.04):
    """
    Ramer-Douglas-Peucker path simplification.
    Removes points that are within `epsilon` metres of the straight line
    between their neighbours.  This collapses long straight segments into
    single lines and keeps only the key turns.
    """
    if len(points) <= 2:
        return points

    # Find the point furthest from the line between first and last
    start = points[0]
    end = points[-1]
    max_dist = 0.0
    max_idx = 0

    for i in range(1, len(points) - 1):
        d = _point_line_distance(points[i], start, end)
        if d > max_dist:
            max_dist = d
            max_idx = i

    if max_dist > epsilon:
        # Recurse on both halves
        left = _rdp_simplify(points[:max_idx + 1], epsilon)
        right = _rdp_simplify(points[max_idx:], epsilon)
        return left[:-1] + right
    else:
        # All intermediate points are close to the line — just keep endpoints
        return [start, end]


def _point_line_distance(point, line_start, line_end):
    """Perpendicular distance from point to line segment."""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    dx = x2 - x1
    dy = y2 - y1
    line_len_sq = dx * dx + dy * dy

    if line_len_sq < 1e-12:
        # line_start == line_end
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

    # Project point onto line
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / line_len_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)


def is_path_blocked(grid, path_world, detector):
    """
    Check if any waypoint in the current path is blocked by an obstacle.
    Used to decide whether replanning is needed.
    """
    for wx, wy in path_world:
        r, c = detector.world_to_grid(wx, wy)
        rows, cols = grid.shape
        if 0 <= r < rows and 0 <= c < cols and grid[r, c] == 1:
            return True
    return False

