from collections import deque
from dataclasses import dataclass
import heapq
import math
from pathlib import Path

import numpy as np
import yaml

from .map_utils import fine_cell_to_world, load_occupancy_map, world_to_fine_cell


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float | None = None
    quaternion: tuple[float, float, float, float] | None = None


@dataclass(frozen=True)
class CoarseMap:
    resolution: float
    block_size: int
    width: int
    height: int
    grid: np.ndarray


def load_waypoints(yaml_path):
    yaml_path = Path(yaml_path)
    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if "waypoints" not in data:
        raise ValueError(f"Expected top-level 'waypoints' key in {yaml_path}")

    parsed = {}
    for name, entry in data["waypoints"].items():
        quaternion = None
        if entry.get("quaternion") is not None:
            q = entry["quaternion"]
            if len(q) != 4:
                raise ValueError(f"Waypoint '{name}' has invalid quaternion length")
            quaternion = tuple(float(value) for value in q)

        parsed[name] = Waypoint(
            name=name,
            x=float(entry["x"]),
            y=float(entry["y"]),
            yaw=float(entry["yaw"]) if entry.get("yaw") is not None else None,
            quaternion=quaternion,
        )

    return parsed


def inflate_grid(grid, inflation_radius_m, resolution):
    radius_cells = int(math.ceil(inflation_radius_m / resolution))
    if radius_cells <= 0:
        return np.where(grid == 1, 1, 0).astype(np.int8)

    # Only inflate known occupied cells. Treating unknown cells as obstacles here
    # can erase narrow restaurant aisles before the planner ever runs.
    blocked = grid == 1
    inflated = blocked.copy()

    occupied_rows, occupied_cols = np.nonzero(blocked)
    for row, col in zip(occupied_rows, occupied_cols):
        row_min = max(0, row - radius_cells)
        row_max = min(grid.shape[0], row + radius_cells + 1)
        col_min = max(0, col - radius_cells)
        col_max = min(grid.shape[1], col + radius_cells + 1)

        for nbr_row in range(row_min, row_max):
            row_offset = nbr_row - row
            remaining = radius_cells * radius_cells - row_offset * row_offset
            if remaining < 0:
                continue
            col_span = int(math.floor(math.sqrt(remaining)))
            left = max(0, col - col_span)
            right = min(grid.shape[1], col + col_span + 1)
            inflated[nbr_row, left:right] = True

    return inflated.astype(np.int8)


def coarsen_grid(grid, block_size, occupied_fraction_threshold=0.25):
    if block_size <= 0:
        raise ValueError("block_size must be positive")
    if not (0.0 < occupied_fraction_threshold <= 1.0):
        raise ValueError("occupied_fraction_threshold must be in (0.0, 1.0]")

    height, width = grid.shape
    coarse_height = int(math.ceil(height / block_size))
    coarse_width = int(math.ceil(width / block_size))
    coarse = np.ones((coarse_height, coarse_width), dtype=np.int8)

    for coarse_row in range(coarse_height):
        row_start = coarse_row * block_size
        row_end = min(height, row_start + block_size)
        for coarse_col in range(coarse_width):
            col_start = coarse_col * block_size
            col_end = min(width, col_start + block_size)
            block = grid[row_start:row_end, col_start:col_end]
            occupied_fraction = np.count_nonzero(block != 0) / block.size
            coarse[coarse_row, coarse_col] = 1 if occupied_fraction >= occupied_fraction_threshold else 0

    return coarse


def world_to_coarse_cell(x, y, grid_map, block_size):
    row, col = world_to_fine_cell(x, y, grid_map)
    return row // block_size, col // block_size


def coarse_cell_to_world(coarse_row, coarse_col, grid_map, block_size):
    center_row = coarse_row * block_size + block_size // 2
    center_col = coarse_col * block_size + block_size // 2
    return fine_cell_to_world(center_row, center_col, grid_map)


def snap_to_nearest_free(coarse_cell, coarse_map):
    start_row, start_col = coarse_cell
    if not (0 <= start_row < coarse_map.height and 0 <= start_col < coarse_map.width):
        raise ValueError(
            f"Coarse cell {coarse_cell} is outside map bounds "
            f"{coarse_map.height}x{coarse_map.width}"
        )

    if coarse_map.grid[start_row, start_col] == 0:
        return coarse_cell

    visited = set()
    queue = deque([coarse_cell])
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while queue:
        row, col = queue.popleft()
        if (row, col) in visited:
            continue
        visited.add((row, col))

        if 0 <= row < coarse_map.height and 0 <= col < coarse_map.width:
            if coarse_map.grid[row, col] == 0:
                return row, col
            for d_row, d_col in neighbors:
                queue.append((row + d_row, col + d_col))

    raise RuntimeError("No free coarse cell found near the requested coordinate")


def astar(coarse_map, start, goal):
    start = tuple(start)
    goal = tuple(goal)

    for label, cell in (("start", start), ("goal", goal)):
        row, col = cell
        if not (0 <= row < coarse_map.height and 0 <= col < coarse_map.width):
            raise ValueError(f"A* {label} cell {cell} is outside the map")
        if coarse_map.grid[row, col] != 0:
            raise ValueError(f"A* {label} cell {cell} is occupied")

    def heuristic(cell_a, cell_b):
        row_a, col_a = cell_a
        row_b, col_b = cell_b
        return math.hypot(col_a - col_b, row_a - row_b)

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    open_heap = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct_path(came_from, current)

        row, col = current
        for d_row, d_col in neighbors:
            nbr_row = row + d_row
            nbr_col = col + d_col
            if not (0 <= nbr_row < coarse_map.height and 0 <= nbr_col < coarse_map.width):
                continue
            if coarse_map.grid[nbr_row, nbr_col] == 1:
                continue
            if d_row != 0 and d_col != 0:
                if coarse_map.grid[row + d_row, col] == 1 or coarse_map.grid[row, col + d_col] == 1:
                    continue

            step_cost = math.sqrt(2.0) if d_row != 0 and d_col != 0 else 1.0
            neighbor = (nbr_row, nbr_col)
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heapq.heappush(open_heap, (tentative_g + heuristic(neighbor, goal), neighbor))

    return []


def simplify_path(path):
    if len(path) < 3:
        return path

    simplified = [path[0]]
    for index in range(1, len(path) - 1):
        prev_row, prev_col = simplified[-1]
        curr_row, curr_col = path[index]
        next_row, next_col = path[index + 1]

        direction_in = (curr_row - prev_row, curr_col - prev_col)
        direction_out = (next_row - curr_row, next_col - curr_col)
        if direction_in != direction_out:
            simplified.append(path[index])

    simplified.append(path[-1])
    return simplified


def path_to_world(path, grid_map, block_size):
    return [coarse_cell_to_world(row, col, grid_map, block_size) for row, col in path]


def build_planner(map_yaml_path, waypoint_yaml_path, block_size=4, inflation_radius_m=0.15):
    grid_map = load_occupancy_map(map_yaml_path)
    inflated = inflate_grid(
        grid_map.occupancy,
        inflation_radius_m=inflation_radius_m,
        resolution=grid_map.meta.resolution,
    )
    coarse_grid = coarsen_grid(inflated, block_size)
    coarse_map = CoarseMap(
        resolution=grid_map.meta.resolution * block_size,
        block_size=block_size,
        width=coarse_grid.shape[1],
        height=coarse_grid.shape[0],
        grid=coarse_grid,
    )
    waypoints = load_waypoints(waypoint_yaml_path)
    return grid_map, coarse_map, waypoints


def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
