import asyncio
import heapq
import numpy as np
import math
import time
from dataclasses import dataclass, field
from heapq import heappush, heappop
from typing import List, Tuple, Optional, Set, Dict

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap

class Pathfinder:
    def __init__(self, world_map, picar):
        self.world_map = world_map
        self.picar = picar
        self.path_rebuild_threshold = 20  # Rebuild path if more than 20cm from expected position
        self.max_path_steps = 50  # Maximum number of steps before forcing a path rebuild

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        x, y = pos
        neighbors = []

        # Check 8 surrounding cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))

        return neighbors

    def find_path(self, start_x: float, start_y: float,
                  goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Find path from start to goal in world coordinates"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        # Initialize data structures for A*
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        found_path = False

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                found_path = True
                break

            for next_pos in self.get_neighbors(current):
                # Calculate movement cost (diagonal moves cost more)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                if dx == 1 and dy == 1:
                    movement_cost = 1.4  # √2 for diagonal
                else:
                    movement_cost = 1.0

                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        if not found_path:
            return []

        # Reconstruct path
        current = goal_grid
        path = []

        while current is not None:
            # Convert grid coordinates back to world coordinates
            world_coords = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_coords)
            current = came_from[current]

        path.reverse()
        return path

    def check_path_validity(self, current_pos: Dict, path: List[Tuple[float, float]]) -> bool:
        """Check if current path is still valid"""
        if not path:
            return False

        # Get current position
        current_x = current_pos['x']
        current_y = current_pos['y']

        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0

        for i, (path_x, path_y) in enumerate(path):
            dist = math.sqrt((current_x - path_x) ** 2 + (current_y - path_y) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # If we're too far from path, it's invalid
        if min_dist > self.path_rebuild_threshold:
            return False

        # Check if any new obstacles block the remaining path
        for i in range(closest_idx, len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            # Convert to grid coordinates
            grid_x1, grid_y1 = self.world_map.world_to_grid(x1, y1)
            grid_x2, grid_y2 = self.world_map.world_to_grid(x2, y2)

            # Check for obstacles along this path segment
            points = self.get_line_points(grid_x1, grid_y1, grid_x2, grid_y2)
            for gx, gy in points:
                if (0 <= gx < self.world_map.grid_size and
                        0 <= gy < self.world_map.grid_size and
                        self.world_map.grid[gy, gx] != 0):
                    return False

        return True

    def get_line_points(self, x1: int, y1: int, x2: int, y2: int) -> List[Tuple[int, int]]:
        """Get all grid points along a line using Bresenham's algorithm"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1

        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1

        if dx > dy:
            err = dx / 2
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        points.append((x, y))
        return points

    def get_next_waypoint(self, current_pos: Dict, path: List[Tuple[float, float]],
                          lookahead_distance: float = 20.0) -> Tuple[float, float]:
        """Get next waypoint along path within lookahead distance"""
        if not path:
            return None

        current_x = current_pos['x']
        current_y = current_pos['y']

        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0

        for i, (path_x, path_y) in enumerate(path):
            dist = math.sqrt((current_x - path_x) ** 2 + (current_y - path_y) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Look ahead along path
        cumulative_dist = 0
        target_idx = closest_idx

        for i in range(closest_idx, len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            segment_dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            if cumulative_dist + segment_dist > lookahead_distance:
                # Interpolate point along this segment
                remaining_dist = lookahead_distance - cumulative_dist
                fraction = remaining_dist / segment_dist
                target_x = x1 + fraction * (x2 - x1)
                target_y = y1 + fraction * (y2 - y1)
                return (target_x, target_y)

            cumulative_dist += segment_dist
            target_idx = i + 1

        # If we reach end of path, return final point
        return path[-1]