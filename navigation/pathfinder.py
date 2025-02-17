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
    def __init__(self, world_map: WorldMap, picarx: PicarXWrapper):
        self.world_map = world_map
        self.picarx = picarx

        # Calculate grid cell size based on map parameters
        self.cell_size = self.world_map.resolution  # Size of each grid cell in cm

        # Define movements relative to grid size
        # Note: Diagonal cost is calculated dynamically based on cell_size
        self.movements = [
            (0, 1),  # up
            (1, 0),  # right
            (0, -1),  # down
            (-1, 0),  # left
            (1, 1),  # diagonal up-right
            (-1, 1),  # diagonal up-left
            (1, -1),  # diagonal down-right
            (-1, -1),  # diagonal down-left
        ]

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        Calculate heuristic (Euclidean distance) between points in grid coordinates
        Scales by cell_size to get real-world distance
        """
        grid_dist = np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
        return grid_dist * self.cell_size

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid positions considering grid boundaries"""
        neighbors = []

        for dx, dy in self.movements:
            new_pos = (pos[0] + dx, pos[1] + dy)

            # Check if within grid bounds
            if (0 <= new_pos[0] < self.world_map.grid_size and
                    0 <= new_pos[1] < self.world_map.grid_size):

                # For diagonal movements, check both cardinal directions for obstacles
                # to prevent cutting corners
                is_diagonal = dx != 0 and dy != 0
                if is_diagonal:
                    cardinal1 = (pos[0] + dx, pos[1])
                    cardinal2 = (pos[0], pos[1] + dy)
                    if (self.world_map.grid[cardinal1[1], cardinal1[0]] != 0 or
                            self.world_map.grid[cardinal2[1], cardinal2[0]] != 0):
                        continue

                # Check if not an obstacle
                if self.world_map.grid[new_pos[1], new_pos[0]] == 0:
                    neighbors.append(new_pos)

        return neighbors

    def calculate_movement_cost(self, current: Tuple[int, int], next_pos: Tuple[int, int]) -> float:
        """Calculate real-world movement cost between adjacent grid cells"""
        dx, dy = next_pos[0] - current[0], next_pos[1] - current[1]
        grid_dist = np.sqrt(dx * dx + dy * dy)  # 1.0 for cardinal, √2 for diagonal
        return grid_dist * self.cell_size  # Convert to real-world distance

    def find_path(self, start_pos: Tuple[float, float],
                  target_pos: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Find path from start to target position using A*

        Args:
            start_pos: Starting position in world coordinates (x, y)
            target_pos: Target position in world coordinates (x, y)

        Returns:
            List of waypoints in world coordinates, or None if no path found
        """
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_pos[0], start_pos[1])
        target_grid = self.world_map.world_to_grid(target_pos[0], target_pos[1])

        # Check if start or target is in obstacle
        if (self.world_map.grid[start_grid[1], start_grid[0]] != 0 or
                self.world_map.grid[target_grid[1], target_grid[0]] != 0):
            return None

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == target_grid:
                break

            for next_pos in self.get_neighbors(current):
                # Calculate real-world movement cost
                new_cost = cost_so_far[current] + self.calculate_movement_cost(current, next_pos)

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(next_pos, target_grid)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # If target not reached, no path exists
        if target_grid not in came_from:
            return None

        # Reconstruct path
        path = []
        current = target_grid
        while current is not None:
            # Convert back to world coordinates
            world_pos = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_pos)
            current = came_from[current]

        path.reverse()

        # Path smoothing: remove unnecessary waypoints
        return self.smooth_path(path)

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Look ahead as far as possible while maintaining clear line of sight
            for look_ahead in range(len(path) - 1, current_idx, -1):
                # Check if direct path is clear
                if self.is_line_of_sight_clear(path[current_idx], path[look_ahead]):
                    smoothed.append(path[look_ahead])
                    current_idx = look_ahead
                    break
            current_idx += 1

        return smoothed

    def is_line_of_sight_clear(self, start: Tuple[float, float],
                               end: Tuple[float, float]) -> bool:
        """Check if there's a clear line of sight between two points"""
        # Convert to grid coordinates
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        end_grid = self.world_map.world_to_grid(end[0], end[1])

        # Use Bresenham's line algorithm to check grid cells along the line
        x0, y0 = start_grid
        x1, y1 = end_grid
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        for _ in range(n):
            if self.world_map.grid[y, x] != 0:
                return False

            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

        return True