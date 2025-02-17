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
        """Calculate heuristic (Euclidean distance) between points"""
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid positions"""
        neighbors = []
        for dx, dy in self.movements:
            new_pos = (pos[0] + dx, pos[1] + dy)

            # Check if within grid bounds
            if (0 <= new_pos[0] < self.world_map.grid_size and
                    0 <= new_pos[1] < self.world_map.grid_size):

                # Check if not an obstacle
                if self.world_map.grid[new_pos[1], new_pos[0]] == 0:
                    neighbors.append(new_pos)

        return neighbors

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
                # Calculate movement cost (diagonal movements cost more)
                dx, dy = next_pos[0] - current[0], next_pos[1] - current[1]
                movement_cost = np.sqrt(dx * dx + dy * dy)
                new_cost = cost_so_far[current] + movement_cost

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
        return path

    def is_path_clear(self, path: List[Tuple[float, float]], threshold: float = 1.0) -> bool:
        """
        Check if path is clear of obstacles

        Args:
            path: List of waypoints in world coordinates
            threshold: Number of grid cells to check for clearance

        Returns:
            True if path is clear, False if blocked
        """
        for x, y in path:
            grid_x, grid_y = self.world_map.world_to_grid(x, y)

            # Check surrounding cells within threshold
            for dx in range(-int(threshold), int(threshold) + 1):
                for dy in range(-int(threshold), int(threshold) + 1):
                    check_x = grid_x + dx
                    check_y = grid_y + dy

                    # Ensure within grid bounds
                    if (0 <= check_x < self.world_map.grid_size and
                            0 <= check_y < self.world_map.grid_size):

                        # If obstacle detected, path is blocked
                        if self.world_map.grid[check_y, check_x] != 0:
                            return False

        return True