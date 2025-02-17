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
    def __init__(self, world_map, picarx):
        self.world_map = world_map
        self.picarx = picarx

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4  # sqrt(2)

        # Possible movement directions (8-directional movement)
        self.DIRECTIONS = [
            (-1, -1), (-1, 0), (-1, 1),  # Northwest, North, Northeast
            (0, -1), (0, 1),  # West, East
            (1, -1), (1, 0), (1, 1)  # Southwest, South, Southeast
        ]

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic (diagonal distance) between two points"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def _get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        neighbors = []

        for dx, dy in self.DIRECTIONS:
            new_x = current[0] + dx
            new_y = current[1] + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))

        return neighbors

    def _movement_cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate cost of movement between adjacent cells"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        # Diagonal movement
        if dx == 1 and dy == 1:
            return self.DIAGONAL_COST
        # Straight movement
        return self.STRAIGHT_COST

    def find_path(self, start_x: float, start_y: float,
                  target_x: float, target_y: float) -> List[Tuple[float, float]]:
        """
        Find a path from start to target position using A* algorithm

        Args:
            start_x, start_y: Starting position in world coordinates (cm)
            target_x, target_y: Target position in world coordinates (cm)

        Returns:
            List of (x, y) waypoints in world coordinates (cm)
        """
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        target_grid = self.world_map.world_to_grid(target_x, target_y)

        # Check if target is reachable
        if self.world_map.grid[target_grid[1], target_grid[0]] != 0:
            print("Target position is blocked by obstacle")
            return []

        # Initialize data structures
        frontier = []  # Priority queue of nodes to explore
        heapq.heappush(frontier, (0, start_grid))

        came_from = {start_grid: None}  # Path tracking
        cost_so_far = {start_grid: 0}  # Cost to reach each node

        # A* search
        while frontier:
            current = heapq.heappop(frontier)[1]

            # Path found
            if current == target_grid:
                break

            # Explore neighbors
            for next_pos in self._get_neighbors(current):
                new_cost = cost_so_far[current] + self._movement_cost(current, next_pos)

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self._heuristic(next_pos, target_grid)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # No path found
        if target_grid not in came_from:
            print("No path found to target")
            return []

        # Reconstruct path
        path = []
        current = target_grid
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()

        # Convert back to world coordinates
        world_path = [self.world_map.grid_to_world(x, y) for x, y in path]
        return world_path

    def smooth_path(self, path: List[Tuple[float, float]],
                    smoothing_strength: float = 0.5) -> List[Tuple[float, float]]:
        """
        Apply path smoothing to reduce sharp turns

        Args:
            path: List of (x, y) waypoints
            smoothing_strength: How much to smooth (0-1)

        Returns:
            Smoothed path
        """
        if len(path) <= 2:
            return path

        smoothed = list(path)
        change = True
        while change:
            change = False
            for i in range(1, len(smoothed) - 1):
                old_x, old_y = smoothed[i]

                # Calculate smoothed position
                smoothed[i] = (
                    old_x + smoothing_strength * (smoothed[i - 1][0] + smoothed[i + 1][0] - 2 * old_x),
                    old_y + smoothing_strength * (smoothed[i - 1][1] + smoothed[i + 1][1] - 2 * old_y)
                )

                # Check if significant change occurred
                if abs(smoothed[i][0] - old_x) > 0.1 or abs(smoothed[i][1] - old_y) > 0.1:
                    change = True

        return smoothed

    def is_path_clear(self, path: List[Tuple[float, float]], clearance: float = 20.0) -> bool:
        """
        Check if path is clear of obstacles

        Args:
            path: List of (x, y) waypoints
            clearance: Minimum clearance from obstacles (cm)

        Returns:
            True if path is clear
        """
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]

            # Check points along the line segment
            steps = int(math.dist(start, end) / clearance)
            for t in range(steps + 1):
                x = start[0] + (end[0] - start[0]) * t / steps
                y = start[1] + (end[1] - start[1]) * t / steps

                grid_x, grid_y = self.world_map.world_to_grid(x, y)
                if self.world_map.grid[grid_y, grid_x] != 0:
                    return False

        return True