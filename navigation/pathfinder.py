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
        self.grid_size = world_map.grid_size
        self.resolution = world_map.resolution

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        x, y = pos
        neighbors = []

        # Check all 8 directions
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if 0 <= new_x < self.grid_size and 0 <= new_y < self.grid_size:
                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))

        return neighbors

    def find_path(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path from start to goal using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_pos[0], start_pos[1])
        goal_grid = self.world_map.world_to_grid(goal_pos[0], goal_pos[1])

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                break

            for next_pos in self.get_neighbors(current):
                # Calculate new cost (diagonal movement costs more)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                movement_cost = 1.4 if dx + dy == 2 else 1.0
                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        if goal_grid not in came_from:
            return []  # No path found

        path = []
        current = goal_grid
        while current is not None:
            # Convert grid coordinates back to world coordinates
            world_x, world_y = self.world_map.grid_to_world(current[0], current[1])
            path.append((world_x, world_y))
            current = came_from[current]

        path.reverse()
        return path

    def is_path_clear(self, path: List[Tuple[float, float]]) -> bool:
        """Check if path is clear of obstacles"""
        for x, y in path:
            grid_x, grid_y = self.world_map.world_to_grid(x, y)
            # Check surrounding cells (1 grid space)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    if (0 <= check_x < self.grid_size and
                            0 <= check_y < self.grid_size and
                            self.world_map.grid[check_y, check_x] != 0):
                        return False
        return True