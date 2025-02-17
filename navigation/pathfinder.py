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
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4  # sqrt(2)
        self.TURN_COST = 0.5  # Additional cost for changing direction

        # Calculate grid cell size based on car's turning radius
        self.min_turn_radius = picar.get_min_turn_radius()
        self.grid_cell_size = world_map.resolution


    def find_path(self, start_x: float, start_y: float, goal_x: float, goal_y: float) -> List[Tuple[int, int]]:
        """Find path from start to goal using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        # Initialize data structures
        frontier = []  # Priority queue
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                break

            # Get valid neighbors considering car's turning radius
            for next_pos in self._get_neighbors(current):
                # Calculate new cost including turning penalties
                new_cost = cost_so_far[current] + self._movement_cost(current, next_pos)

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self._heuristic(next_pos, goal_grid)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        return self._reconstruct_path(came_from, start_grid, goal_grid)


    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        x, y = pos
        neighbors = []

        # Check straight and diagonal moves
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check grid bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if move is valid (no obstacles)
                if self.world_map.grid[new_y, new_x] == 0:
                    # For diagonal moves, check both adjacent cells
                    if abs(dx) == 1 and abs(dy) == 1:
                        if (self.world_map.grid[y, new_x] == 0 and
                                self.world_map.grid[new_y, x] == 0):
                            neighbors.append((new_x, new_y))
                    else:
                        neighbors.append((new_x, new_y))

        return neighbors


    def _movement_cost(self, current: Tuple[int, int], next_pos: Tuple[int, int]) -> float:
        """Calculate movement cost between adjacent cells"""
        dx = abs(next_pos[0] - current[0])
        dy = abs(next_pos[1] - current[1])

        # Diagonal movement
        if dx == 1 and dy == 1:
            return self.DIAGONAL_COST

        # Straight movement
        return self.STRAIGHT_COST


    def _heuristic(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic distance to goal"""
        # Using diagonal distance
        dx = abs(pos[0] - goal[0])
        dy = abs(pos[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


    def _reconstruct_path(self, came_from: dict, start: Tuple[int, int],
                          goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from start to goal"""
        current = goal
        path = []

        while current != start:
            path.append(current)
            if current not in came_from:
                return []  # No path found
            current = came_from[current]

        path.append(start)
        path.reverse()
        return path


    def smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Smooth path to reduce unnecessary turns"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_direction = None

        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            current = path[i]
            next_pos = path[i + 1]

            # Calculate directions
            dx1 = current[0] - prev[0]
            dy1 = current[1] - prev[1]
            dx2 = next_pos[0] - current[0]
            dy2 = next_pos[1] - current[1]

            # If direction changes, keep the point
            if dx1 != dx2 or dy1 != dy2:
                smoothed.append(current)

        smoothed.append(path[-1])
        return smoothed