import heapq
import math
import time

import numpy as np
from typing import List, Tuple, Dict, Set
import asyncio


class Pathfinder:
    def __init__(self, world_map, picar):
        self.world_map = world_map
        self.picar = picar
        self.current_path = []

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4  # sqrt(2)
        self.OBSTACLE_COST = float('inf')

        # Neighboring cells (8-directional movement)
        self.NEIGHBORS = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]

    def heuristic(self, start: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic cost using diagonal distance"""
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def get_valid_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells that aren't obstacles"""
        neighbors = []
        x, y = node

        for dx, dy in self.NEIGHBORS:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    # Check if diagonal move is valid (both adjacent cells must be free)
                    if dx != 0 and dy != 0:
                        if (self.world_map.grid[y, new_x] == 0 and
                                self.world_map.grid[new_y, x] == 0):
                            neighbors.append((new_x, new_y))
                    else:
                        neighbors.append((new_x, new_y))

        return neighbors

    def movement_cost(self, current: Tuple[int, int], next: Tuple[int, int]) -> float:
        """Calculate cost of movement between adjacent cells"""
        dx = abs(current[0] - next[0])
        dy = abs(current[1] - next[1])

        # Diagonal movement
        if dx == 1 and dy == 1:
            return self.DIAGONAL_COST
        # Straight movement
        return self.STRAIGHT_COST

    def find_path(self, start_x: float, start_y: float,
                        goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
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
            _, current = heapq.heappop(frontier)

            if current == goal_grid:
                break

            for next_node in self.get_valid_neighbors(current):
                new_cost = cost_so_far[current] + self.movement_cost(current, next_node)

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal_grid)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        # Reconstruct path
        path_grid = []
        current = goal_grid

        if current not in came_from:  # No path found
            return []

        while current is not None:
            path_grid.append(current)
            current = came_from[current]

        path_grid.reverse()

        # Convert grid coordinates back to world coordinates
        path_world = []
        for grid_x, grid_y in path_grid:
            world_x, world_y = self.world_map.grid_to_world(grid_x, grid_y)
            path_world.append((world_x, world_y))

        self.current_path = path_world
        return path_world

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth the path to make it more suitable for car movement"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            current = path[i]

            # Look ahead to find longest straight-line segment possible
            for j in range(len(path) - 1, i, -1):
                if self.is_clear_path(current, path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            i += 1

        return smoothed

    def is_clear_path(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """Check if there's a clear straight-line path between two points"""
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        end_grid = self.world_map.world_to_grid(end[0], end[1])

        # Use Bresenham's line algorithm to check cells along the path
        x0, y0 = start_grid
        x1, y1 = end_grid
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if x == x1 and y == y1:
                break

            if self.world_map.grid[y, x] != 0:
                return False

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True

    def update_path(self, current_x: float, current_y: float,
                    goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Update path based on current position and world state"""
        new_path = self.find_path(current_x, current_y, goal_x, goal_y)
        if new_path:
            new_path = self.smooth_path(new_path)
        return new_path