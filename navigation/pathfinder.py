import asyncio
import heapq
import math
import time
from heapq import heappush, heappop
from typing import List, Tuple, Optional

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap


class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar
        self.min_turn_radius = picar.get_min_turn_radius()
        self.grid_step = 20  # Use larger grid steps (20cm) for coarser pathfinding

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells using only 4 directions for simpler paths"""
        x, y = node
        neighbors = []

        # Check only cardinal directions (N, E, S, W)
        for dx, dy in [(0, self.grid_step), (self.grid_step, 0),
                       (0, -self.grid_step), (-self.grid_step, 0)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if path to neighbor is clear
                if self.is_path_clear(x, y, new_x, new_y):
                    neighbors.append((new_x, new_y))

        return neighbors

    def is_path_clear(self, x1: int, y1: int, x2: int, y2: int) -> bool:
        """Check if path between two points is clear of obstacles using line sampling"""
        points = self.get_line_points(x1, y1, x2, y2)
        for x, y in points:
            if self.world_map.grid[y, x] != 0:  # Check if point is occupied
                return False
        return True

    def get_line_points(self, x1: int, y1: int, x2: int, y2: int) -> List[Tuple[int, int]]:
        """Get points along line for collision checking"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        n = 1 + dx + dy
        x_inc = 1 if x2 > x1 else -1
        y_inc = 1 if y2 > y1 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        for _ in range(n):
            points.append((x, y))
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

        return points

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path from start to goal using A* with reduced waypoints"""
        # Quick check if direct path is possible
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        goal_grid = self.world_map.world_to_grid(goal[0], goal[1])

        # If direct path is clear, return just the goal point
        if self.is_path_clear(start_grid[0], start_grid[1], goal_grid[0], goal_grid[1]):
            return [start, goal]

        # If direct path not possible, do A* search
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                break

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.heuristic(current, next)

                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal_grid)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        # Reconstruct path
        if goal_grid not in came_from:
            return []  # No path found

        path = []
        current = goal_grid
        while current is not None:
            world_coords = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_coords)
            current = came_from[current]

        path.reverse()
        return self.optimize_path(path)

    def optimize_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Optimize path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path

        # Start with the first point
        optimized = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Look ahead as far as possible while maintaining clear path
            for look_ahead_idx in range(len(path) - 1, current_idx, -1):
                start = path[current_idx]
                end = path[look_ahead_idx]

                # Convert to grid coordinates for collision checking
                start_grid = self.world_map.world_to_grid(start[0], start[1])
                end_grid = self.world_map.world_to_grid(end[0], end[1])

                if self.is_path_clear(start_grid[0], start_grid[1], end_grid[0], end_grid[1]):
                    optimized.append(end)
                    current_idx = look_ahead_idx
                    break

            current_idx += 1

        return optimized