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
        self.px = picarx
        self.min_turn_radius = self.px.get_min_turn_radius()

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        x, y = node
        neighbors = []

        # Check 8 surrounding cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check if within grid bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    # Check turning radius constraint
                    if self.check_turn_feasible(node, (new_x, new_y)):
                        neighbors.append((new_x, new_y))

        return neighbors

    def check_turn_feasible(self, current: Tuple[int, int], next_node: Tuple[int, int]) -> bool:
        """Check if turn is feasible given vehicle turning radius"""
        # Convert grid coordinates to world coordinates
        curr_x, curr_y = self.world_map.grid_to_world(current[0], current[1])
        next_x, next_y = self.world_map.grid_to_world(next_node[0], next_node[1])

        # Get current heading
        curr_heading = math.radians(self.px.heading)

        # Calculate angle to next point
        dx = next_x - curr_x
        dy = next_y - curr_y
        target_heading = math.atan2(dy, dx)

        # Calculate turn angle
        turn_angle = abs(target_heading - curr_heading)
        turn_angle = min(turn_angle, 2 * math.pi - turn_angle)

        # Calculate required turn radius
        distance = math.sqrt(dx * dx + dy * dy)
        if turn_angle > 0:
            required_radius = distance / (2 * math.sin(turn_angle / 2))
            return required_radius >= self.min_turn_radius

        return True

    def find_path(self, start_x: float, start_y: float,
                  goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Find path from start to goal using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                break

            for next_node in self.get_neighbors(current):
                # Calculate movement cost (diagonal moves cost more)
                dx = abs(next_node[0] - current[0])
                dy = abs(next_node[1] - current[1])
                move_cost = 1.4 if dx + dy == 2 else 1.0

                new_cost = cost_so_far[current] + move_cost

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        # Reconstruct path
        if goal_grid not in came_from:
            return []  # No path found

        path = []
        current = goal_grid
        while current is not None:
            # Convert back to world coordinates
            world_x, world_y = self.world_map.grid_to_world(current[0], current[1])
            path.append((world_x, world_y))
            current = came_from.get(current)

        path.reverse()
        return path

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path using simple path smoothing"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Look ahead to find furthest visible point
            for look_ahead in range(len(path) - 1, current_idx, -1):
                if self.is_path_clear(path[current_idx], path[look_ahead]):
                    smoothed.append(path[look_ahead])
                    current_idx = look_ahead
                    break
            current_idx += 1

        return smoothed

    def is_path_clear(self, start: Tuple[float, float],
                      end: Tuple[float, float]) -> bool:
        """Check if direct path between points is clear of obstacles"""
        # Convert to grid coordinates
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        end_grid = self.world_map.world_to_grid(end[0], end[1])

        # Use Bresenham's line algorithm to check cells along path
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