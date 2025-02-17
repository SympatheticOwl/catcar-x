import asyncio
import heapq
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

        # Define movement directions (8-directional movement)
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]


    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic (euclidean distance) between two points"""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        neighbors = []
        for dx, dy in self.directions:
            new_x, new_y = pos[0] + dx, pos[1] + dy

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

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        path_found = False

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                path_found = True
                break

            for next_pos in self.get_neighbors(current):
                # Calculate movement cost (diagonal movements cost more)
                dx, dy = next_pos[0] - current[0], next_pos[1] - current[1]
                movement_cost = math.sqrt(dx * dx + dy * dy)
                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(next_pos, goal_grid)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        if not path_found:
            return []

        # Reconstruct path
        grid_path = []
        current = goal_grid
        while current is not None:
            grid_path.append(current)
            current = came_from[current]
        grid_path.reverse()

        # Convert back to world coordinates
        world_path = [self.world_map.grid_to_world(x, y) for x, y in grid_path]

        # Smooth path by removing unnecessary waypoints
        smoothed_path = self.smooth_path(world_path)

        return smoothed_path


    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Look ahead as far as possible while maintaining clear line of sight
            next_idx = current_idx + 2
            while next_idx < len(path):
                if self.is_clear_path(path[current_idx], path[next_idx]):
                    next_idx += 1
                else:
                    next_idx -= 1
                    break

            current_idx = next_idx - 1
            smoothed.append(path[current_idx])

        return smoothed


    def is_clear_path(self, start: Tuple[float, float],
                      end: Tuple[float, float]) -> bool:
        """Check if there's a clear path between two points"""
        # Convert to grid coordinates
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        end_grid = self.world_map.world_to_grid(end[0], end[1])

        # Use Bresenham's line algorithm to check cells between points
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


    def get_heading_to_point(self, current_x: float, current_y: float,
                             target_x: float, target_y: float) -> float:
        """Calculate heading needed to reach target point"""
        dx = target_x - current_x
        dy = target_y - current_y
        return math.degrees(math.atan2(dy, dx)) % 360