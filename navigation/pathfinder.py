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
        self.path = []
        self.current_target = None

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        x, y = pos
        neighbors = []

        # Check 8 surrounding cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))

        return neighbors

    def find_path(self, start_x: float, start_y: float,
                  target_x: float, target_y: float) -> List[Tuple[float, float]]:
        """Find path from start to target in world coordinates"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        print(f'start_x: {start_x}, start_y: {start_y}')
        print(f'start_grid: {start_grid}')
        target_grid = self.world_map.world_to_grid(target_x, target_y)
        print(f'target_x: {target_x}, target_y: {target_y}')
        print(f'target_grid: {target_grid}')

        # Initialize data structures
        frontier = []
        heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            print('frontier...')
            current = heappop(frontier)[1]

            if current == target_grid:
                break

            for next_pos in self.get_neighbors(current):
                # Calculate new cost (diagonal movement costs more)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                movement_cost = 1.4 if dx + dy == 2 else 1.0
                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(next_pos, target_grid)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        current = target_grid
        path = []

        while current is not None:
            print(f'current: {current}')
            # Convert grid coordinates back to world coordinates
            world_coords = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_coords)
            current = came_from.get(current)

        path.reverse()
        return path

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path to reduce unnecessary turns"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Look ahead to find furthest point we can reach directly
            for look_ahead in range(len(path) - 1, current_idx, -1):
                start = path[current_idx]
                end = path[look_ahead]

                # Check if direct path is clear
                if self.is_path_clear(start, end):
                    smoothed.append(end)
                    current_idx = look_ahead
                    break
            else:
                # If no clear path found, add next point
                current_idx += 1
                smoothed.append(path[current_idx])

        return smoothed

    def is_path_clear(self, start: Tuple[float, float],
                      end: Tuple[float, float]) -> bool:
        """Check if direct path between points is obstacle-free"""
        # Convert to grid coordinates
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        end_grid = self.world_map.world_to_grid(end[0], end[1])

        # Use Bresenham's line algorithm to check cells along path
        x0, y0 = start_grid
        x1, y1 = end_grid
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0

        if x0 < x1:
            sx = 1
        else:
            sx = -1

        if y0 < y1:
            sy = 1
        else:
            sy = -1

        err = dx - dy

        while True:
            if (x < 0 or x >= self.world_map.grid_size or
                    y < 0 or y >= self.world_map.grid_size or
                    self.world_map.grid[y, x] != 0):
                return False

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err = err - dy
                x = x + sx
            if e2 < dx:
                err = err + dx
                y = y + sy

        return True

    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """Get next waypoint from current path"""
        if not self.path:
            return None
        return self.path[0]

    def update_path(self) -> None:
        """Recalculate path based on current position and obstacles"""
        if not self.current_target:
            return

        pos = self.picar.get_position()
        new_path = self.find_path(pos['x'], pos['y'],
                                  self.current_target[0],
                                  self.current_target[1])
        self.path = self.smooth_path(new_path)