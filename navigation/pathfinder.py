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
    def __init__(self, world_map, picar):
        self.world_map = world_map
        self.picar = picar

        # Pathfinding parameters
        self.directions = [
            (0, 1),  # north
            (1, 1),  # northeast
            (1, 0),  # east
            (1, -1),  # southeast
            (0, -1),  # south
            (-1, -1),  # southwest
            (-1, 0),  # west
            (-1, 1)  # northwest
        ]

        # Cost for diagonal vs straight movement
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4

        # Minimum turning radius considerations
        self.MIN_TURN_RADIUS = self.picar.get_min_turn_radius()
        self.GRID_CELL_SIZE = self.world_map.resolution
        self.MIN_TURN_RADIUS_CELLS = int(self.MIN_TURN_RADIUS / self.GRID_CELL_SIZE)

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic distance between two points"""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_movement_cost(self, current: Tuple[int, int], next_pos: Tuple[int, int],
                          came_from: Dict[Tuple[int, int], Tuple[int, int]]) -> float:
        """Calculate movement cost considering turning radius"""
        # Base cost (straight or diagonal)
        dx = abs(next_pos[0] - current[0])
        dy = abs(next_pos[1] - current[1])
        base_cost = self.DIAGONAL_COST if dx + dy == 2 else self.STRAIGHT_COST

        # Check if we need to consider turning radius
        if current in came_from and came_from[current] is not None:
            prev = came_from[current]
            # Calculate angles
            try:
                prev_angle = math.atan2(current[1] - prev[1], current[0] - prev[0])
                next_angle = math.atan2(next_pos[1] - current[1], next_pos[0] - current[0])
                angle_diff = abs(math.degrees(next_angle - prev_angle))

                # Add turning penalty if turn is too sharp
                if angle_diff > 45:  # Sharp turn threshold
                    required_radius = self.MIN_TURN_RADIUS_CELLS
                    actual_radius = math.sqrt((next_pos[0] - current[0]) ** 2 +
                                              (next_pos[1] - current[1]) ** 2)
                    if actual_radius < required_radius:
                        base_cost *= 2.0  # Penalty for sharp turns
            except (TypeError, ValueError):
                # If there's any issue calculating angles, just return base cost
                pass

        return base_cost

    def is_valid_position(self, pos: Tuple[int, int]) -> bool:
        """Check if a position is valid (within bounds and not obstacle)"""
        x, y = pos
        if 0 <= x < self.world_map.grid_size and 0 <= y < self.world_map.grid_size:
            return self.world_map.grid[y, x] == 0
        return False

    def find_path(self, start_world: Tuple[float, float],
                  goal_world: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path from start to goal in world coordinates"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_world[0], start_world[1])
        goal_grid = self.world_map.world_to_grid(goal_world[0], goal_world[1])

        # Initialize data structures
        frontier = []
        heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        found_path = False
        while frontier:
            current = heappop(frontier)[1]

            if current == goal_grid:
                found_path = True
                break

            # Check all neighboring cells
            for dx, dy in self.directions:
                next_pos = (current[0] + dx, current[1] + dy)

                if not self.is_valid_position(next_pos):
                    continue

                # Calculate new cost
                new_cost = cost_so_far[current] + self.get_movement_cost(current, next_pos, came_from)

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_pos)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        if not found_path:
            return []

        # Reconstruct path
        path_grid = []
        current = goal_grid
        while current is not None:
            path_grid.append(current)
            current = came_from[current]
        path_grid.reverse()

        # Convert back to world coordinates
        path_world = [self.world_map.grid_to_world(x, y) for x, y in path_grid]

        # Smooth path
        return self.smooth_path(path_world)

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth the path to make it more drivable"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Look ahead to find furthest point we can directly reach
            for lookahead in range(len(path) - 1, current_idx, -1):
                # Check if direct path to lookahead point is clear
                start = path[current_idx]
                end = path[lookahead]

                if self.is_path_clear(start, end):
                    smoothed.append(end)
                    current_idx = lookahead
                    break
            else:
                # If no clear path found, add next point
                current_idx += 1
                smoothed.append(path[current_idx])

        return smoothed

    def is_path_clear(self, start: Tuple[float, float],
                      end: Tuple[float, float]) -> bool:
        """Check if direct path between points is clear of obstacles"""
        # Convert to grid coordinates
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        end_grid = self.world_map.world_to_grid(end[0], end[1])

        # Get points along line
        points = self.get_line_points(start_grid, end_grid)

        # Check each point
        return all(self.is_valid_position(point) for point in points)

    def get_line_points(self, start: Tuple[int, int],
                        end: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get all grid points along a line using Bresenham's algorithm"""
        x1, y1 = start
        x2, y2 = end
        points = []

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1

        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        points.append((x, y))
        return points