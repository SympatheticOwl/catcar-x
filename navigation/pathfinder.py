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

        # Verify and adjust grid resolution if needed
        if self.world_map.resolution > 10.0:  # If resolution is too coarse
            print(f"Warning: Grid resolution is too coarse ({self.world_map.resolution}cm)")
            print("Consider initializing WorldMap with resolution=10.0 for better navigation")

        # Movement directions (8-directional)
        self.DIRECTIONS = [
            (0, 1),  # N
            (1, 1),  # NE
            (1, 0),  # E
            (1, -1),  # SE
            (0, -1),  # S
            (-1, -1),  # SW
            (-1, 0),  # W
            (-1, 1),  # NW
        ]

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4  # sqrt(2)
        self.TURN_COST = 1.0  # Cost for changing direction

        self.grid_cell_size = world_map.resolution

    def find_path(self, start_x: float, start_y: float, goal_x: float, goal_y: float) -> List[Tuple[int, int]]:
        """Find path from start to goal using A* algorithm with 8-directional movement"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid, None))  # Include current direction
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}
        direction_so_far = {start_grid: None}

        while frontier:
            current_cost, current, current_dir = heapq.heappop(frontier)

            if current == goal_grid:
                break

            for direction in self.DIRECTIONS:
                next_pos = (current[0] + direction[0], current[1] + direction[1])

                # Skip if out of bounds
                if not (0 <= next_pos[0] < self.world_map.grid_size and
                        0 <= next_pos[1] < self.world_map.grid_size):
                    continue

                # Skip if obstacle
                if self.world_map.grid[next_pos[1], next_pos[0]] != 0:
                    continue

                # Calculate movement cost
                movement_cost = self.DIAGONAL_COST if (direction[0] != 0 and direction[1] != 0) else self.STRAIGHT_COST

                # Add turn cost if changing direction
                if current_dir is not None and direction != current_dir:
                    movement_cost += self.TURN_COST

                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self._heuristic(next_pos, goal_grid)
                    heapq.heappush(frontier, (priority, next_pos, direction))
                    came_from[next_pos] = current
                    direction_so_far[next_pos] = direction

        # Reconstruct path
        path = self._reconstruct_path(came_from, start_grid, goal_grid)
        return self._extract_waypoints(path, direction_so_far)

    def _heuristic(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic distance to goal using octile distance"""
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

    def _extract_waypoints(self, path: List[Tuple[int, int]], directions: dict) -> List[Tuple[int, int]]:
        """Extract key waypoints where direction changes"""
        if len(path) <= 2:
            return path

        waypoints = [path[0]]
        current_direction = None

        for i in range(1, len(path)):
            pos = path[i]
            direction = directions.get(pos)

            if direction != current_direction:
                waypoints.append(path[i - 1])
                current_direction = direction

        if waypoints[-1] != path[-1]:
            waypoints.append(path[-1])

        return waypoints

    def get_direction_to_point(self, current_x: int, current_y: int,
                               target_x: int, target_y: int) -> float:
        """Get heading angle for 8-directional movement"""
        dx = target_x - current_x
        dy = target_y - current_y
        angle = math.degrees(math.atan2(dy, dx))

        # Convert to 8 cardinal/ordinal directions
        normalized_angle = (angle + 360) % 360
        direction_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        closest_angle = min(direction_angles, key=lambda x: abs(x - normalized_angle))

        return closest_angle
