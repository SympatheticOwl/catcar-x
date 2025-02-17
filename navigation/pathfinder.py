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
        self.px = picarx

        # Grid movement directions (8-directional)
        self.DIRECTIONS = [
            (0, 1),  # N
            (1, 1),  # NE
            (1, 0),  # E
            (1, -1),  # SE
            (0, -1),  # S
            (-1, -1),  # SW
            (-1, 0),  # W
            (-1, 1)  # NW
        ]

        # Movement parameters (adjusted for grid resolution)
        self.BASE_SPEED = 30
        self.GRID_CELL_SIZE = self.world_map.resolution  # cm per grid cell

        # Calculate turn times based on grid resolution
        # For 45° turns at resolution 40cm
        self.TURN_TIME_45 = 1.0
        self.TURN_TIME_90 = 2.0

        print(f"Initialized Pathfinder with grid size: {self.world_map.grid_size}x{self.world_map.grid_size}")
        print(f"Grid resolution: {self.GRID_CELL_SIZE}cm per cell")


    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate octile distance heuristic for grid-based movement"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        # Use grid-aligned movement costs
        straight = abs(dx - dy)
        diagonal = min(dx, dy)
        return straight + (math.sqrt(2) - 1) * diagonal


    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        neighbors = []
        x, y = pos

        for dx, dy in self.DIRECTIONS:
            new_x = x + dx
            new_y = y + dy

            # Check grid bounds using world_map's grid size
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell and surrounding cells are obstacle-free
                # Add a small buffer for safety
                is_safe = True
                for bx in range(-1, 2):
                    for by in range(-1, 2):
                        check_x = new_x + bx
                        check_y = new_y + by
                        if (0 <= check_x < self.world_map.grid_size and
                                0 <= check_y < self.world_map.grid_size):
                            if self.world_map.grid[check_y, check_x] == 1:
                                is_safe = False
                                break
                    if not is_safe:
                        break

                if is_safe:
                    neighbors.append((new_x, new_y))

        return neighbors


    def find_path(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float]) -> List[Tuple[int, int]]:
        """Find path using A* algorithm, working in grid coordinates"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_pos[0], start_pos[1])
        goal_grid = self.world_map.world_to_grid(goal_pos[0], goal_pos[1])

        print(f"Finding path from grid cell {start_grid} to {goal_grid}")

        # Initialize A* structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                break

            for next_pos in self.get_neighbors(current):
                # Calculate movement cost (diagonal movement costs more)
                movement_cost = (math.sqrt(2) if abs(next_pos[0] - current[0]) +
                                                 abs(next_pos[1] - current[1]) == 2 else 1)

                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        path = []
        current = goal_grid
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()

        if not path or path[0] != start_grid:
            print("No valid path found!")
            return []

        print(f"Found path with {len(path)} waypoints")
        return path


    def get_turn_angle(self, current_pos: Tuple[int, int], next_pos: Tuple[int, int]) -> float:
        """Calculate required turn angle between grid positions"""
        dx = (next_pos[0] - current_pos[0]) * self.GRID_CELL_SIZE
        dy = (next_pos[1] - current_pos[1]) * self.GRID_CELL_SIZE
        target_angle = math.degrees(math.atan2(dy, dx))

        # Convert to vehicle heading (0 degrees is forward/north)
        target_angle = 90 - target_angle

        # Normalize angle difference to [-180, 180]
        angle_diff = target_angle - self.px.heading
        angle_diff = (angle_diff + 180) % 360 - 180

        return angle_diff


    async def execute_path(self, path: List[Tuple[int, int]]) -> bool:
        """Execute path movement, handling turns and forward movement"""
        if len(path) < 2:
            return False

        for i in range(len(path) - 1):
            current_pos = path[i]
            next_pos = path[i + 1]

            # Calculate turn angle
            angle_diff = self.get_turn_angle(current_pos, next_pos)

            # Execute turn if needed
            if abs(angle_diff) > 5:  # 5 degree threshold
                # Calculate turn parameters based on grid resolution
                turn_angle = 30 if angle_diff > 0 else -30

                # Use 45° or 90° turn time based on angle
                turn_time = (self.TURN_TIME_90 if abs(angle_diff) > 45
                             else self.TURN_TIME_45)

                print(f"Turning {angle_diff:.1f}° for {turn_time:.1f}s")

                # Execute turn
                self.px.set_dir_servo_angle(turn_angle)
                self.px.forward(self.BASE_SPEED)
                await asyncio.sleep(turn_time)

                # Straighten wheels
                self.px.set_dir_servo_angle(0)

            # Move forward to next position
            self.px.forward(self.BASE_SPEED)

            # Calculate movement time based on grid cell size
            movement_time = self.GRID_CELL_SIZE / (self.BASE_SPEED * 2)  # Rough estimate
            print(f"Moving forward for {movement_time:.1f}s")

            # Wait for movement to complete
            await asyncio.sleep(movement_time)

            # Check for obstacles
            if self.check_obstacles():
                print("Obstacle detected during path execution!")
                self.px.stop()
                return False

        self.px.stop()
        return True


    def check_obstacles(self) -> bool:
        """Check if obstacles are detected in the immediate vicinity"""
        current_grid = self.world_map.world_to_grid(self.px.x, self.px.y)

        # Check surrounding cells
        for dx, dy in self.DIRECTIONS:
            check_x = current_grid[0] + dx
            check_y = current_grid[1] + dy

            if (0 <= check_x < self.world_map.grid_size and
                    0 <= check_y < self.world_map.grid_size and
                    self.world_map.grid[check_y, check_x] == 1):
                return True

        return False