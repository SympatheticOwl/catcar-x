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

        # Directions for 8-directional movement in degrees (N, NE, E, SE, S, SW, W, NW)
        self.directions = [
            (0, 1, 0),  # North: 0 degrees
            (1, 1, 45),  # Northeast: 45 degrees
            (1, 0, 90),  # East: 90 degrees
            (1, -1, 135),  # Southeast: 135 degrees
            (0, -1, 180),  # South: 180 degrees
            (-1, -1, 225),  # Southwest: 225 degrees
            (-1, 0, 270),  # West: 270 degrees
            (-1, 1, 315),  # Northwest: 315 degrees
        ]

        # Turn parameters from PicarXWrapper
        self.TURN_SPEED = 30
        self.DIAGONAL_TURN_TIME = 1.25  # Time for 45-degree turn
        self.FULL_TURN_TIME = 2.3  # Time for 90-degree turn

    async def execute_turn(self, target_heading: float):
        """Execute a turn to reach target heading"""
        current_heading = self.px.heading % 360
        angle_diff = target_heading - current_heading
        # Normalize to -180 to 180
        angle_diff = (angle_diff + 180) % 360 - 180

        # Calculate turn time based on angle
        turn_time = abs(angle_diff / 90.0) * self.FULL_TURN_TIME

        # Set steering direction
        steering_angle = 30 if angle_diff > 0 else -30
        self.px.set_dir_servo_angle(steering_angle)

        # Execute turn
        if angle_diff > 0:
            self.px.forward(self.TURN_SPEED)
        else:
            self.px.backward(self.TURN_SPEED)

        await asyncio.sleep(turn_time)

        # Stop and straighten wheels
        self.px.stop()
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.1)

    async def move_one_grid(self, direction_idx: int):
        """Move one grid space in the specified direction"""
        dx, dy, heading = self.directions[direction_idx]

        # First turn to the correct heading
        await self.execute_turn(heading)

        # Calculate movement time based on grid resolution
        # Distance = grid_resolution, speed = 30 cm/s
        movement_time = self.world_map.resolution / 30.0

        # Move forward one grid space
        self.px.forward(30)
        await asyncio.sleep(movement_time)
        self.px.stop()

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic (diagonal distance) between points"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int, int]]:
        """Get valid neighboring grid positions with direction index"""
        neighbors = []
        for i, (dx, dy, _) in enumerate(self.directions):
            new_x, new_y = pos[0] + dx, pos[1] + dy

            # Check grid bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if position is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y, i))

        return neighbors

    def find_path(self, start_x: float, start_y: float,
                  target_x: float, target_y: float) -> List[Tuple[int, int, int]]:
        """Find path from start to target using A* algorithm, returns grid positions and directions"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        target_grid = self.world_map.world_to_grid(target_x, target_y)

        # Initialize A* data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == target_grid:
                break

            for next_pos in self._get_neighbors(current):
                new_x, new_y, dir_idx = next_pos

                # Calculate movement cost (diagonal moves cost more)
                dx = abs(new_x - current[0])
                dy = abs(new_y - current[1])
                move_cost = math.sqrt(2) if dx + dy == 2 else 1

                new_cost = cost_so_far[current] + move_cost
                next_pos_grid = (new_x, new_y)

                if next_pos_grid not in cost_so_far or new_cost < cost_so_far[next_pos_grid]:
                    cost_so_far[next_pos_grid] = new_cost
                    priority = new_cost + self._heuristic(next_pos_grid, target_grid)
                    heapq.heappush(frontier, (priority, next_pos_grid))
                    came_from[next_pos_grid] = (current, dir_idx)

        # Reconstruct path
        if target_grid not in came_from:
            return []  # No path found

        path = []
        current = target_grid
        while current is not None:
            if current == start_grid:
                break
            prev_pos, dir_idx = came_from[current]
            path.append((current[0], current[1], dir_idx))
            current = prev_pos

        path.reverse()
        return path


async def navigate_to_target(self, target_x: float, target_y: float):
    """Navigate to target position using grid-based movement"""
    print(f"Starting navigation to target ({target_x}, {target_y})")

    while True:
        current_pos = self.px.get_position()

        # Find path to target
        path = self.find_path(current_pos['x'], current_pos['y'], target_x, target_y)

        if not path:
            print("No valid path found to target")
            return False

        print(f"Found path with {len(path)} grid moves")

        # Follow path grid by grid
        for grid_x, grid_y, direction_idx in path:
            # Check for obstacles before each move
            if self.px.current_distance < self.px.min_distance:
                print("Obstacle detected, stopping to scan...")
                self.px.stop()

                # Scan environment and update map
                await self.scan_environment()
                break  # Recalculate path

            # Execute grid movement
            await self.move_one_grid(direction_idx)

            # Verify position after movement
            current_pos = self.px.get_position()
            world_x, world_y = self.world_map.grid_to_world(grid_x, grid_y)
            distance_to_target = math.sqrt(
                (world_x - current_pos['x']) ** 2 +
                (world_y - current_pos['y']) ** 2
            )

            if distance_to_target > self.world_map.resolution:
                print("Position error too large, recalculating path...")
                break

        # Check if we've reached the final target
        current_pos = self.px.get_position()
        distance_to_target = math.sqrt(
            (target_x - current_pos['x']) ** 2 +
            (target_y - current_pos['y']) ** 2
        )

        if distance_to_target < self.world_map.resolution / 2:
            print("Reached target position!")
            return True

        await asyncio.sleep(0.1)
