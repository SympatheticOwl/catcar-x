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

        # Directions for 8-directional movement (N, NE, E, SE, S, SW, W, NW)
        self.directions = [
            (0, 1), (1, 1), (1, 0), (1, -1),
            (0, -1), (-1, -1), (-1, 0), (-1, 1)
        ]

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic (diagonal distance) between points"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid positions"""
        neighbors = []
        for dx, dy in self.directions:
            new_x, new_y = pos[0] + dx, pos[1] + dy

            # Check grid bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if position is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))

        return neighbors

    def find_path(self, start_x: float, start_y: float,
                  target_x: float, target_y: float) -> List[Tuple[float, float]]:
        """Find path from start to target using A* algorithm"""
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
                # Calculate movement cost (diagonal moves cost more)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                move_cost = math.sqrt(2) if dx + dy == 2 else 1

                new_cost = cost_so_far[current] + move_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self._heuristic(next_pos, target_grid)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        if target_grid not in came_from:
            return []  # No path found

        path = []
        current = target_grid
        while current is not None:
            # Convert grid coordinates back to world coordinates
            world_x, world_y = self.world_map.grid_to_world(current[0], current[1])
            path.append((world_x, world_y))
            current = came_from[current]

        path.reverse()
        return path


async def navigate_to_target(self, target_x: float, target_y: float):
    """Navigate to target position using A* pathfinding"""
    print(f"Starting navigation to target ({target_x}, {target_y})")

    while True:
        current_pos = self.px.get_position()
        current_x, current_y = current_pos['x'], current_pos['y']

        # Find path to target
        path = self.find_path(current_x, current_y, target_x, target_y)

        if not path:
            print("No valid path found to target")
            return False

        print(f"Found path with {len(path)} waypoints")

        # Follow path
        for waypoint_x, waypoint_y in path:
            # Navigate to each waypoint
            while True:
                current_pos = self.px.get_position()
                distance_to_waypoint = math.sqrt(
                    (waypoint_x - current_pos['x']) ** 2 +
                    (waypoint_y - current_pos['y']) ** 2
                )

                # Check if we've reached the waypoint
                if distance_to_waypoint < 5:  # 5cm threshold
                    break

                # Calculate angle to waypoint
                dx = waypoint_x - current_pos['x']
                dy = waypoint_y - current_pos['y']
                target_heading = math.degrees(math.atan2(dy, dx)) % 360

                # Calculate angle difference
                angle_diff = target_heading - current_pos['heading']
                angle_diff = (angle_diff + 180) % 360 - 180

                # Set steering angle
                steering_angle = self.px.calculate_steering_angle(angle_diff)
                self.px.set_dir_servo_angle(steering_angle)

                # Move forward if ultrasonic reading is clear
                if self.px.current_distance >= self.px.min_distance:
                    self.px.forward(30)  # Standard speed
                else:
                    # Obstacle detected - stop and scan
                    print("Obstacle detected, stopping to scan...")
                    self.px.stop()

                    # Scan environment
                    await self.scan_environment()

                    # Recalculate path from current position
                    break  # Exit waypoint loop to recalculate full path

                await asyncio.sleep(0.1)

            # If we broke out of the waypoint loop due to obstacle, break path loop too
            if self.px.current_distance < self.px.min_distance:
                break

        # Check if we've reached the final target
        current_pos = self.px.get_position()
        distance_to_target = math.sqrt(
            (target_x - current_pos['x']) ** 2 +
            (target_y - current_pos['y']) ** 2
        )

        if distance_to_target < 5:  # 5cm threshold
            print("Reached target position!")
            self.px.stop()
            return True

        # If we're here, we hit an obstacle and need to recalculate path
        await asyncio.sleep(0.1)