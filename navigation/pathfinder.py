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

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4  # sqrt(2)

        # Grid directions (8-directional movement)
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

        # Movement timing constants
        self.TURN_45_TIME = 1.0  # seconds for 45-degree turn
        self.TURN_90_TIME = 2.0  # seconds for 90-degree turn
        self.MOVEMENT_SPEED = 30
        self.GRID_MOVE_TIME = 1.0  # seconds to move one grid space

        # Obstacle detection
        self.MIN_OBSTACLE_DISTANCE = self.world_map.resolution  # One grid space
        self.SCANNING_ANGLES = range(-60, 61, 5)  # Scanning sweep angles

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic (diagonal distance) between points"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def get_grid_direction(self, current: Tuple[int, int], next: Tuple[int, int]) -> int:
        """Get direction index (0-7) for movement between grid cells"""
        dx = next[0] - current[0]
        dy = next[1] - current[1]

        for i, (dir_x, dir_y) in enumerate(self.DIRECTIONS):
            if (dir_x, dir_y) == (dx, dy):
                return i
        return 0  # Default to North if no match

    def direction_to_angle(self, direction: int) -> float:
        """Convert direction index to angle in degrees"""
        return direction * 45.0

    async def check_for_obstacles(self) -> bool:
        """Check if there are any obstacles within one grid space"""
        # Get multiple ultrasonic readings for reliability
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            await asyncio.sleep(0.01)

        if not distances:
            return False

        avg_distance = sum(distances) / len(distances)
        return avg_distance <= self.MIN_OBSTACLE_DISTANCE

    async def scan_environment(self):
        """Perform a detailed scan of the environment"""
        print("Scanning environment for obstacles...")
        self.px.forward(0)  # Stop movement

        # Save original camera angle
        original_angle = self.px.current_steering_angle

        # Sweep ultrasonic sensor
        for angle in self.SCANNING_ANGLES:
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.05)  # Let servo settle

            # Get multiple readings at each angle
            distances = []
            for _ in range(3):
                dist = self.px.ultrasonic.read()
                if dist and 0 < dist < 300:
                    distances.append(dist)
                await asyncio.sleep(0.01)

            if distances:
                avg_distance = sum(distances) / len(distances)
                # Convert polar coordinates to world coordinates
                angle_rad = math.radians(angle + self.px.heading)
                obs_x = self.px.x + avg_distance * math.cos(angle_rad)
                obs_y = self.px.y + avg_distance * math.sin(angle_rad)

                # Add obstacle to world map
                self.world_map.add_obstacle(
                    x=obs_x,
                    y=obs_y,
                    radius=5.0,  # 5cm radius for point obstacles
                    confidence=0.8,
                    label="ultrasonic_detection"
                )

        # Reset camera angle
        self.px.set_cam_pan_angle(original_angle)

        # Add padding around detected obstacles
        self.world_map.add_padding()
        print("Environment scan complete")

    async def find_path(self, start_x: float, start_y: float,
                        target_x: float, target_y: float) -> Optional[List[Tuple[int, int]]]:
        """Find path using A* algorithm, operating in grid coordinates"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        target_grid = self.world_map.world_to_grid(target_x, target_y)

        # Check if target is reachable (not in obstacle)
        if self.world_map.grid[target_grid[1], target_grid[0]] != 0:
            print("Target location is blocked by obstacle!")
            return None

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == target_grid:
                break

            # Check all 8 directions
            for dx, dy in self.DIRECTIONS:
                next_x = current[0] + dx
                next_y = current[1] + dy
                next_cell = (next_x, next_y)

                # Check bounds
                if (0 <= next_x < self.world_map.grid_size and
                        0 <= next_y < self.world_map.grid_size):

                    # Check if cell is free (not an obstacle)
                    if self.world_map.grid[next_y, next_x] == 0:
                        # Calculate movement cost
                        movement_cost = (self.DIAGONAL_COST if dx != 0 and dy != 0
                                         else self.STRAIGHT_COST)
                        new_cost = cost_so_far[current] + movement_cost

                        if (next_cell not in cost_so_far or
                                new_cost < cost_so_far[next_cell]):
                            cost_so_far[next_cell] = new_cost
                            priority = new_cost + self.heuristic(next_cell, target_grid)
                            heapq.heappush(frontier, (priority, next_cell))
                            came_from[next_cell] = current

        # Reconstruct path
        if target_grid not in came_from:
            print("No path found to target!")
            return None

        path = []
        current = target_grid
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()

        return path

    def get_turn_angle(self, current_heading: float, target_heading: float) -> float:
        """Calculate the smallest turning angle between current and target heading"""
        angle_diff = target_heading - current_heading
        # Normalize to -180 to 180
        angle_diff = (angle_diff + 180) % 360 - 180
        return angle_diff

    async def execute_path(self, path: List[Tuple[int, int]]) -> bool:
        """Execute the path by controlling the Picarx"""
        if not path or len(path) < 2:
            return False

        for i in range(len(path) - 1):
            # Check for obstacles before each movement
            if await self.check_for_obstacles():
                print("Obstacle detected during path execution!")
                self.px.forward(0)  # Stop movement
                return False

            current = path[i]
            next_point = path[i + 1]

            # Get direction and required heading
            direction = self.get_grid_direction(current, next_point)
            target_heading = self.direction_to_angle(direction)

            # Calculate turn angle
            turn_angle = self.get_turn_angle(self.px.heading, target_heading)

            # Execute turn if needed
            if abs(turn_angle) > 5:  # 5-degree threshold
                # Stop before turning
                self.px.forward(0)
                await self.px.turn_to_heading(target_heading)

            # Move forward one grid space
            self.px.forward(self.MOVEMENT_SPEED)
            await asyncio.sleep(self.GRID_MOVE_TIME)

        self.px.forward(0)
        return True  # Path completed successfully