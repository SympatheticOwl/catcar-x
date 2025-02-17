import heapq
import math
import time

import numpy as np
from typing import List, Tuple, Dict, Set
import asyncio


class Pathfinder:
    def __init__(self, world_map, picar):
        self.world_map = world_map
        self.picar = picar
        self.path = []
        self.current_path_index = 0

        # A* parameters
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
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

            for next_pos in self.get_neighbors(current):
                # Calculate movement cost (diagonal movements cost more)
                dx, dy = next_pos[0] - current[0], next_pos[1] - current[1]
                movement_cost = math.sqrt(dx * dx + dy * dy)
                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        current = goal_grid
        path = []

        while current is not None:
            # Convert grid coordinates back to world coordinates
            world_coords = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_coords)
            current = came_from.get(current)

        path.reverse()
        return path

    def get_steering_angle(self, current_pos: Dict, target_pos: Tuple[float, float]) -> float:
        """Calculate steering angle to next waypoint"""
        dx = target_pos[0] - current_pos['x']
        dy = target_pos[1] - current_pos['y']

        # Calculate target angle in degrees
        target_angle = math.degrees(math.atan2(dy, dx))

        # Calculate angle difference
        angle_diff = target_angle - current_pos['heading']
        # Normalize to -180 to 180
        angle_diff = (angle_diff + 180) % 360 - 180

        # Convert to steering angle
        steering_angle = (angle_diff / 45.0) * 30  # 30 is max steering angle
        return max(-30, min(30, steering_angle))

    async def follow_path(self, path: List[Tuple[float, float]],
                          emergency_stop_flag: bool,
                          current_maneuver: asyncio.Task) -> bool:
        """Follow calculated path while avoiding obstacles"""
        if not path:
            return False

        self.path = path
        self.current_path_index = 0

        while self.current_path_index < len(self.path):
            # Check for emergency stop or ongoing maneuver
            if emergency_stop_flag or current_maneuver:
                return False

            current_pos = self.picar.get_position()
            target = self.path[self.current_path_index]

            # Calculate distance to current waypoint
            dx = target[0] - current_pos['x']
            dy = target[1] - current_pos['y']
            distance = math.sqrt(dx * dx + dy * dy)

            # If close enough to waypoint, move to next one
            if distance < 5:  # 5cm threshold
                self.current_path_index += 1
                continue

            # Calculate and set steering angle
            steering_angle = self.get_steering_angle(current_pos, target)
            self.picar.set_dir_servo_angle(steering_angle)

            # Adjust speed based on turn sharpness
            speed = 30 * (1 - abs(steering_angle) / 60)  # Reduce speed in turns
            self.picar.forward(speed)

            await asyncio.sleep(0.1)

        self.picar.stop()
        return True

    async def navigate_to_goal(self, goal_x: float, goal_y: float,
                               emergency_stop_flag: bool,
                               current_maneuver: asyncio.Task,
                               replanning_interval: float = 2.0) -> bool:
        """Navigate to goal with periodic replanning"""
        last_planning_time = 0

        while True:
            current_time = time.time()
            current_pos = self.picar.get_position()

            # Check if we've reached the goal
            dx = goal_x - current_pos['x']
            dy = goal_y - current_pos['y']
            if math.sqrt(dx * dx + dy * dy) < 5:
                self.picar.stop()
                return True

            # Replan path periodically or if no current path
            if (current_time - last_planning_time > replanning_interval or
                    not self.path):
                self.path = self.find_path(
                    current_pos['x'], current_pos['y'],
                    goal_x, goal_y
                )
                last_planning_time = current_time
                self.current_path_index = 0

                if not self.path:
                    print("No valid path found!")
                    self.picar.stop()
                    return False

            # Follow current path
            path_complete = await self.follow_path(
                self.path[self.current_path_index:],
                emergency_stop_flag,
                current_maneuver
            )

            if not path_complete:
                # Path following was interrupted
                await asyncio.sleep(0.5)  # Wait for obstacle avoidance to complete
                continue

            await asyncio.sleep(0.1)