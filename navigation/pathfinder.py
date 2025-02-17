import heapq
import math
import numpy as np
from typing import List, Tuple, Dict, Set
import asyncio


class Pathfinder:
    def __init__(self, world_map, picar):
        self.world_map = world_map
        self.picar = picar

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4  # sqrt(2)

        # Define possible movements (8 directions)
        # (dx, dy, cost)
        self.MOVEMENTS = [
            (0, 1, self.STRAIGHT_COST),  # N
            (1, 1, self.DIAGONAL_COST),  # NE
            (1, 0, self.STRAIGHT_COST),  # E
            (1, -1, self.DIAGONAL_COST),  # SE
            (0, -1, self.STRAIGHT_COST),  # S
            (-1, -1, self.DIAGONAL_COST),  # SW
            (-1, 0, self.STRAIGHT_COST),  # W
            (-1, 1, self.DIAGONAL_COST)  # NW
        ]

        # Navigation parameters
        self.path = []
        self.current_path_index = 0
        self.is_navigating = False
        self.target_position = None
        self.repath_threshold = 5  # Number of grid cells before forcing a new path calculation

        # Movement control
        self.turn_threshold = 5  # Degrees within which we consider heading aligned
        self.base_speed = 30
        self.turning_speed = 20

    def heuristic(self, start: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic cost between two points using diagonal distance"""
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        return self.STRAIGHT_COST * (dx + dy) + (self.DIAGONAL_COST - 2 * self.STRAIGHT_COST) * min(dx, dy)

    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """A* pathfinding implementation"""
        if not (0 <= goal[0] < self.world_map.grid_size and 0 <= goal[1] < self.world_map.grid_size):
            print("Goal outside grid bounds")
            return []

        if self.world_map.grid[goal[1]][goal[0]] == 1:
            print("Goal in obstacle")
            return []  # Goal is in obstacle

        # Priority queue for open set
        open_set = []
        heapq.heappush(open_set, (0, start))

        # Tracking dictionaries
        came_from = {start: None}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        closed_set = set()

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                # Reconstruct path
                path = []
                while current:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            closed_set.add(current)

            # Check all possible movements
            for dx, dy, cost in self.MOVEMENTS:
                neighbor = (current[0] + dx, current[1] + dy)

                # Skip if out of bounds
                if not (0 <= neighbor[0] < self.world_map.grid_size and
                        0 <= neighbor[1] < self.world_map.grid_size):
                    continue

                # Skip if in closed set or is obstacle
                if (neighbor in closed_set or
                        self.world_map.grid[neighbor[1]][neighbor[0]] == 1):
                    continue

                # Calculate new cost
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print("No path found")
        return []  # No path found

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-180, 180] range"""
        angle = angle % 360
        if angle > 180:
            angle -= 360
        return angle

    def get_movement_direction(self, current_pos: Tuple[int, int],
                               next_pos: Tuple[int, int]) -> Tuple[float, float]:
        """Calculate movement direction and steering angle for next waypoint"""
        # Get current position in world coordinates
        current_world_x, current_world_y = self.world_map.grid_to_world(*current_pos)
        next_world_x, next_world_y = self.world_map.grid_to_world(*next_pos)

        # Calculate relative movement vector
        dx = next_world_x - current_world_x
        dy = next_world_y - current_world_y

        # Calculate target heading (in degrees)
        target_heading = math.degrees(math.atan2(dy, dx))

        # Get current car heading
        current_heading = self.picar.heading

        # Calculate steering angle (difference between target and current heading)
        steering_angle = self.normalize_angle(target_heading - current_heading)

        # Clamp steering angle to picar limits
        steering_angle = max(-30, min(30, steering_angle))

        # Calculate movement distance
        distance = math.sqrt(dx ** 2 + dy ** 2)

        print(f"Current heading: {current_heading:.1f}°")
        print(f"Target heading: {target_heading:.1f}°")
        print(f"Steering angle: {steering_angle:.1f}°")
        print(f"Distance: {distance:.1f}cm")

        return steering_angle, distance

    async def turn_to_heading(self, target_heading: float) -> None:
        """Turn the car to face a specific heading"""
        current_heading = self.picar.heading
        angle_diff = self.normalize_angle(target_heading - current_heading)

        # If already aligned, return
        if abs(angle_diff) <= self.turn_threshold:
            return

        print(f"Turning from {current_heading:.1f}° to {target_heading:.1f}°")

        # Set steering angle
        steering_angle = max(-30, min(30, angle_diff))
        self.picar.set_dir_servo_angle(steering_angle)

        # Calculate turn time based on angle difference
        turn_time = abs(angle_diff) / 45.0  # Assuming 45 degrees per second

        # Turn in place
        if angle_diff > 0:
            self.picar.forward(self.turning_speed)
        else:
            self.picar.backward(self.turning_speed)

        await asyncio.sleep(turn_time)
        self.picar.stop()
        self.picar.set_dir_servo_angle(0)
        await asyncio.sleep(0.1)  # Small pause after turning

    async def navigate_to_target(self, target_x: float, target_y: float):
        """Navigate to target position while continuously updating path"""
        self.is_navigating = True
        self.target_position = (target_x, target_y)

        print(f"Starting navigation to ({target_x}, {target_y})")

        while self.is_navigating:
            # Get current position
            current_pos = self.picar.get_position()
            current_grid_x, current_grid_y = self.world_map.world_to_grid(
                current_pos['x'], current_pos['y'])

            # Convert target to grid coordinates
            target_grid_x, target_grid_y = self.world_map.world_to_grid(target_x, target_y)

            print(f"\nCurrent position: ({current_pos['x']:.1f}, {current_pos['y']:.1f})")
            print(f"Current grid: ({current_grid_x}, {current_grid_y})")
            print(f"Target grid: ({target_grid_x}, {target_grid_y})")

            # Check if we've reached the target
            if (abs(current_pos['x'] - target_x) < self.world_map.resolution and
                    abs(current_pos['y'] - target_y) < self.world_map.resolution):
                print("Target reached!")
                self.is_navigating = False
                self.picar.stop()
                break

            # Calculate new path if needed
            if not self.path or self.current_path_index >= len(self.path) - 1:
                self.path = self.find_path(
                    (current_grid_x, current_grid_y),
                    (target_grid_x, target_grid_y)
                )
                self.current_path_index = 0

                if not self.path:
                    print("No path found to target!")
                    self.is_navigating = False
                    self.picar.stop()
                    break

                print(f"New path calculated: {self.path}")

            # Get next waypoint
            next_waypoint = self.path[self.current_path_index + 1]
            print(f"Next waypoint: {next_waypoint}")

            # Calculate steering angle and distance
            steering_angle, distance = self.get_movement_direction(
                (current_grid_x, current_grid_y),
                next_waypoint
            )

            # Turn to face the correct direction
            target_heading = self.picar.heading + steering_angle
            await self.turn_to_heading(target_heading)

            # Move forward one grid cell
            print("Moving forward...")
            self.picar.forward(self.base_speed)

            # Wait for movement to complete
            movement_time = distance / (self.base_speed * self.picar.WHEEL_CIRCUMFERENCE / 60)
            await asyncio.sleep(movement_time)

            self.picar.stop()
            self.current_path_index += 1

            # Brief pause between movements
            await asyncio.sleep(0.1)

    def stop_navigation(self):
        """Stop current navigation task"""
        self.is_navigating = False
        self.path = []
        self.current_path_index = 0
        self.picar.stop()
        self.picar.set_dir_servo_angle(0)