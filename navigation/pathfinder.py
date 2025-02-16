import asyncio
import heapq
import math
import time
from heapq import heappush, heappop
from typing import List, Tuple, Optional

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap


class PathNode:
    def __init__(self, x: int, y: int, g_cost: float = float('inf'),
                 h_cost: float = 0, parent: Optional['PathNode'] = None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # Cost from start to current node
        self.h_cost = h_cost  # Estimated cost from current node to goal
        self.f_cost = g_cost + h_cost
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar
        self.current_path = []
        self.path_index = 0
        self.waiting_for_dynamic_obstacle = False
        self.stop_sign_waiting = False

        # Movement parameters
        self.node_reached_threshold = 5.0  # cm
        self.path_complete_threshold = 10.0  # cm
        self.replanning_distance = 15.0  # cm, distance to trigger replanning

        # Initialize waypoint tracking
        self.current_waypoint = None
        self.target_position = None

    def calculate_path(self, start_x: float, start_y: float,
                       goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Calculate path using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        start_node = PathNode(start_grid[0], start_grid[1], 0,
                              self._heuristic(start_grid, goal_grid))
        goal_node = PathNode(goal_grid[0], goal_grid[1])

        open_set = []
        closed_set = set()
        heapq.heappush(open_set, start_node)
        node_lookup = {(start_node.x, start_node.y): start_node}

        while open_set:
            current = heapq.heappop(open_set)

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                return self._reconstruct_path(current)

            closed_set.add((current.x, current.y))

            for next_pos in self._get_neighbors(current):
                if (next_pos[0], next_pos[1]) in closed_set:
                    continue

                g_cost = current.g_cost + self._distance(
                    (current.x, current.y), next_pos)

                next_node = node_lookup.get((next_pos[0], next_pos[1]))
                if not next_node:
                    next_node = PathNode(next_pos[0], next_pos[1])
                    node_lookup[(next_pos[0], next_pos[1])] = next_node

                if g_cost < next_node.g_cost:
                    next_node.parent = current
                    next_node.g_cost = g_cost
                    next_node.h_cost = self._heuristic(
                        (next_node.x, next_node.y), goal_grid)
                    next_node.f_cost = next_node.g_cost + next_node.h_cost

                    if next_node not in open_set:
                        heapq.heappush(open_set, next_node)

        return []  # No path found

    def _heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate heuristic cost between two positions"""
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    def _distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate actual distance between two positions"""
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    def _get_neighbors(self, node: PathNode) -> List[Tuple[int, int]]:
        """Get valid neighboring positions"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                new_x = node.x + dx
                new_y = node.y + dy

                # Check bounds
                if (0 <= new_x < self.world_map.grid_size and
                        0 <= new_y < self.world_map.grid_size):
                    # Check if position is obstacle-free
                    if self.world_map.grid[new_y, new_x] == 0:
                        neighbors.append((new_x, new_y))

        return neighbors

    def _reconstruct_path(self, end_node: PathNode) -> List[Tuple[float, float]]:
        """Reconstruct path from end node to start node"""
        path = []
        current = end_node

        while current:
            # Convert grid coordinates back to world coordinates
            world_pos = self.world_map.grid_to_world(current.x, current.y)
            path.append(world_pos)
            current = current.parent

        return list(reversed(path))

    async def navigate_to_target(self, target_x: float, target_y: float):
        """Navigate to target position while avoiding obstacles"""
        self.target_position = (target_x, target_y)
        self.current_path = []
        self.path_index = 0
        self.waiting_for_dynamic_obstacle = False
        self.stop_sign_waiting = False

        while True:
            current_pos = self.picar.get_position()
            distance_to_target = math.sqrt(
                (target_x - current_pos['x']) ** 2 +
                (target_y - current_pos['y']) ** 2)

            # Check if we've reached the target
            if distance_to_target < self.path_complete_threshold:
                print("Target reached!")
                self.picar.stop()
                return True

            # Handle dynamic obstacles (people, cats)
            if self.waiting_for_dynamic_obstacle:
                print("Waiting for dynamic obstacle to clear...")
                self.picar.stop()
                await asyncio.sleep(0.5)
                continue

            # Handle stop signs
            if self.stop_sign_waiting:
                print("Stopping at stop sign...")
                self.picar.stop()
                await asyncio.sleep(3.0)
                self.stop_sign_waiting = False
                continue

            # Check if we need to calculate or recalculate path
            if (not self.current_path or
                    self._should_replan(current_pos['x'], current_pos['y'])):
                # Scan environment before planning
                await self.scan_environment()

                # Calculate new path
                self.current_path = self.calculate_path(
                    current_pos['x'], current_pos['y'], target_x, target_y)
                self.path_index = 0

                if not self.current_path:
                    print("No valid path found!")
                    self.picar.stop()
                    return False

            # Navigate current path
            await self._follow_path()

            await asyncio.sleep(0.1)

    def _should_replan(self, current_x: float, current_y: float) -> bool:
        """Determine if replanning is needed"""
        if not self.current_path:
            return True

        if self.path_index >= len(self.current_path):
            return True

        # Check distance to current waypoint
        waypoint = self.current_path[self.path_index]
        distance = math.sqrt(
            (waypoint[0] - current_x) ** 2 + (waypoint[1] - current_y) ** 2)

        # Replan if we're too far from expected position
        return distance > self.replanning_distance

    async def _follow_path(self):
        """Follow the current path"""
        if not self.current_path or self.path_index >= len(self.current_path):
            return

        current_pos = self.picar.get_position()
        waypoint = self.current_path[self.path_index]

        # Calculate distance and angle to waypoint
        dx = waypoint[0] - current_pos['x']
        dy = waypoint[1] - current_pos['y']
        distance = math.sqrt(dx ** 2 + dy ** 2)
        target_angle = math.degrees(math.atan2(dy, dx))

        # If we've reached current waypoint, move to next
        if distance < self.node_reached_threshold:
            self.path_index += 1
            return

        # Calculate steering angle
        angle_diff = target_angle - current_pos['heading']
        angle_diff = (angle_diff + 180) % 360 - 180  # Normalize to [-180, 180]

        steering_angle = self.picar.calculate_steering_angle(angle_diff)
        self.picar.set_dir_servo_angle(steering_angle)

        # Adjust speed based on turn sharpness
        speed = 30 * (1 - abs(steering_angle) / (2 * self.picar.MAX_STEERING_ANGLE))
        self.picar.forward(speed)

    def handle_vision_detection(self, detected_objects: List[dict]):
        """Handle detected objects from vision system"""
        for obj in detected_objects:
            if obj['label'] in ['person', 'cat']:
                self.waiting_for_dynamic_obstacle = True
                return
            elif obj['label'] == 'stop sign':
                self.stop_sign_waiting = True
                return

        self.waiting_for_dynamic_obstacle = False

    async def scan_environment(self):
        """Scan environment for obstacles"""
        scan_range = (-60, 60)
        scan_step = 5

        for angle in range(scan_range[0], scan_range[1] + 1, scan_step):
            self.picar.set_cam_pan_angle(angle)
            await asyncio.sleep(0.05)