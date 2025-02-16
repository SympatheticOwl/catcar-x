import asyncio
import math
from heapq import heappush, heappop
from typing import List, Tuple, Set, Optional
import numpy as np
from picarx_wrapper import PicarXWrapper
from world_map import WorldMap

class Node:
    def __init__(self, x: int, y: int, g_cost: float = float('inf'),
                 h_cost: float = 0, parent: 'Node' = None, heading: float = None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # Cost from start to this node
        self.h_cost = h_cost  # Heuristic cost to goal
        self.f_cost = g_cost + h_cost
        self.parent = parent
        self.heading = heading  # Current heading in degrees

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

class Pathfinder:
    def __init__(self, world_map: WorldMap, robot: PicarXWrapper):
        self.world_map = world_map
        self.px = robot

        # Navigation parameters
        self.min_turn_radius = robot.get_min_turn_radius()
        self.grid_turn_radius = int(self.min_turn_radius / world_map.resolution)

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4
        self.TURN_COST = 2.0

        # Navigation state
        self.current_path: List[Tuple[float, float]] = []
        self.current_target: Optional[Tuple[float, float]] = None
        self.navigation_active = False
        self.path_recalculation_needed = False

        # Define possible movements (8-directional)
        self.movements = [
            (0, 1), (1, 1), (1, 0), (1, -1),
            (0, -1), (-1, -1), (-1, 0), (-1, 1)
        ]

    def heuristic(self, current: Tuple[int, int], goal: Tuple[int, int]) -> float:
        dx = abs(current[0] - goal[0])
        dy = abs(current[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def is_valid_move(self, x: int, y: int, current_heading: float,
                      next_heading: float) -> bool:
        # Check grid boundaries and obstacles
        if not (0 <= x < self.world_map.grid_size and 0 <= y < self.world_map.grid_size):
            return False
        if self.world_map.grid[y, x] == 1:
            return False

        # Check turning radius constraint
        if current_heading is not None:
            heading_diff = abs(((next_heading - current_heading + 180) % 360) - 180)
            if heading_diff > self.px.MAX_STEERING_ANGLE:
                turn_space = self._check_turn_space(x, y, self.grid_turn_radius)
                if not turn_space:
                    return False
        return True

    def _check_turn_space(self, x: int, y: int, radius: int) -> bool:
        x_min = max(0, x - radius)
        x_max = min(self.world_map.grid_size, x + radius + 1)
        y_min = max(0, y - radius)
        y_max = min(self.world_map.grid_size, y + radius + 1)
        return not np.any(self.world_map.grid[y_min:y_max, x_min:x_max])

    def calculate_path(self, start_x: float, start_y: float,
                       goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        start_node = Node(start_grid[0], start_grid[1], g_cost=0,
                          h_cost=self.heuristic(start_grid, goal_grid),
                          heading=self.px.heading)
        goal_node = Node(goal_grid[0], goal_grid[1])

        open_set: List[Node] = []
        closed_set: Set[Node] = set()
        heappush(open_set, start_node)
        node_dict = {(start_node.x, start_node.y): start_node}

        while open_set:
            current = heappop(open_set)

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                return self._reconstruct_path(current)

            closed_set.add(current)

            for dx, dy in self.movements:
                new_x = current.x + dx
                new_y = current.y + dy
                new_heading = math.degrees(math.atan2(dy, dx)) % 360

                if not self.is_valid_move(new_x, new_y, current.heading, new_heading):
                    continue

                movement_cost = self.DIAGONAL_COST if dx != 0 and dy != 0 else self.STRAIGHT_COST
                if current.heading is not None:
                    heading_diff = abs(((new_heading - current.heading + 180) % 360) - 180)
                    if heading_diff > 45:
                        movement_cost += self.TURN_COST

                g_cost = current.g_cost + movement_cost
                h_cost = self.heuristic((new_x, new_y), (goal_node.x, goal_node.y))
                neighbor = Node(new_x, new_y, g_cost, h_cost, current, new_heading)

                if neighbor in closed_set:
                    continue

                existing = node_dict.get((new_x, new_y))
                if not existing or g_cost < existing.g_cost:
                    node_dict[(new_x, new_y)] = neighbor
                    if not existing:
                        heappush(open_set, neighbor)

        return []  # No path found

    def _reconstruct_path(self, goal_node: Node) -> List[Tuple[float, float]]:
        path = []
        current = goal_node
        while current:
            world_coords = self.world_map.grid_to_world(current.x, current.y)
            path.append(world_coords)
            current = current.parent
        return list(reversed(path))

    def check_path_validity(self) -> bool:
        """Check if current path is still valid given updated obstacle information"""
        if not self.current_path:
            return False

        for x, y in self.current_path:
            grid_x, grid_y = self.world_map.world_to_grid(x, y)
            if self.world_map.grid[grid_y, grid_x] == 1:
                return False
        return True

    async def navigate_to_point(self, target_x: float, target_y: float, speed: int = 30):
        """Main navigation function that handles path planning and execution"""
        self.current_target = (target_x, target_y)
        self.navigation_active = True
        self.path_recalculation_needed = True

        while self.navigation_active:
            current_pos = self.px.get_position()

            # Check if we've reached the target
            distance_to_target = math.sqrt(
                (target_x - current_pos['x']) ** 2 +
                (target_y - current_pos['y']) ** 2
            )
            if distance_to_target < 5:  # 5cm threshold
                self.navigation_active = False
                self.px.stop()
                return True

            # Recalculate path if needed
            if self.path_recalculation_needed or not self.check_path_validity():
                print("Recalculating path...")
                self.current_path = self.calculate_path(
                    current_pos['x'], current_pos['y'],
                    target_x, target_y
                )

                if not self.current_path:
                    print("No valid path found!")
                    self.navigation_active = False
                    self.px.stop()
                    return False

                self.path_recalculation_needed = False

            # Follow the current path
            if self.current_path:
                next_point = self.current_path[0]
                success = await self.px.navigate_to_point(next_point[0], next_point[1], speed)

                if success:
                    self.current_path.pop(0)
                else:
                    self.path_recalculation_needed = True

            await asyncio.sleep(0.1)

        return False

    def request_path_recalculation(self):
        """Request path recalculation on next navigation cycle"""
        self.path_recalculation_needed = True

    def stop_navigation(self):
        """Stop current navigation"""
        self.navigation_active = False
        self.current_path = []
        self.current_target = None
        self.px.stop()