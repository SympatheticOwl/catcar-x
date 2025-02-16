import asyncio
import heapq
import math
import time
from heapq import heappush, heappop
from typing import List, Tuple, Optional

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap


class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar
        self.min_turn_radius = picar.get_min_turn_radius()

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
        x, y = node
        neighbors = []

        # Check 8 surrounding cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    # Check turning radius constraint
                    if self.check_turn_feasible(node, (new_x, new_y)):
                        neighbors.append((new_x, new_y))

        return neighbors

    def check_turn_feasible(self, current: Tuple[int, int], next: Tuple[int, int]) -> bool:
        """Check if turn is feasible given car's minimum turning radius"""
        # Convert grid coordinates to world coordinates
        curr_world = self.world_map.grid_to_world(current[0], current[1])
        next_world = self.world_map.grid_to_world(next[0], next[1])

        # Get current heading
        current_heading = self.picar.heading

        # Calculate required heading change
        dx = next_world[0] - curr_world[0]
        dy = next_world[1] - curr_world[1]
        target_heading = math.degrees(math.atan2(dy, dx))

        # Calculate angle difference
        angle_diff = abs((target_heading - current_heading + 180) % 360 - 180)

        # Calculate distance between points
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Check if turn is possible given minimum turning radius
        if angle_diff > 0:
            required_radius = distance / (2 * math.sin(math.radians(angle_diff / 2)))
            return required_radius >= self.min_turn_radius

        return True

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path from start to goal using A*"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        goal_grid = self.world_map.world_to_grid(goal[0], goal[1])

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                break

            for next in self.get_neighbors(current):
                # Calculate movement cost (diagonal movement costs more)
                dx = abs(next[0] - current[0])
                dy = abs(next[1] - current[1])
                movement_cost = 1.4 if dx + dy == 2 else 1.0

                new_cost = cost_so_far[current] + movement_cost

                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal_grid)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        # Reconstruct path
        if goal_grid not in came_from:
            return []  # No path found

        path = []
        current = goal_grid
        while current is not None:
            # Convert back to world coordinates
            world_coords = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_coords)
            current = came_from[current]

        path.reverse()
        return path

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path to make it more drivable"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]

        # Use path smoothing to reduce unnecessary turns
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next = path[i + 1]

            # Calculate angles
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next[1] - curr[1], next[0] - curr[0])

            # If turn is less than 45 degrees, skip middle point
            if abs(angle1 - angle2) < math.pi / 4:
                continue

            smoothed.append(curr)

        smoothed.append(path[-1])
        return smoothed