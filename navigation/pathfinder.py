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
        """Euclidean distance heuristic"""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

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

                # Debug print
                print(f"Checking neighbor ({new_x}, {new_y})")
                print(f"Grid value: {self.world_map.grid[new_y, new_x]}")

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    # Check turning radius constraint
                    if self.check_turn_feasible(node, (new_x, new_y)):
                        neighbors.append((new_x, new_y))
                        print(f"Added valid neighbor: ({new_x}, {new_y})")

        return neighbors

    def check_turn_feasible(self, current: Tuple[int, int], next: Tuple[int, int]) -> bool:
        """Check if turn is feasible given car's minimum turning radius"""
        # For initial testing, always return True to debug path finding
        return True

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path from start to goal using A*"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start[0], start[1])
        goal_grid = self.world_map.world_to_grid(goal[0], goal[1])

        print(f"Finding path from {start} to {goal}")
        print(f"Grid coordinates: from {start_grid} to {goal_grid}")

        # Validate coordinates
        if not (0 <= start_grid[0] < self.world_map.grid_size and
                0 <= start_grid[1] < self.world_map.grid_size and
                0 <= goal_grid[0] < self.world_map.grid_size and
                0 <= goal_grid[1] < self.world_map.grid_size):
            print("Start or goal coordinates out of bounds!")
            return []

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        print("Starting pathfinding loop...")

        while frontier:
            current = heapq.heappop(frontier)[1]
            print(f"Exploring node: {current}")

            if current == goal_grid:
                print("Goal reached!")
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
                    print(f"Added node {next} to frontier with priority {priority}")

        # Reconstruct path
        if goal_grid not in came_from:
            print("No path found!")
            return []  # No path found

        path = []
        current = goal_grid
        while current is not None:
            # Convert back to world coordinates
            world_coords = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_coords)
            current = came_from[current]

        path.reverse()
        print(f"Found path: {path}")
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