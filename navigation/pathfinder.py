import asyncio
import heapq
import math
from typing import List, Tuple, Optional, Dict
from picarx_wrapper import PicarXWrapper
from vision_system import VisionSystem
from world_map import WorldMap


class Pathfinder:
    def __init__(self, world_map: WorldMap):
        self.world_map = world_map
        self.path_length = 10  # Number of steps to return in partial path

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
                    neighbors.append((new_x, new_y))

        return neighbors

    def heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic (Euclidean distance)"""
        return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    def find_path(self, start_x: float, start_y: float,
                  goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Find path from start to goal using A*"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        path_found = False

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                path_found = True
                break

            for next_node in self.get_neighbors(current):
                # Calculate movement cost (diagonal movements cost more)
                dx = abs(next_node[0] - current[0])
                dy = abs(next_node[1] - current[1])
                movement_cost = math.sqrt(2) if dx + dy == 2 else 1

                new_cost = cost_so_far[current] + movement_cost

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal_grid)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        if not path_found:
            return []

        # Reconstruct path
        current = goal_grid
        path = []

        while current is not None:
            path.append(current)
            current = came_from[current]

        path.reverse()

        # Convert path back to world coordinates
        world_path = []
        for grid_x, grid_y in path:
            world_x, world_y = self.world_map.grid_to_world(grid_x, grid_y)
            world_path.append((world_x, world_y))

        # Return partial path if it's longer than path_length
        if len(world_path) > self.path_length:
            return world_path[:self.path_length]
        return world_path

    def update_path_length(self, length: int):
        """Update the number of path steps to return"""
        self.path_length = max(1, length)  # Ensure at least 1 step