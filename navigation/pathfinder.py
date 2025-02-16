import asyncio
import heapq
import math
from typing import List, Tuple, Optional, Dict
from world_map import WorldMap


class Pathfinder:
    def __init__(self, world_map: WorldMap):
        """
        Initialize pathfinder with WorldMap instance

        Args:
            world_map: WorldMap instance containing obstacle information
        """
        self.world_map = world_map
        self.movements = [
            (0, 1),  # up
            (1, 0),  # right
            (0, -1),  # down
            (-1, 0),  # left
            (1, 1),  # diagonal up-right
            (-1, 1),  # diagonal up-left
            (1, -1),  # diagonal down-right
            (-1, -1)  # diagonal down-left
        ]

    def find_path(self, start_x: float, start_y: float,
                  target_x: float, target_y: float) -> Optional[List[Tuple[float, float]]]:
        """
        Find path from start to target position using A* algorithm

        Args:
            start_x: Starting X coordinate in world space (cm)
            start_y: Starting Y coordinate in world space (cm)
            target_x: Target X coordinate in world space (cm)
            target_y: Target Y coordinate in world space (cm)

        Returns:
            List of (x,y) coordinates forming the path, or None if no path found
        """
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        target_grid = self.world_map.world_to_grid(target_x, target_y)

        # Check if start or target is in obstacle
        if (self.world_map.grid[start_grid[1], start_grid[0]] == 1 or
                self.world_map.grid[target_grid[1], target_grid[0]] == 1):
            print("Start or target position is in obstacle!")
            return None

        # Initialize open and closed sets
        open_set = []
        closed_set = set()
        came_from = {}

        # Cost from start to current node
        g_score = {start_grid: 0}
        # Estimated total cost from start to goal through current node
        f_score = {start_grid: self._heuristic(start_grid, target_grid)}

        # Add start node to open set
        heapq.heappush(open_set, (f_score[start_grid], start_grid))

        while open_set:
            # Get node with lowest f_score
            current = heapq.heappop(open_set)[1]

            # If we reached the target
            if current == target_grid:
                return self._reconstruct_path(came_from, current)

            closed_set.add(current)

            # Check all possible movements
            for dx, dy in self.movements:
                neighbor = (current[0] + dx, current[1] + dy)

                # Skip if out of bounds
                if (neighbor[0] < 0 or neighbor[0] >= self.world_map.grid_size or
                        neighbor[1] < 0 or neighbor[1] >= self.world_map.grid_size):
                    continue

                # Skip if in closed set
                if neighbor in closed_set:
                    continue

                # Skip if neighbor is obstacle
                if self.world_map.grid[neighbor[1], neighbor[0]] == 1:
                    continue

                # Calculate movement cost (diagonal movements cost more)
                movement_cost = math.sqrt(dx * dx + dy * dy)
                tentative_g_score = g_score[current] + movement_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor, target_grid)

                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # No path found
        print("No valid path found!")
        return None

    def _heuristic(self, node: Tuple[int, int], target: Tuple[int, int]) -> float:
        """
        Calculate heuristic estimate of distance between nodes using Euclidean distance

        Args:
            node: Current node grid coordinates
            target: Target node grid coordinates

        Returns:
            Estimated distance between nodes
        """
        return math.sqrt((node[0] - target[0]) ** 2 + (node[1] - target[1]) ** 2)

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[float, float]]:
        """
        Reconstruct path from came_from dictionary

        Args:
            came_from: Dictionary tracking path
            current: Current node

        Returns:
            List of (x,y) coordinates in world space
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)

        # Convert grid coordinates back to world coordinates
        world_path = []
        for node in reversed(path):
            x, y = self.world_map.grid_to_world(node[0], node[1])
            world_path.append((x, y))

        return world_path

    def smooth_path(self, path: List[Tuple[float, float]],
                    smoothing_weight: float = 0.5) -> List[Tuple[float, float]]:
        """
        Apply path smoothing to reduce sharp turns

        Args:
            path: Original path as list of (x,y) coordinates
            smoothing_weight: Weight of smoothing (0-1)

        Returns:
            Smoothed path as list of (x,y) coordinates
        """
        if len(path) <= 2:
            return path

        smoothed_path = [list(coord) for coord in path]
        change = True
        while change:
            change = False
            for i in range(1, len(path) - 1):
                for j in range(2):  # For both x and y coordinates
                    old = smoothed_path[i][j]
                    smoothed_path[i][j] += smoothing_weight * (
                            path[i][j] - smoothed_path[i][j] +
                            smoothed_path[i - 1][j] + smoothed_path[i + 1][j] -
                            2 * smoothed_path[i][j]
                    )
                    if abs(old - smoothed_path[i][j]) > 0.1:
                        change = True

        return [tuple(coord) for coord in smoothed_path]