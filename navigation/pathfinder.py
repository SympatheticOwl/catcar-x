import asyncio
import heapq
import math
import time
from heapq import heappush, heappop
from typing import List, Tuple, Optional

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap

class Node:
    def __init__(self, x: int, y: int, g_cost: float = float('inf'),
                 h_cost: float = 0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # Cost from start to current node
        self.h_cost = h_cost  # Estimated cost from current node to goal
        self.f_cost = g_cost + h_cost  # Total cost
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
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]
        self.min_turn_radius = 20  # cm, approximate minimum turning radius
        self.path_smoothing_iterations = 3

    def get_neighbors(self, node: Node) -> List[Node]:
        """Get valid neighboring nodes considering turning radius constraints"""
        neighbors = []
        for dx, dy in self.directions:
            new_x = node.x + dx
            new_y = node.y + dy

            # Check grid bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if not self.world_map.grid[new_y, new_x]:
                    # Convert to world coordinates
                    world_x, world_y = self.world_map.grid_to_world(new_x, new_y)
                    world_current_x, world_current_y = self.world_map.grid_to_world(node.x, node.y)

                    # Calculate angle change
                    if node.parent:
                        parent_x, parent_y = self.world_map.grid_to_world(node.parent.x, node.parent.y)
                        angle1 = math.atan2(world_current_y - parent_y, world_current_x - parent_x)
                        angle2 = math.atan2(world_y - world_current_y, world_x - world_current_x)
                        angle_change = abs(angle2 - angle1)

                        # Skip if angle change is too sharp for turning radius
                        if angle_change > math.pi / 4:  # 45 degrees
                            continue

                    neighbors.append(Node(new_x, new_y))

        return neighbors

    def heuristic(self, node: Node, goal: Node) -> float:
        """Calculate heuristic cost using Euclidean distance"""
        return math.sqrt((node.x - goal.x) ** 2 + (node.y - goal.y) ** 2)

    def find_path(self, start_x: float, start_y: float,
                  goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Find path from start to goal using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid_x, start_grid_y = self.world_map.world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self.world_map.world_to_grid(goal_x, goal_y)

        start_node = Node(start_grid_x, start_grid_y, g_cost=0)
        goal_node = Node(goal_grid_x, goal_grid_y)

        # Initialize open and closed sets
        open_set = []
        closed_set = set()

        # Add start node to open set
        start_node.h_cost = self.heuristic(start_node, goal_node)
        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                return self._reconstruct_path(current)

            closed_set.add((current.x, current.y))

            for neighbor in self.get_neighbors(current):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                # Calculate new g_cost
                movement_cost = math.sqrt((neighbor.x - current.x) ** 2 +
                                          (neighbor.y - current.y) ** 2)
                new_g_cost = current.g_cost + movement_cost

                # If this path is better than previous ones
                if neighbor not in open_set or new_g_cost < neighbor.g_cost:
                    neighbor.g_cost = new_g_cost
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    neighbor.parent = current

                    if neighbor not in open_set:
                        heapq.heappush(open_set, neighbor)

        return []  # No path found

    def _reconstruct_path(self, goal_node: Node) -> List[Tuple[float, float]]:
        """Reconstruct the path from goal to start"""
        path = []
        current = goal_node

        while current:
            # Convert grid coordinates back to world coordinates
            world_x, world_y = self.world_map.grid_to_world(current.x, current.y)
            path.append((world_x, world_y))
            current = current.parent

        path.reverse()
        return self._smooth_path(path)

    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Apply path smoothing to reduce sharp turns"""
        if len(path) <= 2:
            return path

        for _ in range(self.path_smoothing_iterations):
            i = 1
            while i < len(path) - 1:
                prev_x, prev_y = path[i - 1]
                curr_x, curr_y = path[i]
                next_x, next_y = path[i + 1]

                # Calculate angles
                angle1 = math.atan2(curr_y - prev_y, curr_x - prev_x)
                angle2 = math.atan2(next_y - curr_y, next_x - curr_x)
                angle_diff = abs(angle2 - angle1)

                # If turn is too sharp, interpolate a new point
                if angle_diff > math.pi / 4:
                    new_x = (prev_x + next_x) / 2
                    new_y = (prev_y + next_y) / 2
                    path[i] = (new_x, new_y)

                i += 1

        return path