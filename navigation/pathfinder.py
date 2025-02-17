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


@dataclass
class Node:
    x: int  # grid coordinates
    y: int
    g_cost: float = float('inf')  # cost from start
    h_cost: float = float('inf')  # heuristic cost to goal
    parent: 'Node' = None

    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar
        self.current_path = []
        self.current_path_index = 0

        # Movement directions (8-directional)
        self.directions = [
            (0, 1),  # N
            (1, 1),  # NE
            (1, 0),  # E
            (1, -1),  # SE
            (0, -1),  # S
            (-1, -1),  # SW
            (-1, 0),  # W
            (-1, 1)  # NW
        ]

    def heuristic(self, node: Node, goal: Node) -> float:
        """Calculate heuristic cost using diagonal distance"""
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        return math.sqrt(dx * dx + dy * dy)

    def get_neighbors(self, node: Node) -> List[Node]:
        """Get valid neighboring nodes"""
        neighbors = []

        for dx, dy in self.directions:
            new_x = node.x + dx
            new_y = node.y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbor = Node(new_x, new_y)

                    # Calculate movement cost (diagonal movement costs more)
                    if dx == 0 or dy == 0:
                        movement_cost = 1.0
                    else:
                        movement_cost = 1.4  # sqrt(2)

                    neighbor.g_cost = node.g_cost + movement_cost
                    neighbors.append(neighbor)

        return neighbors

    def find_path(self, start_x: float, start_y: float,
                  goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Find path from start to goal using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid_x, start_grid_y = self.world_map.world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self.world_map.world_to_grid(goal_x, goal_y)

        start_node = Node(start_grid_x, start_grid_y, g_cost=0)
        goal_node = Node(goal_grid_x, goal_grid_y)

        # Initialize open and closed sets
        open_set = {start_node}
        closed_set = set()

        # Calculate initial heuristic
        start_node.h_cost = self.heuristic(start_node, goal_node)

        while open_set:
            # Get node with lowest f_cost
            current = min(open_set, key=lambda n: n.f_cost)

            # Check if we've reached the goal
            if current.x == goal_node.x and current.y == goal_node.y:
                return self._reconstruct_path(current)

            open_set.remove(current)
            closed_set.add(current)

            # Check neighbors
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue

                # Calculate heuristic if not already in open set
                if neighbor not in open_set:
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.parent = current
                    open_set.add(neighbor)
                else:
                    # Check if this path is better than previous
                    if neighbor.g_cost > current.g_cost + 1:
                        neighbor.g_cost = current.g_cost + 1
                        neighbor.parent = current

        return []  # No path found

    def _reconstruct_path(self, goal_node: Node) -> List[Tuple[float, float]]:
        """Reconstruct path from goal node back to start"""
        path = []
        current = goal_node

        while current is not None:
            # Convert grid coordinates back to world coordinates
            world_x, world_y = self.world_map.grid_to_world(current.x, current.y)
            path.append((world_x, world_y))
            current = current.parent

        return list(reversed(path))

    def is_near_target(self, current_x: float, current_y: float,
                       target_x: float, target_y: float) -> bool:
        """Check if current position is within one grid space of target"""
        distance = math.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2)
        return distance <= self.world_map.resolution