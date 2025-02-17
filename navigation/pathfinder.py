import asyncio
import heapq
import math
import time
from dataclasses import dataclass, field
from heapq import heappush, heappop
from typing import List, Tuple, Optional, Set

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap


@dataclass(order=True)
class Node:
    position: Tuple[int, int] = field(compare=False)
    g_cost: float = field(compare=False)
    h_cost: float = field(compare=False)
    f_cost: float = field(default=float('inf'))
    parent: 'Node' = field(default=None, compare=False)

    def __post_init__(self):
        self.f_cost = self.g_cost + self.h_cost


class Pathfinder:
    def __init__(self, world_map: WorldMap):
        self.world_map = world_map
        self.grid_cell_size = world_map.resolution  # 40cm per grid cell

        # Define movement directions (8-directional)
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]

    def heuristic(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic cost using diagonal distance"""
        dx = abs(pos[0] - goal[0])
        dy = abs(pos[1] - goal[1])
        # Scale by grid cell size since we're working in grid coordinates
        return (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)) * self.grid_cell_size

    def get_neighbors(self, node: Node, goal: Tuple[int, int]) -> List[Node]:
        """Get valid neighboring nodes"""
        neighbors = []

        for dx, dy in self.directions:
            new_x = node.position[0] + dx
            new_y = node.position[1] + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    # Calculate real-world distance for this movement
                    movement_cost = (math.sqrt(2) if dx != 0 and dy != 0 else 1.0) * self.grid_cell_size
                    g_cost = node.g_cost + movement_cost
                    h_cost = self.heuristic((new_x, new_y), goal)

                    neighbors.append(Node(
                        position=(new_x, new_y),
                        g_cost=g_cost,
                        h_cost=h_cost,
                        parent=node
                    ))

        return neighbors

    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[float, float]]:
        """Find path using A* algorithm"""
        # Initialize open and closed sets
        open_set: List[Node] = []
        closed_set: Set[Tuple[int, int]] = set()

        # Create start node
        start_node = Node(
            position=start,
            g_cost=0,
            h_cost=self.heuristic(start, goal)
        )

        # Add start node to open set
        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            # If we reached the goal
            if current.position == goal:
                path = []
                while current:
                    # Convert grid coordinates to world coordinates
                    world_x, world_y = self.world_map.grid_to_world(
                        current.position[0], current.position[1]
                    )
                    path.append((world_x, world_y))
                    current = current.parent
                return path[::-1]  # Reverse path to get start-to-goal order

            # Add current node to closed set
            closed_set.add(current.position)

            # Check neighbors
            for neighbor in self.get_neighbors(current, goal):
                if neighbor.position in closed_set:
                    continue

                existing = next((node for node in open_set
                                 if node.position == neighbor.position), None)

                if not existing or neighbor.g_cost < existing.g_cost:
                    if existing:
                        open_set.remove(existing)
                    heapq.heappush(open_set, neighbor)

        return []  # No path found

    def post_process_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Post-process path to add intermediate waypoints for long segments"""
        if not path:
            return path

        final_path = []
        max_segment_length = self.grid_cell_size  # One grid cell length

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            # Add first point
            final_path.append(p1)

            # Calculate distance between points
            dist = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

            # Add intermediate points if segment is too long
            if dist > max_segment_length:
                num_points = int(dist / max_segment_length)
                for j in range(1, num_points):
                    t = j / num_points
                    x = p1[0] + t * (p2[0] - p1[0])
                    y = p1[1] + t * (p2[1] - p1[1])
                    final_path.append((x, y))

        # Add final point
        final_path.append(path[-1])
        return final_path