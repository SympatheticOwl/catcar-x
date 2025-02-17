import asyncio
import heapq
import math
import time
from dataclasses import dataclass, field
from heapq import heappush, heappop
from typing import List, Tuple, Optional

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap


@dataclass(order=True)
class Node:
    position: Tuple[int, int] = field(compare=False)
    g_cost: float = field(compare=False)  # Cost from start to current node
    h_cost: float = field(compare=False)  # Estimated cost to goal
    f_cost: float = field(default=float('inf'))  # Total cost (g + h)
    parent: 'Node' = field(default=None, compare=False)

    def __post_init__(self):
        self.f_cost = self.g_cost + self.h_cost

class Pathfinder:
    def __init__(self, world_map, picar):
        self.world_map = world_map
        self.picar = picar
        self.movement_cost = 1.0
        self.diagonal_cost = 1.4  # sqrt(2)

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
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

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
                    movement_cost = self.diagonal_cost if dx != 0 and dy != 0 else self.movement_cost
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

                # Check if this path is better than any previous one
                existing = next((node for node in open_set
                                 if node.position == neighbor.position), None)

                if not existing or neighbor.g_cost < existing.g_cost:
                    if existing:
                        open_set.remove(existing)
                    heapq.heappush(open_set, neighbor)

        return []  # No path found

    def smooth_path(self, path: List[Tuple[float, float]],
                    smoothing_weight: float = 0.5) -> List[Tuple[float, float]]:
        """Apply path smoothing to reduce sharp turns"""
        if len(path) <= 2:
            return path

        smoothed = path.copy()
        change = True
        while change:
            change = False
            for i in range(1, len(smoothed) - 1):
                old_x, old_y = smoothed[i]

                # Calculate smoothed position
                new_x = smoothed[i][0] + smoothing_weight * (
                        smoothed[i - 1][0] + smoothed[i + 1][0] - 2 * smoothed[i][0]
                )
                new_y = smoothed[i][1] + smoothing_weight * (
                        smoothed[i - 1][1] + smoothed[i + 1][1] - 2 * smoothed[i][1]
                )

                # Update if the change is significant
                if abs(new_x - old_x) > 0.1 or abs(new_y - old_y) > 0.1:
                    smoothed[i] = (new_x, new_y)
                    change = True

        return smoothed

    def post_process_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Apply post-processing to path"""
        if not path:
            return path

        # First smooth the path
        smoothed = self.smooth_path(path)

        # Add intermediate points for long segments
        final_path = []
        max_segment_length = 20  # cm

        for i in range(len(smoothed) - 1):
            p1 = smoothed[i]
            p2 = smoothed[i + 1]

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
        final_path.append(smoothed[-1])
        return final_path