import asyncio
import heapq
import math
from typing import List, Tuple, Set

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap

class Node:
    def __init__(self, x: int, y: int, heading: int, g_cost: float = float('inf'),
                 h_cost: float = 0.0, parent=None):
        self.x = x
        self.y = y
        self.heading = heading  # Discretized heading (0-359 degrees)
        self.g_cost = g_cost  # Cost from start
        self.h_cost = h_cost  # Heuristic cost to goal
        self.parent = parent

    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y
                and self.heading == other.heading)

    def __hash__(self):
        return hash((self.x, self.y, self.heading))


class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar

        # Movement constraints
        self.min_turn_radius = picar.get_min_turn_radius()
        self.heading_increments = 45  # Discretize heading into 45-degree increments
        self.movement_increments = 10  # cm per movement

        # Neighbor generation parameters
        self.possible_steering_angles = [-28, -14, 0, 14, 28]  # Discrete steering angles

    def find_path(self, start_x: float, start_y: float, start_heading: float,
                  goal_x: float, goal_y: float) -> List[Tuple[float, float, float]]:
        """
        Find a path from start to goal position considering vehicle constraints
        Returns list of (x, y, heading) waypoints
        """
        # Convert world coordinates to grid coordinates
        start_grid_x, start_grid_y = self.world_map.world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self.world_map.world_to_grid(goal_x, goal_y)

        # Discretize heading
        start_heading_disc = round(start_heading / self.heading_increments) * self.heading_increments

        # Initialize start and goal nodes
        start_node = Node(start_grid_x, start_grid_y, start_heading_disc, 0.0,
                          self._heuristic(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y))

        # Initialize open and closed sets
        open_set: List[Node] = [start_node]
        closed_set: Set[Node] = set()

        print(open_set, closed_set)

        while open_set:
            current = heapq.heappop(open_set)

            # Check if we've reached the goal (within threshold)
            if self._is_goal(current, goal_grid_x, goal_grid_y):
                return self._reconstruct_path(current)

            closed_set.add(current)

            # Generate neighbors based on possible movements
            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set:
                    continue

                # Calculate new g_cost
                new_g_cost = current.g_cost + self._movement_cost(current, neighbor)

                # If this is a better path or new node
                if new_g_cost < neighbor.g_cost:
                    neighbor.g_cost = new_g_cost
                    neighbor.parent = current

                    if neighbor not in open_set:
                        heapq.heappush(open_set, neighbor)

        print('no path found...')
        return []  # No path found

    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Calculate heuristic cost between two grid positions"""
        # Using diagonal distance as heuristic
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        return math.sqrt(dx * dx + dy * dy) * self.world_map.resolution

    def _is_goal(self, node: Node, goal_x: int, goal_y: int, threshold: int = 3) -> bool:
        """Check if node is within threshold of goal"""
        return (abs(node.x - goal_x) <= threshold and
                abs(node.y - goal_y) <= threshold)

    def _get_neighbors(self, node: Node) -> List[Node]:
        """Generate valid neighbor nodes based on vehicle movement constraints"""
        neighbors = []

        for steering_angle in self.possible_steering_angles:
            # Calculate movement arc
            if abs(steering_angle) < 1:  # Straight movement
                new_x = node.x + self.movement_increments * math.cos(math.radians(node.heading))
                new_y = node.y + self.movement_increments * math.sin(math.radians(node.heading))
                new_heading = node.heading
            else:
                # Calculate turning radius for this steering angle
                turn_radius = self.min_turn_radius * (self.picar.MAX_STEERING_ANGLE / abs(steering_angle))

                # Calculate change in heading
                arc_length = self.movement_increments
                angle_change = math.degrees(arc_length / turn_radius)
                if steering_angle < 0:
                    angle_change = -angle_change

                new_heading = (node.heading + angle_change) % 360

                # Calculate new position
                turning_center_x = node.x - turn_radius * math.sin(math.radians(node.heading))
                turning_center_y = node.y + turn_radius * math.cos(math.radians(node.heading))

                angle = math.radians(angle_change)
                new_x = (turning_center_x +
                         turn_radius * math.sin(math.radians(node.heading + angle_change)))
                new_y = (turning_center_y -
                         turn_radius * math.cos(math.radians(node.heading + angle_change)))

            # Convert to grid coordinates and check validity
            grid_x = round(new_x)
            grid_y = round(new_y)

            if self._is_valid_position(grid_x, grid_y):
                # Discretize heading
                disc_heading = round(new_heading / self.heading_increments) * self.heading_increments
                neighbor = Node(grid_x, grid_y, disc_heading)
                neighbors.append(neighbor)

        return neighbors

    def _is_valid_position(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid position is valid and obstacle-free"""
        if (grid_x < 0 or grid_x >= self.world_map.grid_size or
                grid_y < 0 or grid_y >= self.world_map.grid_size):
            return False

        # Check for obstacles (including padding)
        return self.world_map.grid[grid_y, grid_x] == 0

    def _movement_cost(self, from_node: Node, to_node: Node) -> float:
        """Calculate cost of movement between nodes"""
        # Base cost is Euclidean distance
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.sqrt(dx * dx + dy * dy) * self.world_map.resolution

        # Add turning penalty
        heading_diff = abs(to_node.heading - from_node.heading)
        heading_diff = min(heading_diff, 360 - heading_diff)
        turning_penalty = heading_diff * 0.1  # Penalize sharp turns

        return distance + turning_penalty

    def _reconstruct_path(self, goal_node: Node) -> List[Tuple[float, float, float]]:
        """Reconstruct path from goal node back to start"""
        path = []
        current = goal_node

        while current is not None:
            # Convert grid coordinates back to world coordinates
            world_x, world_y = self.world_map.grid_to_world(current.x, current.y)
            path.append((world_x, world_y, current.heading))
            current = current.parent

        return list(reversed(path))  # Return path from start to goal
