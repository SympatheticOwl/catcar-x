import asyncio
import heapq
import math

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap


class Node:
    def __init__(self, position, g_cost=float('inf'), h_cost=0):
        self.position = position  # (x, y) tuple
        self.g_cost = g_cost  # Cost from start to current node
        self.h_cost = h_cost  # Estimated cost from current node to goal
        self.parent = None

    @property
    def f_cost(self):
        return self.g_cost + self.h_cost

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f_cost < other.f_cost


class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar
        self.path = []
        self.current_segment = 0
        self.segment_length = 50  # cm - distance to travel before rescanning
        self.min_turn_radius = picar.get_min_turn_radius()

    def heuristic(self, pos1, pos2):
        """Calculate heuristic cost between two positions"""
        x1, y1 = pos1
        x2, y2 = pos2
        # Using Euclidean distance as heuristic
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_neighbors(self, node):
        """Get valid neighboring positions"""
        x, y = node.position
        neighbors = []

        # Define possible movements (8-directional)
        moves = [(0, 1), (1, 0), (0, -1), (-1, 0),
                 (1, 1), (1, -1), (-1, 1), (-1, -1)]

        for dx, dy in moves:
            new_x = x + dx
            new_y = y + dy

            # Check if position is within grid bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):

                # Check if position is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    # Check turning radius constraint
                    if self.check_turn_feasible(node.position, (new_x, new_y)):
                        neighbors.append(Node((new_x, new_y)))

        return neighbors

    def check_turn_feasible(self, current_pos, next_pos):
        """Check if turn is feasible given car's turning radius"""
        if not self.path:  # No previous position to check angle
            return True

        if len(self.path) < 2:  # Need at least 2 points to check angle
            return True

        # Get previous position from path
        prev_pos = self.path[-2].position if len(self.path) >= 2 else current_pos

        # Calculate angles
        angle1 = math.atan2(current_pos[1] - prev_pos[1],
                            current_pos[0] - prev_pos[0])
        angle2 = math.atan2(next_pos[1] - current_pos[1],
                            next_pos[0] - current_pos[0])

        # Calculate angle difference
        angle_diff = abs(math.degrees(angle2 - angle1))
        if angle_diff > 180:
            angle_diff = 360 - angle_diff

        # Check if turn is too sharp given minimum turning radius
        # Simplified check - can be made more precise
        return angle_diff <= 45  # Maximum 45-degree turn between segments

    async def find_path(self, start_pos, goal_pos):
        """Find path using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(*start_pos)
        goal_grid = self.world_map.world_to_grid(*goal_pos)

        start_node = Node(start_grid, g_cost=0)
        start_node.h_cost = self.heuristic(start_grid, goal_grid)
        goal_node = Node(goal_grid)

        open_set = [start_node]
        closed_set = set()

        while open_set:
            current = min(open_set)
            open_set.remove(current)

            if current == goal_node:
                # Reconstruct path
                path = []
                while current:
                    path.append(current)
                    current = current.parent
                self.path = path[::-1]
                return True

            closed_set.add(current.position)

            for neighbor in self.get_neighbors(current):
                if neighbor.position in closed_set:
                    continue

                # Calculate new g_cost
                new_g_cost = current.g_cost + self.heuristic(
                    current.position, neighbor.position)

                if neighbor not in open_set:
                    open_set.append(neighbor)
                elif new_g_cost >= neighbor.g_cost:
                    continue

                neighbor.parent = current
                neighbor.g_cost = new_g_cost
                neighbor.h_cost = self.heuristic(
                    neighbor.position, goal_grid)

        return False  # No path found

    def get_next_segment(self):
        """Get next path segment to navigate"""
        if not self.path or self.current_segment >= len(self.path):
            return None

        # Get current segment start
        start_idx = self.current_segment
        current_length = 0
        end_idx = start_idx

        # Find segment end point based on segment_length
        while end_idx < len(self.path) - 1:
            pos1 = self.path[end_idx].position
            pos2 = self.path[end_idx + 1].position

            # Convert grid coordinates to world coordinates
            world_pos1 = self.world_map.grid_to_world(*pos1)
            world_pos2 = self.world_map.grid_to_world(*pos2)

            # Calculate distance
            dist = math.sqrt((world_pos2[0] - world_pos1[0]) ** 2 +
                             (world_pos2[1] - world_pos1[1]) ** 2)

            if current_length + dist > self.segment_length:
                break

            current_length += dist
            end_idx += 1

        # Update current segment
        self.current_segment = end_idx + 1

        # Return world coordinates of end point
        end_point = self.path[end_idx].position
        return self.world_map.grid_to_world(*end_point)

    async def navigate_to_goal(self, goal_x, goal_y):
        """Navigate to goal position with obstacle avoidance"""
        print(f"Starting navigation to goal: ({goal_x}, {goal_y})")

        while True:
            # Get current position
            pos = self.picar.get_position()
            current_x, current_y = pos['x'], pos['y']

            # Scan environment
            print("Scanning environment...")
            await self.picar.scan_environment()
            self.world_map.add_padding()

            # Find path to goal
            print("Finding path to goal...")
            path_found = await self.find_path(
                (current_x, current_y), (goal_x, goal_y))

            if not path_found:
                print("No valid path found!")
                return False

            # Get next segment endpoint
            next_point = self.get_next_segment()
            if not next_point:
                print("Reached goal!")
                return True

            print(f"Navigating to intermediate point: {next_point}")
            # Navigate to segment endpoint
            await self.picar.navigate_to_point(*next_point)

            # Small delay before next iteration
            await asyncio.sleep(0.1)