import asyncio
import heapq
import math
from typing import List, Tuple, Optional, Dict
from picarx_wrapper import PicarXWrapper
from vision_system import VisionSystem
from world_map import WorldMap


class Pathfinder:
    def __init__(self, world_map: 'WorldMap'):
        self.world_map = world_map
        self.path: List[Tuple[int, int]] = []
        self.replanning_interval = 10  # Number of steps before replanning
        self.steps_since_replanning = 0

        # Movement costs
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4  # sqrt(2)
        self.OBSTACLE_COST = float('inf')

        # Directions for pathfinding (8-directional movement)
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]

    def _get_path_cost(self, current: Tuple[int, int], neighbor: Tuple[int, int]) -> float:
        """Calculate the cost of moving from current to neighbor"""
        dx = abs(current[0] - neighbor[0])
        dy = abs(current[1] - neighbor[1])
        return self.DIAGONAL_COST if dx + dy == 2 else self.STRAIGHT_COST

    def _heuristic(self, point: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic distance (diagonal distance)"""
        dx = abs(point[0] - goal[0])
        dy = abs(point[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def _get_neighbors(self, point: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring points"""
        neighbors = []
        for dx, dy in self.directions:
            new_x = point[0] + dx
            new_y = point[1] + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):
                # Check if point is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))
        return neighbors

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[int, int]]:
        """Find path from start to goal using A* algorithm"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(*start)
        goal_grid = self.world_map.world_to_grid(*goal)

        # Initialize data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal_grid:
                break

            for neighbor in self._get_neighbors(current):
                new_cost = cost_so_far[current] + self._get_path_cost(current, neighbor)

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor, goal_grid)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        # Reconstruct path
        self.path = []
        current = goal_grid
        while current is not None:
            self.path.append(current)
            current = came_from.get(current)
        self.path.reverse()

        return self.path if self.path else None

    async def execute_path(self, picar: PicarXWrapper, vision_system: VisionSystem) -> bool:
        """Execute the planned path while monitoring for obstacles"""
        if not self.path:
            return False

        stop_sign_wait_complete = False

        for i, (grid_x, grid_y) in enumerate(self.path):
            # Convert grid coordinates back to world coordinates
            target_x, target_y = self.world_map.grid_to_world(grid_x, grid_y)

            # Check for dynamic obstacles
            while True:
                # Get vision system updates
                objects = vision_system.get_obstacle_info()

                # Handle detected objects
                if objects:
                    person_or_cat = any(obj['label'] in ['person', 'cat'] for obj in objects)
                    stop_sign = any(obj['label'] == 'stop sign' for obj in objects)

                    if person_or_cat:
                        print("Person or cat detected - waiting...")
                        picar.stop()
                        await asyncio.sleep(1)
                        continue

                    if stop_sign and not stop_sign_wait_complete:
                        print("Stop sign detected - stopping for 3 seconds...")
                        picar.stop()
                        await asyncio.sleep(3)
                        stop_sign_wait_complete = True

                # No obstacles detected, proceed with movement
                break

            # Move to next point
            success = await picar.navigate_to_point(target_x, target_y)
            if not success:
                return False

            # Check if replanning is needed
            self.steps_since_replanning += 1
            if self.steps_since_replanning >= self.replanning_interval:
                return True  # Signal that replanning is needed

        return True

    def visualize_path(self):
        """Visualize the planned path on the world map"""
        if not self.path:
            return

        # Create a copy of the grid for visualization
        viz_grid = self.world_map.grid.copy()

        # Mark path points
        for x, y in self.path:
            viz_grid[y, x] = 2  # Use 2 to distinguish path from obstacles

        # Print visualization
        print("\nPath visualization (0=free, 1=obstacle, 2=path):")
        for row in viz_grid:
            print(''.join(['2' if cell == 2 else '1' if cell else '0' for cell in row]))