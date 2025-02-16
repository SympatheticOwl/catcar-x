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
        self.grid_size = 42  # cm per grid cell
        self.path = []
        self.current_target = None

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        x, y = pos
        neighbors = []

        # Check 8 surrounding cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if 0 <= new_x < self.world_map.grid_size and 0 <= new_y < self.world_map.grid_size:
                # Check if cell is obstacle-free
                if not self.world_map.grid[new_y, new_x]:
                    neighbors.append((new_x, new_y))

        return neighbors

    def find_path(self, start_pos: Tuple[float, float], target_pos: Tuple[float, float]) -> List[Tuple[int, int]]:
        """Find path using A* algorithm, converting between world and grid coordinates"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(*start_pos)
        target_grid = self.world_map.world_to_grid(*target_pos)

        # Initialize data structures
        frontier = []
        heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heappop(frontier)[1]

            if current == target_grid:
                break

            for next_pos in self._get_neighbors(current):
                # Calculate movement cost (diagonal movements cost more)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                movement_cost = 1.4 if dx + dy == 2 else 1.0

                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self._heuristic(target_grid, next_pos)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        path = []
        current = target_grid
        while current is not None:
            path.append(current)
            current = came_from.get(current)

        path.reverse()
        return path if path else []

    async def navigate_to_target(self, target_x: float, target_y: float):
        """Navigate to target position while avoiding obstacles"""
        self.current_target = (target_x, target_y)

        while True:
            # Get current position
            pos = self.picar.get_position()
            current_pos = (pos['x'], pos['y'])

            # Check if we've reached the target
            distance_to_target = math.sqrt(
                (current_pos[0] - target_x) ** 2 +
                (current_pos[1] - target_y) ** 2
            )
            if distance_to_target < self.grid_size / 2:
                print(f"Reached target position: ({target_x}, {target_y})")
                self.picar.stop()
                return True

            # Find path to target
            path = self.find_path(current_pos, (target_x, target_y))
            if not path:
                print("No valid path found to target!")
                self.picar.stop()
                return False

            # Get next waypoint
            next_waypoint = self.world_map.grid_to_world(*path[1]) if len(path) > 1 else (target_x, target_y)

            # Check vision system for obstacles
            objects = self.picar.vision.get_obstacle_info()
            if objects:
                for obj in objects:
                    if obj['label'] in ['person', 'cat']:
                        print(f"Detected {obj['label']}, waiting...")
                        self.picar.stop()
                        await asyncio.sleep(1)  # Check again in 1 second
                        continue
                    elif obj['label'] == 'stop sign':
                        print("Stop sign detected, waiting 3 seconds")
                        self.picar.stop()
                        await asyncio.sleep(3)

            # Navigate to next waypoint
            try:
                await self.picar.navigate_to_point(next_waypoint[0], next_waypoint[1])
            except asyncio.CancelledError:
                self.picar.stop()
                raise

            await asyncio.sleep(0.1)  # Control loop rate

    def update_path(self):
        """Update path based on new obstacle information"""
        if self.current_target:
            pos = self.picar.get_position()
            current_pos = (pos['x'], pos['y'])
            self.path = self.find_path(current_pos, self.current_target)
            return bool(self.path)
        return False