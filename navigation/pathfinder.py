import asyncio
import heapq
import math
from heapq import heappush, heappop
from typing import List, Tuple, Set, Optional
import numpy as np
from picarx_wrapper import PicarXWrapper
from world_map import WorldMap

class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar

        # Navigation parameters
        self.path_segment_length = 50  # cm - distance to travel before rebuilding map
        self.rebuild_threshold = 0.8  # Rebuild if 80% of segment is complete
        self.heuristic_weight = 1.1  # Slight weight to favor shorter paths

        # Current navigation state
        self.current_path = []
        self.current_segment_start = None
        self.target_position = None
        self.is_navigating = False

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring grid cells"""
        x, y = pos
        neighbors = []

        # Check 8 adjacent cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):
                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))

        return neighbors

    def find_path(self, start_x: float, start_y: float,
                  target_x: float, target_y: float) -> Optional[List[Tuple[float, float]]]:
        """Find path from start to target position using A*"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        target_grid = self.world_map.world_to_grid(target_x, target_y)

        # Initialize A* data structures
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == target_grid:
                break

            for next_pos in self.get_neighbors(current):
                # Calculate movement cost (diagonal moves cost more)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                move_cost = 1.4 if dx + dy == 2 else 1.0

                new_cost = cost_so_far[current] + move_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic_weight * self.heuristic(next_pos, target_grid)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        if target_grid not in came_from:
            return None

        path = []
        current = target_grid
        while current is not None:
            # Convert grid coordinates back to world coordinates
            world_pos = self.world_map.grid_to_world(current[0], current[1])
            path.append(world_pos)
            current = came_from[current]

        return list(reversed(path))

    def simplify_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Simplify path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Look ahead for longest valid straight line segment
            for look_ahead in range(len(path) - 1, current_idx, -1):
                start = path[current_idx]
                end = path[look_ahead]

                # Check if direct path is clear
                is_clear = True
                interpolated_points = self._interpolate_path(start, end)

                for point in interpolated_points:
                    grid_x, grid_y = self.world_map.world_to_grid(point[0], point[1])
                    if (grid_x >= self.world_map.grid_size or grid_y >= self.world_map.grid_size or
                            grid_x < 0 or grid_y < 0 or
                            self.world_map.grid[grid_y, grid_x] == 1):
                        is_clear = False
                        break

                if is_clear:
                    simplified.append(end)
                    current_idx = look_ahead
                    break

            current_idx += 1

        return simplified

    def _interpolate_path(self, start: Tuple[float, float],
                          end: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Generate points along line between start and end"""
        points = []
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = ((dx ** 2) + (dy ** 2)) ** 0.5

        # Generate points every 5cm
        steps = max(1, int(distance / 5))
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * dx
            y = start[1] + t * dy
            points.append((x, y))

        return points

    async def navigate_to_target(self, target_x: float, target_y: float):
        """Navigate to target position while avoiding obstacles"""
        self.is_navigating = True
        self.target_position = (target_x, target_y)

        while self.is_navigating:
            # Get current position
            pos = self.picar.get_position()
            current_x, current_y = pos['x'], pos['y']

            # Check if we're close enough to target
            distance_to_target = ((current_x - target_x) ** 2 +
                                  (current_y - target_y) ** 2) ** 0.5
            if distance_to_target < 5:  # 5cm threshold
                print("Reached target position!")
                self.is_navigating = False
                break

            # Find new path if needed
            if not self.current_path:
                print("Finding new path to target...")
                path = self.find_path(current_x, current_y, target_x, target_y)
                if not path:
                    print("No valid path found!")
                    self.is_navigating = False
                    break

                self.current_path = self.simplify_path(path)
                self.current_segment_start = (current_x, current_y)
                print(f"Path found with {len(self.current_path)} waypoints")

            # Navigate to next waypoint
            next_waypoint = self.current_path[0]
            print(f"Navigating to waypoint: ({next_waypoint[0]:.1f}, {next_waypoint[1]:.1f})")

            # Check if we need to rebuild path
            segment_progress = self._get_segment_progress(self.current_segment_start,
                                                          next_waypoint,
                                                          (current_x, current_y))

            if segment_progress > self.rebuild_threshold:
                print("Rebuilding path...")
                self.current_path = []
                continue

            # Use existing navigate_to_point for waypoint navigation
            reached = await self.picar.navigate_to_point(next_waypoint[0], next_waypoint[1])

            if reached:
                print("Reached waypoint")
                self.current_path.pop(0)
                self.current_segment_start = (current_x, current_y)

            await asyncio.sleep(0.1)

    def _get_segment_progress(self, start: Tuple[float, float],
                              end: Tuple[float, float],
                              current: Tuple[float, float]) -> float:
        """Calculate progress along current path segment"""
        segment_vector = np.array([end[0] - start[0], end[1] - start[1]])
        current_vector = np.array([current[0] - start[0], current[1] - start[1]])

        segment_length = np.linalg.norm(segment_vector)
        if segment_length == 0:
            return 1.0

        # Project current position onto segment
        progress = np.dot(current_vector, segment_vector) / segment_length
        return max(0.0, min(1.0, progress / segment_length))

    def stop_navigation(self):
        """Stop current navigation"""
        self.is_navigating = False
        self.current_path = []
        self.target_position = None