import asyncio
import heapq
import math
import time
from typing import List, Tuple, Optional

from picarx_wrapper import PicarXWrapper
from world_map import WorldMap

class Pathfinder:
    def __init__(self, world_map: WorldMap, picar: PicarXWrapper):
        self.world_map = world_map
        self.picar = picar

        # Path planning parameters
        self.replan_interval = 5.0  # Seconds between path replanning
        self.path_segment_length = 3  # Number of points to follow before replanning
        self.last_plan_time = 0
        self.current_path: List[Tuple[int, int]] = []
        self.current_path_index = 0

        # Navigation states
        self.is_navigating = False
        self.target_x = 0
        self.target_y = 0
        self.stop_sign_pause = False
        self.detected_obstacles = set()  # Track detected dynamic obstacles

    def _get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
        x, y = node
        neighbors = []

        # Check 8 surrounding cells
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                new_x = x + dx
                new_y = y + dy

                # Check bounds
                if (0 <= new_x < self.world_map.grid_size and
                        0 <= new_y < self.world_map.grid_size):
                    # Check if cell is obstacle-free
                    if self.world_map.grid[new_y, new_x] == 0:
                        neighbors.append((new_x, new_y))

        return neighbors

    def _heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic distance between nodes"""
        return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    def _find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding implementation"""
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next_node in self._get_neighbors(current):
                # Calculate new cost (diagonal movement costs more)
                dx = abs(next_node[0] - current[0])
                dy = abs(next_node[1] - current[1])
                movement_cost = 1.4 if dx + dy == 2 else 1.0
                new_cost = cost_so_far[current] + movement_cost

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self._heuristic(next_node, goal)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        # Reconstruct path
        if goal not in came_from:
            return None

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()

        return path

    def _smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Simple path smoothing to reduce sharp turns"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_point = path[i + 1]

            # Check if we can skip this point
            dx1 = curr[0] - prev[0]
            dy1 = curr[1] - prev[1]
            dx2 = next_point[0] - curr[0]
            dy2 = next_point[1] - curr[1]

            # If direction changes significantly, keep the point
            if dx1 != dx2 or dy1 != dy2:
                smoothed.append(curr)

        smoothed.append(path[-1])
        return smoothed

    async def navigate_to_goal(self, target_x: float, target_y: float):
        """Main navigation loop with dynamic replanning"""
        self.is_navigating = True
        self.target_x = target_x
        self.target_y = target_y

        while self.is_navigating:
            current_time = time.time()

            # Get current position in grid coordinates
            current_pos = self.picar.get_position()
            start_grid_x, start_grid_y = self.world_map.world_to_grid(
                current_pos['x'], current_pos['y'])
            goal_grid_x, goal_grid_y = self.world_map.world_to_grid(
                target_x, target_y)

            # Check if we've reached the goal
            distance_to_goal = math.sqrt(
                (current_pos['x'] - target_x) ** 2 +
                (current_pos['y'] - target_y) ** 2)
            if distance_to_goal < 5:  # 5cm threshold
                print("Reached goal!")
                self.is_navigating = False
                self.picar.stop()
                break

            # Replan if needed
            if (current_time - self.last_plan_time > self.replan_interval or
                    not self.current_path or
                    self.current_path_index >= len(self.current_path)):

                # Find new path
                new_path = self._find_path(
                    (start_grid_x, start_grid_y),
                    (goal_grid_x, goal_grid_y)
                )

                if new_path is None:
                    print("No valid path found!")
                    self.is_navigating = False
                    self.picar.stop()
                    break

                # Smooth and update path
                self.current_path = self._smooth_path(new_path)
                self.current_path_index = 0
                self.last_plan_time = current_time

            # Get next few waypoints to follow
            current_segment = self.current_path[
                              self.current_path_index:
                              self.current_path_index + self.path_segment_length]

            # Follow path segment
            for waypoint in current_segment:
                # Convert grid coordinates back to world coordinates
                way_x, way_y = self.world_map.grid_to_world(*waypoint)

                # Check for obstacles before moving
                vision_clear = await self._check_vision_obstacles()
                if not vision_clear:
                    continue

                # Navigate to waypoint
                print(f"Moving to waypoint: ({way_x:.1f}, {way_y:.1f})")
                await self.picar.navigate_to_point(way_x, way_y)
                self.current_path_index += 1

            await asyncio.sleep(0.1)

    async def _check_vision_obstacles(self) -> bool:
        """Check for obstacles detected by vision system"""
        detected_objects = self.picar.vision.get_obstacle_info()
        if not detected_objects:
            return True

        for obj in detected_objects:
            if obj['label'] == "stop sign" and not self.stop_sign_pause:
                print("Stop sign detected - pausing for 3 seconds")
                self.picar.stop()
                self.stop_sign_pause = True
                await asyncio.sleep(3)
                self.stop_sign_pause = False
                return True

            # Wait for dynamic obstacles to clear
            if obj['label'] in ['person', 'cat']:
                print(f"Waiting for {obj['label']} to clear path")
                self.picar.stop()
                return False

        return True

    def stop_navigation(self):
        """Stop current navigation"""
        self.is_navigating = False
        self.current_path = []
        self.current_path_index = 0
        self.picar.stop()