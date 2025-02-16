import heapq
from typing import List, Tuple, Optional, Dict
from world_map import WorldMap


class PathPlanner:
    def __init__(self, world_map: WorldMap):
        self.world_map = world_map
        self.path: List[Tuple[int, int]] = []
        self.path_index = 0
        self.replanning_threshold = 10  # Number of steps before replanning
        self.steps_since_replan = 0

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
        x, y = pos
        neighbors = []

        # Check 8 surrounding cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy

            # Check bounds
            if (0 <= new_x < self.world_map.grid_size and
                    0 <= new_y < self.world_map.grid_size):
                # Check if cell is obstacle-free
                if self.world_map.grid[new_y, new_x] == 0:
                    neighbors.append((new_x, new_y))

        return neighbors

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding algorithm"""
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        cost_so_far: Dict[Tuple[int, int], float] = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next_pos in self.get_neighbors(current):
                # Calculate movement cost (diagonal movement costs more)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                movement_cost = 1.4 if dx + dy == 2 else 1.0

                new_cost = cost_so_far[current] + movement_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

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

    def plan_path(self, start_x: float, start_y: float, goal_x: float, goal_y: float) -> Optional[
        List[Tuple[float, float]]]:
        """Plan a path from current position to goal in world coordinates"""
        # Convert world coordinates to grid coordinates
        start_grid = self.world_map.world_to_grid(start_x, start_y)
        goal_grid = self.world_map.world_to_grid(goal_x, goal_y)

        # Find path in grid coordinates
        grid_path = self.a_star(start_grid, goal_grid)

        if grid_path is None:
            return None

        # Convert back to world coordinates
        world_path = []
        for grid_x, grid_y in grid_path:
            world_x, world_y = self.world_map.grid_to_world(grid_x, grid_y)
            world_path.append((world_x, world_y))

        self.path = world_path
        self.path_index = 0
        return world_path

    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """Get next waypoint from current path"""
        if not self.path or self.path_index >= len(self.path):
            return None

        waypoint = self.path[self.path_index]
        self.path_index += 1
        self.steps_since_replan += 1

        return waypoint

    def should_replan(self) -> bool:
        """Check if we should replan the path"""
        return self.steps_since_replan >= self.replanning_threshold

    def reset_replan_counter(self):
        """Reset the replanning counter"""
        self.steps_since_replan = 0