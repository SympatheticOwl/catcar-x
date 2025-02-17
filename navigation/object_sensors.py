import time
from typing import List, Dict, Tuple
import numpy as np
import asyncio
import math

from pathfinder import Pathfinder
from world_map import WorldMap
from picarx_wrapper import PicarXWrapper
from vision_system import VisionSystem

class AsyncObstacleAvoidance:
    def __init__(self):
        # World mapping
        self.px = PicarXWrapper()
        self.world_map = WorldMap(map_size=400, resolution=40.0)  # 4m x 4m map, 1cm resolution
        self.pathfinder = Pathfinder(self.world_map, self.px)

        # Sensor offsets from center
        self.ULTRASONIC_OFFSET_X = 5.0  # cm forward
        self.ULTRASONIC_OFFSET_Y = 0.0  # cm sideways
        self.CAMERA_OFFSET_X = 5.0  # cm forward
        self.CAMERA_OFFSET_Y = 0.0  # cm sideways

        # configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05

        # state management
        self.current_distance = 100
        self.is_moving = False
        self.current_maneuver = None
        self.emergency_stop_flag = False
        self.is_backing_up = False  # since this is async we don't want to interrupt evasive backup maneuvers while obejcts are still too close
        self.is_cliff = False

        # vision
        self.vision = VisionSystem()
        self.vision_enabled = True
        self.vision_clear = True

        # scanning parameters
        self.scan_range = (-60, 60)
        self.scan_step = 5

        self.navigation_target = None
        self.navigation_task = None
        self.is_navigating = False

        self.current_path = []
        self.current_path_index = 0

    def _update_ultrasonic_detection(self, distance: float):
        """Update map with obstacle detected by ultrasonic sensor"""
        if not (0 < distance < 300):  # Ignore invalid readings
            return

        # Calculate obstacle position in world coordinates
        sensor_angle_rad = math.radians(self.px.heading)
        sensor_x = self.px.x + self.ULTRASONIC_OFFSET_X * math.cos(sensor_angle_rad)
        sensor_y = self.px.y + self.ULTRASONIC_OFFSET_X * math.sin(sensor_angle_rad)

        obstacle_x = sensor_x + distance * math.cos(sensor_angle_rad)
        obstacle_y = sensor_y + distance * math.sin(sensor_angle_rad)

        # Add to world map
        self.world_map.add_obstacle(
            x=obstacle_x,
            y=obstacle_y,
            radius=5.0,  # Assume 5cm radius for ultrasonic detections
            confidence=0.8,
            label="ultrasonic_detection"
        )

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            await asyncio.sleep(0.01)

        print(f'distances: {distances}')
        if distances:
            self._update_ultrasonic_detection(sum(distances) / len(distances))
        return distances

    async def scan_environment(self):
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.05)
            await self.scan_avg()

        self.px.set_cam_pan_angle(0)
        return scan_data

    def _polar_to_cartesian(self, angle_deg, distance):
        """Convert polar coordinates to cartesian"""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return np.array([x, y])

    async def ultrasonic_monitoring(self):
        while True:
            distances = await self.scan_avg()
            if distances:
                self.current_distance = sum(distances) / len(distances)

                print(f"Distance: {self.current_distance:.1f} cm")

                # emergency stop if too close during forward movement only
                if (self.current_distance < self.min_distance and
                        self.is_moving and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):  # Don't interrupt backup
                    print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                    await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            self.is_cliff = self.px.get_cliff_status(self.px.get_grayscale_data())

            if (self.is_cliff and
                    self.is_moving and
                    not self.emergency_stop_flag and
                    not self.is_backing_up):
                print(f"Emergency stop, cliff detected!")
                await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)


    async def emergency_stop(self):
        self.emergency_stop_flag = True
        # cancel any ongoing maneuver except backup
        if self.current_maneuver:
            self.current_maneuver.cancel()

        self.is_moving = False
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

    def find_best_direction(self):
        """Analyze scan data to find the best direction to move"""
        max_distance = 0
        best_angle = 0

        # Convert current position to grid coordinates
        curr_pos = self.px.get_position()
        curr_grid_x, curr_grid_y = self.world_map.world_to_grid(curr_pos['x'], curr_pos['y'])

        # Check angles in scan range
        start_angle, end_angle = self.scan_range
        for angle in range(start_angle, end_angle + 1, self.scan_step):
            # Convert angle to radians
            angle_rad = math.radians(angle)

            # Look ahead in this direction (check multiple distances)
            max_check_distance = 100  # cm
            check_step = 5  # cm

            # Find distance to first obstacle in this direction
            distance_to_obstacle = max_check_distance

            for dist in range(check_step, max_check_distance, check_step):
                # Calculate point to check in grid coordinates
                check_x = curr_grid_x + int(dist * math.cos(angle_rad) / self.world_map.resolution)
                check_y = curr_grid_y + int(dist * math.sin(angle_rad) / self.world_map.resolution)

                # Ensure within grid bounds
                if (0 <= check_x < self.world_map.grid_size and
                        0 <= check_y < self.world_map.grid_size):

                    # If we hit an obstacle, record distance and stop checking this direction
                    if self.world_map.grid[check_y, check_x] != 0:
                        distance_to_obstacle = dist
                        break

            # Update best direction if this is the clearest path
            if distance_to_obstacle > max_distance:
                max_distance = distance_to_obstacle
                best_angle = angle

        return best_angle, max_distance

    async def evasive_maneuver(self):
        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            await self.scan_environment()
            # add padding once scanning is done
            self.world_map.add_padding()

            best_angle, max_distance = self.find_best_direction()

            # normal evasive maneuver
            print("Backing up...")
            self.is_moving = True
            self.is_backing_up = True
            self.px.backward(self.speed)
            await asyncio.sleep(self.backup_time)
            self.is_backing_up = False

            print(f"Turning to {best_angle}° (clearest path: {max_distance:.1f}cm)")
            self.px.set_dir_servo_angle(best_angle)
            self.px.forward(self.speed)
            await asyncio.sleep(self.turn_time)

            if not self.emergency_stop_flag:
                self.px.set_dir_servo_angle(0)
                self.is_moving = True
                self.px.forward(self.speed)

            self.px.set_dir_servo_angle(0)

        except asyncio.CancelledError:
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            self.is_moving = False
            raise

        finally:
            self.current_maneuver = None

    import numpy as np
    from typing import List, Tuple, Dict, Set
    from heapq import heappush, heappop
    import math
    import asyncio

    class Pathfinder:
        def __init__(self, world_map, picar):
            self.world_map = world_map
            self.picar = picar

            # Pathfinding parameters
            self.directions = [
                (0, 1),  # north
                (1, 1),  # northeast
                (1, 0),  # east
                (1, -1),  # southeast
                (0, -1),  # south
                (-1, -1),  # southwest
                (-1, 0),  # west
                (-1, 1)  # northwest
            ]

            # Cost for diagonal vs straight movement
            self.STRAIGHT_COST = 1.0
            self.DIAGONAL_COST = 1.4

            # Minimum turning radius considerations
            self.MIN_TURN_RADIUS = self.picar.get_min_turn_radius()
            self.GRID_CELL_SIZE = self.world_map.resolution
            self.MIN_TURN_RADIUS_CELLS = int(self.MIN_TURN_RADIUS / self.GRID_CELL_SIZE)

        def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
            """Calculate heuristic distance between two points"""
            return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

        def get_movement_cost(self, current: Tuple[int, int], next_pos: Tuple[int, int],
                              came_from: Dict[Tuple[int, int], Tuple[int, int]]) -> float:
            """Calculate movement cost considering turning radius"""
            # Base cost (straight or diagonal)
            dx = abs(next_pos[0] - current[0])
            dy = abs(next_pos[1] - current[1])
            base_cost = self.DIAGONAL_COST if dx + dy == 2 else self.STRAIGHT_COST

            # Check if we need to consider turning radius
            if current in came_from and came_from[current] is not None:
                prev = came_from[current]
                # Calculate angles
                try:
                    prev_angle = math.atan2(current[1] - prev[1], current[0] - prev[0])
                    next_angle = math.atan2(next_pos[1] - current[1], next_pos[0] - current[0])
                    angle_diff = abs(math.degrees(next_angle - prev_angle))

                    # Add turning penalty if turn is too sharp
                    if angle_diff > 45:  # Sharp turn threshold
                        required_radius = self.MIN_TURN_RADIUS_CELLS
                        actual_radius = math.sqrt((next_pos[0] - current[0]) ** 2 +
                                                  (next_pos[1] - current[1]) ** 2)
                        if actual_radius < required_radius:
                            base_cost *= 2.0  # Penalty for sharp turns
                except (TypeError, ValueError):
                    # If there's any issue calculating angles, just return base cost
                    pass

            return base_cost

        def is_valid_position(self, pos: Tuple[int, int]) -> bool:
            """Check if a position is valid (within bounds and not obstacle)"""
            x, y = pos
            if 0 <= x < self.world_map.grid_size and 0 <= y < self.world_map.grid_size:
                return self.world_map.grid[y, x] == 0
            return False

        def find_path(self, start_world: Tuple[float, float],
                      goal_world: Tuple[float, float]) -> List[Tuple[float, float]]:
            """Find path from start to goal in world coordinates"""
            # Convert world coordinates to grid coordinates
            start_grid = self.world_map.world_to_grid(start_world[0], start_world[1])
            goal_grid = self.world_map.world_to_grid(goal_world[0], goal_world[1])

            # Initialize data structures
            frontier = []
            heappush(frontier, (0, start_grid))
            came_from = {start_grid: None}
            cost_so_far = {start_grid: 0}

            found_path = False
            while frontier:
                current = heappop(frontier)[1]

                if current == goal_grid:
                    found_path = True
                    break

                # Check all neighboring cells
                for dx, dy in self.directions:
                    next_pos = (current[0] + dx, current[1] + dy)

                    if not self.is_valid_position(next_pos):
                        continue

                    # Calculate new cost
                    new_cost = cost_so_far[current] + self.get_movement_cost(current, next_pos, came_from)

                    if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                        cost_so_far[next_pos] = new_cost
                        priority = new_cost + self.heuristic(goal_grid, next_pos)
                        heappush(frontier, (priority, next_pos))
                        came_from[next_pos] = current

            if not found_path:
                return []

            # Reconstruct path
            path_grid = []
            current = goal_grid
            while current is not None:
                path_grid.append(current)
                current = came_from[current]
            path_grid.reverse()

            # Convert back to world coordinates
            path_world = [self.world_map.grid_to_world(x, y) for x, y in path_grid]

            # Smooth path
            return self.smooth_path(path_world)

        def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
            """Smooth the path to make it more drivable"""
            if len(path) <= 2:
                return path

            smoothed = [path[0]]
            current_idx = 0

            while current_idx < len(path) - 1:
                # Look ahead to find furthest point we can directly reach
                for lookahead in range(len(path) - 1, current_idx, -1):
                    # Check if direct path to lookahead point is clear
                    start = path[current_idx]
                    end = path[lookahead]

                    if self.is_path_clear(start, end):
                        smoothed.append(end)
                        current_idx = lookahead
                        break
                else:
                    # If no clear path found, add next point
                    current_idx += 1
                    smoothed.append(path[current_idx])

            return smoothed

        def is_path_clear(self, start: Tuple[float, float],
                          end: Tuple[float, float]) -> bool:
            """Check if direct path between points is clear of obstacles"""
            # Convert to grid coordinates
            start_grid = self.world_map.world_to_grid(start[0], start[1])
            end_grid = self.world_map.world_to_grid(end[0], end[1])

            # Get points along line
            points = self.get_line_points(start_grid, end_grid)

            # Check each point
            return all(self.is_valid_position(point) for point in points)

        def get_line_points(self, start: Tuple[int, int],
                            end: Tuple[int, int]) -> List[Tuple[int, int]]:
            """Get all grid points along a line using Bresenham's algorithm"""
            x1, y1 = start
            x2, y2 = end
            points = []

            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            x, y = x1, y1
            sx = 1 if x1 < x2 else -1
            sy = 1 if y1 < y2 else -1

            if dx > dy:
                err = dx / 2.0
                while x != x2:
                    points.append((x, y))
                    err -= dy
                    if err < 0:
                        y += sy
                        err += dx
                    x += sx
            else:
                err = dy / 2.0
                while y != y2:
                    points.append((x, y))
                    err -= dx
                    if err < 0:
                        x += sx
                        err += dy
                    y += sy

            points.append((x, y))
            return points

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            # Add vision task to existing tasks
            pos_track_task = asyncio.create_task(self.px.continuous_position_tracking())
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            # navigation_task = asyncio.create_task(self.px.navigate_to_point(100, 50))
            navigation_task = asyncio.create_task(self.navigate_to_target(100, 50))

            tasks = [pos_track_task, vision_task, ultrasonic_task, cliff_task, navigation_task]
            await asyncio.gather(navigation_task)
        except asyncio.CancelledError:
            print("\nShutting down gracefully...")
        finally:
            for task in tasks:
                task.cancel()
            try:
                await asyncio.gather(*tasks, return_exceptions=True)
            except asyncio.CancelledError:
                pass

            self.vision.cleanup()
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            print("Shutdown complete")