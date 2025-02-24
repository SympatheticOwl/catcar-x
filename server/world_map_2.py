import asyncio
import math
import time
import numpy as np
from typing import Tuple, Dict, List
from scipy import ndimage
from picarx_wrapper import PicarXWrapper
from state_handler import State


class WorldMap2:
    def __init__(self, state, map_size: int = 200, resolution: float = 1.0):
        """
        Initialize world map with improved sensor integration

        Args:
            state: State instance containing car configuration
            map_size: Size of map in cm
            resolution: cm per grid cell (1.0 = 1cm per cell)
        """
        self.state = state
        self.resolution = resolution
        self.grid_size = int(map_size / resolution)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        # Center origin
        self.origin = np.array([self.grid_size // 2, self.grid_size // 2])

        # Sensor configuration
        self.max_sensor_range = 300  # Maximum reliable sensor range in cm
        self.min_sensor_range = 25  # Minimum reliable sensor range in cm

        # For confidence tracking
        self.confidence_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.confidence_decay = 0.95  # Decay factor for old readings
        self.min_confidence = 0.3  # Minimum confidence to keep obstacle

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (cm) to grid coordinates"""
        grid_x = int(x / self.resolution) + self.origin[0]
        grid_y = int(y / self.resolution) + self.origin[1]
        return self._clamp_coordinates(grid_x, grid_y)

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates (cm)"""
        x = (grid_x - self.origin[0]) * self.resolution
        y = (grid_y - self.origin[1]) * self.resolution
        return x, y

    def polar_to_cartesian(self, distance: float, angle_deg: float) -> Tuple[float, float]:
        """
        Convert polar coordinates (distance, angle) to cartesian coordinates (x, y)

        Args:
            distance: Distance from origin in cm
            angle_deg: Angle in degrees (0 is forward, positive clockwise)

        Returns:
            Tuple of (x, y) coordinates in cm
        """
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return x, y

    def _clamp_coordinates(self, x: int, y: int) -> Tuple[int, int]:
        """Ensure coordinates stay within grid bounds"""
        x = max(0, min(x, self.grid_size - 1))
        y = max(0, min(y, self.grid_size - 1))
        return x, y

    def _connect_points(self, x1: int, y1: int, x2: int, y2: int):
        """Connect two points with a line of obstacles using Bresenham's algorithm"""
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        steep = dy > dx

        if steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        dx = x2 - x1
        dy = abs(y2 - y1)
        error = dx // 2
        y = y1
        y_step = 1 if y1 < y2 else -1

        for x in range(x1, x2 + 1):
            if steep:
                self._set_obstacle(y, x)
            else:
                self._set_obstacle(x, y)

            error -= dy
            if error < 0:
                y += y_step
                error += dx

    def _set_obstacle(self, x: int, y: int, confidence: float = 1.0):
        """Set a grid cell as containing an obstacle"""
        x, y = self._clamp_coordinates(x, y)
        self.confidence_grid[y, x] = confidence
        self.grid[y, x] = 1

    def update_from_sensor_reading(self, distance: float, angle: float):
        """
        Update map based on a single ultrasonic sensor reading

        Args:
            distance: Distance reading in cm
            angle: Sensor angle in degrees (0 is forward)
        """
        if not self.min_sensor_range <= distance <= self.max_sensor_range:
            return

        # Convert sensor reading to cartesian coordinates
        x, y = self.polar_to_cartesian(distance, angle)

        # Convert to grid coordinates
        grid_x, grid_y = self.world_to_grid(x, y)
        center_x, center_y = self.origin

        # Connect the detected obstacle point with neighboring points
        # to represent continuous surfaces
        if hasattr(self, 'last_grid_x') and hasattr(self, 'last_grid_y'):
            # Only connect if points are reasonably close
            point_distance = math.sqrt((grid_x - self.last_grid_x) ** 2 +
                                       (grid_y - self.last_grid_y) ** 2)
            if point_distance < 10:  # Adjust threshold as needed
                self._connect_points(self.last_grid_x, self.last_grid_y,
                                     grid_x, grid_y)

        # Store current point for next reading
        self.last_grid_x = grid_x
        self.last_grid_y = grid_y

        # Mark the obstacle
        self._set_obstacle(grid_x, grid_y)

    def _update_confidence(self, grid_x: int, grid_y: int, confidence: float):
        """Update confidence value for a grid cell"""
        # Apply decay to existing confidence
        self.confidence_grid *= self.confidence_decay

        # Update with new reading
        self.confidence_grid[grid_y, grid_x] = max(
            self.confidence_grid[grid_y, grid_x],
            confidence
        )

        # Update obstacle grid based on confidence
        self.grid[grid_y, grid_x] = self.confidence_grid[grid_y, grid_x] > self.min_confidence

    def _interpolate_to_point(self, start_x: int, start_y: int, end_x: int, end_y: int):
        """Interpolate between two points using Bresenham's line algorithm"""
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        steep = dy > dx

        if steep:
            start_x, start_y = start_y, start_x
            end_x, end_y = end_y, end_x

        if start_x > end_x:
            start_x, end_x = end_x, start_x
            start_y, end_y = end_y, start_y

        dx = end_x - start_x
        dy = abs(end_y - start_y)
        error = dx // 2
        y = start_y
        y_step = 1 if start_y < end_y else -1

        # Mark empty space along the line
        for x in range(start_x, end_x):
            if steep:
                self._mark_empty(y, x)
            else:
                self._mark_empty(x, y)

            error -= dy
            if error < 0:
                y += y_step
                error += dx

    def _mark_empty(self, x: int, y: int):
        """Mark a grid cell as empty space"""
        x, y = self._clamp_coordinates(x, y)
        # Only mark as empty if confidence is low
        if self.confidence_grid[y, x] < 0.8:
            self.grid[y, x] = 0
            self.confidence_grid[y, x] *= 0.5

    def scan_surroundings(self, sensor_func, angle_range: Tuple[int, int] = (-60, 60),
                          angle_step: int = 5):
        """
        Perform a full scan of surroundings

        Args:
            sensor_func: Function that returns distance reading for given angle
            angle_range: Tuple of (min_angle, max_angle)
            angle_step: Degrees between readings
        """
        # Clear last point tracking at start of new scan
        if hasattr(self, 'last_grid_x'):
            delattr(self, 'last_grid_x')
        if hasattr(self, 'last_grid_y'):
            delattr(self, 'last_grid_y')

        # Perform scan
        angles = range(angle_range[0], angle_range[1] + 1, angle_step)
        for angle in angles:
            distance = sensor_func(angle)
            if distance is not None:
                self.update_from_sensor_reading(distance, angle)

    def get_local_obstacles(self, range_cm: float = 50.0) -> List[Tuple[float, float]]:
        """Get list of obstacle coordinates within range of car"""
        obstacles = []
        range_cells = int(range_cm / self.resolution)

        # Get subsection of grid around car
        x_min = max(0, self.origin[0] - range_cells)
        x_max = min(self.grid_size, self.origin[0] + range_cells)
        y_min = max(0, self.origin[1] - range_cells)
        y_max = min(self.grid_size, self.origin[1] + range_cells)

        # Find obstacle coordinates
        for y in range(y_min, y_max):
            for x in range(x_min, x_max):
                if self.grid[y, x] == 1:
                    world_x, world_y = self.grid_to_world(x, y)
                    obstacles.append((world_x, world_y))

        return obstacles

    def world_map(self):
        print("\nWorld Map (car at center):")
        # Add car marker at center
        temp_grid = self.grid.copy()
        center_x, center_y = self.origin
        temp_grid[center_y, center_x] = 2

        map = []
        # Print grid with different markers
        for row in temp_grid:
            row = ''.join(['C' if cell == 2 else '#' if cell == 1 else '.' for cell in row])
            print(row)
            map.append(row)

        return map

    def visualize(self):
        """Print ASCII visualization of the map with car at center"""
        print("\nWorld Map (car at center):")
        # Add car marker at center
        temp_grid = self.grid.copy()
        center_x, center_y = self.origin
        temp_grid[center_y, center_x] = 2

        # Print grid with different markers
        for row in temp_grid:
            print(''.join(['C' if cell == 2 else '#' if cell == 1 else '.' for cell in row]))