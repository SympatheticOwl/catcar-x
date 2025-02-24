import asyncio
import base64
import io
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Dict, List
from scipy import ndimage
from picarx_wrapper import PicarXWrapper
from state_handler import State


class WorldMap2:
    def __init__(self, state: State, map_size: int = 400, resolution: float = 5.0):
        """
        Initialize the world map

        Args:
            state: State object containing car position and heading
            map_size: Size of the map in cm
            resolution: cm per grid cell
        """
        self.state = state
        self.resolution = resolution
        self.grid_size = int(map_size / resolution)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.car_marker = 2  # Use 2 to represent car position in grid

        # Center of grid coordinates
        self.origin = np.array([self.grid_size // 2, self.grid_size // 2])

        # Track scanned areas
        self.scanned_areas = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.scan_times = np.zeros((self.grid_size, self.grid_size))

        # Initialize car position at center
        self.update_car_position()

        # Last scan data for interpolation
        self.last_scan = None  # (angle, distance)

    def polar_to_cartesian(self, distance: float, angle: float) -> Tuple[float, float]:
        """
        Convert polar coordinates (distance, angle) to cartesian (x, y)
        Angle is in degrees, distance in cm
        """
        # Convert angle to radians and adjust for car's heading
        total_angle = math.radians(angle + self.state.heading)

        # Convert to cartesian coordinates
        x = distance * math.cos(total_angle)
        y = distance * math.sin(total_angle)

        return x, y

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (cm) to grid coordinates"""
        # Adjust for car's current position
        rel_x = x + self.state.x
        rel_y = y + self.state.y

        grid_x = int(rel_x / self.resolution) + self.origin[0]
        grid_y = int(rel_y / self.resolution) + self.origin[1]

        # Ensure coordinates are within bounds
        grid_x = max(0, min(grid_x, self.grid_size - 1))
        grid_y = max(0, min(grid_y, self.grid_size - 1))

        return grid_x, grid_y

    def update_car_position(self):
        """Update car position in grid"""
        # Clear previous car position
        self.grid[self.grid == self.car_marker] = 0

        # Convert car's world position to grid coordinates
        car_x, car_y = self.world_to_grid(0, 0)  # Car's position relative to itself

        # Mark car position
        self.grid[car_y, car_x] = self.car_marker

    def add_obstacle_reading(self, distance: float, angle: float):
        """
        Add an obstacle reading from ultrasonic sensor

        Args:
            distance: Distance to obstacle in cm
            angle: Angle of sensor in degrees (0 is forward, positive is clockwise)
        """
        if distance >= self.state.min_distance:
            # Convert polar coordinates to cartesian
            x, y = self.polar_to_cartesian(distance, angle)

            # Convert to grid coordinates
            grid_x, grid_y = self.world_to_grid(x, y)

            # Mark as scanned
            self.mark_scanned_path(angle, distance)

            # Add obstacle to grid
            self.grid[grid_y, grid_x] = 1

            # Update scan times for this point
            self.scan_times[grid_y, grid_x] = time.time()

            # If we have a previous scan, interpolate between them
            if self.last_scan is not None:
                last_angle, last_distance = self.last_scan
                if abs(last_angle - angle) <= self.state.scan_step:  # Only interpolate within scan step
                    self.interpolate_obstacles(last_angle, last_distance, angle, distance)

            # Update last scan
            self.last_scan = (angle, distance)

    def mark_scanned_path(self, angle: float, distance: float):
        """Mark the path of the ultrasonic scan as checked"""
        start_x, start_y = self.world_to_grid(0, 0)  # Car position
        end_x, end_y = self.world_to_grid(*self.polar_to_cartesian(distance, angle))

        # Create points along the scan line
        points = np.linspace([start_x, start_y], [end_x, end_y],
                             num=int(distance / self.resolution))

        current_time = time.time()

        # Mark all points as scanned
        for point in points:
            x, y = int(point[0]), int(point[1])
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                self.scanned_areas[y, x] = 1
                self.scan_times[y, x] = current_time

    def interpolate_obstacles(self, angle1: float, dist1: float,
                              angle2: float, dist2: float):
        """Interpolate obstacles between two readings"""
        # Convert both readings to cartesian coordinates
        x1, y1 = self.polar_to_cartesian(dist1, angle1)
        x2, y2 = self.polar_to_cartesian(dist2, angle2)

        # Calculate number of interpolation points based on distance
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        num_points = int(distance / self.resolution)

        if num_points > 1:  # Only interpolate if points are far enough apart
            # Interpolate points
            x_points = np.linspace(x1, x2, num_points)
            y_points = np.linspace(y1, y2, num_points)

            # Add interpolated obstacles
            current_time = time.time()
            for x, y in zip(x_points, y_points):
                grid_x, grid_y = self.world_to_grid(x, y)
                self.grid[grid_y, grid_x] = 1
                self.scan_times[grid_y, grid_x] = current_time

    # async def scan_surroundings(self, sensor_func, angle_range: Tuple[int, int] = (-60, 60),
    #                       angle_step: int = 5):
    #     """
    #     Perform a full scan of surroundings
    #
    #     Args:
    #         sensor_func: Function that returns distance reading for given angle
    #         angle_range: Tuple of (min_angle, max_angle)
    #         angle_step: Degrees between readings
    #     """
    #     # Clear last point tracking at start of new scan
    #     if hasattr(self, 'last_grid_x'):
    #         delattr(self, 'last_grid_x')
    #     if hasattr(self, 'last_grid_y'):
    #         delattr(self, 'last_grid_y')
    #
    #     # Perform scan
    #     angles = range(angle_range[0], angle_range[1] + 1, angle_step)
    #     for angle in angles:
    #         distance = await sensor_func(angle)
    #         if distance is not None:
    #             self.update_from_sensor_reading(distance, angle)

    async def scan_surroundings(self, sensor_func):
        """
        Perform a complete scan of surroundings using sensor readings

        Args:
            sensor_readings: Dictionary mapping angles to distances
        """
        # Reset last scan for new scanning sequence
        self.last_scan = None

        start_angle, end_angle = self.state.scan_range
        for angle in range(start_angle, end_angle + 1, self.state.scan_step):
            distance = await sensor_func(angle)
            if distance is not None:
                self.add_obstacle_reading(distance, angle)

        # Clear old readings after scan
        self.clear_old_readings()

    def clear_old_readings(self, max_age: float = 5.0):
        """Clear readings older than max_age seconds"""
        current_time = time.time()
        old_areas = (current_time - self.scan_times) > max_age
        self.grid[old_areas & (self.grid == 1)] = 0  # Only clear obstacles, not car position
        self.scanned_areas[old_areas] = 0
        self.scan_times[old_areas] = 0

    def visualize(self, return_image=False):
        """
        Visualize the map with matplotlib

        Args:
            return_image: If True, returns the plot as a base64 string instead of displaying
        """
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid, cmap='gray_r', origin='lower')

        # Highlight car position
        car_pos = np.where(self.grid == self.car_marker)
        if len(car_pos[0]) > 0:
            plt.plot(car_pos[1], car_pos[0], 'ro', markersize=10, label='Car')

        # Add heading indicator
        heading_rad = math.radians(self.state.heading)
        heading_length = 3
        dx = heading_length * math.cos(heading_rad)
        dy = heading_length * math.sin(heading_rad)
        plt.arrow(car_pos[1][0], car_pos[0][0], dx, dy,
                  head_width=1, head_length=1, fc='r', ec='r')

        plt.grid(True)
        plt.colorbar(label='Obstacle (1) / Car (2)')
        plt.title('World Map')
        plt.xlabel('X Grid Position')
        plt.ylabel('Y Grid Position')
        plt.legend()

        if return_image:
            # Save plot to a bytes buffer
            buf = io.BytesIO()
            plt.savefig(buf, format='png', bbox_inches='tight')
            plt.close()  # Close the figure to free memory

            # Encode the bytes as base64
            buf.seek(0)
            image_base64 = base64.b64encode(buf.getvalue()).decode('utf-8')
            buf.close()

            return image_base64
        else:
            plt.show()
            plt.close()  # Close the figure to free memory

    def get_visualization_data(self):
        """Get both the grid data and visualization"""
        return {
            'grid_data': self.get_grid_data(),
            'visualization': self.visualize(return_image=True)
        }

    def ascii_visualize(self):
        """Print ASCII visualization of the map"""
        rows = []
        for row in self.grid:
            rows.append(''.join(['C' if cell == self.car_marker else
                                 '#' if cell == 1 else '.' for cell in row]))
        print('\n'.join(rows))
        return rows

    def get_grid_data(self):
        """Return grid data in a format suitable for API response"""
        return {
            'grid': self.grid.tolist(),
            'car_position': {
                'x': self.state.x,
                'y': self.state.y,
                'heading': self.state.heading
            },
            'resolution': self.resolution,
            'grid_size': self.grid_size,
            'car_marker': self.car_marker
        }

    # def __init__(self, state: State, map_size: int = 400, resolution: float = 5.0):
    #     """
    #     Initialize the world map
    #
    #     Args:
    #         state: State object containing car position and heading
    #         map_size: Size of the map in cm
    #         resolution: cm per grid cell
    #     """
    #     self.state = state
    #     self.resolution = resolution
    #     self.grid_size = int(map_size / resolution)
    #     self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
    #     self.car_marker = 2  # Use 2 to represent car position in grid
    #
    #     # Center of grid coordinates
    #     self.origin = np.array([self.grid_size // 2, self.grid_size // 2])
    #
    #     # Track scanned areas
    #     self.scanned_areas = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
    #
    #     # Initialize car position at center
    #     self.update_car_position()
    #
    # def polar_to_cartesian(self, distance: float, angle: float) -> Tuple[float, float]:
    #     """
    #     Convert polar coordinates (distance, angle) to cartesian (x, y)
    #     Angle is in degrees, distance in cm
    #     """
    #     # Convert angle to radians and adjust for car's heading
    #     total_angle = math.radians(angle + self.state.heading)
    #
    #     # Convert to cartesian coordinates
    #     x = distance * math.cos(total_angle)
    #     y = distance * math.sin(total_angle)
    #
    #     return x, y
    #
    # def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
    #     """Convert world coordinates (cm) to grid coordinates"""
    #     # Adjust for car's current position
    #     rel_x = x + self.state.x
    #     rel_y = y + self.state.y
    #
    #     grid_x = int(rel_x / self.resolution) + self.origin[0]
    #     grid_y = int(rel_y / self.resolution) + self.origin[1]
    #
    #     # Ensure coordinates are within bounds
    #     grid_x = max(0, min(grid_x, self.grid_size - 1))
    #     grid_y = max(0, min(grid_y, self.grid_size - 1))
    #
    #     return grid_x, grid_y
    #
    # def update_car_position(self):
    #     """Update car position in grid"""
    #     # Clear previous car position
    #     self.grid[self.grid == self.car_marker] = 0
    #
    #     # Convert car's world position to grid coordinates
    #     car_x, car_y = self.world_to_grid(0, 0)  # Car's position relative to itself
    #
    #     # Mark car position
    #     self.grid[car_y, car_x] = self.car_marker
    #
    # def add_obstacle_reading(self, distance: float, angle: float):
    #     """
    #     Add an obstacle reading from ultrasonic sensor
    #
    #     Args:
    #         distance: Distance to obstacle in cm
    #         angle: Angle of sensor in degrees (0 is forward, positive is clockwise)
    #     """
    #     if distance >= self.state.min_distance:
    #         # Convert polar coordinates to cartesian
    #         x, y = self.polar_to_cartesian(distance, angle)
    #
    #         # Convert to grid coordinates
    #         grid_x, grid_y = self.world_to_grid(x, y)
    #
    #         # Mark as scanned
    #         self.mark_scanned_path(angle, distance)
    #
    #         # Add obstacle to grid
    #         self.grid[grid_y, grid_x] = 1
    #
    # def mark_scanned_path(self, angle: float, distance: float):
    #     """Mark the path of the ultrasonic scan as checked"""
    #     start_x, start_y = self.world_to_grid(0, 0)  # Car position
    #     end_x, end_y = self.world_to_grid(*self.polar_to_cartesian(distance, angle))
    #
    #     # Create points along the scan line
    #     points = np.linspace([start_x, start_y], [end_x, end_y],
    #                          num=int(distance / self.resolution))
    #
    #     # Mark all points as scanned
    #     for point in points:
    #         x, y = int(point[0]), int(point[1])
    #         if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
    #             self.scanned_areas[y, x] = 1
    #
    # def interpolate_obstacles(self, angle1: float, dist1: float,
    #                           angle2: float, dist2: float):
    #     """Interpolate obstacles between two readings"""
    #     # Convert both readings to cartesian coordinates
    #     x1, y1 = self.polar_to_cartesian(dist1, angle1)
    #     x2, y2 = self.polar_to_cartesian(dist2, angle2)
    #
    #     # Calculate number of interpolation points based on distance
    #     distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    #     num_points = int(distance / self.resolution)
    #
    #     # Interpolate points
    #     x_points = np.linspace(x1, x2, num_points)
    #     y_points = np.linspace(y1, y2, num_points)
    #
    #     # Add interpolated obstacles
    #     for x, y in zip(x_points, y_points):
    #         grid_x, grid_y = self.world_to_grid(x, y)
    #         self.grid[grid_y, grid_x] = 1
    #
    # def clear_old_readings(self, max_age: float = 5.0):
    #     """Clear readings older than max_age seconds"""
    #     current_time = time.time()
    #     areas_to_clear = self.scanned_areas & (current_time - self.scan_times > max_age)
    #     self.grid[areas_to_clear] = 0
    #     self.scanned_areas[areas_to_clear] = 0
    #
    # def visualize(self):
    #     """Visualize the map with matplotlib"""
    #     plt.figure(figsize=(10, 10))
    #     plt.imshow(self.grid, cmap='gray_r', origin='lower')
    #
    #     # Highlight car position
    #     car_pos = np.where(self.grid == self.car_marker)
    #     if len(car_pos[0]) > 0:
    #         plt.plot(car_pos[1], car_pos[0], 'ro', markersize=10, label='Car')
    #
    #     # Add heading indicator
    #     heading_rad = math.radians(self.state.heading)
    #     heading_length = 3
    #     dx = heading_length * math.cos(heading_rad)
    #     dy = heading_length * math.sin(heading_rad)
    #     plt.arrow(car_pos[1][0], car_pos[0][0], dx, dy,
    #               head_width=1, head_length=1, fc='r', ec='r')
    #
    #     plt.grid(True)
    #     plt.colorbar(label='Obstacle (1) / Car (2)')
    #     plt.title('World Map')
    #     plt.xlabel('X Grid Position')
    #     plt.ylabel('Y Grid Position')
    #     plt.legend()
    #     plt.show()
    #
    # def ascii_visualize(self):
    #     """Print ASCII visualization of the map"""
    #     for row in self.grid:
    #         print(''.join(['C' if cell == self.car_marker else
    #                        '#' if cell == 1 else '.' for cell in row]))




    # def __init__(self, state, map_size: int = 200, resolution: float = 1.0):
    #     """
    #     Initialize world map with improved sensor integration
    #
    #     Args:
    #         state: State instance containing car configuration
    #         map_size: Size of map in cm
    #         resolution: cm per grid cell (1.0 = 1cm per cell)
    #     """
    #     self.state = state
    #     self.resolution = resolution
    #     self.grid_size = int(map_size / resolution)
    #     self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
    #
    #     # Center origin
    #     self.origin = np.array([self.grid_size // 2, self.grid_size // 2])
    #
    #     # Sensor configuration
    #     self.max_sensor_range = 300  # Maximum reliable sensor range in cm
    #     self.min_sensor_range = 25  # Minimum reliable sensor range in cm
    #
    #     # For confidence tracking
    #     self.confidence_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
    #     self.confidence_decay = 0.95  # Decay factor for old readings
    #     self.min_confidence = 0.3  # Minimum confidence to keep obstacle
    #
    # def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
    #     """Convert world coordinates (cm) to grid coordinates"""
    #     grid_x = int(x / self.resolution) + self.origin[0]
    #     grid_y = int(y / self.resolution) + self.origin[1]
    #     return self._clamp_coordinates(grid_x, grid_y)
    #
    # def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
    #     """Convert grid coordinates to world coordinates (cm)"""
    #     x = (grid_x - self.origin[0]) * self.resolution
    #     y = (grid_y - self.origin[1]) * self.resolution
    #     return x, y
    #
    # def polar_to_cartesian(self, distance: float, angle_deg: float) -> Tuple[float, float]:
    #     """
    #     Convert polar coordinates (distance, angle) to cartesian coordinates (x, y)
    #
    #     Args:
    #         distance: Distance from origin in cm
    #         angle_deg: Angle in degrees (0 is forward, positive clockwise)
    #
    #     Returns:
    #         Tuple of (x, y) coordinates in cm
    #     """
    #     angle_rad = math.radians(angle_deg)
    #     x = distance * math.cos(angle_rad)
    #     y = distance * math.sin(angle_rad)
    #     return x, y
    #
    # def _clamp_coordinates(self, x: int, y: int) -> Tuple[int, int]:
    #     """Ensure coordinates stay within grid bounds"""
    #     x = max(0, min(x, self.grid_size - 1))
    #     y = max(0, min(y, self.grid_size - 1))
    #     return x, y
    #
    # def _connect_points(self, x1: int, y1: int, x2: int, y2: int):
    #     """Connect two points with a line of obstacles using Bresenham's algorithm"""
    #     dx = abs(x2 - x1)
    #     dy = abs(y2 - y1)
    #     steep = dy > dx
    #
    #     if steep:
    #         x1, y1 = y1, x1
    #         x2, y2 = y2, x2
    #
    #     if x1 > x2:
    #         x1, x2 = x2, x1
    #         y1, y2 = y2, y1
    #
    #     dx = x2 - x1
    #     dy = abs(y2 - y1)
    #     error = dx // 2
    #     y = y1
    #     y_step = 1 if y1 < y2 else -1
    #
    #     for x in range(x1, x2 + 1):
    #         if steep:
    #             self._set_obstacle(y, x)
    #         else:
    #             self._set_obstacle(x, y)
    #
    #         error -= dy
    #         if error < 0:
    #             y += y_step
    #             error += dx
    #
    # def _set_obstacle(self, x: int, y: int, confidence: float = 1.0):
    #     """Set a grid cell as containing an obstacle"""
    #     x, y = self._clamp_coordinates(x, y)
    #     self.confidence_grid[y, x] = confidence
    #     self.grid[y, x] = 1
    #
    # def update_from_sensor_reading(self, distance: float, angle: float):
    #     """
    #     Update map based on a single ultrasonic sensor reading
    #
    #     Args:
    #         distance: Distance reading in cm
    #         angle: Sensor angle in degrees (0 is forward)
    #     """
    #     if not self.min_sensor_range <= distance <= self.max_sensor_range:
    #         return
    #
    #     # Convert sensor reading to cartesian coordinates
    #     x, y = self.polar_to_cartesian(distance, angle)
    #
    #     # Convert to grid coordinates
    #     grid_x, grid_y = self.world_to_grid(x, y)
    #     center_x, center_y = self.origin
    #
    #     # Connect the detected obstacle point with neighboring points
    #     # to represent continuous surfaces
    #     if hasattr(self, 'last_grid_x') and hasattr(self, 'last_grid_y'):
    #         # Only connect if points are reasonably close
    #         point_distance = math.sqrt((grid_x - self.last_grid_x) ** 2 +
    #                                    (grid_y - self.last_grid_y) ** 2)
    #         if point_distance < 10:  # Adjust threshold as needed
    #             self._connect_points(self.last_grid_x, self.last_grid_y,
    #                                  grid_x, grid_y)
    #
    #     # Store current point for next reading
    #     self.last_grid_x = grid_x
    #     self.last_grid_y = grid_y
    #
    #     # Mark the obstacle
    #     self._set_obstacle(grid_x, grid_y)
    #
    # def _update_confidence(self, grid_x: int, grid_y: int, confidence: float):
    #     """Update confidence value for a grid cell"""
    #     # Apply decay to existing confidence
    #     self.confidence_grid *= self.confidence_decay
    #
    #     # Update with new reading
    #     self.confidence_grid[grid_y, grid_x] = max(
    #         self.confidence_grid[grid_y, grid_x],
    #         confidence
    #     )
    #
    #     # Update obstacle grid based on confidence
    #     self.grid[grid_y, grid_x] = self.confidence_grid[grid_y, grid_x] > self.min_confidence
    #
    # def _interpolate_to_point(self, start_x: int, start_y: int, end_x: int, end_y: int):
    #     """Interpolate between two points using Bresenham's line algorithm"""
    #     dx = abs(end_x - start_x)
    #     dy = abs(end_y - start_y)
    #     steep = dy > dx
    #
    #     if steep:
    #         start_x, start_y = start_y, start_x
    #         end_x, end_y = end_y, end_x
    #
    #     if start_x > end_x:
    #         start_x, end_x = end_x, start_x
    #         start_y, end_y = end_y, start_y
    #
    #     dx = end_x - start_x
    #     dy = abs(end_y - start_y)
    #     error = dx // 2
    #     y = start_y
    #     y_step = 1 if start_y < end_y else -1
    #
    #     # Mark empty space along the line
    #     for x in range(start_x, end_x):
    #         if steep:
    #             self._mark_empty(y, x)
    #         else:
    #             self._mark_empty(x, y)
    #
    #         error -= dy
    #         if error < 0:
    #             y += y_step
    #             error += dx
    #
    # def _mark_empty(self, x: int, y: int):
    #     """Mark a grid cell as empty space"""
    #     x, y = self._clamp_coordinates(x, y)
    #     # Only mark as empty if confidence is low
    #     if self.confidence_grid[y, x] < 0.8:
    #         self.grid[y, x] = 0
    #         self.confidence_grid[y, x] *= 0.5
    #
    # async def scan_surroundings(self, sensor_func, angle_range: Tuple[int, int] = (-60, 60),
    #                       angle_step: int = 5):
    #     """
    #     Perform a full scan of surroundings
    #
    #     Args:
    #         sensor_func: Function that returns distance reading for given angle
    #         angle_range: Tuple of (min_angle, max_angle)
    #         angle_step: Degrees between readings
    #     """
    #     # Clear last point tracking at start of new scan
    #     if hasattr(self, 'last_grid_x'):
    #         delattr(self, 'last_grid_x')
    #     if hasattr(self, 'last_grid_y'):
    #         delattr(self, 'last_grid_y')
    #
    #     # Perform scan
    #     angles = range(angle_range[0], angle_range[1] + 1, angle_step)
    #     for angle in angles:
    #         distance = await sensor_func(angle)
    #         if distance is not None:
    #             self.update_from_sensor_reading(distance, angle)
    #
    # def get_local_obstacles(self, range_cm: float = 50.0) -> List[Tuple[float, float]]:
    #     """Get list of obstacle coordinates within range of car"""
    #     obstacles = []
    #     range_cells = int(range_cm / self.resolution)
    #
    #     # Get subsection of grid around car
    #     x_min = max(0, self.origin[0] - range_cells)
    #     x_max = min(self.grid_size, self.origin[0] + range_cells)
    #     y_min = max(0, self.origin[1] - range_cells)
    #     y_max = min(self.grid_size, self.origin[1] + range_cells)
    #
    #     # Find obstacle coordinates
    #     for y in range(y_min, y_max):
    #         for x in range(x_min, x_max):
    #             if self.grid[y, x] == 1:
    #                 world_x, world_y = self.grid_to_world(x, y)
    #                 obstacles.append((world_x, world_y))
    #
    #     return obstacles
    #
    # def world_map(self):
    #     print("\nWorld Map (car at center):")
    #     # Add car marker at center
    #     temp_grid = self.grid.copy()
    #     center_x, center_y = self.origin
    #     temp_grid[center_y, center_x] = 2
    #
    #     map = []
    #     # Print grid with different markers
    #     for row in temp_grid:
    #         row = ''.join(['C' if cell == 2 else '#' if cell == 1 else '.' for cell in row])
    #         print(row)
    #         map.append(row)
    #
    #     return map
    #
    # def visualize(self):
    #     """Print ASCII visualization of the map with car at center"""
    #     print("\nWorld Map (car at center):")
    #     # Add car marker at center
    #     temp_grid = self.grid.copy()
    #     center_x, center_y = self.origin
    #     temp_grid[center_y, center_x] = 2
    #
    #     # Print grid with different markers
    #     for row in temp_grid:
    #         print(''.join(['C' if cell == 2 else '#' if cell == 1 else '.' for cell in row]))