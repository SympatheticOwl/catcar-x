import numpy as np
import math
import base64
import io
from typing import Dict, List, Tuple, Callable, Optional, Any
# Set matplotlib to use non-interactive Agg backend
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from state_handler import State


class WorldMap:
    def __init__(self, state: State, grid_size: int = 101, grid_resolution: float = 2.0):
        """
        Initialize world map grid for object tracking

        Args:
            state: Shared state object
            grid_size: Size of the grid (odd number recommended)
            grid_resolution: Resolution of grid in cm per cell
        """
        self.__state = state
        self.grid_size = grid_size
        self.grid_resolution = grid_resolution

        # Ensure grid_size is odd so car is at exact center
        if self.grid_size % 2 == 0:
            self.grid_size += 1

        # Initialize grid (0 = unknown, 1 = free space, 2 = object)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        # Center of the grid (coordinates of the car)
        self.center = self.grid_size // 2

        # Tracking for latest scan data
        self.last_scan_points = []
        self.detected_objects = []

        # The car's representation on the grid
        self.car_length = 20  # cm
        self.car_width = 15  # cm

        # Cell markers
        self.UNKNOWN = 0
        self.FREE = 1
        self.OBJECT = 2
        self.CLIFF = 3
        self.CAR = 4

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates (cm) to grid coordinates

        Args:
            world_x: X coordinate in world space (cm)
            world_y: Y coordinate in world space (cm)

        Returns:
            Tuple of grid coordinates (row, col)
        """
        # Calculate grid coordinates (car is at center)
        grid_x = self.center + int(world_x / self.grid_resolution)
        grid_y = self.center - int(world_y / self.grid_resolution)  # Y-axis flipped in grid

        # Constrain to grid boundaries
        grid_x = max(0, min(grid_x, self.grid_size - 1))
        grid_y = max(0, min(grid_y, self.grid_size - 1))

        return grid_y, grid_x  # Return as (row, col)

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates (cm)

        Args:
            row: Row in grid
            col: Column in grid

        Returns:
            Tuple of world coordinates (x, y)
        """
        world_x = (col - self.center) * self.grid_resolution
        world_y = (self.center - row) * self.grid_resolution

        return world_x, world_y

    def add_object_point(self, angle: float, distance: float):
        """
        Add a detected object point to the grid

        Args:
            angle: Angle in degrees (0 is front, positive is right)
            distance: Distance in cm
        """
        # Convert to world coordinates from polar (relative to car)
        heading_rad = math.radians(self.__state.heading)
        angle_rad = math.radians(angle)
        total_angle = heading_rad + angle_rad

        # Account for ultrasonic sensor position offset
        sensor_x = self.__state.ULTRASONIC_OFFSET_X
        sensor_y = self.__state.ULTRASONIC_OFFSET_Y

        # Calculate object position relative to car
        rel_x = sensor_x + distance * math.cos(total_angle)
        rel_y = sensor_y + distance * math.sin(total_angle)

        # Convert to absolute world coordinates
        world_x = self.__state.x + rel_x
        world_y = self.__state.y + rel_y

        # Convert to grid coordinates and mark as object
        grid_row, grid_col = self.world_to_grid(world_x, world_y)
        self.grid[grid_row, grid_col] = self.OBJECT

        # Add to detected points
        self.last_scan_points.append({
            'angle': angle,
            'distance': distance,
            'world_x': world_x,
            'world_y': world_y,
            'grid_row': grid_row,
            'grid_col': grid_col
        })

    def interpolate_between_points(self):
        """
        Interpolate objects between detected points with similar distances
        """
        if len(self.last_scan_points) < 2:
            return

        # Sort points by angle
        sorted_points = sorted(self.last_scan_points, key=lambda p: p['angle'])

        # Interpolate between consecutive points
        for i in range(len(sorted_points) - 1):
            p1 = sorted_points[i]
            p2 = sorted_points[i + 1]

            # Check if points are close enough in angle and distance to be considered same object
            angle_diff = abs(p2['angle'] - p1['angle'])
            distance_diff = abs(p2['distance'] - p1['distance'])

            # If points are close enough, interpolate between them
            if angle_diff < 15 and distance_diff < 20:  # 15 degrees and 20cm thresholds
                row1, col1 = p1['grid_row'], p1['grid_col']
                row2, col2 = p2['grid_row'], p2['grid_col']

                # Use Bresenham's line algorithm to interpolate
                for point in self.bresenham_line(row1, col1, row2, col2):
                    row, col = point
                    if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                        self.grid[row, col] = self.OBJECT

    def bresenham_line(self, row1: int, col1: int, row2: int, col2: int) -> List[Tuple[int, int]]:
        """
        Bresenham's line algorithm for grid interpolation

        Args:
            row1, col1: Starting point
            row2, col2: Ending point

        Returns:
            List of points (row, col) along the line
        """
        points = []

        dx = abs(col2 - col1)
        dy = -abs(row2 - row1)

        sx = 1 if col1 < col2 else -1
        sy = 1 if row1 < row2 else -1

        err = dx + dy

        while True:
            points.append((row1, col1))

            if row1 == row2 and col1 == col2:
                break

            e2 = 2 * err

            if e2 >= dy:
                if col1 == col2:
                    break
                err += dy
                col1 += sx

            if e2 <= dx:
                if row1 == row2:
                    break
                err += dx
                row1 += sy

        return points

    def mark_free_space(self):
        """
        Mark space between car and detected objects as free
        """
        car_row, car_col = self.world_to_grid(self.__state.x, self.__state.y)

        # For each object point
        for point in self.last_scan_points:
            obj_row, obj_col = point['grid_row'], point['grid_col']

            # Interpolate between car and object
            for row, col in self.bresenham_line(car_row, car_col, obj_row, obj_col)[:-1]:
                if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    if self.grid[row, col] == self.UNKNOWN:
                        self.grid[row, col] = self.FREE

    async def scan_surroundings(self, sensor_func: Callable):
        """
        Scan surroundings and update world map

        Args:
            sensor_func: Async function that measures distance at a given angle
        """
        # Clear last scan points
        self.last_scan_points = []

        # Scan range
        scan_range = range(self.__state.scan_range[0], self.__state.scan_range[1] + 1, self.__state.scan_step)

        # Perform scan and add points to the map
        for angle in scan_range:
            distance = await sensor_func(angle)
            if distance and distance < 300:  # Valid reading
                self.add_object_point(angle, distance)

        # Interpolate between points and mark free space
        self.interpolate_between_points()
        self.mark_free_space()

        # Add car to the grid
        self.update_car_position()

    def update_car_position(self):
        """
        Update car's position in the grid based on current state
        """
        # Get car's current grid position
        car_row, car_col = self.world_to_grid(self.__state.x, self.__state.y)

        # Calculate car's bounding box in grid
        car_length_cells = int(self.car_length / self.grid_resolution)
        car_width_cells = int(self.car_width / self.grid_resolution)

        # Calculate orientation
        heading_rad = math.radians(self.__state.heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        # Calculate corners of car bounding box (simplified)
        half_length = car_length_cells // 2
        half_width = car_width_cells // 2

        for dy in range(-half_width, half_width + 1):
            for dx in range(-half_length, half_length + 1):
                # Rotate coordinates based on heading
                rot_dx = dx * cos_h - dy * sin_h
                rot_dy = dx * sin_h + dy * cos_h

                # Calculate grid position
                grid_row = int(car_row + rot_dy)
                grid_col = int(car_col + rot_dx)

                # Check if within grid bounds
                if 0 <= grid_row < self.grid_size and 0 <= grid_col < self.grid_size:
                    # Only mark as car if not already an object
                    if self.grid[grid_row, grid_col] != self.OBJECT:
                        self.grid[grid_row, grid_col] = self.CAR

    def get_ascii_map(self) -> str:
        """
        Get ASCII representation of the world map

        Returns:
            ASCII map as a string
        """
        # Symbols for different cell types
        symbols = {
            self.UNKNOWN: ' ',  # Unknown
            self.FREE: '.',  # Free space
            self.OBJECT: 'X',  # Object
            self.CLIFF: '!',  # Cliff
            self.CAR: 'C'  # Car
        }

        # Get current grid representation
        grid_copy = self.grid.copy()
        self.update_car_position()  # Ensure car is shown

        # Build ASCII map
        ascii_map = ""
        for row in range(self.grid_size):
            line = ""
            for col in range(self.grid_size):
                line += symbols[grid_copy[row, col]]
            ascii_map += line + "\n"

        return ascii_map

    def visualize(self, return_image: bool = False) -> Dict[str, Any]:
        """
        Visualize the world map using matplotlib

        Args:
            return_image: Whether to return a base64-encoded image

        Returns:
            Dictionary with visualization data
        """
        # Create a colormap
        colors = ['lightgray', 'white', 'red', 'black', 'green']
        cmap = LinearSegmentedColormap.from_list('world_map', colors, N=5)

        # Update car position
        self.update_car_position()

        # Create figure and plot
        fig, ax = plt.subplots(figsize=(10, 10))
        im = ax.imshow(self.grid, cmap=cmap, vmin=0, vmax=4)

        # Add car position marker
        car_row, car_col = self.world_to_grid(self.__state.x, self.__state.y)
        ax.plot(car_col, car_row, 'bo', markersize=10)

        # Draw car heading
        heading_rad = math.radians(self.__state.heading)
        heading_x = math.cos(heading_rad) * 5
        heading_y = -math.sin(heading_rad) * 5  # Negative because y-axis is flipped in grid
        ax.arrow(car_col, car_row, heading_x, heading_y,
                 head_width=2, head_length=2, fc='blue', ec='blue')

        # Add grid
        ax.grid(True, which='both', linestyle='-', linewidth=0.5, alpha=0.3)

        # Add labels and title
        ax.set_title(
            f'World Map (Position: {self.__state.x:.1f}, {self.__state.y:.1f}, Heading: {self.__state.heading:.1f}°)')

        # Add color bar
        cbar = plt.colorbar(im, ax=ax, ticks=[0, 1, 2, 3, 4])
        cbar.set_ticklabels(['Unknown', 'Free', 'Object', 'Cliff', 'Car'])

        # Add grid coordinates
        ax.set_xticks(np.arange(0, self.grid_size, 10))
        ax.set_yticks(np.arange(0, self.grid_size, 10))

        # If requested, convert to base64 for web display
        img_data = None
        if return_image:
            buf = io.BytesIO()
            plt.savefig(buf, format='png')
            buf.seek(0)
            img_data = base64.b64encode(buf.read()).decode('utf-8')
            plt.close(fig)

        return {
            'grid_data': self.grid.tolist(),
            'plot': img_data
        }