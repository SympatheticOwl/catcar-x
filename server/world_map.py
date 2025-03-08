import numpy as np
import math
import base64
import io
from typing import Dict, List, Tuple, Callable
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from state_handler import State


class WorldMap:
    def __init__(self, state: State, grid_size: int = 400, resolution: float = 5.0):
        """
        Initialize world map grid for object tracking

        Args:
            state: Shared state object
            grid_size: Size of the grid (grid_size x grid_size)
            resolution: Resolution of grid in cm per cell
        """
        self.__state = state
        self.grid_size = grid_size
        self.resolution = resolution

        # Create a grid centered at (grid_size/2, grid_size/2)
        # 0 = unknown/free, 1 = obstacle
        self.grid = np.zeros((grid_size, grid_size), dtype=np.uint8)

        # Center point (car's position in grid coordinates)
        self.center_x = grid_size // 2
        self.center_y = grid_size // 2

        # Track scanned points for visualization
        self.scanned_points = []

        # Object detection threshold (cm)
        self.object_threshold = 100  # Maximum distance to track objects

    def clear_grid(self):
        """Reset the grid to all zeros"""
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.scanned_points = []

    def polar_to_cartesian(self, angle: float, distance: float) -> Tuple[float, float]:
        """
        Convert polar coordinates (angle, distance) to cartesian coordinates

        Args:
            angle: Angle in degrees (0 is front, positive is right)
            distance: Distance in cm

        Returns:
            (x, y) coordinates in cm
        """
        # Convert angle to radians
        angle_rad = math.radians(angle)

        # Calculate cartesian coordinates
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)

        return x, y

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid indices

        Args:
            world_x: X coordinate in world space (cm)
            world_y: Y coordinate in world space (cm)

        Returns:
            (row, col) grid indices
        """
        # Convert to grid coordinates (car is at center)
        col = self.center_x + int(world_x / self.resolution)
        row = self.center_y - int(world_y / self.resolution)  # Y-axis is flipped in grid

        # Ensure within grid bounds
        col = max(0, min(col, self.grid_size - 1))
        row = max(0, min(row, self.grid_size - 1))

        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert grid indices to world coordinates

        Args:
            row: Grid row
            col: Grid column

        Returns:
            (x, y) world coordinates in cm
        """
        world_x = (col - self.center_x) * self.resolution
        world_y = (self.center_y - row) * self.resolution

        return world_x, world_y

    def mark_obstacle(self, angle: float, distance: float):
        """
        Mark an obstacle on the grid

        Args:
            angle: Angle in degrees (0 is front, positive is right)
            distance: Distance in cm
        """
        # Only mark obstacles within threshold distance
        if distance > self.object_threshold:
            return

        # Convert to world coordinates (relative to car)
        world_x, world_y = self.polar_to_cartesian(angle, distance)

        # Account for car's position and orientation
        heading_rad = math.radians(self.__state.heading)
        rotated_x = world_x * math.cos(heading_rad) - world_y * math.sin(heading_rad)
        rotated_y = world_x * math.sin(heading_rad) + world_y * math.cos(heading_rad)

        # Add car's position offset
        abs_x = self.__state.x + rotated_x
        abs_y = self.__state.y + rotated_y

        # Convert to grid coordinates
        row, col = self.world_to_grid(abs_x, abs_y)

        # Mark as obstacle in grid
        self.grid[row, col] = 1

        # Store point for visualization
        self.scanned_points.append({
            'angle': angle,
            'distance': distance,
            'world_x': abs_x,
            'world_y': abs_y,
            'grid_row': row,
            'grid_col': col
        })

    def interpolate_obstacles(self):
        """
        Interpolate between scanned points to create continuous obstacles
        """
        if len(self.scanned_points) < 2:
            return

        # Sort points by angle
        sorted_points = sorted(self.scanned_points, key=lambda p: p['angle'])

        # Interpolate between consecutive points
        for i in range(len(sorted_points) - 1):
            p1 = sorted_points[i]
            p2 = sorted_points[i + 1]

            # Check if points are close enough in angle and distance
            angle_diff = abs(p2['angle'] - p1['angle'])
            distance_diff = abs(p2['distance'] - p1['distance'])

            # Only interpolate between points that are likely part of the same object
            # and not too far apart in angle
            if angle_diff <= 10 and distance_diff < 20:
                # Use Bresenham's line algorithm to interpolate
                line_points = self.bresenham_line(
                    p1['grid_row'], p1['grid_col'],
                    p2['grid_row'], p2['grid_col']
                )

                # Mark all points on the line as obstacles
                for row, col in line_points:
                    if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                        self.grid[row, col] = 1

    def bresenham_line(self, row1: int, col1: int, row2: int, col2: int) -> List[Tuple[int, int]]:
        """
        Bresenham's line algorithm to find grid cells on a line

        Args:
            row1, col1: Starting point
            row2, col2: Ending point

        Returns:
            List of (row, col) points on the line
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

    async def scan_surroundings(self, sensor_func: Callable):
        """
        Scan surroundings using the ultrasonic sensor

        Args:
            sensor_func: Async function that takes an angle and returns distance
        """
        # Clear previous scan data
        self.clear_grid()

        # Define scan range and step from state
        scan_range = range(self.__state.scan_range[0],
                           self.__state.scan_range[1] + 1,
                           self.__state.scan_step)

        # Perform scan
        for angle in scan_range:
            distance = await sensor_func(angle)

            # Skip invalid readings
            if distance is None or distance <= 0:
                continue

            # Mark obstacle on grid
            self.mark_obstacle(angle, distance)

        # Interpolate to create continuous obstacles
        self.interpolate_obstacles()

    def get_ascii_map(self) -> str:
        """
        Generate ASCII representation of the map

        Returns:
            String representing the grid with car position
        """
        # Get a copy of the grid
        grid_copy = self.grid.copy()

        # Mark car's position
        car_row, car_col = self.world_to_grid(self.__state.x, self.__state.y)
        grid_copy[car_row, car_col] = 2  # 2 represents car

        # Generate ASCII representation
        result = ""
        for row in range(self.grid_size):
            line = ""
            for col in range(self.grid_size):
                if grid_copy[row, col] == 0:
                    line += " "  # Empty/unknown
                elif grid_copy[row, col] == 1:
                    line += "X"  # Obstacle
                elif grid_copy[row, col] == 2:
                    line += "C"  # Car
            result += line + "\n"

        return result

    def visualize(self, return_image: bool = False) -> Dict:
        """
        Visualize the world map

        Args:
            return_image: If True, return base64-encoded image

        Returns:
            Dictionary with grid data and visualization
        """
        # Create a colormap: 0=white (empty), 1=red (obstacle), 2=green (car)
        colors = ['white', 'red', 'green']
        cmap = LinearSegmentedColormap.from_list("world_map", colors, N=3)

        # Create a copy of the grid for visualization
        grid_viz = self.grid.copy()

        # Mark car's position
        car_row, car_col = self.world_to_grid(self.__state.x, self.__state.y)
        grid_viz[car_row, car_col] = 2  # Car is represented as 2

        # Create plot
        fig, ax = plt.subplots(figsize=(8, 8))
        im = ax.imshow(grid_viz, cmap=cmap, vmin=0, vmax=2)

        # Add heading indicator
        heading_rad = math.radians(self.__state.heading)
        dx = 5 * math.cos(heading_rad)
        dy = -5 * math.sin(heading_rad)  # Negative because y-axis is flipped in grid
        ax.arrow(car_col, car_row, dx, dy,
                 head_width=2, head_length=2, fc='blue', ec='blue')

        # Add gridlines
        ax.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.3)

        # Add coordinate axes
        # Mark the grid center (0,0 in world coordinates)
        center_row, center_col = self.world_to_grid(0, 0)
        ax.plot(center_col, center_row, 'bx', markersize=10)

        # Calculate grid for axis labels
        # (show world coordinates, not grid indices)
        x_ticks = np.arange(0, self.grid_size, 10)
        y_ticks = np.arange(0, self.grid_size, 10)
        x_labels = [f"{self.grid_to_world(0, x)[0]:.0f}" for x in x_ticks]
        y_labels = [f"{self.grid_to_world(y, 0)[1]:.0f}" for y in y_ticks]

        ax.set_xticks(x_ticks)
        ax.set_xticklabels(x_labels)
        ax.set_yticks(y_ticks)
        ax.set_yticklabels(y_labels)

        # Add title with car position info
        ax.set_title(
            f"World Map (Pos: {self.__state.x:.1f}, {self.__state.y:.1f}, Heading: {self.__state.heading:.1f}°)")

        # Add legend
        cbar = plt.colorbar(im, ax=ax, ticks=[0, 1, 2])
        cbar.set_ticklabels(['Empty', 'Obstacle', 'Car'])

        # Return as base64 if requested
        img_data = None
        if return_image:
            buf = io.BytesIO()
            plt.savefig(buf, format='png')
            buf.seek(0)
            img_data = base64.b64encode(buf.read()).decode('utf-8')
            plt.close(fig)
        else:
            plt.close(fig)

        return {
            'grid_data': self.grid.tolist(),
            'plot': img_data
        }