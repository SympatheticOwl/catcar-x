import numpy as np
import math
import matplotlib
# Force matplotlib to use non-interactive backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io
import base64
from matplotlib.colors import LinearSegmentedColormap
from typing import Callable, List, Tuple, Optional, Any, Awaitable
import asyncio

from state_handler import State


class WorldMap:
    def __init__(self, state: State, grid_size: int = 100, cell_size: float = 1.0):
        """
        Initialize the world map

        Args:
            state: State object containing robot's status
            grid_size: Size of the grid (grid_size x grid_size)
            cell_size: Size of each grid cell in cm
        """
        self.state = state
        self.grid_size = grid_size
        self.cell_size = cell_size

        # Create grid with default unknown state (-1)
        # 0 = Free space, 1 = Occupied, -1 = Unknown
        self.grid = np.ones((grid_size, grid_size)) * -1

        # Center of the grid is the origin (0,0)
        self.origin = grid_size // 2

        # Store detected objects with their world coordinates
        self.objects = []

        # Create custom colormap: unknown=grey, free=white, occupied=black
        self.cmap = LinearSegmentedColormap.from_list('worldmap_cmap',
                                                      [(0.5, 0.5, 0.5),  # Unknown (grey)
                                                       (1, 1, 1),  # Free (white)
                                                       (0, 0, 0)])  # Occupied (black)
        self.norm = plt.Normalize(-1, 1)

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates

        Args:
            grid_x: X coordinate in grid
            grid_y: Y coordinate in grid

        Returns:
            (world_x, world_y): Coordinates in world space (cm)
        """
        world_x = (grid_x - self.origin) * self.cell_size
        world_y = (grid_y - self.origin) * self.cell_size
        return world_x, world_y

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates

        Args:
            world_x: X coordinate in world space (cm)
            world_y: Y coordinate in world space (cm)

        Returns:
            (grid_x, grid_y): Coordinates in grid
        """
        grid_x = int(round(world_x / self.cell_size)) + self.origin
        grid_y = int(round(world_y / self.cell_size)) + self.origin

        # Ensure coordinates are within grid bounds
        grid_x = max(0, min(grid_x, self.grid_size - 1))
        grid_y = max(0, min(grid_y, self.grid_size - 1))

        return grid_x, grid_y

    def polar_to_cartesian(self, angle: float, distance: float) -> Tuple[float, float]:
        """
        Convert polar coordinates (angle, distance) to cartesian coordinates
        relative to the robot's current position and heading

        Args:
            angle: Angle in degrees (relative to robot's front, where 0 is forward)
            distance: Distance in cm

        Returns:
            (x, y): Cartesian coordinates (cm) in world space
        """
        # First, we need to understand the two coordinate systems:
        # 1. Robot heading (0° is east, 90° is north)
        # 2. Sensor angle (0° is robot's front, positive clockwise)

        # Calculate the absolute angle in world coordinates
        # We need to add the sensor angle to the robot's heading,
        # but the sign may need to be flipped because of differences in convention

        # Try flipping the sensor angle sign
        absolute_angle = (self.state.heading - angle) % 360

        # Convert to radians
        angle_rad = math.radians(absolute_angle)

        # Standard polar to Cartesian conversion
        x_offset = distance * math.cos(angle_rad)
        y_offset = distance * math.sin(angle_rad)

        # Add robot's position
        world_x = self.state.x + x_offset
        world_y = self.state.y + y_offset

        return world_x, world_y

    def update_grid_with_ray(self, start_x: float, start_y: float, end_x: float, end_y: float) -> None:
        """
        Update grid using ray casting from start to end point

        Args:
            start_x, start_y: Start coordinates in world space (cm)
            end_x, end_y: End coordinates in world space (cm)
        """
        # Convert to grid coordinates
        start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
        end_grid_x, end_grid_y = self.world_to_grid(end_x, end_y)

        # Use Bresenham's line algorithm to get all cells along the ray
        line_points = self._bresenham_line(start_grid_x, start_grid_y, end_grid_x, end_grid_y)

        # Mark all cells along the ray as free except the last one
        for i, (grid_x, grid_y) in enumerate(line_points):
            # Skip if out of bounds
            if not (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size):
                continue

            if i == len(line_points) - 1:
                # Last point is occupied
                self.grid[grid_y, grid_x] = 1
            else:
                # Path to obstacle is free space
                self.grid[grid_y, grid_x] = 0

    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """
        Implementation of Bresenham's line algorithm

        Args:
            x0, y0: Starting point
            x1, y1: Ending point

        Returns:
            List of points (x, y) along the line
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                if x0 == x1:
                    break
                err -= dy
                x0 += sx
            if e2 < dx:
                if y0 == y1:
                    break
                err += dx
                y0 += sy

        return points

    async def scan_surroundings(self, sensor_func: Callable[[float], Awaitable[float]],
                                angle_range: Tuple[int, int] = None,
                                angle_step: int = None) -> None:
        """
        Scan surroundings using the ultrasonic sensor

        Args:
            sensor_func: Async function that takes an angle and returns a distance
            angle_range: Range of angles to scan (min, max)
            angle_step: Step size for scanning in degrees
        """
        # Use default values from state if not provided
        if angle_range is None:
            angle_range = self.state.scan_range
        if angle_step is None:
            angle_step = self.state.scan_step

        # Current robot position in world coordinates
        robot_x, robot_y = self.state.x, self.state.y

        # Scan surroundings
        for angle in range(angle_range[0], angle_range[1] + 1, angle_step):
            # Get distance reading from sensor
            distance = await sensor_func(angle)

            # Convert to world coordinates
            if distance < 300:  # Valid reading (not too far)
                object_x, object_y = self.polar_to_cartesian(angle, distance)

                # Store object
                self.objects.append((object_x, object_y))

                # Update grid with ray from robot to object
                self.update_grid_with_ray(robot_x, robot_y, object_x, object_y)
            else:
                # If no object detected, mark maximum range as free space
                max_dist = 50  # Only mark up to 50cm as definitely free
                far_x, far_y = self.polar_to_cartesian(angle, max_dist)

                # Update grid with ray from robot to maximum range
                self.update_grid_with_ray(robot_x, robot_y, far_x, far_y)

    def visualize(self, return_image: bool = True) -> dict:
        """
        Visualize the world map

        Args:
            return_image: If True, return base64-encoded PNG image (default: True)

        Returns:
            Dictionary with visualization data and grid data
        """
        # Use non-interactive Agg backend to avoid plt.show() issues in headless environments
        plt.switch_backend('Agg')

        fig, ax = plt.subplots(figsize=(8, 8))

        # Plot the grid
        im = ax.imshow(self.grid, cmap=self.cmap, norm=self.norm,
                       extent=[-self.origin * self.cell_size, (self.grid_size - self.origin) * self.cell_size,
                               -self.origin * self.cell_size, (self.grid_size - self.origin) * self.cell_size])

        # Add colorbar
        cbar = plt.colorbar(im, ax=ax, ticks=[-1, 0, 1])
        cbar.ax.set_yticklabels(['Unknown', 'Free', 'Occupied'])

        # Plot robot position (as a triangle pointing in the heading direction)
        robot_marker_size = 10
        heading_rad = math.radians(self.state.heading)
        dx = robot_marker_size * math.cos(heading_rad)
        dy = robot_marker_size * math.sin(heading_rad)

        ax.arrow(self.state.x, self.state.y, dx, dy,
                 head_width=5, head_length=5, fc='r', ec='r')

        # Plot robot position
        ax.plot(self.state.x, self.state.y, 'ro', markersize=5)

        # Add grid
        ax.grid(True, linestyle='--', alpha=0.7)

        # Set title and labels
        ax.set_title(f'World Map (Robot at {self.state.x:.1f}, {self.state.y:.1f}, heading {self.state.heading:.1f}°)')
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')

        # Set axis equal and limits
        ax.set_aspect('equal')

        # Prepare grid data for ASCII representation
        grid_data = {
            'grid': self.grid.tolist(),
            'position': {
                'x': float(self.state.x),
                'y': float(self.state.y),
                'heading': float(self.state.heading)
            },
            'dimensions': {
                'width': self.grid_size,
                'height': self.grid_size,
                'cell_size': self.cell_size,
                'origin': self.origin
            }
        }

        # Always save to buffer
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=100)
        buf.seek(0)

        result = {'grid_data': grid_data}

        if return_image:
            # Convert to base64
            img_str = base64.b64encode(buf.read()).decode('utf-8')
            plt.close(fig)
            result['visualization'] = img_str
        else:
            # No need for plt.show(), just return None for visualization
            plt.close(fig)
            result['visualization'] = None

        return result

    def get_ascii_map(self, width: int = 40, height: int = 20) -> str:
        """
        Generate ASCII representation of the map

        Args:
            width: Width of ASCII map
            height: Height of ASCII map

        Returns:
            ASCII map as string
        """
        # Create empty ASCII map
        ascii_map = [['·' for _ in range(width)] for _ in range(height)]

        # Calculate scaling factors
        scale_x = self.grid_size / width
        scale_y = self.grid_size / height

        # Fill ASCII map based on grid data
        for y in range(height):
            for x in range(width):
                # Convert ASCII coordinates to grid coordinates
                grid_x = int(x * scale_x)
                grid_y = int(y * scale_y)

                # Get cell value
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    cell = self.grid[grid_y, grid_x]

                    if cell == 1:  # Occupied
                        ascii_map[y][x] = '#'
                    elif cell == 0:  # Free
                        ascii_map[y][x] = ' '
                    # else unknown ('·')

        # Add robot position
        robot_grid_x, robot_grid_y = self.world_to_grid(self.state.x, self.state.y)
        robot_ascii_x = int(robot_grid_x / scale_x)
        robot_ascii_y = int(robot_grid_y / scale_y)

        # Only add robot if within ASCII map bounds
        if 0 <= robot_ascii_x < width and 0 <= robot_ascii_y < height:
            # Direction markers based on heading
            heading_markers = {
                0: '^',  # North
                45: '/',  # Northeast
                90: '>',  # East
                135: '\\',  # Southeast
                180: 'v',  # South
                225: '/',  # Southwest
                270: '<',  # West
                315: '\\'  # Northwest
            }

            # Get closest direction
            closest_heading = min(heading_markers.keys(),
                                  key=lambda h: abs((self.state.heading - h + 180) % 360 - 180))

            ascii_map[robot_ascii_y][robot_ascii_x] = heading_markers[closest_heading]

        # Convert to string
        ascii_str = '\n'.join([''.join(row) for row in ascii_map])

        # Add legend
        legend = (
            f"\nLegend:\n"
            f"  · = Unknown\n"
            f"    = Free space\n"
            f"  # = Obstacle\n"
            f"  ^>v< = Robot position and direction\n"
            f"\nRobot: ({self.state.x:.1f}, {self.state.y:.1f}), Heading: {self.state.heading:.1f}°"
        )

        return ascii_str + legend