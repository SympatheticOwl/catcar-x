import base64
import io
import math
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Tuple, Dict, List
from state_handler import State


class WorldMap3:
    def __init__(self, state: State, map_size: int = 400, resolution: float = 5.0):
        """
        Initialize the world map with absolute coordinates

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

        # Last scan data for interpolation
        self.last_scan = None  # (angle, distance)

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert absolute world coordinates (cm) to grid coordinates"""
        grid_x = int(world_x / self.resolution) + self.origin[0]
        grid_y = int(world_y / self.resolution) + self.origin[1]

        # Ensure coordinates are within bounds
        grid_x = max(0, min(grid_x, self.grid_size - 1))
        grid_y = max(0, min(grid_y, self.grid_size - 1))

        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to absolute world coordinates (cm)"""
        world_x = (grid_x - self.origin[0]) * self.resolution
        world_y = (grid_y - self.origin[1]) * self.resolution
        return world_x, world_y

    def update_car_position(self):
        """Update car position in grid based on current state"""
        # Clear previous car position
        self.grid[self.grid == self.car_marker] = 0

        # Convert car's world position to grid coordinates
        car_grid_x, car_grid_y = self.world_to_grid(self.state.x, self.state.y)

        # Mark car position
        self.grid[car_grid_y, car_grid_x] = self.car_marker

    def add_obstacle_reading(self, distance: float, angle: float):
        """
        Add an obstacle reading from ultrasonic sensor, translating to absolute world coordinates

        Args:
            distance: Distance to obstacle in cm
            angle: Angle of sensor in degrees (relative to car's heading)
        """
        if distance >= self.state.min_distance:
            # Calculate absolute world coordinates of the obstacle
            # First, get the relative coordinates from car's perspective
            rel_angle_rad = math.radians(angle + self.state.heading)
            rel_x = distance * math.cos(rel_angle_rad)
            rel_y = distance * math.sin(rel_angle_rad)

            # Then, translate to absolute world coordinates by adding car's position
            abs_x = self.state.x + rel_x
            abs_y = self.state.y + rel_y

            # Convert to grid coordinates
            grid_x, grid_y = self.world_to_grid(abs_x, abs_y)

            # Mark path as scanned
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
        """Mark the path of the ultrasonic scan as checked in absolute coordinates"""
        # Calculate absolute endpoint of scan
        rel_angle_rad = math.radians(angle + self.state.heading)
        end_x = self.state.x + distance * math.cos(rel_angle_rad)
        end_y = self.state.y + distance * math.sin(rel_angle_rad)

        # Convert car position and scan endpoint to grid
        start_x, start_y = self.world_to_grid(self.state.x, self.state.y)
        end_x, end_y = self.world_to_grid(end_x, end_y)

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
        """Interpolate obstacles between two readings in absolute coordinates"""
        # Calculate absolute world coordinates of first reading
        rel_angle1_rad = math.radians(angle1 + self.state.heading)
        x1 = self.state.x + dist1 * math.cos(rel_angle1_rad)
        y1 = self.state.y + dist1 * math.sin(rel_angle1_rad)

        # Calculate absolute world coordinates of second reading
        rel_angle2_rad = math.radians(angle2 + self.state.heading)
        x2 = self.state.x + dist2 * math.cos(rel_angle2_rad)
        y2 = self.state.y + dist2 * math.sin(rel_angle2_rad)

        # Calculate number of interpolation points based on distance
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        num_points = int(distance / self.resolution)

        if num_points > 1:  # Only interpolate if points are far enough apart
            # Interpolate points in world coordinates
            x_points = np.linspace(x1, x2, num_points)
            y_points = np.linspace(y1, y2, num_points)

            # Add interpolated obstacles
            current_time = time.time()
            for x, y in zip(x_points, y_points):
                grid_x, grid_y = self.world_to_grid(x, y)
                self.grid[grid_y, grid_x] = 1
                self.scan_times[grid_y, grid_x] = current_time

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

    def visualize(self, return_image=True):
        """
        Visualize the map with matplotlib in absolute world coordinates

        Args:
            return_image: If True, returns the plot as a base64 string
        """
        try:
            # Create a new figure with a white background
            fig = plt.figure(figsize=(10, 10), facecolor='white')
            ax = fig.add_subplot(111)

            # Calculate world coordinate extent based on grid size
            world_min_x = -self.resolution * self.origin[0]
            world_max_x = self.resolution * (self.grid_size - self.origin[0])
            world_min_y = -self.resolution * self.origin[1]
            world_max_y = self.resolution * (self.grid_size - self.origin[1])

            # Create world coordinate meshgrid
            world_x = np.linspace(world_min_x, world_max_x, self.grid_size)
            world_y = np.linspace(world_min_y, world_max_y, self.grid_size)
            X, Y = np.meshgrid(world_x, world_y)

            # Plot grid cells with proper colors
            cmap = plt.cm.get_cmap('gray_r', 3)  # 3 distinct colors for 0, 1, 2
            im = ax.pcolormesh(X, Y, self.grid, cmap=cmap, shading='nearest')

            # Add colorbar and make it discrete
            cbar = plt.colorbar(im, ax=ax, ticks=[0.33, 1, 1.67])
            cbar.ax.set_yticklabels(['Empty', 'Obstacle', 'Car'])

            # Plot car position
            car_grid_x, car_grid_y = self.world_to_grid(self.state.x, self.state.y)
            car_x, car_y = self.grid_to_world(car_grid_x, car_grid_y)
            ax.plot(car_x, car_y, 'ro', markersize=10, label=f'Car ({car_x:.1f}, {car_y:.1f})')

            # Add car heading indicator
            heading_rad = math.radians(self.state.heading)
            heading_length = 20  # cm
            dx = heading_length * math.cos(heading_rad)
            dy = heading_length * math.sin(heading_rad)
            ax.arrow(car_x, car_y, dx, dy, head_width=5, head_length=10, fc='r', ec='r')

            # Add grid lines at regular intervals
            grid_interval = 50  # cm
            ax.grid(True, linestyle='--', alpha=0.5)
            ax.set_xticks(np.arange(math.floor(world_min_x / grid_interval) * grid_interval,
                                    math.ceil(world_max_x / grid_interval) * grid_interval,
                                    grid_interval))
            ax.set_yticks(np.arange(math.floor(world_min_y / grid_interval) * grid_interval,
                                    math.ceil(world_max_y / grid_interval) * grid_interval,
                                    grid_interval))

            # Add axis labels and title
            ax.set_title('World Map (Absolute Coordinates)')
            ax.set_xlabel('X Position (cm)')
            ax.set_ylabel('Y Position (cm)')

            # Add origin indicators
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax.axvline(x=0, color='k', linestyle='--', alpha=0.3)

            # Set equal aspect ratio
            ax.set_aspect('equal')

            # Add car position and heading info
            ax.text(0.02, 0.98,
                    f'Car: ({car_x:.1f}, {car_y:.1f}) cm\n' +
                    f'Heading: {self.state.heading:.1f}°',
                    transform=ax.transAxes,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

            # Add legend
            ax.legend(loc='lower right')

            if return_image:
                # Save plot to a bytes buffer
                buf = io.BytesIO()
                fig.savefig(buf, format='png', bbox_inches='tight',
                            facecolor=fig.get_facecolor(), edgecolor='none')
                plt.close(fig)  # Close the figure to free memory

                # Encode the bytes as base64
                buf.seek(0)
                image_base64 = base64.b64encode(buf.getvalue()).decode('utf-8')
                buf.close()

                return image_base64
            else:
                plt.show()
                plt.close(fig)

        except Exception as e:
            print(f"Error in visualization: {str(e)}")
            return None

    def get_visualization_data(self):
        """Get both the grid data and visualization"""
        return {
            'grid_data': self.get_grid_data(),
            'visualization': self.visualize()
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