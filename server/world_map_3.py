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

        # Store actual obstacle points for better interpolation
        self.obstacle_points = []

        # Time-based obstacle decay
        self.obstacle_confidence = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

        # Debug info
        self.last_scan_data = {}

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
        if 0 <= car_grid_x < self.grid_size and 0 <= car_grid_y < self.grid_size:
            self.grid[car_grid_y, car_grid_x] = self.car_marker

    def add_obstacle_reading(self, distance: float, angle: float):
        """
        Add an obstacle reading from ultrasonic sensor, translating to absolute world coordinates

        Args:
            distance: Distance to obstacle in cm
            angle: Angle of sensor in degrees (relative to car's heading)
        """
        # Store scan data for debugging
        self.last_scan_data[angle] = distance

        # Always mark the path as scanned, even if no obstacle detected
        self.mark_scanned_path(angle, distance)

        # Only add obstacle point if distance is valid and not too far
        if self.state.min_distance <= distance < 150:  # Limit to more reliable range
            # Calculate absolute world coordinates of the obstacle
            # First, get the relative coordinates from car's perspective
            rel_angle_rad = math.radians(angle + self.state.heading)
            rel_x = distance * math.cos(rel_angle_rad)
            rel_y = distance * math.sin(rel_angle_rad)

            # Then, translate to absolute world coordinates by adding car's position
            abs_x = self.state.x + rel_x
            abs_y = self.state.y + rel_y

            # Store this obstacle point for future interpolation
            self.obstacle_points.append((abs_x, abs_y, time.time()))

            # Convert to grid coordinates
            grid_x, grid_y = self.world_to_grid(abs_x, abs_y)

            # Check bounds before setting
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                # Add obstacle to grid
                self.grid[grid_y, grid_x] = 1

                # Increase confidence for this cell
                self.obstacle_confidence[grid_y, grid_x] = min(1.0,
                                                               self.obstacle_confidence[grid_y, grid_x] + 0.3)

                # Update scan times for this point
                self.scan_times[grid_y, grid_x] = time.time()

                # Add additional points in a small radius around the detected point
                # to make walls more visible
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue  # Skip the center point (already set)

                        nx, ny = grid_x + dx, grid_y + dy
                        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                            self.grid[ny, nx] = 1
                            self.obstacle_confidence[ny, nx] = min(1.0,
                                                                   self.obstacle_confidence[ny, nx] + 0.2)
                            self.scan_times[ny, nx] = time.time()

            # If we have a previous scan, interpolate between them
            if self.last_scan is not None:
                last_angle, last_distance = self.last_scan
                angle_diff = abs(last_angle - angle)

                # Only interpolate if angles are close enough and distances are similar
                # This prevents connecting unrelated obstacles
                distance_diff = abs(last_distance - distance)
                distance_avg = (last_distance + distance) / 2

                # More strict conditions for interpolation to avoid false positives
                if (angle_diff <= 2 * self.state.scan_step and
                        distance_diff < distance_avg * 0.2):  # 20% difference max
                    self.interpolate_obstacles(last_angle, last_distance, angle, distance)

            # Update last scan
            self.last_scan = (angle, distance)

    def mark_scanned_path(self, angle: float, distance: float):
        """Mark the path of the ultrasonic scan as checked in absolute coordinates"""
        # Calculate absolute endpoint of scan
        rel_angle_rad = math.radians(angle + self.state.heading)

        # Limit maximum scan distance for marking scanned areas
        actual_distance = min(distance, 100)  # Limit to 100cm for scanned path

        end_x = self.state.x + actual_distance * math.cos(rel_angle_rad)
        end_y = self.state.y + actual_distance * math.sin(rel_angle_rad)

        # Convert car position and scan endpoint to grid
        start_x, start_y = self.world_to_grid(self.state.x, self.state.y)
        end_x, end_y = self.world_to_grid(end_x, end_y)

        # Use Bresenham's line algorithm for more efficient line drawing
        points = self.bresenham_line(start_x, start_y, end_x, end_y)

        current_time = time.time()

        # Mark all points as scanned
        for x, y in points:
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                self.scanned_areas[y, x] = 1
                self.scan_times[y, x] = current_time

                # Gradually decay confidence of empty space where no obstacle is detected
                if self.grid[y, x] == 1:
                    # If we scan through a point but find no obstacle, reduce confidence
                    cell_distance = np.sqrt(
                        (x - start_x) ** 2 + (y - start_y) ** 2) * self.resolution

                    # Only decay if this point is closer than the detected distance
                    if cell_distance < actual_distance - self.resolution * 2:
                        self.obstacle_confidence[y, x] *= 0.9  # Decay factor

                        # Remove obstacle if confidence drops too low
                        if self.obstacle_confidence[y, x] < 0.3:
                            self.grid[y, x] = 0

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for efficient line drawing"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while x0 != x1 or y0 != y1:
            points.append((x0, y0))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        points.append((x1, y1))  # Add the last point
        return points

    def interpolate_obstacles(self, angle1: float, dist1: float,
                              angle2: float, dist2: float):
        """Interpolate obstacles between two readings in absolute coordinates"""
        # Only interpolate if both distances are valid
        if (dist1 < self.state.min_distance or dist2 < self.state.min_distance or
                dist1 > 150 or dist2 > 150):  # Don't trust readings that are too far
            return

        # Calculate absolute world coordinates of first reading
        rel_angle1_rad = math.radians(angle1 + self.state.heading)
        x1 = self.state.x + dist1 * math.cos(rel_angle1_rad)
        y1 = self.state.y + dist1 * math.sin(rel_angle1_rad)

        # Calculate absolute world coordinates of second reading
        rel_angle2_rad = math.radians(angle2 + self.state.heading)
        x2 = self.state.x + dist2 * math.cos(rel_angle2_rad)
        y2 = self.state.y + dist2 * math.sin(rel_angle2_rad)

        # Calculate distance between points
        point_distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Calculate angle difference to check if we're seeing the same surface
        angle_diff = abs(angle2 - angle1)

        # Don't interpolate if points are too far apart or angle change is too large
        # This avoids creating phantom obstacles between unrelated readings
        if angle_diff > 15 or point_distance > 30:  # Max 15 degrees, 30cm
            return

        # Calculate number of interpolation points based on distance
        # Use more points for higher precision but not too many to be efficient
        num_points = max(3, min(10, int(point_distance / (self.resolution * 0.5))))

        if num_points > 1:
            # Interpolate points in world coordinates
            x_points = np.linspace(x1, x2, num_points)
            y_points = np.linspace(y1, y2, num_points)

            # Add interpolated obstacles
            current_time = time.time()
            for x, y in zip(x_points, y_points):
                grid_x, grid_y = self.world_to_grid(x, y)
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    self.grid[grid_y, grid_x] = 1
                    self.obstacle_confidence[grid_y, grid_x] = min(1.0,
                                                                   self.obstacle_confidence[grid_y, grid_x] + 0.2)
                    self.scan_times[grid_y, grid_x] = current_time

    async def scan_surroundings(self, sensor_func):
        """
        Perform a complete scan of surroundings using sensor readings

        Args:
            sensor_func: Async function that takes an angle and returns a distance
        """
        # Reset last scan for new scanning sequence
        self.last_scan = None
        self.last_scan_data = {}  # Clear previous scan data

        # Update car position before scan
        self.update_car_position()

        # Scan from left to right for more consistent interpolation
        start_angle, end_angle = self.state.scan_range
        scan_step = self.state.scan_step

        for angle in range(start_angle, end_angle + 1, scan_step):
            distance = await sensor_func(angle)
            if distance is not None:
                self.add_obstacle_reading(distance, angle)

        # Now clear old readings based on confidence after the full scan
        self.apply_confidence_and_clear_old()

        # Force redraw of car position to ensure it's visible
        self.update_car_position()

    def apply_confidence_and_clear_old(self, max_age: float = 5.0, confidence_threshold: float = 0.3):
        """Apply confidence thresholds and clear old readings"""
        current_time = time.time()

        # Age-based decay of confidence
        time_factor = np.clip(
            (current_time - self.scan_times) / max_age, 0, 1)
        self.obstacle_confidence *= (1 - time_factor * 0.5)  # Gradually decay by 50% max

        # Clear obstacles with low confidence
        low_confidence = self.obstacle_confidence < confidence_threshold
        self.grid[low_confidence & (self.grid == 1)] = 0

        # Clear very old readings completely
        very_old_areas = (current_time - self.scan_times) > max_age * 2
        self.grid[very_old_areas & (self.grid == 1)] = 0
        self.scanned_areas[very_old_areas] = 0
        self.scan_times[very_old_areas] = 0
        self.obstacle_confidence[very_old_areas] = 0

        # Filter old points from obstacle_points
        current_points = []
        for x, y, t in self.obstacle_points:
            if current_time - t <= max_age:
                current_points.append((x, y, t))
        self.obstacle_points = current_points

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

            # Use a better color map for visualization
            # White for empty space, black for obstacles, red for car
            custom_cmap = matplotlib.colors.ListedColormap(['white', 'gray', 'red'])
            bounds = [-0.5, 0.5, 1.5, 2.5]
            norm = matplotlib.colors.BoundaryNorm(bounds, custom_cmap.N)

            # Plot grid cells with proper colors
            im = ax.pcolormesh(X, Y, self.grid, cmap=custom_cmap, norm=norm, shading='auto')

            # Add grid lines at regular intervals (in cm)
            grid_interval = 10  # cm (smaller grid for more detail)
            ax.grid(True, linestyle='-', alpha=0.3)
            ax.set_xticks(np.arange(math.floor(world_min_x / grid_interval) * grid_interval,
                                    math.ceil(world_max_x / grid_interval) * grid_interval,
                                    grid_interval))
            ax.set_yticks(np.arange(math.floor(world_min_y / grid_interval) * grid_interval,
                                    math.ceil(world_max_y / grid_interval) * grid_interval,
                                    grid_interval))

            # Plot car position more precisely (actual coordinates, not grid-aligned)
            ax.plot(self.state.x, self.state.y, 'ro', markersize=8, label='Car')

            # Add car heading indicator
            heading_rad = math.radians(self.state.heading)
            heading_length = 15  # cm
            dx = heading_length * math.cos(heading_rad)
            dy = heading_length * math.sin(heading_rad)
            ax.arrow(self.state.x, self.state.y, dx, dy,
                     head_width=3, head_length=5, fc='r', ec='r')

            # Add axis labels and title
            ax.set_title('World Map')
            ax.set_xlabel('X Position (cm)')
            ax.set_ylabel('Y Position (cm)')

            # Add origin indicators
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            ax.axvline(x=0, color='k', linestyle='--', alpha=0.5)

            # Set equal aspect ratio and limit view to reasonable area
            ax.set_aspect('equal')

            # Focus the view on the area around the car (with some margin)
            view_margin = 50  # cm
            ax.set_xlim(self.state.x - view_margin, self.state.x + view_margin)
            ax.set_ylim(self.state.y - view_margin, self.state.y + view_margin)

            # Add car position and heading info
            ax.text(0.02, 0.98,
                    f'Car: ({self.state.x:.1f}, {self.state.y:.1f}) cm\n' +
                    f'Heading: {self.state.heading:.1f}°',
                    transform=ax.transAxes,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

            # Add scan data visualization if available
            if self.last_scan_data:
                angles = list(self.last_scan_data.keys())
                distances = list(self.last_scan_data.values())

                # Add a small legend showing scan data
                max_display = 3  # Show only a few key points
                if len(angles) > max_display:
                    # Pick evenly spaced angles
                    indices = np.linspace(0, len(angles) - 1, max_display, dtype=int)
                    sample_text = "\n".join([f"Angle {angles[i]}°: {distances[i]:.1f} cm"
                                             for i in indices])
                else:
                    sample_text = "\n".join([f"Angle {a}°: {d:.1f} cm"
                                             for a, d in self.last_scan_data.items()])

                ax.text(0.02, 0.12,
                        f"Scan Data Sample:\n{sample_text}",
                        transform=ax.transAxes,
                        verticalalignment='bottom',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

            # Add legend
            ax.legend(loc='upper right')

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