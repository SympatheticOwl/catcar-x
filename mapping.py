import numpy as np
from math import cos, sin, radians
import asyncio


class MapBuilder:
    def __init__(self, map_size=100, resolution=1):
        """
        Initialize the mapping system

        Args:
            map_size (int): Size of the square grid in cm
            resolution (int): cm per grid cell
        """
        self.map_size = map_size
        self.resolution = resolution
        self.grid_size = map_size // resolution

        # Create occupancy grid centered on car's position
        # -1: unknown, 0: free, 1: occupied
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1)

        # Car position is center of grid
        self.car_pos = (self.grid_size // 2, self.grid_size // 2)

        # Sensor parameters
        self.max_sensor_range = 300  # cm
        self.angle_increment = 2  # degrees
        self.interpolation_resolution = 1  # cm

    def world_to_grid(self, x, y):
        """Convert world coordinates (cm) to grid indices"""
        grid_x = int(x / self.resolution) + self.car_pos[0]
        grid_y = int(y / self.resolution) + self.car_pos[1]
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates (cm)"""
        world_x = (grid_x - self.car_pos[0]) * self.resolution
        world_y = (grid_y - self.car_pos[1]) * self.resolution
        return world_x, world_y

    def update_grid(self, angle, distance):
        """
        Update occupancy grid with new sensor reading

        Args:
            angle (float): Sensor angle in degrees
            distance (float): Measured distance in cm
        """
        if not (0 < distance < self.max_sensor_range):
            return

        # Convert polar to cartesian coordinates
        x = distance * cos(radians(angle))
        y = distance * sin(radians(angle))

        # Get grid coordinates of obstacle
        obstacle_x, obstacle_y = self.world_to_grid(x, y)

        # Check if point is within grid bounds
        if (0 <= obstacle_x < self.grid_size and
                0 <= obstacle_y < self.grid_size):

            # Mark cells along ray as free
            ray_points = self.get_ray_points(angle, distance)
            for rx, ry in ray_points:
                if (0 <= rx < self.grid_size and
                        0 <= ry < self.grid_size):
                    self.occupancy_grid[rx, ry] = 0

            # Mark obstacle point
            self.occupancy_grid[obstacle_x, obstacle_y] = 1

    def get_ray_points(self, angle, distance):
        """Get all grid points along sensor ray"""
        points = []

        # Generate points along ray at interpolation resolution
        for d in range(0, int(distance), self.interpolation_resolution):
            x = d * cos(radians(angle))
            y = d * sin(radians(angle))
            grid_x, grid_y = self.world_to_grid(x, y)
            points.append((grid_x, grid_y))

        return points

    def interpolate_obstacles(self, angle1, dist1, angle2, dist2):
        """
        Interpolate obstacles between two sensor readings

        Args:
            angle1, angle2 (float): Angles of readings in degrees
            dist1, dist2 (float): Measured distances in cm
        """
        if abs(angle1 - angle2) > self.angle_increment:
            return

        # Convert both points to cartesian
        x1 = dist1 * cos(radians(angle1))
        y1 = dist1 * sin(radians(angle1))
        x2 = dist2 * cos(radians(angle2))
        y2 = dist2 * sin(radians(angle2))

        # Calculate points along line between obstacles
        points = []
        dx = x2 - x1
        dy = y2 - y1
        steps = max(abs(dx), abs(dy)) // self.interpolation_resolution

        if steps > 0:
            x_step = dx / steps
            y_step = dy / steps

            for i in range(int(steps)):
                x = x1 + i * x_step
                y = y1 + i * y_step
                grid_x, grid_y = self.world_to_grid(x, y)
                if (0 <= grid_x < self.grid_size and
                        0 <= grid_y < self.grid_size):
                    self.occupancy_grid[grid_x, grid_y] = 1


class MappingSystem:
    def __init__(self, picarx):
        self.px = picarx
        self.map = MapBuilder()
        self.last_angle = None
        self.last_distance = None

        self.scan_range = (-60, 60)  # degrees
        self.scan_step = 5  # degrees

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            await asyncio.sleep(0.01)
        return distances

    async def scan_surroundings(self):
        """Perform a full scan and update map"""
        angles = range(-45, 46, 5)  # -45 to +45 degrees

        for angle in angles:
            # Set servo angle
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)  # Allow servo to move

            # Take multiple readings and average
            distances = self.scan_avg()

            if distances:
                avg_distance = sum(distances) / len(distances)

                # Update map with new reading
                self.map.update_grid(angle, avg_distance)

                # Interpolate between consecutive readings
                if self.last_angle is not None:
                    self.map.interpolate_obstacles(
                        self.last_angle,
                        self.last_distance,
                        angle,
                        avg_distance
                    )

                self.last_angle = angle
                self.last_distance = avg_distance

        # Reset servo to center
        self.px.set_cam_pan_angle(0)

    def get_obstacles_in_range(self, range_cm):
        """
        Get array of obstacle angles within range
        Returns array where 1 indicates obstacle present at that angle
        """
        angles = []
        center_x = self.map.car_pos[0]
        center_y = self.map.car_pos[1]

        # Convert range to grid cells
        range_cells = int(range_cm / self.map.resolution)

        # Check each angle
        for angle in range(-45, 46):
            # Get point at this angle and range
            x = center_x + int(range_cells * cos(radians(angle)))
            y = center_y + int(range_cells * sin(radians(angle)))

            # Check if any obstacles along ray to this point
            ray_points = self.map.get_ray_points(angle, range_cells)
            has_obstacle = False

            for rx, ry in ray_points:
                if (0 <= rx < self.map.grid_size and
                        0 <= ry < self.map.grid_size and
                        self.map.occupancy_grid[rx, ry] == 1):
                    has_obstacle = True
                    break

            angles.append(1 if has_obstacle else 0)

        return np.array(angles)