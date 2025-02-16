import time
import numpy as np
from typing import Tuple, Dict

class WorldMap:
    def __init__(self, map_size: int = 400, resolution: float = 1.0):
        """
        Initialize world map for obstacle tracking

        Args:
            map_size: Size of the map in cm
            resolution: cm per grid cell (1.0 = 1cm per cell)
        """
        self.resolution = resolution
        self.grid_size = int(map_size / resolution)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.origin = np.array([self.grid_size // 2, self.grid_size // 2])

        # Obstacle tracking
        self.obstacles: Dict[str, Dict] = {}  # Track individual obstacles
        self.obstacle_id_counter = 0

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (cm) to grid coordinates"""
        grid_x = int(x / self.resolution) + self.origin[0]
        grid_y = int(y / self.resolution) + self.origin[1]
        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates (cm)"""
        x = (grid_x - self.origin[0]) * self.resolution
        y = (grid_y - self.origin[1]) * self.resolution
        return x, y

    def add_obstacle(self, x: float, y: float, radius: float = 10.0, confidence: float = 1.0,
                     label: str = "unknown") -> str:
        """
        Add an obstacle to the map

        Args:
            x, y: World coordinates of obstacle center (cm)
            radius: Radius of obstacle (cm)
            confidence: Detection confidence (0-1)
            label: Object label from vision system

        Returns:
            obstacle_id: Unique ID for tracking this obstacle
        """
        obstacle_id = f"obs_{self.obstacle_id_counter}"
        self.obstacle_id_counter += 1

        # Store obstacle metadata
        self.obstacles[obstacle_id] = {
            'x': x,
            'y': y,
            'radius': radius,
            'confidence': confidence,
            'label': label,
            'last_seen': time.time()
        }

        # Update grid
        self._update_grid_for_obstacle(x, y, radius)

        return obstacle_id

    def _update_grid_for_obstacle(self, x: float, y: float, radius: float):
        """Update grid cells for an obstacle"""
        grid_x, grid_y = self.world_to_grid(x, y)
        grid_radius = int(radius / self.resolution)

        # Create a circle mask
        y_indices, x_indices = np.ogrid[-grid_radius:grid_radius + 1, -grid_radius:grid_radius + 1]
        mask = x_indices ** 2 + y_indices ** 2 <= grid_radius ** 2

        # Calculate grid boundaries
        x_min = max(0, grid_x - grid_radius)
        x_max = min(self.grid_size, grid_x + grid_radius + 1)
        y_min = max(0, grid_y - grid_radius)
        y_max = min(self.grid_size, grid_y + grid_radius + 1)

        # Apply mask to grid
        mask_height, mask_width = mask.shape
        grid_height = y_max - y_min
        grid_width = x_max - x_min

        # Trim mask if needed
        mask = mask[:grid_height, :grid_width]

        # Update grid
        self.grid[y_min:y_max, x_min:x_max][mask] = 1

    def update_obstacle_position(self, obstacle_id: str, new_x: float, new_y: float):
        """Update position of a tracked obstacle"""
        if obstacle_id in self.obstacles:
            # Clear old position
            old_x = self.obstacles[obstacle_id]['x']
            old_y = self.obstacles[obstacle_id]['y']
            radius = self.obstacles[obstacle_id]['radius']
            self._clear_grid_area(old_x, old_y, radius)

            # Update position
            self.obstacles[obstacle_id]['x'] = new_x
            self.obstacles[obstacle_id]['y'] = new_y
            self.obstacles[obstacle_id]['last_seen'] = time.time()

            # Add to new position
            self._update_grid_for_obstacle(new_x, new_y, radius)

    def _clear_grid_area(self, x: float, y: float, radius: float):
        """Clear grid cells in an area"""
        grid_x, grid_y = self.world_to_grid(x, y)
        grid_radius = int(radius / self.resolution)

        x_min = max(0, grid_x - grid_radius)
        x_max = min(self.grid_size, grid_x + grid_radius + 1)
        y_min = max(0, grid_y - grid_radius)
        y_max = min(self.grid_size, grid_y + grid_radius + 1)

        self.grid[y_min:y_max, x_min:x_max] = 0

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.grid:
            print(''.join(['1' if cell else '0' for cell in row]))
