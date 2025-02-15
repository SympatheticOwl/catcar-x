import math
import asyncio
import numpy as np
from enum import Enum
from typing import List, Tuple, Optional
from vision_system import VisionSystem
from picar_wrapper import PicarXMovement

class NavigationState(Enum):
    MOVING = "moving"
    SCANNING = "scanning"
    PLANNING = "planning"
    OBSTACLE_DETECTED = "obstacle_detected"
    REACHED_TARGET = "reached_target"

class NavigationSystem:
    def __init__(self, movement_controller: PicarXMovement, vision_system: VisionSystem):
        self.movement = movement_controller
        self.vision = vision_system
        
        # Navigation parameters
        self.target_x = 0  # cm
        self.target_y = 0  # cm
        self.position_tolerance = 10  # cm
        self.heading_tolerance = 5  # degrees
        
        # Obstacle avoidance parameters
        self.min_obstacle_distance = 20  # cm
        self.scan_range = (-60, 60)  # degrees
        self.scan_step = 5  # degrees
        self.safe_distance = 30  # cm
        
        # Path planning
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint = 0
        self.state = NavigationState.PLANNING
        
        # Virtual grid for pathfinding
        self.grid_size = 10  # cm per grid cell
        self.grid_width = 500  # 5 meters
        self.grid_height = 500  # 5 meters
        self.obstacle_grid = np.zeros((self.grid_height, self.grid_width))
        
        # Initialize grid with known position at center
        self.grid_center = (self.grid_height // 2, self.grid_width // 2)
        
    def _world_to_grid(self, x_cm: float, y_cm: float) -> Tuple[int, int]:
        """Convert world coordinates (cm) to grid coordinates"""
        grid_x = int(self.grid_center[1] + (x_cm / self.grid_size))
        grid_y = int(self.grid_center[0] + (y_cm / self.grid_size))
        return max(0, min(grid_y, self.grid_height-1)), max(0, min(grid_x, self.grid_width-1))
        
    def _grid_to_world(self, grid_y: int, grid_x: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates (cm)"""
        x_cm = (grid_x - self.grid_center[1]) * self.grid_size
        y_cm = (grid_y - self.grid_center[0]) * self.grid_size
        return x_cm, y_cm
        
    async def set_target(self, x_cm: float, y_cm: float):
        """Set target destination in centimeters"""
        self.target_x = x_cm
        self.target_y = y_cm
        self.waypoints = [(x_cm, y_cm)]
        self.current_waypoint = 0
        self.state = NavigationState.PLANNING
        
    async def update_obstacle_map(self, scan_data: List[Tuple[float, float]]):
        """Update obstacle grid based on sensor scan data"""
        current_x, current_y, current_heading = self.movement.get_position()
        
        # Clear area around current position
        current_grid_pos = self._world_to_grid(current_x, current_y)
        clear_radius = int(self.safe_distance / self.grid_size)
        
        y_indices, x_indices = np.ogrid[max(0, current_grid_pos[0]-clear_radius):min(self.grid_height, current_grid_pos[0]+clear_radius+1),
                                      max(0, current_grid_pos[1]-clear_radius):min(self.grid_width, current_grid_pos[1]+clear_radius+1)]
        self.obstacle_grid[y_indices, x_indices] = 0
        
        # Add detected obstacles to grid
        for angle, distance in scan_data:
            if distance < 300:  # Filter out invalid readings
                # Convert polar coordinates to Cartesian
                abs_angle = current_heading + angle
                rad_angle = math.radians(abs_angle)
                obstacle_x = current_x + (distance * math.cos(rad_angle))
                obstacle_y = current_y + (distance * math.sin(rad_angle))
                
                # Mark obstacle in grid
                obstacle_grid_pos = self._world_to_grid(obstacle_x, obstacle_y)
                self.obstacle_grid[obstacle_grid_pos] = 1
                
                # Add safety margin around obstacle
                margin = int(self.safe_distance / self.grid_size)
                y_min = max(0, obstacle_grid_pos[0] - margin)
                y_max = min(self.grid_height, obstacle_grid_pos[0] + margin + 1)
                x_min = max(0, obstacle_grid_pos[1] - margin)
                x_max = min(self.grid_width, obstacle_grid_pos[1] + margin + 1)
                
                self.obstacle_grid[y_min:y_max, x_min:x_max] = 1
                
    def _find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding algorithm"""
        from heapq import heappush, heappop
        
        def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
            
        def get_neighbors(pos: Tuple[int, int]) -> List[Tuple[int, int]]:
            y, x = pos
            neighbors = []
            for dy, dx in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                ny, nx = y + dy, x + dx
                if (0 <= ny < self.grid_height and 
                    0 <= nx < self.grid_width and 
                    self.obstacle_grid[ny, nx] == 0):
                    neighbors.append((ny, nx))
            return neighbors
            
        frontier = []
        heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while frontier:
            current = heappop(frontier)[1]
            
            if current == goal:
                break
                
            for next_pos in get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
                    
        if goal not in came_from:
            return None
            
        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        
        return path
        
    async def plan_path(self):
        """Plan path to target avoiding obstacles"""
        current_x, current_y, _ = self.movement.get_position()
        start_grid = self._world_to_grid(current_x, current_y)
        goal_grid = self._world_to_grid(self.target_x, self.target_y)
        
        path = self._find_path(start_grid, goal_grid)
        if path:
            # Convert path to waypoints
            self.waypoints = [self._grid_to_world(y, x) for y, x in path]
            self.current_waypoint = 0
            return True
        return False
        
    async def scan_environment(self) -> List[Tuple[float, float]]:
        """Perform sensor sweep and get obstacle data"""
        scan_data = []
        
        # Check vision system first
        vision_objects = self.vision.get_obstacle_info()
        if vision_objects:
            for obj in vision_objects:
                # Convert bounding box to approximate distance and angle
                xmin, ymin, xmax, ymax = obj['box']
                center_x = (xmin + xmax) / 2
                width = xmax - xmin
                
                # Rough distance estimation based on object width
                # This would need calibration for your specific camera setup
                distance = 100 * (100 / width)  # Very rough approximation
                
                # Calculate angle from center of image
                angle = (center_x - 320) / 640 * 60  # Assuming 60 degree FOV
                
                scan_data.append((angle, distance))
        
        # Get ultrasonic scan data
        for angle in range(self.scan_range[0], self.scan_range[1] + 1, self.scan_step):
            self.movement.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)
            
            distances = []
            for _ in range(3):
                dist = self.movement.px.ultrasonic.read()
                if dist and 0 < dist < 300:
                    distances.append(dist)
                await asyncio.sleep(0.01)
                
            if distances:
                scan_data.append((angle, sum(distances) / len(distances)))
                
        self.movement.px.set_cam_pan_angle(0)
        return scan_data
        
    async def navigate(self):
        """Main navigation loop"""
        while True:
            current_x, current_y, current_heading = self.movement.get_position()
            
            # Check if we've reached the target
            distance_to_target = math.sqrt(
                (self.target_x - current_x)**2 + 
                (self.target_y - current_y)**2
            )
            
            if distance_to_target <= self.position_tolerance:
                self.state = NavigationState.REACHED_TARGET
                break
                
            # State machine
            if self.state == NavigationState.PLANNING:
                # Scan environment and update obstacle map
                scan_data = await self.scan_environment()
                await self.update_obstacle_map(scan_data)
                
                # Plan new path
                if await self.plan_path():
                    self.state = NavigationState.MOVING
                else:
                    # No path found, need more scanning
                    self.state = NavigationState.SCANNING
                    
            elif self.state == NavigationState.MOVING:
                if self.current_waypoint < len(self.waypoints):
                    waypoint_x, waypoint_y = self.waypoints[self.current_waypoint]
                    
                    # Check for obstacles before moving
                    dist = self.movement.px.ultrasonic.read()
                    if dist and dist < self.min_obstacle_distance:
                        self.state = NavigationState.OBSTACLE_DETECTED
                        continue
                        
                    # Move to waypoint
                    success, _ = await self.movement.move_to_position(waypoint_x, waypoint_y)
                    if success:
                        self.current_waypoint += 1
                    else:
                        self.state = NavigationState.PLANNING
                else:
                    self.state = NavigationState.PLANNING
                    
            elif self.state == NavigationState.OBSTACLE_DETECTED:
                # Backup and replan
                await self.movement.move_distance(-20)  # Back up 20cm
                self.state = NavigationState.PLANNING
                
            elif self.state == NavigationState.SCANNING:
                # Do a full scan and try planning again
                scan_data = await self.scan_environment()
                await self.update_obstacle_map(scan_data)
                self.state = NavigationState.PLANNING
                
            await asyncio.sleep(0.1)
            
        return self.state == NavigationState.REACHED_TARGET

async def navigate_to_target(x_feet: float, y_feet: float):
    """Helper function to navigate to a target position specified in feet"""
    # Convert feet to centimeters
    x_cm = x_feet * 30.48
    y_cm = y_feet * 30.48
    
    # Initialize systems
    movement = PicarXMovement()
    vision = VisionSystem()
    navigation = NavigationSystem(movement, vision)
    
    # Set target and navigate
    await navigation.set_target(x_cm, y_cm)
    success = await navigation.navigate()
    
    if success:
        print(f"Successfully reached target position ({x_feet}ft, {y_feet}ft)")
    else:
        print("Failed to reach target position")
        
    return success