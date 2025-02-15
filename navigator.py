import asyncio
import math
import numpy as np
from enum import Enum
from typing import Tuple, List, Optional
from picar_wrapper import PicarXMovement
from async_obj_avoider import EnhancedObstacleAvoidance

class NavigationState(Enum):
    MOVING = "moving"
    SCANNING = "scanning"
    PLANNING = "planning"
    STOPPED = "stopped"

class AutonomousNavigation:
    def __init__(self, movement_system: PicarXMovement, sensor_system: EnhancedObstacleAvoidance):
        self.movement = movement_system
        self.sensors = sensor_system
        
        # Navigation parameters
        self.target_x = 0  # cm
        self.target_y = 0  # cm
        self.min_distance = 15  # cm, minimum safe distance from obstacles
        self.scan_radius = 50  # cm, how far to look for obstacles
        self.state = NavigationState.STOPPED
        
        # Path planning parameters
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_idx = 0
        self.path_retries = 0
        self.max_retries = 3
        self.obstacle_memory = []  # List of detected obstacles and their positions
        
        # Movement parameters
        self.move_chunk_size = 30  # cm, distance to move before rechecking path
        self.speed = 30
        
        # Initialize sensor monitoring
        self.obstacle_detected = asyncio.Event()
        self.is_moving = False

    def set_destination(self, x_feet: float, y_feet: float):
        """Set the target destination in feet"""
        # Convert feet to centimeters
        self.target_x = x_feet * 30.48  # 1 foot = 30.48 cm
        self.target_y = y_feet * 30.48
        self.waypoints = [(self.target_x, self.target_y)]
        self.current_waypoint_idx = 0
        print(f"Target set to: ({x_feet}ft, {y_feet}ft) = ({self.target_x}cm, {self.target_y}cm)")

    async def obstacle_monitoring(self):
        """Monitor for obstacles using both ultrasonic and vision systems"""
        while True:
            if self.is_moving:
                # Check ultrasonic sensor
                if self.sensors.current_distance < self.min_distance:
                    print(f"Ultrasonic detected obstacle at {self.sensors.current_distance}cm")
                    self.obstacle_detected.set()
                
                # Check vision system
                if self.sensors.vision_enabled:
                    objects = self.sensors.vision.get_obstacle_info()
                    if objects:
                        print(f"Vision system detected: {[obj['label'] for obj in objects]}")
                        self.obstacle_detected.set()
                
            await asyncio.sleep(0.05)  # 50ms check interval

    def update_obstacle_memory(self, scan_data):
        """Update internal map of obstacles"""
        current_x, current_y, _ = self.movement.get_position()
        
        # Convert scan data to obstacle positions
        new_obstacles = []
        for angle, distance in scan_data:
            if distance < self.scan_radius:
                # Convert polar coordinates to cartesian
                rad_angle = math.radians(angle)
                obs_x = current_x + (distance * math.cos(rad_angle))
                obs_y = current_y + (distance * math.sin(rad_angle))
                new_obstacles.append((obs_x, obs_y))
        
        # Update obstacle memory
        self.obstacle_memory.extend(new_obstacles)
        # Limit memory size by removing old obstacles
        if len(self.obstacle_memory) > 100:
            self.obstacle_memory = self.obstacle_memory[-100:]

    def find_clear_path(self) -> Optional[Tuple[float, float]]:
        """Find a clear path towards the goal"""
        current_x, current_y, current_heading = self.movement.get_position()
        target = self.waypoints[self.current_waypoint_idx]
        
        # Calculate direct path to target
        dx = target[0] - current_x
        dy = target[1] - current_y
        direct_distance = math.sqrt(dx**2 + dy**2)
        
        if direct_distance < self.move_chunk_size:
            return target  # Close enough to move directly to target
            
        # Check if direct path is clear
        direct_angle = math.degrees(math.atan2(dy, dx))
        is_path_blocked = any(
            self._point_near_obstacle(
                current_x + (dx * t),
                current_y + (dy * t)
            )
            for t in np.linspace(0, 1, 10)
        )
        
        if not is_path_blocked:
            # Move in the direction of target, but only by chunk_size
            progress_ratio = self.move_chunk_size / direct_distance
            return (
                current_x + (dx * progress_ratio),
                current_y + (dy * progress_ratio)
            )
            
        # If direct path is blocked, try finding alternative directions
        best_direction = None
        best_score = float('-inf')
        
        for angle in range(0, 360, 30):  # Check every 30 degrees
            rad_angle = math.radians(angle)
            test_x = current_x + (self.move_chunk_size * math.cos(rad_angle))
            test_y = current_y + (self.move_chunk_size * math.sin(rad_angle))
            
            if not self._point_near_obstacle(test_x, test_y):
                # Score this direction based on:
                # 1. Distance to target
                # 2. Alignment with target direction
                # 3. Clearance from obstacles
                distance_to_target = math.sqrt(
                    (target[0] - test_x)**2 + 
                    (target[1] - test_y)**2
                )
                
                angle_to_target = abs(
                    angle - math.degrees(math.atan2(dy, dx))
                )
                
                clearance = min(
                    math.sqrt((ox - test_x)**2 + (oy - test_y)**2)
                    for ox, oy in self.obstacle_memory
                ) if self.obstacle_memory else self.scan_radius
                
                score = (
                    -distance_to_target +  # Shorter distance is better
                    -angle_to_target * 0.5 +  # Better alignment is better
                    clearance * 2  # More clearance is better
                )
                
                if score > best_score:
                    best_score = score
                    best_direction = (test_x, test_y)
        
        return best_direction

    def _point_near_obstacle(self, x: float, y: float) -> bool:
        """Check if a point is too close to any known obstacle"""
        return any(
            math.sqrt((ox - x)**2 + (oy - y)**2) < self.min_distance
            for ox, oy in self.obstacle_memory
        )

    async def navigate(self):
        """Main navigation loop"""
        self.state = NavigationState.MOVING
        self.is_moving = True
        
        # Start obstacle monitoring
        monitor_task = asyncio.create_task(self.obstacle_monitoring())
        
        try:
            while self.current_waypoint_idx < len(self.waypoints):
                current_x, current_y, _ = self.movement.get_position()
                target = self.waypoints[self.current_waypoint_idx]
                
                # Check if we've reached the current waypoint
                distance_to_target = math.sqrt(
                    (target[0] - current_x)**2 +
                    (target[1] - current_y)**2
                )
                
                if distance_to_target < 10:  # Within 10cm of target
                    print(f"Reached waypoint {self.current_waypoint_idx}")
                    self.current_waypoint_idx += 1
                    continue
                
                # Plan next movement
                self.state = NavigationState.PLANNING
                next_point = self.find_clear_path()
                
                if next_point is None:
                    print("No clear path found!")
                    # Perform a scan to update obstacle memory
                    self.state = NavigationState.SCANNING
                    scan_data = await self.sensors.scan_environment()
                    self.update_obstacle_memory(scan_data)
                    self.path_retries += 1
                    
                    if self.path_retries >= self.max_retries:
                        print("Failed to find path after max retries")
                        break
                    continue
                
                # Move to next point
                self.state = NavigationState.MOVING
                self.is_moving = True
                self.obstacle_detected.clear()
                
                # Create tasks for both movement and obstacle detection
                move_task = asyncio.create_task(
                    self.movement.move_to_position(
                        next_point[0],
                        next_point[1],
                        self.speed
                    )
                )
                obstacle_task = asyncio.create_task(self.obstacle_detected.wait())
                
                # Wait for either movement completion or obstacle detection
                done, pending = await asyncio.wait(
                    {move_task, obstacle_task},
                    return_when=asyncio.FIRST_COMPLETED
                )
                
                # Cancel pending tasks
                for task in pending:
                    task.cancel()
                
                # Handle obstacle detection
                if self.obstacle_detected.is_set():
                    print("Obstacle detected! Replanning route...")
                    self.state = NavigationState.SCANNING
                    scan_data = await self.sensors.scan_environment()
                    self.update_obstacle_memory(scan_data)
                    continue
                
                self.path_retries = 0  # Reset retry counter on successful movement
                
            # Navigation complete
            self.state = NavigationState.STOPPED
            self.is_moving = False
            print("Navigation complete!")
            
        except asyncio.CancelledError:
            print("Navigation cancelled!")
            raise
        finally:
            monitor_task.cancel()
            try:
                await monitor_task
            except asyncio.CancelledError:
                pass
            self.state = NavigationState.STOPPED
            self.is_moving = False
async def main():
    # Initialize systems
    movement_system = PicarXMovement()
    sensor_system = EnhancedObstacleAvoidance(movement_system.px)
    navigation = AutonomousNavigation(movement_system, sensor_system)
    
    try:
        # Set destination (10 feet north, 5 feet east)
        navigation.set_destination(5, 1)
        
        # Start sensor system
        sensor_task = asyncio.create_task(sensor_system.run())
        
        # Start navigation
        nav_task = asyncio.create_task(navigation.navigate())
        
        # Wait for navigation to complete
        await nav_task
        
    except KeyboardInterrupt:
        print("\nNavigation interrupted!")
    finally:
        # Cleanup
        sensor_task.cancel()
        nav_task.cancel()
        try:
            await asyncio.gather(sensor_task, nav_task, return_exceptions=True)
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    asyncio.run(main())