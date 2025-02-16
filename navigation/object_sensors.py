import time
from typing import List, Dict, Tuple
import numpy as np
import asyncio
import math

from pathfinder import Pathfinder
from world_map import WorldMap
from picarx_wrapper import PicarXWrapper
from vision_system import VisionSystem

class AsyncObstacleAvoidance:
    def __init__(self):
        # World mapping
        self.px = PicarXWrapper()
        self.world_map = WorldMap(map_size=200, resolution=1.0)  # 4m x 4m map, 1cm resolution
        self.pathfinder = Pathfinder(self.world_map, self.px)

        # Sensor offsets from center
        self.ULTRASONIC_OFFSET_X = 5.0  # cm forward
        self.ULTRASONIC_OFFSET_Y = 0.0  # cm sideways
        self.CAMERA_OFFSET_X = 5.0  # cm forward
        self.CAMERA_OFFSET_Y = 0.0  # cm sideways

        # configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05

        # state management
        self.current_distance = 100
        self.is_moving = False
        self.current_maneuver = None
        self.emergency_stop_flag = False
        self.is_backing_up = False  # since this is async we don't want to interrupt evasive backup maneuvers while obejcts are still too close
        self.is_cliff = False

        # vision
        self.vision = VisionSystem()
        self.vision_enabled = True
        self.vision_clear = True

        # scanning parameters
        self.scan_range = (-60, 60)
        self.scan_step = 5

        self.navigation_target = None
        self.navigation_task = None
        self.is_navigating = False

    def _update_ultrasonic_detection(self, distance: float):
        """Update map with obstacle detected by ultrasonic sensor"""
        if not (0 < distance < 300):  # Ignore invalid readings
            return

        # Calculate obstacle position in world coordinates
        sensor_angle_rad = math.radians(self.px.heading)
        sensor_x = self.px.x + self.ULTRASONIC_OFFSET_X * math.cos(sensor_angle_rad)
        sensor_y = self.px.y + self.ULTRASONIC_OFFSET_X * math.sin(sensor_angle_rad)

        obstacle_x = sensor_x + distance * math.cos(sensor_angle_rad)
        obstacle_y = sensor_y + distance * math.sin(sensor_angle_rad)

        # Add to world map
        self.world_map.add_obstacle(
            x=obstacle_x,
            y=obstacle_y,
            radius=5.0,  # Assume 5cm radius for ultrasonic detections
            confidence=0.8,
            label="ultrasonic_detection"
        )

    def _update_vision_detections(self, detected_objects: List[Dict]):
        """Update map with obstacles detected by vision system"""
        if not detected_objects:
            return

        for obj in detected_objects:
            # Calculate relative position from bounding box
            box = obj['box']  # (xmin, ymin, xmax, ymax)
            box_center_x = (box[0] + box[2]) / 2
            box_width = box[2] - box[0]

            # Estimate distance based on box width
            estimated_distance = 100 * (100 / box_width)  # Simplified example

            # Calculate object position in world coordinates
            camera_angle_rad = math.radians(self.px.heading)
            camera_x = (self.px.x + self.CAMERA_OFFSET_X * math.cos(camera_angle_rad))
            camera_y = (self.px.y + self.CAMERA_OFFSET_X * math.sin(camera_angle_rad))

            obstacle_x = camera_x + estimated_distance * math.cos(camera_angle_rad)
            obstacle_y = camera_y + estimated_distance * math.sin(camera_angle_rad)

            # Add to world map
            self.world_map.add_obstacle(
                x=obstacle_x,
                y=obstacle_y,
                radius=10.0,  # Adjust based on object type/size
                confidence=obj['confidence'],
                label=obj['label']
            )

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            await asyncio.sleep(0.01)

        print(f'distances: {distances}')
        if distances:
            self._update_ultrasonic_detection(sum(distances) / len(distances))
        return distances

    async def scan_environment(self):
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.05)
            await self.scan_avg()

        self.px.set_cam_pan_angle(0)
        return scan_data

    def _polar_to_cartesian(self, angle_deg, distance):
        """Convert polar coordinates to cartesian"""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return np.array([x, y])

    async def ultrasonic_monitoring(self):
        while True:
            distances = await self.scan_avg()
            if distances:
                self.current_distance = sum(distances) / len(distances)

                print(f"Distance: {self.current_distance:.1f} cm")

                # emergency stop if too close during forward movement only
                if (self.current_distance < self.min_distance and
                        self.is_moving and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):  # Don't interrupt backup
                    print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                    await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            self.is_cliff = self.px.get_cliff_status(self.px.get_grayscale_data())

            if (self.is_cliff and
                    self.is_moving and
                    not self.emergency_stop_flag and
                    not self.is_backing_up):
                print(f"Emergency stop, cliff detected!")
                await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)


    async def emergency_stop(self):
        self.emergency_stop_flag = True
        # cancel any ongoing maneuver except backup
        if self.current_maneuver:
            self.current_maneuver.cancel()

        self.is_moving = False
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

    def find_best_direction(self):
        """Analyze scan data to find the best direction to move"""
        max_distance = 0
        best_angle = 0

        # Convert current position to grid coordinates
        curr_pos = self.px.get_position()
        curr_grid_x, curr_grid_y = self.world_map.world_to_grid(curr_pos['x'], curr_pos['y'])

        # Check angles in scan range
        start_angle, end_angle = self.scan_range
        for angle in range(start_angle, end_angle + 1, self.scan_step):
            # Convert angle to radians
            angle_rad = math.radians(angle)

            # Look ahead in this direction (check multiple distances)
            max_check_distance = 100  # cm
            check_step = 5  # cm

            # Find distance to first obstacle in this direction
            distance_to_obstacle = max_check_distance

            for dist in range(check_step, max_check_distance, check_step):
                # Calculate point to check in grid coordinates
                check_x = curr_grid_x + int(dist * math.cos(angle_rad) / self.world_map.resolution)
                check_y = curr_grid_y + int(dist * math.sin(angle_rad) / self.world_map.resolution)

                # Ensure within grid bounds
                if (0 <= check_x < self.world_map.grid_size and
                        0 <= check_y < self.world_map.grid_size):

                    # If we hit an obstacle, record distance and stop checking this direction
                    if self.world_map.grid[check_y, check_x] != 0:
                        distance_to_obstacle = dist
                        break

            # Update best direction if this is the clearest path
            if distance_to_obstacle > max_distance:
                max_distance = distance_to_obstacle
                best_angle = angle

        return best_angle, max_distance

    async def evasive_maneuver(self):
        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            await self.scan_environment()
            # add padding once scanning is done
            self.world_map.add_padding()

            best_angle, max_distance = self.find_best_direction()

            # normal evasive maneuver
            print("Backing up...")
            self.is_moving = True
            self.is_backing_up = True
            self.px.backward(self.speed)
            await asyncio.sleep(self.backup_time)
            self.is_backing_up = False

            print(f"Turning to {best_angle}° (clearest path: {max_distance:.1f}cm)")
            self.px.set_dir_servo_angle(best_angle)
            self.px.forward(self.speed)
            await asyncio.sleep(self.turn_time)

            if not self.emergency_stop_flag:
                self.px.set_dir_servo_angle(0)
                self.is_moving = True
                self.px.forward(self.speed)

            self.px.set_dir_servo_angle(0)

        except asyncio.CancelledError:
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            self.is_moving = False
            raise

        finally:
            self.current_maneuver = None

    async def forward_movement(self):
        while True:
            position = self.px.get_position()
            print(f'{position}')
            if not self.emergency_stop_flag and not self.current_maneuver:
                # Check both ultrasonic and vision systems
                if self.vision_enabled:
                    objects = self.vision.get_obstacle_info()
                    if objects:
                        self._update_vision_detections(objects)
                        for obj in objects:
                            print(f"Vision system detected: {obj['label']}")
                            if obj['label'] == "stop sign":
                                print("STOP!!!!")
                                self.vision_clear = False

                if (self.current_distance >= self.min_distance and not self.is_cliff):
                    if not self.is_moving:
                        print("Moving forward...")
                        self.is_moving = True
                        self.px.forward(self.speed)
                else:
                    if self.is_moving:
                        if self.is_cliff:
                            print("Cliff detected!")
                        elif not self.vision_clear:
                            print("Vision system detected obstacle!")
                        else:
                            print(f"Ultrasonic detected obstacle at {self.current_distance:.1f}cm")
                        self.is_moving = False
                        self.px.forward(0)
                        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

            # Periodically visualize the map (for debugging)
            if time.time() % 5 < 0.1:  # Every 5 seconds
                print("\nCurrent World Map:")
                self.world_map.visualize_map()
                pos = self.px.get_position()
                print(f"Position: x={pos['x']:.1f}, y={pos['y']:.1f}, heading={pos['heading']:.1f}°")

            await asyncio.sleep(0.1)

    async def navigate_to_target(self, target_x: float, target_y: float):
        """Navigate to target coordinates while avoiding obstacles using forward_movement"""
        self.is_navigating = True
        current_path = None
        current_waypoint_idx = 1  # Skip first waypoint (current position)
        movement_task = None

        try:
            while True:
                # Get current position
                current_pos = self.px.get_position()
                current_x = current_pos['x']
                current_y = current_pos['y']

                # Check if we've reached the final target
                distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
                if distance_to_target < 5:  # 5cm threshold
                    print(f"Reached target: ({target_x}, {target_y})")
                    if movement_task:
                        movement_task.cancel()
                    self.px.stop()
                    self.is_navigating = False
                    return True

                # Plan or update path if needed
                if current_path is None or self.emergency_stop_flag:
                    # Cancel existing movement if any
                    if movement_task:
                        movement_task.cancel()
                        movement_task = None
                        await asyncio.sleep(0.5)  # Let cancellation complete

                    # Get new path
                    current_path = self.pathfinder.find_path(
                        (current_x, current_y),
                        (target_x, target_y)
                    )
                    if not current_path:
                        print("No valid path found. Scanning environment...")
                        self.px.stop()
                        await self.scan_environment()
                        await asyncio.sleep(1)
                        continue
                    current_waypoint_idx = 1
                    print(f"New path planned: {current_path}")

                # Get current waypoint
                if current_waypoint_idx >= len(current_path):
                    current_waypoint_idx = len(current_path) - 1
                waypoint_x, waypoint_y = current_path[current_waypoint_idx]

                # Check distance to current waypoint
                distance_to_waypoint = math.sqrt(
                    (waypoint_x - current_x) ** 2 + (waypoint_y - current_y) ** 2
                )

                # Move to next waypoint if we've reached the current one
                if distance_to_waypoint < 5:  # 5cm threshold
                    current_waypoint_idx += 1
                    if current_waypoint_idx >= len(current_path):
                        continue  # Will trigger new path planning
                    waypoint_x, waypoint_y = current_path[current_waypoint_idx]

                # Calculate angle to waypoint
                dx = waypoint_x - current_x
                dy = waypoint_y - current_y
                target_angle = math.degrees(math.atan2(dy, dx))

                # Calculate angle difference
                angle_diff = target_angle - self.px.heading
                # Normalize to -180 to 180
                angle_diff = (angle_diff + 180) % 360 - 180

                # If we need to turn more than 45 degrees, stop and turn first
                if abs(angle_diff) > 45:
                    if movement_task:
                        movement_task.cancel()
                        movement_task = None
                        await asyncio.sleep(0.5)

                    print(f"Turning to angle: {target_angle}")
                    await self.px.turn_to_heading(target_angle)
                    continue

                # Set steering angle for minor corrections
                steering_angle = self.px.calculate_steering_angle(angle_diff)
                self.px.set_dir_servo_angle(steering_angle)

                # Start or maintain forward movement
                if not movement_task or movement_task.done():
                    print(f"Starting forward movement toward waypoint: ({waypoint_x}, {waypoint_y})")
                    movement_task = asyncio.create_task(self.forward_movement())

                # Check if we need to replan due to obstacles
                if self.emergency_stop_flag or self.current_maneuver:
                    print("Obstacle detected, replanning path...")
                    current_path = None  # Force path replanning
                    continue

                await asyncio.sleep(0.1)

        except asyncio.CancelledError:
            print("Navigation cancelled!")
            if movement_task:
                movement_task.cancel()
            self.px.stop()
            self.is_navigating = False
            raise

        finally:
            self.is_navigating = False
            if movement_task:
                movement_task.cancel()
            self.px.stop()

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            await self.scan_environment()
            self.world_map.add_padding()

            # Add vision task to existing tasks
            pos_track_task = asyncio.create_task(self.px.continuous_position_tracking())
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            navigation_task = asyncio.create_task(self.navigate_to_target(100, 50))

            tasks = [pos_track_task, vision_task, ultrasonic_task, cliff_task, navigation_task]
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            print("\nShutting down gracefully...")
        finally:
            for task in tasks:
                task.cancel()
            try:
                await asyncio.gather(*tasks, return_exceptions=True)
            except asyncio.CancelledError:
                pass

            self.vision.cleanup()
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            print("Shutdown complete")