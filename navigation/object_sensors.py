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
        self.world_map = WorldMap(map_size=400, resolution=40.0)  # 4m x 4m map, 1cm resolution
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

        self.current_path = []
        self.current_path_index = 0

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
            if not self.emergency_stop_flag and not self.current_maneuver:
                # Check both ultrasonic and vision systems
                if self.vision_enabled:
                    objects = self.vision.get_obstacle_info()
                    if objects:
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

            await asyncio.sleep(0.1)

    async def navigate_to_target(self, target_x: float, target_y: float):
        """Navigate to target position while avoiding obstacles"""
        self.is_navigating = True
        self.navigation_target = (target_x, target_y)

        print(f"\nStarting navigation to target ({target_x}, {target_y})")

        try:
            while self.is_navigating:
                current_pos = self.px.get_position()
                print(f"\nCurrent position: ({current_pos['x']:.1f}, {current_pos['y']:.1f}), " +
                      f"heading: {current_pos['heading']:.1f}°")

                # Check if we've reached the target
                dist_to_target = math.sqrt(
                    (target_x - current_pos['x']) ** 2 +
                    (target_y - current_pos['y']) ** 2
                )
                print(f"Distance to target: {dist_to_target:.1f} cm")

                if dist_to_target < 20:  # Within 20cm
                    print("Reached target!")
                    self.px.stop()
                    self.is_navigating = False
                    return True

                # Check if we need to replan path
                need_replan = False

                if not self.current_path:
                    print("No current path - planning required")
                    need_replan = True
                elif self.current_distance < self.min_distance:
                    print(f"Obstacle detected at {self.current_distance:.1f} cm - replanning required")
                    need_replan = True
                elif not self.pathfinder.is_path_clear(self.current_path):
                    print("Current path blocked - replanning required")
                    need_replan = True

                # Plan or update path
                if need_replan:
                    print("\nPlanning new path...")
                    # Scan environment if obstacle detected
                    if self.current_distance < self.min_distance:
                        print("Scanning environment...")
                        await self.scan_environment()
                        self.world_map.add_padding()

                    # Find new path
                    self.current_path = self.pathfinder.find_path(
                        current_pos['x'], current_pos['y'],
                        target_x, target_y
                    )

                    if not self.current_path:
                        print("No valid path found - stopping navigation")
                        self.px.stop()
                        self.is_navigating = False
                        return False

                    print("Smoothing path...")
                    self.current_path = self.pathfinder.smooth_path(self.current_path)
                    self.current_path_index = 0
                    print(f"New path created with {len(self.current_path)} waypoints")

                # Follow current path
                if self.current_path_index < len(self.current_path):
                    next_x, next_y = self.current_path[self.current_path_index]
                    print(f"\nMoving to waypoint {self.current_path_index + 1}/{len(self.current_path)}: " +
                          f"({next_x:.1f}, {next_y:.1f})")

                    # Calculate heading to next waypoint
                    target_heading = self.px.get_target_heading(next_x, next_y)
                    angle_diff = target_heading - current_pos['heading']
                    # Normalize to -180 to 180
                    angle_diff = (angle_diff + 180) % 360 - 180

                    print(f"Target heading: {target_heading:.1f}°, " +
                          f"current heading: {current_pos['heading']:.1f}°, " +
                          f"difference: {angle_diff:.1f}°")

                    # Set steering and speed
                    steering_angle = self.px.calculate_steering_angle(angle_diff)
                    self.px.set_dir_servo_angle(steering_angle)

                    # Adjust speed based on turn sharpness
                    speed = self.speed * (1 - abs(steering_angle) / self.px.MAX_STEERING_ANGLE * 0.5)
                    print(f"Steering angle: {steering_angle:.1f}°, adjusted speed: {speed:.1f}")

                    self.px.forward(speed)
                    self.is_moving = True

                    # Check if waypoint reached
                    dist_to_waypoint = math.sqrt(
                        (next_x - current_pos['x']) ** 2 +
                        (next_y - current_pos['y']) ** 2
                    )
                    print(f"Distance to waypoint: {dist_to_waypoint:.1f} cm")

                    if dist_to_waypoint < 20:  # Within 20cm
                        print(f"Reached waypoint {self.current_path_index + 1}")
                        self.current_path_index += 1

                await asyncio.sleep(0.1)

        except asyncio.CancelledError:
            print("Navigation cancelled")
            self.px.stop()
            self.is_navigating = False
            raise
        except Exception as e:
            print(f"Navigation error: {e}")
            self.px.stop()
            self.is_navigating = False

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            # Add vision task to existing tasks
            pos_track_task = asyncio.create_task(self.px.continuous_position_tracking())
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            # navigation_task = asyncio.create_task(self.px.navigate_to_point(100, 50))
            navigation_task = asyncio.create_task(self.navigate_to_target(100, 50))

            tasks = [pos_track_task, vision_task, ultrasonic_task, cliff_task, navigation_task]
            await asyncio.gather(navigation_task)
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