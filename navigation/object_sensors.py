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
        self.px = PicarXWrapper()
        self.world_map = WorldMap()  # 4m x 4m map, 1cm resolution
        self.pathfinder = Pathfinder(self.world_map, self.px)

        # Sensor offsets from center
        self.ULTRASONIC_OFFSET_X = 5.0  # cm forward
        self.ULTRASONIC_OFFSET_Y = 0.0  # cm sideways
        self.CAMERA_OFFSET_X = 5.0  # cm forward
        self.CAMERA_OFFSET_Y = 0.0  # cm sideways

        # configuration parameters
        self.min_distance = 25
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05
        self.scan_range = (-60, 60)
        self.scan_step = 5

        # state management
        self.current_distance = 100
        self.is_moving = False
        self.current_maneuver = None
        self.emergency_stop_flag = False
        self.is_backing_up = False
        self.is_cliff = False

        # vision
        self.vision = VisionSystem()
        self.vision_enabled = True
        self.vision_clear = True

        map_size = 100
        self.map_size = map_size
        self.map = np.zeros((map_size, map_size))
        self.car_pos = np.array([0, map_size // 2])
        self.car_angle = 0


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

        if distances:
            self._update_ultrasonic_detection(sum(distances) / len(distances))
        return distances

    async def scan_environment(self):
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.05)
            distance = await self.scan_avg()
            print(f'Environment Scan Distance: {distance}')

        self.px.set_cam_pan_angle(0)
        return scan_data

    async def old_scan_environment(self):
        """Perform a sensor sweep and return scan data"""
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)

            distances = await self.scan_avg()

            if distances:
                avg_dist = sum(distances) / len(distances)
                scan_data.append((angle, avg_dist))

        self.px.set_cam_pan_angle(0)
        self.update_map(scan_data)

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
        print("emergency stop!")
        self.emergency_stop_flag = True
        # cancel any ongoing maneuver except backup
        if self.current_maneuver:
            self.current_maneuver.cancel()

        self.is_moving = False
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        # self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

    def find_best_direction(self, scan_data):
        """Analyze scan data to find the best direction to move, accounting for padded obstacles"""
        max_distance = 0
        best_angle = 0

        # Consider the padded map when finding the best direction
        for angle, distance in scan_data:
            # Convert the measured point to map coordinates
            point = self._polar_to_cartesian(angle, distance)
            map_x = int(point[0] + self.map_size // 2)
            map_y = int(point[1] + self.map_size // 2)

            # Check if the point and its surrounding area (padding) are clear
            if (0 <= map_x < self.map_size and
                    0 <= map_y < self.map_size and
                    not self.map[map_y, map_x]):  # Check padded map
                if distance > max_distance:
                    max_distance = distance
                    best_angle = angle

        return best_angle, max_distance

    def update_map(self, scan_data):
        # Create initial map from scan data
        for i in range(len(scan_data) - 1):
            angle1, dist1 = scan_data[i]
            angle2, dist2 = scan_data[i + 1]

            # Convert to cartesian coordinates
            point1 = self._polar_to_cartesian(angle1, dist1)
            point2 = self._polar_to_cartesian(angle2, dist2)

            # Interpolate between points
            num_points = max(
                abs(int(point2[0] - point1[0])),
                abs(int(point2[1] - point1[1]))
            ) + 1

            if num_points > 1:
                x_points = np.linspace(point1[0], point2[0], num_points)
                y_points = np.linspace(point1[1], point2[1], num_points)

                # Mark interpolated points
                for x, y in zip(x_points, y_points):
                    map_x = int(x + self.map_size // 2)
                    map_y = int(y + self.map_size // 2)

                    if (0 <= map_x < self.map_size and
                            0 <= map_y < self.map_size):
                        self.map[map_y, map_x] = 1

        print("\nCurrent Map (with padding):")
        self.visualize_map()

    def visualize_map(self):
        for row in self.map:
            print(''.join(['1' if cell else '0' for cell in row]))

    async def evasive_maneuver(self):
        try:
            print("begin evasive maneuver")
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            await self.scan_environment()
            # add padding once scanning is done
            # self.world_map.add_padding()

            best_angle, max_distance = self.find_best_direction(self.map)

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

    async def navigate_to_point(self, target_x, target_y, speed=30):
        """Navigate to a target point using A* pathfinding"""
        print(f"Starting navigation to ({target_x}, {target_y})")

        while True:
            current_pos = self.px.get_position()
            current_x, current_y = current_pos['x'], current_pos['y']

            # Check if we've reached the target
            distance_to_target = math.sqrt(
                (target_x - current_x) ** 2 +
                (target_y - current_y) ** 2
            )
            if distance_to_target < 5:  # 5cm threshold
                print("Reached target!")
                self.px.stop()
                return True

            # Get current path or calculate new one
            if not self.pathfinder.current_path:
                print("Calculating initial path...")
                path = await self.pathfinder.find_path(
                    current_x, current_y,
                    target_x, target_y
                )
                if not path:
                    print("No valid path found!")
                    self.px.stop()
                    return False

            # Scan environment and update world map
            if self.current_distance < self.min_distance or not self.vision_clear:
                print("Obstacle detected! Stopping and scanning environment...")
                self.px.stop()

                # Perform a complete environment scan
                scan_data = await self.old_scan_environment()

                # Find best direction to turn
                print("Finding best direction to escape obstacle...")
                best_angle, max_distance = self.find_best_direction(scan_data)
                print(f"Best escape direction: {best_angle}° (clear path: {max_distance:.1f}cm)")

                # Update world map with scan data
                for angle, distance in scan_data:
                    self._update_ultrasonic_detection(distance)
                self.world_map.add_padding()

                # Turn to the best direction
                await self.px.turn_to_heading(self.px.heading + best_angle)

                # Update current position
                current_pos = self.px.get_position()
                current_x, current_y = current_pos['x'], current_pos['y']

                # Recalculate path from new heading
                print("Recalculating path from new heading...")
                path = await self.pathfinder.update_path(
                    current_x, current_y,
                    target_x, target_y
                )

                if not path:
                    print("No valid path found! Trying to find alternate route...")
                    # Try to find a path to a point slightly offset from the target
                    for offset in [(30, 0), (-30, 0), (0, 30), (0, -30)]:
                        alt_target_x = target_x + offset[0]
                        alt_target_y = target_y + offset[1]
                        path = await self.pathfinder.update_path(
                            current_x, current_y,
                            alt_target_x, alt_target_y
                        )
                        if path:
                            print("Found alternate route!")
                            break

                if not path:
                    print("No valid path found after obstacle detection!")
                    return False

            # Get next waypoint
            if len(self.pathfinder.current_path) > 1:
                next_x, next_y = self.pathfinder.current_path[1]

                # Calculate angle to next waypoint
                dx = next_x - current_x
                dy = next_y - current_y
                target_angle = math.degrees(math.atan2(dy, dx))

                # Calculate angle difference
                angle_diff = target_angle - self.px.heading
                # Normalize to -180 to 180
                angle_diff = (angle_diff + 180) % 360 - 180

                # Calculate steering angle
                steering_angle = self.px.calculate_steering_angle(angle_diff)
                self.px.set_dir_servo_angle(steering_angle)

                # Adjust speed based on turn sharpness
                adjusted_speed = speed * (1 - abs(steering_angle) / (2 * self.px.MAX_STEERING_ANGLE))

                # Move forward if no obstacles detected
                if self.current_distance >= self.min_distance and self.vision_clear:
                    self.px.forward(adjusted_speed)
                else:
                    self.px.stop()

                # Remove waypoint if we're close enough
                waypoint_distance = math.sqrt(
                    (next_x - current_x) ** 2 +
                    (next_y - current_y) ** 2
                )
                if waypoint_distance < 10:  # 10cm threshold
                    self.pathfinder.current_path.pop(0)

            await asyncio.sleep(0.1)

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            # Create all tasks
            pos_track_task = asyncio.create_task(self.px.continuous_position_tracking())
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            navigation_task = asyncio.create_task(self.navigate_to_point(100, 50))

            tasks = [
                pos_track_task,
                vision_task,
                ultrasonic_task,
                cliff_task,
                navigation_task
            ]

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
