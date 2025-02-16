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

        for angle, distance in self.world_map.grid:
            if distance > max_distance:
                max_distance = distance
                best_angle = angle

        return best_angle, max_distance

    async def evasive_maneuver(self):
        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            await self.scan_environment()
            # add padding once scanning is done
            # self.world_map.add_padding()

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

    async def navigate_to_goal(self, target_x: float, target_y: float):
        """Main navigation loop with dynamic replanning"""
        self.pathfinder.is_navigating = True
        self.pathfinder.target_x = target_x
        self.pathfinder.target_y = target_y

        while self.is_navigating:
            current_time = time.time()

            # Get current position in grid coordinates
            current_pos = self.px.get_position()
            start_grid_x, start_grid_y = self.world_map.world_to_grid(
                current_pos['x'], current_pos['y'])
            goal_grid_x, goal_grid_y = self.world_map.world_to_grid(
                target_x, target_y)

            # Check if we've reached the goal
            distance_to_goal = math.sqrt(
                (current_pos['x'] - target_x) ** 2 +
                (current_pos['y'] - target_y) ** 2)
            if distance_to_goal < 5:  # 5cm threshold
                print("Reached goal!")
                self.is_navigating = False
                self.px.stop()
                break

            # Replan if needed
            if (current_time - self.pathfinder.last_plan_time > self.pathfinder.replan_interval or
                    not self.pathfinder.current_path or
                    self.pathfinder.current_path_index >= len(self.pathfinder.current_path)):

                # Find new path
                new_path = self.pathfinder.find_path(
                    (start_grid_x, start_grid_y),
                    (goal_grid_x, goal_grid_y)
                )

                if new_path is None:
                    print("No valid path found!")
                    self.is_navigating = False
                    self.px.stop()
                    break

                # Smooth and update path
                self.pathfinder.current_path = self.pathfinder.smooth_path(new_path)
                self.pathfinder.current_path_index = 0
                self.pathfinder.last_plan_time = current_time

            # Get next few waypoints to follow
            current_segment = self.pathfinder.current_path[
                              self.pathfinder.current_path_index:
                              self.pathfinder.current_path_index + self.pathfinder.path_segment_length]

            # Follow path segment
            for waypoint in current_segment:
                # Convert grid coordinates back to world coordinates
                way_x, way_y = self.world_map.grid_to_world(*waypoint)

                # Check for obstacles before moving
                vision_clear = await self.pathfinder.check_vision_obstacles()
                if not vision_clear:
                    continue

                # Navigate to waypoint
                print(f"Moving to waypoint: ({way_x:.1f}, {way_y:.1f})")
                await self.px.navigate_to_point(way_x, way_y)
                self.pathfinder.current_path_index += 1

            await asyncio.sleep(0.1)

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            # Add vision task to existing tasks
            pos_track_task = asyncio.create_task(self.px.continuous_position_tracking())
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            navigation_task = asyncio.create_task(self.navigate_to_goal(100, 50))
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