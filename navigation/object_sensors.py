import numpy as np
import asyncio
import math
from vision_system import VisionSystem
from scipy import ndimage
from picarx_wrapper import PicarXWrapper


class AsyncObstacleAvoidance:
    def __init__(self):
        self.pxw = PicarXWrapper()

        # configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05

        # Add padding configuration
        self.padding_size = 2
        self.padding_structure = np.ones((3, 3))

        # state management
        self.current_distance = 100
        self.emergency_stop_flag = False
        self.is_backing_up = False
        self.is_cliff = False

        # vision
        self.vision = VisionSystem()
        self.vision_enabled = True
        self.vision_clear = True

        # scanning parameters
        map_size = 100
        self.map_size = map_size
        self.map = np.zeros((map_size, map_size))
        self.scan_range = (-60, 60)
        self.scan_step = 5

        # navigation parameters
        self.current_goal = None
        self.navigation_active = False

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.map:
            print(''.join(['1' if cell else '0' for cell in row]))

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.pxw.px.ultrasonic.read()  # Note the nested px access
            if dist and 0 < dist < 300:
                distances.append(dist)
            await asyncio.sleep(0.01)
        return distances

    async def scan_environment(self):
        """Perform a sensor sweep and return scan data"""
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.pxw.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)

            distances = await self.scan_avg()
            if distances:
                avg_dist = sum(distances) / len(distances)
                scan_data.append((angle, avg_dist))

        self.pxw.px.set_cam_pan_angle(0)
        return scan_data

    def update_map(self, scan_data):
        """Update internal map with new scan data and add padding around obstacles"""
        self.map = np.zeros((self.map_size, self.map_size))

        current_pos = self.pxw.get_position()

        for angle, distance in scan_data:
            # Convert to global coordinates using current position
            point = self._polar_to_cartesian(angle, distance)
            map_x = int(point[0] + current_pos['x'] + self.map_size // 2)
            map_y = int(point[1] + current_pos['y'] + self.map_size // 2)

            if (0 <= map_x < self.map_size and 0 <= map_y < self.map_size):
                self.map[map_y, map_x] = 1
                self.pxw.add_obstacle(distance, angle)

        # Apply padding using binary dilation
        self.map = ndimage.binary_dilation(
            self.map,
            structure=self.padding_structure,
            iterations=self.padding_size
        )

        print("\nCurrent Map (with padding):")
        self.visualize_map()

    def _polar_to_cartesian(self, angle_deg, distance):
        """Convert polar coordinates to cartesian"""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return np.array([x, y])

    async def ultrasonic_monitoring(self):
        while True:
            if self.navigation_active:
                distances = await self.scan_avg()
                if distances:
                    self.current_distance = sum(distances) / len(distances)
                    print(f"Distance: {self.current_distance:.1f} cm")

                    if (self.current_distance < self.min_distance and
                            not self.emergency_stop_flag and
                            not self.is_backing_up):
                        print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                        await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            if self.navigation_active:
                self.is_cliff = self.pxw.px.get_cliff_status(self.pxw.px.get_grayscale_data())

                if (self.is_cliff and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):
                    print(f"Emergency stop, cliff detected!")
                    await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def emergency_stop(self):
        self.emergency_stop_flag = True
        self.pxw.stop()
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        await self.evasive_maneuver()

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

    async def evasive_maneuver(self):
        try:
            self.pxw.stop()
            await asyncio.sleep(0.5)

            scan_data = await self.scan_environment()
            self.update_map(scan_data)

            print("Backing up...")
            self.is_backing_up = True
            await self.pxw.move_distance(-self.backup_time * self.speed, self.speed)
            self.is_backing_up = False

            # Find clearest path
            max_distance = 0
            best_angle = 0
            for angle, distance in scan_data:
                if distance > max_distance:
                    max_distance = distance
                    best_angle = angle

            print(f"Turning to {best_angle}° (clearest path: {max_distance:.1f}cm)")
            await self.pxw.turn_to_angle(best_angle)

            # Resume navigation if we have a goal
            if self.current_goal and not self.emergency_stop_flag:
                await self.navigate_to_goal(*self.current_goal)

        except asyncio.CancelledError:
            self.pxw.stop()
            raise

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
                        self.pxw.forward(self.speed)
                else:
                    if self.is_moving:
                        if self.is_cliff:
                            print("Cliff detected!")
                        elif not self.vision_clear:
                            print("Vision system detected obstacle!")
                        else:
                            print(f"Ultrasonic detected obstacle at {self.current_distance:.1f}cm")
                        self.is_moving = False
                        self.pxw.forward(0)
                        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

            await asyncio.sleep(0.1)

    async def navigate_to_goal(self, goal_x, goal_y):
        """Navigate to a goal position while avoiding obstacles"""
        self.current_goal = (goal_x, goal_y)
        self.navigation_active = True

        try:
            while True:
                if self.emergency_stop_flag:
                    await asyncio.sleep(0.1)
                    continue

                success = await self.pxw.navigate_to_goal(goal_x, goal_y, self.speed)
                if success:
                    print(f"Reached goal: ({goal_x}, {goal_y})")
                    self.current_goal = None
                    self.navigation_active = False
                    return True
                elif not success and not self.emergency_stop_flag:
                    # If navigation failed but not due to emergency stop,
                    # scan environment and try again
                    scan_data = await self.scan_environment()
                    self.update_map(scan_data)
                    await asyncio.sleep(0.5)

                await asyncio.sleep(0.1)

        except asyncio.CancelledError:
            self.pxw.stop()
            self.current_goal = None
            self.navigation_active = False
            raise

    async def run(self):
        """Main control loop"""
        print("Starting enhanced navigation system...")
        tasks = []
        try:
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            tasks = [vision_task, ultrasonic_task, cliff_task]
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
            self.pxw.stop()
            print("Shutdown complete")
