import numpy as np
import asyncio
import math
from vision_system import VisionSystem
from scipy import ndimage
from picarx_wrapper import PicarXWrapper


class AsyncObstacleAvoidance:
    def __init__(self):
        self.px = PicarXWrapper()

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
        self.current_maneuver = None  # Track current maneuver task

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
        self.navigation_paused = False

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.map:
            print(''.join(['1' if cell else '0' for cell in row]))

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.px.ultrasonic.read()
            if dist and 0 < dist < 300:
                distances.append(dist)
            await asyncio.sleep(0.01)
        return distances

    async def scan_environment(self):
        """Perform a sensor sweep and return scan data"""
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)

            distances = await self.scan_avg()
            if distances:
                avg_dist = sum(distances) / len(distances)
                scan_data.append((angle, avg_dist))

        self.px.px.set_cam_pan_angle(0)
        return scan_data

    def update_map(self, scan_data):
        """Update internal map with new scan data and add padding around obstacles"""
        self.map = np.zeros((self.map_size, self.map_size))

        current_pos = self.px.get_position()

        for angle, distance in scan_data:
            # Convert to global coordinates using current position
            point = self._polar_to_cartesian(angle, distance)
            map_x = int(point[0] + current_pos['x'] + self.map_size // 2)
            map_y = int(point[1] + current_pos['y'] + self.map_size // 2)

            if (0 <= map_x < self.map_size and 0 <= map_y < self.map_size):
                self.map[map_y, map_x] = 1
                self.px.add_obstacle(distance, angle)

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
            if not self.emergency_stop_flag:
                distances = await self.scan_avg()
                if distances:
                    self.current_distance = sum(distances) / len(distances)
                    print(f"Distance: {self.current_distance:.1f} cm")

                    if (self.current_distance < self.min_distance and
                            self.px.is_moving and
                            not self.emergency_stop_flag):
                        print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                        await self.emergency_stop()
                        # Wait for maneuver to complete
                        while self.current_maneuver:
                            await asyncio.sleep(0.1)

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            if not self.emergency_stop_flag:
                self.is_cliff = self.px.px.get_cliff_status(self.px.px.get_grayscale_data())

                if (self.is_cliff and
                        self.px.is_moving and
                        not self.emergency_stop_flag):
                    print(f"Emergency stop, cliff detected!")
                    await self.emergency_stop()
                    # Wait for maneuver to complete
                    while self.current_maneuver:
                        await asyncio.sleep(0.1)

            await asyncio.sleep(self.sensor_read_freq)

    async def emergency_stop(self):
        """Emergency stop that pauses navigation and initiates evasive maneuver"""
        self.emergency_stop_flag = True
        self.navigation_paused = True

        # Cancel current maneuver if exists
        if self.current_maneuver:
            self.current_maneuver.cancel()
            self.current_maneuver = None

        self.px.stop()
        await asyncio.sleep(0.5)

        self.emergency_stop_flag = False
        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())
        await self.current_maneuver

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
        """Evasive maneuver that can be interrupted"""
        try:
            self.px.stop()
            await asyncio.sleep(0.5)

            scan_data = await self.scan_environment()
            self.update_map(scan_data)

            print("Backing up...")
            self.is_backing_up = True
            backup_start = asyncio.get_event_loop().time()

            # Interruptible backup movement
            self.px.px.backward(self.speed)
            while (asyncio.get_event_loop().time() - backup_start) < self.backup_time:
                if self.emergency_stop_flag:
                    break
                await asyncio.sleep(0.1)

            self.px.stop()
            self.is_backing_up = False

            if not self.emergency_stop_flag:
                # Find clearest path
                max_distance = 0
                best_angle = 0
                for angle, distance in scan_data:
                    if distance > max_distance:
                        max_distance = distance
                        best_angle = angle

                print(f"Turning to {best_angle}° (clearest path: {max_distance:.1f}cm)")
                await self.px.turn_to_angle(best_angle)

            self.navigation_paused = False

        except asyncio.CancelledError:
            self.px.stop()
            self.is_backing_up = False
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

    async def navigate_to_goal(self, goal_x, goal_y):
        """Navigate to a goal position while avoiding obstacles"""
        self.current_goal = (goal_x, goal_y)
        self.navigation_active = True
        self.navigation_paused = False

        try:
            while True:
                if self.emergency_stop_flag or self.navigation_paused:
                    await asyncio.sleep(0.1)
                    continue

                success = await self.px.navigate_to_goal(goal_x, goal_y, self.speed)
                if success:
                    print(f"Reached goal: ({goal_x}, {goal_y})")
                    self.current_goal = None
                    self.navigation_active = False
                    return True
                elif not success and not self.emergency_stop_flag:
                    # If navigation failed but not due to emergency stop,
                    # scan environment and try again
                    self.navigation_paused = True
                    scan_data = await self.scan_environment()
                    self.update_map(scan_data)
                    self.navigation_paused = False
                    await asyncio.sleep(0.5)

                await asyncio.sleep(0.1)

        except asyncio.CancelledError:
            self.px.stop()
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
            self.px.stop()
            print("Shutdown complete")
