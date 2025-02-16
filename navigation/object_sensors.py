import numpy as np
import asyncio
import math
from picarx_wrapper import PicarXWrapper
from vision_system import VisionSystem
from scipy import ndimage


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
        map_size = 100
        self.map_size = map_size
        self.map = np.zeros((map_size, map_size))
        self.scan_range = (-60, 60)
        self.scan_step = 5

        self.padding_size = 2  # Number of cells to pad around obstacles
        self.padding_structure = np.ones((2, 2))  # 3x3 structuring element for dilation

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.map:
            print(''.join(['1' if cell else '0' for cell in row]))

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            await asyncio.sleep(0.01)
        return distances

    async def scan_environment(self):
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
        return scan_data

    def update_map(self, scan_data):
        self.map = np.zeros((self.map_size, self.map_size))

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

        # Apply padding
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

    def find_best_direction(self, scan_data):
        """Analyze scan data to find the best direction to move"""
        max_distance = 0
        best_angle = 0

        for angle, distance in scan_data:
            if distance > max_distance:
                max_distance = distance
                best_angle = angle

        return best_angle, max_distance

    async def evasive_maneuver(self):
        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            scan_data = await self.scan_environment()
            self.update_map(scan_data)

            best_angle, max_distance = self.find_best_direction(scan_data)

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

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            # Add vision task to existing tasks
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            movement_task = asyncio.create_task(self.run_navigation_test())
            tasks = [vision_task, ultrasonic_task, cliff_task, movement_task]
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
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            print("Shutdown complete")