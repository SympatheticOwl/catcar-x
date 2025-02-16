import numpy as np
import asyncio
import math
from vision_system import VisionSystem
from scipy import ndimage

class AsyncObstacleAvoidance:
    def __init__(self, picarx_wrapper):
        self.px = picarx_wrapper

        # configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05

        # padding configuration
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
        self.goal_x = 0
        self.goal_y = 0
        self.navigation_active = False

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.map:
            print(''.join(['1' if cell else '0' for cell in row]))

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.px.ultrasonic.read()  # Note the nested px access
            if dist and 0 < dist < 300:
                distances.append(dist)
            await asyncio.sleep(0.01)
        return distances

    async def scan_environment(self):
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)

            distances = await self.scan_avg()
            if distances:
                avg_dist = sum(distances) / len(distances)
                scan_data.append((angle, avg_dist))
                # Update wrapper's obstacle tracking
                self.px.add_obstacle(avg_dist, angle)

        self.px.px.set_cam_pan_angle(0)
        return scan_data

    def update_map(self, scan_data):
        self.map = np.zeros((self.map_size, self.map_size))

        # Convert car's position to map coordinates
        car_pos = np.array([self.px.x, self.px.y])

        for angle, distance in scan_data:
            # Convert to global coordinates using wrapper's position
            x = self.px.x + distance * math.cos(math.radians(angle + math.degrees(self.px.heading)))
            y = self.px.y + distance * math.sin(math.radians(angle + math.degrees(self.px.heading)))

            # Convert to map coordinates
            map_x = int(x + self.map_size // 2)
            map_y = int(y + self.map_size // 2)

            if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
                self.map[map_y, map_x] = 1

        # Apply padding
        self.map = ndimage.binary_dilation(
            self.map, structure=self.padding_structure,
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
            if not self.is_backing_up:  # Don't interrupt backup maneuvers
                distances = await self.scan_avg()
                if distances:
                    self.current_distance = sum(distances) / len(distances)
                    if (self.current_distance < self.min_distance and
                            self.px.is_moving and
                            not self.emergency_stop_flag):
                        await self.emergency_stop()
            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            if not self.is_backing_up:  # Don't interrupt backup maneuvers
                self.is_cliff = self.px.px.get_cliff_status(self.px.px.get_grayscale_data())
                if (self.is_cliff and
                        self.px.is_moving and
                        not self.emergency_stop_flag):
                    await self.emergency_stop()
            await asyncio.sleep(self.sensor_read_freq)

    async def emergency_stop(self):
        self.emergency_stop_flag = True
        self.px.stop()
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        await self.evasive_maneuver()

    def find_best_direction(self, scan_data):
        max_distance = 0
        best_angle = 0

        if self.navigation_active:
            # Bias towards goal direction
            angle_to_goal = math.degrees(self.px.get_angle_to_goal(self.goal_x, self.goal_y))

            for angle, distance in scan_data:
                # Weight distance by how close the angle is to goal direction
                angle_diff = min(abs(angle - angle_to_goal), 360 - abs(angle - angle_to_goal))
                weighted_distance = distance * (1 - angle_diff / 180)

                if weighted_distance > max_distance:
                    max_distance = weighted_distance
                    best_angle = angle
        else:
            # Simple distance-based selection
            for angle, distance in scan_data:
                if distance > max_distance:
                    max_distance = distance
                    best_angle = angle

        return best_angle, max_distance

    async def evasive_maneuver(self):
        try:
            self.is_backing_up = True  # Prevent interruption during backup

            # Scan environment for obstacles
            scan_data = await self.scan_environment()
            self.update_map(scan_data)

            # Backup
            print("Backing up...")
            await self.px.move_distance(-20, self.speed)
            await asyncio.sleep(self.backup_time)

            # Find best direction
            best_angle, _ = self.find_best_direction(scan_data)

            # Turn and move forward
            print(f"Turning to {best_angle}°")
            await self.px.turn_to_angle(best_angle)
            await asyncio.sleep(0.5)

            if self.navigation_active:
                # Resume goal navigation
                angle_to_goal = math.degrees(self.px.get_angle_to_goal(self.goal_x, self.goal_y))
                await self.px.turn_to_angle(angle_to_goal)
            else:
                # Move forward in new direction
                await self.px.move_distance(20, self.speed)

        finally:
            self.is_backing_up = False

    async def navigate_to_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.navigation_active = True

        while self.navigation_active:
            if not self.emergency_stop_flag and not self.is_backing_up:
                success = await self.px.navigate_to_goal(x, y, self.speed)
                if success:
                    print(f"Reached goal: ({x}, {y})")
                    self.navigation_active = False
                    break
            await asyncio.sleep(0.1)

    async def run(self):
        print("Starting enhanced obstacle avoidance with goal navigation...")
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
