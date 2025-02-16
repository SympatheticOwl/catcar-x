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

        # padding configuration
        self.padding_size = 2
        self.padding_structure = np.ones((3, 3))

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

        # scanning parameters
        map_size = 100
        self.map_size = map_size
        self.map = np.zeros((map_size, map_size))
        self.scan_range = (-60, 60)
        self.scan_step = 5

        # Goal tracking
        self.current_goal = None
        self.goal_reached = False

    def visualize_map(self):
        for row in self.map:
            print(''.join(['1' if cell else '0' for cell in row]))

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.px.ultrasonic.read()
            if dist and 0 < dist < 300:
                distances.append(dist)
            else:
                distances.append(300)
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

        current_pos = self.px.get_position()
        car_x, car_y = current_pos['x'], current_pos['y']

        for angle, distance in scan_data:
            x = car_x + distance * math.cos(math.radians(angle))
            y = car_y + distance * math.sin(math.radians(angle))

            # Convert to map coordinates
            map_x = int(x + self.map_size // 2)
            map_y = int(y + self.map_size // 2)

            if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
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

                if (self.current_distance < self.min_distance and
                        self.is_moving and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):
                    print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                    await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            self.is_cliff = self.px.px.get_cliff_status(self.px.px.get_grayscale_data())

            if (self.is_cliff and
                    self.is_moving and
                    not self.emergency_stop_flag and
                    not self.is_backing_up):
                print(f"Emergency stop, cliff detected!")
                await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def emergency_stop(self):
        self.emergency_stop_flag = True
        if self.current_maneuver and not self.is_backing_up:
            self.current_maneuver.cancel()

        self.is_moving = False
        self.px.stop()
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

    def find_best_direction(self, scan_data, current_pos):
        # Consider current position, obstacles, and goal when choosing direction
        if self.current_goal:
            goal_x, goal_y = self.current_goal
            goal_angle = math.degrees(self.px.get_angle_to_goal(goal_x, goal_y))

            # Find the scan direction closest to goal that's clear of obstacles
            best_angle = None
            min_angle_diff = float('inf')

            for angle, distance in scan_data:
                if distance > self.min_distance * 1.5:  # Add safety margin
                    angle_diff = abs(angle - goal_angle)
                    if angle_diff < min_angle_diff:
                        min_angle_diff = angle_diff
                        best_angle = angle

            if best_angle is not None:
                return best_angle

        # If no goal or no clear path toward goal, use original logic
        max_distance = 0
        best_angle = 0
        for angle, distance in scan_data:
            if distance > max_distance:
                max_distance = distance
                best_angle = angle
        return best_angle

    async def evasive_maneuver(self):
        try:
            self.is_moving = False
            self.px.stop()
            await asyncio.sleep(0.5)

            scan_data = await self.scan_environment()
            self.update_map(scan_data)

            # Find best direction using wrapper's position tracking
            current_pos = self.px.get_position()
            best_angle = self.find_best_direction(scan_data, current_pos)

            print("Backing up...")
            self.is_moving = True
            self.is_backing_up = True
            await self.px.move_distance(-self.backup_time * self.speed)  # Approximate distance
            self.is_backing_up = False

            print(f"Turning to {best_angle}°")
            await self.px.turn_to_angle(best_angle)

            if not self.emergency_stop_flag:
                self.is_moving = True
                await self.px.move_distance(self.turn_time * self.speed)  # Approximate distance

        except asyncio.CancelledError:
            self.px.stop()
            self.is_moving = False
            raise
        finally:
            self.current_maneuver = None

    async def goal_navigation(self):
        while True:
            if self.current_goal and not self.emergency_stop_flag and not self.current_maneuver:
                goal_x, goal_y = self.current_goal
                success = await self.px.navigate_to_goal(goal_x, goal_y, self.speed)

                if success:
                    print(f"Goal reached: ({goal_x}, {goal_y})")
                    self.goal_reached = True
                    self.current_goal = None
                elif not self.emergency_stop_flag:
                    # If navigation failed but not due to emergency stop,
                    # wait for evasive maneuver to complete before retrying
                    await asyncio.sleep(1.0)

            await asyncio.sleep(0.1)

    async def run(self, goal_x=None, goal_y=None):
        print("Starting enhanced obstacle avoidance with goal navigation...")
        if goal_x is not None and goal_y is not None:
            self.current_goal = (goal_x, goal_y)

        tasks = []
        try:
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            navigation_task = asyncio.create_task(self.goal_navigation())

            tasks = [vision_task, ultrasonic_task, cliff_task, navigation_task]
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
