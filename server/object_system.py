import time
from typing import List, Dict, Tuple
import numpy as np
import asyncio
import math

from world_map import WorldMap
from picarx_wrapper import PicarXWrapper
from vision_system import VisionSystem


class ObjectSystem:
    def __init__(self):
        self.px = PicarXWrapper()
        self.world_map = WorldMap()
        self.vision = VisionSystem()

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

        # task states
        # always running for safety
        self.ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
        self.cliff_task = asyncio.create_task(self.cliff_monitoring())
        self.pos_track_task = asyncio.create_task(self.px.continuous_position_tracking())
        self.vision_task = None
        self.movement_task = None

        # sensor state
        self.current_distance = 300
        self.is_moving = False
        self.is_cliff = False
        self.emergency_stop_flag = False
        self.vision_clear = True

        self.scan_range = (-60, 60)
        self.scan_step = 5


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

        # TODO add padding before danger check but dont add to base map?
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

                if (self.current_distance < self.min_distance and
                        self.is_moving and
                        not self.emergency_stop_flag):
                    print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                    await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            self.is_cliff = self.px.get_cliff_status(self.px.get_grayscale_data())

            if (self.is_cliff and
                    self.is_moving and
                    not self.emergency_stop_flag):
                print(f"Emergency stop, cliff detected!")
                await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    # cancel movement and set emergency stop
    async def emergency_stop(self):
        print("emergency stop, hazard detected!")
        self.emergency_stop_flag = True
        # cancel any ongoing maneuver except backup
        if self.movement_task:
            self.movement_task.cancel()

        self.is_moving = False
        self.px.forward(0)
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False

    def start_vision(self):
        self.vision_task = asyncio.create_task(self.vision.capture_and_detect())

    def stop_vision(self):
        if self.vision_task:
            self.vision_task.cancel()
            self.vision_task = None

    def get_objects(self):
        if self.vision_task:
            return self.vision.get_obstacle_info()
        return []

    # def start_ultrasonic(self):
    #     self.ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())

    # def stop_ultrasonic(self):
    #     if self.ultrasonic_task:
    #         self.ultrasonic_task.cancel()
    #         self.ultrasonic_task = None

    def get_object_distance(self):
        return self.current_distance

    # TODO: set speed state
    def forward(self):
        if self.emergency_stop_flag:
            print('hazard detected ahead, unable to move')
            return False

        self.is_moving = True
        self.movement_task = asyncio.create_task(self.px.forward(self.speed))
        return True

    def backward(self, direction: int):
        self.is_moving = True
        self.emergency_stop_flag = False
        self.movement_task = asyncio.create_task(self.px.backward(direction * self.speed))

    # TODO: adjust angle state
    def turn(self, direction: int):
        if self.emergency_stop_flag:
            print('hazard detected ahead, unable to move')
            return False

        self.is_moving = True
        self.px.set_dir_servo_angle(direction * 30)
        time.sleep(0.1)
        self.movement_task = asyncio.create_task(self.px.forward(self.speed))
        return True

    def cancel_movement(self):
        self.is_moving = False
        self.px.stop()
        time.sleep(0.1)
        self.movement_task = None


    # async def run(self):
    #     print("Starting enhanced obstacle avoidance program...")
    #     tasks = []
    #     try:
    #         # Create all tasks
    #
    #         ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
    #
    #
    #         tasks = [
    #             pos_track_task,
    #             ultrasonic_task,
    #             cliff_task,
    #         ]
    #
    #         await asyncio.gather(*tasks)
    #
    #     except asyncio.CancelledError:
    #         print("\nShutting down gracefully...")
    #     finally:
    #         for task in tasks:
    #             task.cancel()
    #         try:
    #             await asyncio.gather(*tasks, return_exceptions=True)
    #         except asyncio.CancelledError:
    #             pass
    #
    #         self.vision.cleanup()
    #         self.px.stop()
    #         self.px.set_dir_servo_angle(0)
    #         print("Shutdown complete")
