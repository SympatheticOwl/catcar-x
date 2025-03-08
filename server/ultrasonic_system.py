from typing import List, Optional

import numpy as np
import asyncio

from state_handler import State
from world_map import WorldMap
from picarx_wrapper import PicarXWrapper


# TODO:
#  separate commands.py and object system
#  attach detected labels to objects in world grid
#  add label to center of printed world grid surrounding picar?
class UltrasonicSystem:
    def __init__(self, state: State, px: PicarXWrapper):
        self.px = px
        self.__state = state
        self.world_map = WorldMap(self.__state)

        # Buffer for recent readings to help with filtering
        self.recent_readings = []
        self.max_buffer_size = 5

        # Debug counter for scan cycles
        self.scan_count = 0

    async def scan_avg(self, num_samples: int = 3) -> List[float]:
        distances = []
        for _ in range(num_samples):
            dist = self.px.ultrasonic.read()
            # Only add valid readings (not too close, not too far)
            if dist and 5 < dist < 300:
                distances.append(dist)
            await asyncio.sleep(0.01)

        # Store readings in buffer for tracking over time
        if distances:
            median_dist = sorted(distances)[len(distances) // 2]
            self.recent_readings.append(median_dist)
            # Keep buffer at max size
            if len(self.recent_readings) > self.max_buffer_size:
                self.recent_readings.pop(0)

        # Calculate average with noise rejection
        average = self.calculate_filtered_distance(distances)

        self.__state.current_distance = average
        return distances

    def calculate_filtered_distance(self, distances: List[float]) -> float:
        if not distances:
            return 300  # Default safe distance if no valid readings

        # If we have enough readings, remove outliers
        if len(distances) >= 3:
            # Sort distances and remove highest and lowest
            sorted_dist = sorted(distances)
            filtered_dist = sorted_dist[1:-1]

            # Return average of remaining values
            return sum(filtered_dist) / len(filtered_dist)
        else:
            # Not enough readings to filter outliers
            return sum(distances) / len(distances)

    async def scan_environment(self):
        self.scan_count += 1
        print(f"Starting scan cycle #{self.scan_count}")

        async def __sensor_func(angle):
            # Set servo angle and wait for it to stabilize
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(self.__state.scan_frequency)

            # Take multiple readings at this angle to improve reliability
            distances = await self.scan_avg()

            # Log reading for debugging
            if distances:
                median = sorted(distances)[len(distances) // 2]
                print(f"Angle: {angle:3d}°, Distance: {median:6.2f} cm, Readings: {len(distances)}")
            else:
                print(f"Angle: {angle:3d}°, No valid readings")

            return self.__state.current_distance

        # Perform the scan and update the world map
        await self.world_map.scan_surroundings(sensor_func=__sensor_func)

        # Return servo to center position
        self.px.set_cam_pan_angle(0)

        print(f"Scan cycle #{self.scan_count} completed")

    async def ultrasonic_monitoring(self):
        while True:
            # Get current distance reading
            await self.scan_avg()

            # Check for emergency stop conditions
            if (self.__state.current_distance < self.__state.min_distance and
                    self.__state.is_moving and
                    not self.__state.is_backing_up and
                    not self.__state.emergency_stop_flag):
                print(f"Emergency stop! Object detected at {self.__state.current_distance:.1f}cm")
                await self.emergency_stop()
            elif not self.__state.is_moving:
                self.__state.emergency_stop_flag = False

            await asyncio.sleep(self.__state.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            self.__state.is_cliff = self.px.get_cliff_status(self.px.get_grayscale_data())

            if (self.__state.is_cliff and
                    self.__state.is_moving and
                    not self.__state.is_backing_up and
                    not self.__state.emergency_stop_flag):
                print(f"Emergency stop, cliff detected!")
                await self.emergency_stop()

            await asyncio.sleep(self.__state.sensor_read_freq)

    async def emergency_stop(self):
        print("Emergency stop, hazard detected!")
        self.__state.emergency_stop_flag = True

        self.px.stop()
        await asyncio.sleep(0.1)
        if self.__state.movement_task:
            self.__state.movement_task.cancel()
            self.__state.movement_task = None

    def get_current_distance(self) -> float:
        return self.__state.current_distance

    def reset(self):
        self.scan_count = 0
        self.recent_readings = []
        self.world_map.clear_grid()
        self.world_map.clear_scan_points()
