import numpy as np
import asyncio
import math
from state_handler import State
from world_map import WorldMap
from world_map_2 import WorldMap2
from picarx_wrapper import PicarXWrapper


# TODO:
#  separate commands.py and object system
#  attach detected labels to objects in world grid
#  add label to center of printed world grid surrounding picar?
class UltrasonicSystem:
    def __init__(self, state: State, px: PicarXWrapper):
        self.px = px
        self.__state = state
        # self.world_map = WorldMap()
        self.world_map = WorldMap2(self.__state)

    def __update_ultrasonic_detection(self, distance: float):
        """Update map with obstacle detected by ultrasonic sensor"""
        if not (0 < distance < 300):  # Ignore invalid readings
            return

        # Calculate obstacle position in world coordinates
        sensor_angle_rad = math.radians(self.__state.heading)
        sensor_x = self.__state.x + self.__state.ULTRASONIC_OFFSET_X * math.cos(sensor_angle_rad)
        sensor_y = self.__state.y + self.__state.ULTRASONIC_OFFSET_X * math.sin(sensor_angle_rad)

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
        return distances

    async def __sensor_func(self, angle):
        self.px.set_cam_pan_angle(angle)
        await asyncio.sleep(self.__state.scan_frequency)
        return await self.scan_avg()


    async def scan_environment(self):
        await self.world_map.scan_surroundings(
            sensor_func=self.__sensor_func,
            angle_range=self.__state.scan_range,
            angle_step=self.__state.scan_step,
        )
        # start_angle, end_angle = self.__state.scan_range
        #
        # for angle in range(start_angle, end_angle + 1, self.__state.scan_step):
        #     self.px.set_cam_pan_angle(angle)
        #     await asyncio.sleep(self.__state.scan_frequency)
        #     distance = await self.scan_avg()
        #     print(f'Environment Scan Distance: {distance}')
        # #
        self.px.set_cam_pan_angle(0)

    def __polar_to_cartesian(self, angle_deg, distance):
        """Convert polar coordinates to cartesian"""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return np.array([x, y])

    async def ultrasonic_monitoring(self):
        while True:
            distances = await self.scan_avg()
            if distances:
                self.__state.current_distance = sum(distances) / len(distances)

                if (self.__state.current_distance < self.__state.min_distance and
                        self.__state.is_moving and
                        not self.__state.emergency_stop_flag):
                    print(f"Emergency stop! Object detected at {self.__state.current_distance:.1f}cm")
                    await self.emergency_stop()

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

    # cancel movement and set emergency stop
    async def emergency_stop(self):
        print("emergency stop, hazard detected!")
        self.__state.emergency_stop_flag = True

        self.px.stop()
        await asyncio.sleep(0.1)
        if self.__state.movement_task:
            self.__state.movement_task.cancel()
            self.__state.movement_task = None

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
