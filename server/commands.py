import time
from typing import List, Dict, Tuple
import numpy as np
import asyncio
import math
from state_handler import State
from streaming_vision_system import VisionSystem
from picarx_wrapper import PicarXWrapper
from ultrasonic_system import UltrasonicSystem

# TODO:
#  attach detected labels to objects in world grid
#  add label to center of printed world grid surrounding picar?
class Commands:
    def __init__(self):
        self.state = State()
        self.px = PicarXWrapper(self.state)
        self.vision = VisionSystem(self.state)
        self.object_system = UltrasonicSystem(self.state, self.px)

        # task states
        # always running for safety
        self.state.ultrasonic_task = None
        self.state.scan_task = None
        self.state.cliff_task = None
        self.state.pos_track_task = None

    def __post_init__(self):
        self.state.ultrasonic_task = asyncio.create_task(self.object_system.ultrasonic_monitoring())
        self.state.cliff_task = asyncio.create_task(self.object_system.cliff_monitoring())
        self.state.pos_track_task = asyncio.create_task(self.object_system.px.continuous_position_tracking())

    async def scan_env(self):
        """
        Scan the environment and return the world state

        Returns:
            Dictionary with scan results and visualization data
        """
        # Perform a scan
        print("Starting environment scan...")
        await self.object_system.scan_environment()
        print("Environment scan completed")

        # Generate visualization right after scan completes
        visualization_data = self.object_system.world_map.visualize()

        # Get the world state
        world_data = self.world_state()

        # Include visualization data in response
        return {
            "status": "success",
            "data": {
                "grid_data": visualization_data['grid_data'],
                "plot_image": visualization_data['visualization'],
                "world_state": world_data
            }
        }

    def start_vision(self):
        self.state.vision_task = asyncio.create_task(self.vision.capture_and_detect())

    def stop_vision(self):
        if self.state.vision_task:
            self.state.vision_task.cancel()
            self.state.vision_task = None

    def get_objects(self):
        if self.state.vision_task:
            return self.vision.get_obstacle_info()
        return []

    # def start_ultrasonic(self):
    #     self.ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())

    # def stop_ultrasonic(self):
    #     if self.ultrasonic_task:
    #         self.ultrasonic_task.cancel()
    #         self.ultrasonic_task = None

    def get_object_distance(self):
        return self.state.current_distance

    # TODO: set speed state
    def forward(self):
        if self.state.emergency_stop_flag:
            print('hazard detected ahead, unable to move')
            return False

        self.state.movement_task = asyncio.create_task(self.px.forward(self.state.speed))
        return True

    def backward(self):
        self.state.emergency_stop_flag = False
        self.state.movement_task = asyncio.create_task(self.px.backward(self.state.speed))

    def turn(self, angle: int):
        self.object_system.px.set_dir_servo_angle(angle)
        time.sleep(0.1)
        return True

    def cancel_movement(self):
        self.object_system.px.stop()
        time.sleep(0.1)
        if self.state.movement_task:
            self.state.movement_task.cancel()
            self.state.movement_task = None

    def world_state(self):
        """
        Get the current world state including robot position, heading, and detected objects

        Returns:
            Dictionary with world state data
        """
        # Get position data
        position = self.object_system.px.get_position()

        # Get ultrasonic data
        distance = self.object_system.get_current_distance()

        # Get detected objects if vision system is active
        objects = []
        if self.state.vision_task and not self.state.vision_task.done():
            objects = self.vision.detected_objects

        return {
            "position": position,
            "distance": distance,
            "objects": objects,
            "emergency_stop": self.state.emergency_stop_flag,
            "is_moving": self.state.is_moving,
            "is_cliff": self.state.is_cliff,
            'ascii_map': self.object_system.world_map.get_ascii_map(),
            'visualization': self.object_system.world_map.visualize(return_image=True)
        }
