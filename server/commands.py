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
        self.state.scan_task = asyncio.create_task(self.object_system.scan_environment())
        await self.state.scan_task
        return self.world_state()

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
        """Return both ASCII and grid data representations"""
        return {
            'ascii_map': self.object_system.world_map.ascii_visualize(),
            'grid_data': self.object_system.world_map.get_grid_data()
        }