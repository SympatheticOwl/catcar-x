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
#  separate commands.py and object system
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

    def scan_env(self):
        self.state.scan_task = asyncio.create_task(self.object_system.scan_environment())

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

    def backward(self, direction: int):
        self.state.emergency_stop_flag = False
        self.state.movement_task = asyncio.create_task(self.px.backward(direction * self.state.speed))

    # TODO: adjust angle state
    # TODO: turn + forward? or turn left -> -=30, right -> +=30
    def turn(self, angle: int):
        if self.state.emergency_stop_flag:
            print('hazard detected ahead, unable to move')
            return False

        self.object_system.px.set_dir_servo_angle(angle)
        time.sleep(0.1)
        # self.state.movement_task = asyncio.create_task(self.px.forward(self.state.speed))
        return True

    def cancel_movement(self):
        self.object_system.px.stop()
        time.sleep(0.1)
        if self.state.movement_task:
            self.state.movement_task.cancel()
            self.state.movement_task = None

    # def main():
    #     try:
    #         loop = asyncio.get_event_loop()
    #         runner = loop.create_task(avoider.run())
    #         loop.run_until_complete(runner)
    #     except KeyboardInterrupt:
    #         print("\nKeyboard interrupt received")
    #         runner.cancel()
    #         loop.run_until_complete(runner)
    #     finally:
    #         loop.close()
    #
    # if __name__ == "__main__":
    #     main()

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
