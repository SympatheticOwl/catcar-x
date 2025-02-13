import asyncio
from picarx import Picarx
from robot_hat import TTS
import numpy as np
from collections import deque
from datetime import datetime, timedelta
from mapping import MappingSystem, MapBuilder


class IntegratedObstacleAvoidance:
    def __init__(self):
        self.px = Picarx()
        self.tts = TTS()
        self.tts.lang("en-US")
        self.mapping_system = MappingSystem(self.px)

        # configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05

        # turn history tracking
        self.turn_history = deque(maxlen=10)
        self.turn_timestamps = deque(maxlen=10)
        self.stuck_threshold = timedelta(seconds=5)
        self.pattern_threshold = 5

        # state management
        self.current_distance = 100
        self.is_moving = False
        self.current_maneuver = None
        self.emergency_stop_flag = False
        self.is_backing_up = False
        self.is_cliff = False
        self.initial_scan_complete = False
        self.is_scanning = False
        self.stuck = False

    async def initial_environment_scan(self):
        """Perform initial 360-degree scan of environment"""
        print("Performing initial environment scan...")
        self.is_scanning = True

        try:
            await self.mapping_system.scan_surroundings()

            # Check if we're too close to objects
            obstacles = self.mapping_system.get_obstacles_in_range(self.min_distance)
            if np.any(obstacles):
                print("Objects detected too close during initial scan!")
                self.current_maneuver = asyncio.create_task(self.evasive_maneuver())
                await self.current_maneuver
        finally:
            self.is_scanning = False
            self.initial_scan_complete = True

    def find_best_turn_direction(self):
        """Use mapping data to find the best direction to turn"""
        # Get obstacle data for 45 degrees in each direction
        obstacles = self.mapping_system.get_obstacles_in_range(50)  # Check 50cm range

        # Split into left and right regions
        left_region = obstacles[:45]  # -45 to 0 degrees
        right_region = obstacles[45:]  # 0 to 45 degrees

        # Count free space in each region
        left_free = np.sum(left_region == 0)
        right_free = np.sum(right_region == 0)

        # Add some randomness when regions are similar
        if abs(left_free - right_free) < 5:
            return random.choice([-1, 1])

        # Return -1 for left, 1 for right
        return -1 if left_free > right_free else 1

    async def scan_and_plan(self):
        """Perform a new scan and plan best direction"""
        self.is_scanning = True
        try:
            await self.mapping_system.scan_surroundings()
            return self.find_best_turn_direction()
        finally:
            self.is_scanning = False

    async def ultrasonic_monitoring(self):
        while True:
            if not self.is_scanning:  # Don't interfere with mapping scans
                distances = []
                for _ in range(3):
                    distance = self.px.ultrasonic.read()
                    if distance and 0 < distance < 300:
                        distances.append(distance)
                    await asyncio.sleep(0.01)

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

    async def emergency_stop(self):
        self.emergency_stop_flag = True
        if self.current_maneuver and not self.is_backing_up:
            self.current_maneuver.cancel()

        self.is_moving = False
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False

        # Perform new scan before evasive maneuver
        direction = await self.scan_and_plan()
        self.current_maneuver = asyncio.create_task(
            self.evasive_maneuver(forced_direction=direction)
        )

    async def evasive_maneuver(self, forced_direction=None):
        # stuck = self.is_stuck_in_pattern()

        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            print("Backing up...")
            self.is_moving = True
            self.is_backing_up = True
            self.px.backward(self.speed)
            await asyncio.sleep(self.backup_time)
            self.is_backing_up = False

            # Use forced_direction if provided, otherwise use mapping to choose
            direction = forced_direction if forced_direction is not None else await self.scan_and_plan()

            turn_angle = 30 * direction
            print(f"Turning {'left' if direction < 0 else 'right'}...")
            self.turn_history.append(direction)
            self.turn_timestamps.append(datetime.now())
            self.px.set_dir_servo_angle(turn_angle)
            self.px.forward(self.speed)
            await asyncio.sleep(self.turn_time)

            if not self.emergency_stop_flag:
                self.px.set_dir_servo_angle(0)
                self.is_moving = True
                self.px.forward(self.speed)

        except asyncio.CancelledError:
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            self.is_moving = False
            raise

        finally:
            self.current_maneuver = None

    async def forward_movement(self):
        # Wait for initial scan to complete
        while not self.initial_scan_complete:
            await asyncio.sleep(0.1)

        while True:
            if not self.emergency_stop_flag and not self.current_maneuver and not self.is_scanning:
                if self.current_distance >= self.min_distance and not self.is_cliff:
                    if not self.is_moving:
                        print("Moving forward...")
                        self.is_moving = True
                        self.px.forward(self.speed)
                else:
                    if self.is_moving:
                        if self.is_cliff:
                            print("Cliff detected!")
                        else:
                            print(f"Obstacle detected at {self.current_distance:.1f}cm")
                        self.is_moving = False
                        self.px.forward(0)
                        direction = await self.scan_and_plan()
                        self.current_maneuver = asyncio.create_task(
                            self.evasive_maneuver(forced_direction=direction)
                        )

            await asyncio.sleep(0.1)

    async def run(self):
        print("Starting integrated obstacle avoidance system...")
        tasks = []
        try:
            # Perform initial scan before starting other tasks
            await self.initial_environment_scan()

            # Start monitoring and movement tasks
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            # cliff_task = asyncio.create_task(self.cliff_monitoring())
            movement_task = asyncio.create_task(self.forward_movement())

            tasks = [ultrasonic_task, movement_task]
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

            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            print("Shutdown complete")


def main():
    avoider = IntegratedObstacleAvoidance()
    try:
        loop = asyncio.get_event_loop()
        runner = loop.create_task(avoider.run())
        loop.run_until_complete(runner)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
        runner.cancel()
        loop.run_until_complete(runner)
    finally:
        loop.close()


if __name__ == "__main__":
    main()