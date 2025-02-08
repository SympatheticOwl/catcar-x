import asyncio
from picarx import Picarx
from robot_hat import TTS
import random
from collections import deque
from datetime import datetime, timedelta


class AsyncObstacleAvoidance:
    def __init__(self):
        self.px = Picarx()
        self.tts = TTS()
        self.tts.lang("en-US")

        # Configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.2
        self.speed = 30

        # Turn history tracking
        self.turn_history = deque(maxlen=5)
        self.turn_timestamps = deque(maxlen=5)
        self.stuck_threshold = timedelta(seconds=10)
        self.pattern_threshold = 4

        # State management
        self.current_distance = 100
        self.is_moving = False
        self.current_maneuver = None
        self.emergency_stop_flag = False
        self.is_backing_up = False  # Flag to track backup operations
        self.is_cliff = False

    async def ultrasonic_monitoring(self):
        while True:
            distances = []
            for _ in range(3):
                # I don't really understand what the purpose of ultrasonic.read() accepting an optional int value
                # to loop readings but doesn't do any averaging itself...
                distance = self.px.ultrasonic.read()
                if distance and 0 < distance < 300:
                    distances.append(distance)
                await asyncio.sleep(0.01)

            if distances:
                self.current_distance = sum(distances) / len(distances)

                print(f"Distance: {self.current_distance:.1f} cm")

                # Emergency stop if too close during forward movement only
                if (self.current_distance < self.min_distance and
                        self.is_moving and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):  # Don't interrupt backup
                    print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                    self.emergency_stop_flag = True
                    # Cancel any ongoing maneuver except backup
                    if self.current_maneuver:
                        self.current_maneuver.cancel()
                    await self.emergency_stop()

            await asyncio.sleep(0.05)  # Sensor read frequency

    async def cliff_monitoring(self):
        while True:
            self.is_cliff = self.px.get_cliff_status(self.px.get_grayscale_data())

            if self.is_cliff:
                # stop immediately
                self.px.forward(0)

                if (self.is_moving and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):  # Don't interrupt backup
                    print(f"Emergency stop, cliff detected!")
                    self.emergency_stop_flag = True
                    # Cancel any ongoing maneuver except backup
                    if self.current_maneuver:
                        self.current_maneuver.cancel()
                    await self.emergency_stop()

            await asyncio.sleep(0.05)  # Sensor read frequency

    async def emergency_stop(self):
        """Immediately stop all movement"""
        self.is_moving = False
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.5)  # Brief pause after emergency stop
        self.emergency_stop_flag = False
        # Schedule evasive maneuver
        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

    def is_stuck_in_pattern(self):
        print("check stuck...")
        """Check if car is stuck in an alternating turn pattern"""
        print(len(self.turn_history))
        if len(self.turn_history) < self.pattern_threshold:
            return False

        # if we have too many terms in a certain amount of time we'll assume we're stuck
        # going back and forth in somewhere like a corner
        time_window = datetime.now() - self.stuck_threshold
        recent_turns = sum(1 for timestamp in self.turn_timestamps
                           if timestamp > time_window)

        return recent_turns < self.pattern_threshold

    async def spin_turn_180(self):
        """Execute 180-degree spin turn"""
        print("Executing 180-degree spin turn...")
        self.is_moving = True

        # Stop first
        self.px.forward(0)
        await asyncio.sleep(0.2)

        if not self.emergency_stop_flag:
            # idk why this works
            self.px.set_motor_speed(1, self.speed)
            self.px.set_motor_speed(2, self.speed)
            await asyncio.sleep(4.5)

            # Stop spinning
            self.px.forward(0)
            await asyncio.sleep(0.2)

        self.px.set_dir_servo_angle(0)
        self.is_moving = False

    def choose_turn_direction(self):
        """Choose turn direction with basic pattern avoidance"""
        # if len(self.turn_history) > 0:
        #     # Avoid repeating the last turn direction
        #     last_turn = self.turn_history[-1]
        #     return -last_turn  # Choose opposite direction
        return random.choice([-1, 1])

    async def evasive_maneuver(self):
        """Execute smart evasive maneuver"""
        stuck = self.is_stuck_in_pattern()

        try:
            # Stop first
            self.is_moving = False
            self.px.forward(0)
            # self.tts.say("obstacle detected")
            await asyncio.sleep(0.5)

            # await asyncio.create_task(self.tts.say("obstacle detected"))

            if stuck:
                print("Stuck pattern detected! Executing escape maneuver...")
                self.tts.say("am stuck, performing really cool spin move to escape")

                self.is_moving = True
                self.is_backing_up = True
                self.px.backward(self.speed)
                await asyncio.sleep(self.long_backup_time)
                self.is_backing_up = False

                if not self.emergency_stop_flag:
                    await self.spin_turn_180()

                    print("clearing turns...")
                    self.turn_history.clear()
                    self.turn_timestamps.clear()

            else:
                # Normal evasive maneuver
                print("Backing up...")
                self.is_moving = True
                self.is_backing_up = True
                self.px.backward(self.speed)
                await asyncio.sleep(self.backup_time)
                self.is_backing_up = False

                direction = self.choose_turn_direction()
                turn_angle = 30 * direction
                print(f"Turning {'left' if direction < 0 else 'right'}...")
                self.turn_history.append(direction)
                self.turn_timestamps.append(datetime.now())
                self.px.set_dir_servo_angle(turn_angle)
                self.px.forward(self.speed)
                await asyncio.sleep(self.turn_time)

            # Reset to forward movement
            if not self.emergency_stop_flag:
                self.px.set_dir_servo_angle(0)
                self.is_moving = True
                self.px.forward(self.speed)

        except asyncio.CancelledError:
            # Handle cancellation gracefully
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            self.is_moving = False
            raise

        finally:
            self.current_maneuver = None

    async def forward_movement(self):
        """Manage forward movement with obstacle checking"""
        while True:
            if not self.emergency_stop_flag and not self.current_maneuver:
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
                        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

            await asyncio.sleep(0.1)

    async def run(self):
        print("Starting async obstacle avoidance program...")
        tasks = []
        try:
            # Create tasks for concurrent execution
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            movement_task = asyncio.create_task(self.forward_movement())
            tasks = [ultrasonic_task, cliff_task, movement_task]

            # Wait for both tasks (will run indefinitely until interrupted)
            await asyncio.gather(*tasks)

        except asyncio.CancelledError:
            print("\nShutting down gracefully...")
        finally:
            # Cancel all running tasks
            for task in tasks:
                task.cancel()

            # Wait for tasks to complete their cancellation
            try:
                await asyncio.gather(*tasks, return_exceptions=True)
            except asyncio.CancelledError:
                pass

            # Clean shutdown
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            print("Shutdown complete")


def main():
    avoider = AsyncObstacleAvoidance()
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
