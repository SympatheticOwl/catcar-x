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
        self.is_backing_up = False  # since this is async we don't want to interrupt evasive backup maneuvers while obejcts are still too close
        self.is_cliff = False

    async def ultrasonic_monitoring(self):
        while True:
            distances = []
            for _ in range(3):
                # I don't really understand what the purpose of ultrasonic.read() accepting an optional int value
                # to loop readings but doesn't do any averaging itself unless I'm misunderstanding the code...
                distance = self.px.ultrasonic.read()
                if distance and 0 < distance < 300:
                    distances.append(distance)
                await asyncio.sleep(0.01)

            if distances:
                self.current_distance = sum(distances) / len(distances)

                print(f"Distance: {self.current_distance:.1f} cm")

                # emergency stop if too close during forward movement only
                if (self.current_distance < self.min_distance and
                        self.is_moving and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):  # Don't interrupt backup
                    print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                    await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            self.is_cliff = self.px.get_cliff_status(self.px.get_grayscale_data())

            if (self.is_cliff and
                    self.is_moving and
                    not self.emergency_stop_flag and
                    not self.is_backing_up):
                print(f"Emergency stop, cliff detected!")
                await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def emergency_stop(self):
        self.emergency_stop_flag = True
        # cancel any ongoing maneuver except backup
        if self.current_maneuver:
            self.current_maneuver.cancel()

        self.is_moving = False
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

    def is_stuck_in_pattern(self):
        print(f"Checking stuck pattern...")
        print(f"Stuck pattern history length: {self.turn_history}...")
        print(len(self.turn_history))
        if len(self.turn_history) < self.pattern_threshold:
            return False

        # if we have too many turns in a certain amount of time we'll assume we're stuck
        # going back and forth in somewhere like a corner
        time_window = datetime.now() - self.stuck_threshold
        recent_turns = sum(1 for timestamp in self.turn_timestamps
                           if timestamp > time_window)
        print(f"Recent turns within time window: {recent_turns}...")

        return recent_turns < self.pattern_threshold

    async def spin_turn_180(self):
        print("Executing signature spin turn...")
        self.is_moving = True

        self.px.forward(0)
        await asyncio.sleep(0.2)

        if not self.emergency_stop_flag:
            # idk why this works
            # spins wheels in opposite direction
            # depending on friction of terrain will usually achieve
            # somewhere between 90-180 degress
            spin_direction = self.choose_turn_direction()
            self.px.set_motor_speed(1, spin_direction * self.speed)
            self.px.set_motor_speed(2, spin_direction * self.speed)
            await asyncio.sleep(5)

            self.px.forward(0)
            await asyncio.sleep(0.2)

        self.px.set_dir_servo_angle(0)
        self.is_moving = False

    def choose_turn_direction(self):
        # there was an attempt to detect stuck pattern by checking left/right
        # alternating pattern on top of time
        # despite being more consistent it defeated the purpose
        # of chosing a "random" direction
        # if len(self.turn_history) > 0:
        #     # Avoid repeating the last turn direction
        #     last_turn = self.turn_history[-1]
        #     return -last_turn  # Choose opposite direction
        return random.choice([-1, 1])

    async def evasive_maneuver(self):
        stuck = self.is_stuck_in_pattern()

        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            if stuck:
                print("Stuck pattern detected! Executing escape maneuver...")
                # tts only works half the time and idk why
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
                # normal evasive maneuver
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
        print("Starting obstacle avoidance program...")
        tasks = []
        try:
            # allow ultrasonic and cliff scanning to run concurrently with movement
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            movement_task = asyncio.create_task(self.forward_movement())
            tasks = [ultrasonic_task, cliff_task, movement_task]
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

            # reset position on shutdown
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
