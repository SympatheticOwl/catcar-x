import numpy as np
from scipy.interpolate import interp1d
import math
import asyncio
from picarx import Picarx
from robot_hat import TTS
import random
from collections import deque
from datetime import datetime, timedelta


class PicarXMapper:
    def __init__(self, grid_size=100, cm_per_cell=1):
        """
        Initialize the mapping system

        Args:
            grid_size: Size of the square grid (grid_size x grid_size)
            cm_per_cell: How many centimeters each cell represents
        """
        self.grid_size = grid_size
        self.cm_per_cell = cm_per_cell

        # Create occupancy grid with car at bottom center
        self.map = np.zeros((grid_size, grid_size))
        self.car_pos = (grid_size - 1, grid_size // 2)  # (y, x)

        # Store raw sensor readings for interpolation
        self.angle_readings = {}  # {angle: distance}

    def polar_to_cartesian(self, distance, angle_deg):
        """Convert polar coordinates to cartesian grid coordinates"""
        # Convert angle to radians
        angle_rad = math.radians(angle_deg)

        # Calculate x and y offsets from car position
        x = distance * math.sin(angle_rad)  # sin for x because 0° is forward
        y = distance * math.cos(angle_rad)  # cos for y because 0° is forward

        # Scale to grid cells and add car position offset
        grid_x = int(self.car_pos[1] + (x / self.cm_per_cell))
        grid_y = int(self.car_pos[0] - (y / self.cm_per_cell))  # Subtract because y increases downward

        return grid_y, grid_x

    def add_sensor_reading(self, angle_deg, distance_cm):
        """
        Add a new sensor reading and update the map

        Args:
            angle_deg: Angle of the sensor reading in degrees
            distance_cm: Distance to obstacle in centimeters
        """
        # Store the reading
        self.angle_readings[angle_deg] = distance_cm

        # Only proceed if we have at least two readings
        if len(self.angle_readings) < 2:
            return

        # Sort angles for interpolation
        angles = sorted(self.angle_readings.keys())
        distances = [self.angle_readings[angle] for angle in angles]

        # Create interpolation function
        interp_func = interp1d(angles, distances, kind='linear')

        # Interpolate between min and max angles at 1 degree intervals
        interp_angles = np.arange(min(angles), max(angles) + 1, 1)
        interp_distances = interp_func(interp_angles)

        # Reset the map (keep only the latest scan)
        self.map = np.zeros((self.grid_size, self.grid_size))

        # Add interpolated points to map
        for angle, distance in zip(interp_angles, interp_distances):
            if 0 < distance < self.grid_size * self.cm_per_cell:
                grid_y, grid_x = self.polar_to_cartesian(distance, angle)

                # Check if coordinates are within grid bounds
                if (0 <= grid_y < self.grid_size and
                        0 <= grid_x < self.grid_size):
                    self.map[grid_y, grid_x] = 1

                    # Add some thickness to make obstacles more visible
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            ny, nx = grid_y + dy, grid_x + dx
                            if (0 <= ny < self.grid_size and
                                    0 <= nx < self.grid_size):
                                self.map[ny, nx] = 1

    def clear_map(self):
        """Reset the map and stored readings"""
        self.map = np.zeros((self.grid_size, self.grid_size))
        self.angle_readings.clear()

    def get_obstacle_directions(self):
        """
        Returns a list of angles (in degrees) where obstacles are detected,
        relative to the car's forward direction
        """
        obstacle_angles = []

        # Check each angle where we have readings
        for angle in sorted(self.angle_readings.keys()):
            distance = self.angle_readings[angle]
            if distance < 30:  # Consider obstacles within 30cm
                obstacle_angles.append(angle)

        return obstacle_angles


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

        # slam
        self.mapper = PicarXMapper()
        self.scan_range = (-60, 60)  # Scan from -60° to +60°
        self.scan_step = 5  # Degrees between readings

    async def scan_surroundings(self):
        """Perform a full scan of surroundings"""
        self.mapper.clear_map()

        original_angle = self.px.dir_servo_angle

        for angle in range(self.scan_range[0], self.scan_range[1] + 1, self.scan_step):
            # Move servo to angle
            self.px.set_dir_servo_angle(angle)
            await asyncio.sleep(0.1)  # Allow servo to move

            # Take multiple readings and average them
            distances = []
            for _ in range(3):
                distance = self.px.ultrasonic.read()
                if distance and 0 < distance < 300:
                    distances.append(distance)
                await asyncio.sleep(0.01)

            if distances:
                avg_distance = sum(distances) / len(distances)
                self.mapper.add_sensor_reading(angle, avg_distance)

        # Return servo to original position
        self.px.set_dir_servo_angle(original_angle)

        # Get list of directions with obstacles
        return self.mapper.get_obstacle_directions()

    async def enhanced_evasive_maneuver(self):
        """Enhanced evasive maneuver using mapping data"""
        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            # Perform a full scan
            obstacle_directions = await self.scan_surroundings()

            if not obstacle_directions:
                # No obstacles detected, proceed normally
                return await super().evasive_maneuver()

            # Find the largest gap between obstacles
            obstacle_directions.sort()
            gaps = []
            for i in range(len(obstacle_directions)):
                next_i = (i + 1) % len(obstacle_directions)
                gap = obstacle_directions[next_i] - obstacle_directions[i]
                if gap < 0:  # Handle wraparound
                    gap += 360
                gaps.append((gap, i))

            # Choose the middle of the largest gap as escape direction
            largest_gap = max(gaps, key=lambda x: x[0])
            gap_start = obstacle_directions[largest_gap[1]]
            escape_angle = gap_start + (largest_gap[0] / 2)

            # Normalize angle to (-180, 180)
            if escape_angle > 180:
                escape_angle -= 360

            # Execute the turn
            self.px.set_dir_servo_angle(int(escape_angle))
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
        self.current_maneuver = asyncio.create_task(self.enhanced_evasive_maneuver())

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
            # spins wheels in opposite direction
            # depending on friction of terrain will usually achieve
            # somewhere between 90-180 degrees
            spin_direction = self.choose_turn_direction()
            self.px.set_motor_speed(1, spin_direction * self.speed)
            self.px.set_motor_speed(2, spin_direction * self.speed)
            await asyncio.sleep(5)

            self.px.forward(0)
            await asyncio.sleep(0.2)

        self.px.set_dir_servo_angle(0)
        self.is_moving = False

    def choose_turn_direction(self):
        return random.choice([-1, 1])

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
                        self.current_maneuver = asyncio.create_task(self.enhanced_evasive_maneuver())

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