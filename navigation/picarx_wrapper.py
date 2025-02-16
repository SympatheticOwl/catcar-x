import math
import time
from typing import Tuple
from picarx import Picarx

class PicarXWrapper:
    def __init__(self):
        self.px = Picarx()

        # Position tracking
        self.x = 0.0  # cm
        self.y = 0.0  # cm
        self.heading = 0.0  # degrees, 0 is north/forward, increases clockwise

        # Movement constants
        self.WHEEL_CIRCUMFERENCE = 20.0  # cm (approximate)
        self.WHEELBASE = 15.0  # cm (approximate)
        self.MAX_SPEED = 50  # max speed value

        # Speed to distance conversion (approximate)
        # At speed 30, car moves roughly 15cm/second
        self.SPEED_TO_CM_PER_SEC = 0.5  # cm/speed unit/second

        # Movement state
        self.last_move_time = time.time()
        self.is_moving = False
        self.current_speed = 0
        self.current_angle = 0
        self.target_x = None
        self.target_y = None
        self.movement_task = None

    def _update_position(self, elapsed_time: float) -> None:
        """Update position based on current speed and steering angle"""
        if not self.is_moving or self.current_speed == 0:
            return

        # Calculate distance traveled
        distance = abs(self.current_speed) * self.SPEED_TO_CM_PER_SEC * elapsed_time

        # Convert steering angle to radians
        steering_rad = -math.radians(self.current_angle)
        heading_rad = math.radians(self.heading)

        if abs(self.current_angle) < 1.0:
            # Straight line movement
            self.x += distance * math.sin(heading_rad)
            self.y += distance * math.cos(heading_rad)
        else:
            # Arc movement
            turn_radius = self.WHEELBASE / math.tan(steering_rad)
            angle_change = (distance / turn_radius)
            new_heading = self.heading + math.degrees(angle_change)

            dx = turn_radius * (math.cos(heading_rad) - math.cos(heading_rad + angle_change))
            dy = turn_radius * (math.sin(heading_rad + angle_change) - math.sin(heading_rad))

            self.x += dx
            self.y += dy
            self.heading = new_heading % 360

    def get_position(self) -> Tuple[float, float, float]:
        """Returns current position and heading"""
        elapsed = time.time() - self.last_move_time
        self._update_position(elapsed)
        self.last_move_time = time.time()
        return (self.x, self.y, self.heading)

    async def move_to_target(self, target_x: float, target_y: float, speed: int = 30) -> bool:
        """
        Asynchronously move to target position, can be interrupted
        Returns True if target reached, False if interrupted
        """
        try:
            self.target_x = target_x
            self.target_y = target_y

            while True:
                current_x, current_y, _ = self.get_position()

                # Check if we've reached the target (within 5cm tolerance)
                dx = target_x - current_x
                dy = target_y - current_y
                distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

                if distance_to_target < 5.0:
                    await self.stop()
                    return True

                # Calculate heading to target
                target_heading = math.degrees(math.atan2(dx, dy)) % 360

                # Turn towards target
                await self.turn_to_heading(target_heading)

                # Move forward a small increment (10cm or remaining distance)
                increment = min(10.0, distance_to_target)
                await self.forward_distance(increment, speed)

                # Allow other tasks to run
                await asyncio.sleep(0.1)

        except asyncio.CancelledError:
            await self.stop()
            return False

    async def turn_to_heading(self, target_heading: float, speed: int = 30) -> None:
        """Turn to face a specific heading"""
        _, _, current_heading = self.get_position()

        # Calculate shortest turning direction
        angle_diff = (target_heading - current_heading) % 360
        if angle_diff > 180:
            angle_diff -= 360

        # Set steering angle (max 40 degrees)
        steering_angle = max(-40, min(40, angle_diff))
        self.current_angle = steering_angle
        self.px.set_dir_servo_angle(steering_angle)

        # Calculate turn time
        turn_time = abs(angle_diff) / 45.0  # ~45 degrees per second at speed 30

        # Execute turn
        self.is_moving = True
        self.current_speed = speed if angle_diff > 0 else -speed
        self.px.forward(self.current_speed)

        try:
            await asyncio.sleep(turn_time)
        except asyncio.CancelledError:
            raise
        finally:
            # Stop and straighten wheels
            await self.stop()
            self.current_angle = 0
            self.px.set_dir_servo_angle(0)
            self.heading = target_heading

    async def forward_distance(self, distance: float, speed: int = 30) -> None:
        """Move forward a specific distance in cm"""
        time_needed = abs(distance) / (abs(speed) * self.SPEED_TO_CM_PER_SEC)

        try:
            self.is_moving = True
            self.current_speed = speed if distance > 0 else -speed
            self.px.forward(self.current_speed)
            await asyncio.sleep(time_needed)
        except asyncio.CancelledError:
            raise
        finally:
            await self.stop()

    async def stop(self) -> None:
        """Stop all movement"""
        self.is_moving = False
        self.current_speed = 0
        self.px.forward(0)

        elapsed = time.time() - self.last_move_time
        self._update_position(elapsed)
        self.last_move_time = time.time()

        # Small delay to ensure motors have stopped
        await asyncio.sleep(0.1)

    def reset_position(self) -> None:
        """Reset position tracking to origin"""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.target_x = None
        self.target_y = None