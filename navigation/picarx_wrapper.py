import asyncio
import math
import time
from picarx import Picarx

class PicarXWrapper:
    def __init__(self):
        self.px = Picarx()

        # Position tracking
        self.x = 0.0  # cm
        self.y = 0.0  # cm
        self.heading = 0.0  # degrees, 0 is forward, positive is clockwise

        # Movement tracking
        self.last_movement_time = time.time()
        self.last_position_update = time.time()
        self.current_speed = 0  # Speed in cm/s
        self.is_moving = False
        self.current_steering_angle = 0

        # Physical constants
        self.WHEEL_DIAMETER = 6.5  # cm
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.WHEELBASE = 9.7  # Distance between wheels in cm
        self.MAX_STEERING_ANGLE = 30  # Maximum steering angle in degrees

        # Movement constants
        self.MAX_MOTOR_SPEED = 100  # Maximum motor speed value
        self.MAX_RPM = 150  # Approximate max RPM at full speed
        self.TURNING_SPEED = 30  # Standard speed for turning
        self.MIN_TURN_RADIUS = self.WHEELBASE / math.tan(math.radians(self.MAX_STEERING_ANGLE))
        self.NINETY_DEG_TURN_TIME = 2.5  # Time to complete a 90-degree turn
        self.TURN_RATE = 90 / self.NINETY_DEG_TURN_TIME  # degrees per second at max steering

    def _speed_to_cm_per_sec(self, speed_value):
        """Convert motor speed value to cm/s"""
        rpm = (abs(speed_value) / self.MAX_MOTOR_SPEED) * self.MAX_RPM
        return (rpm * self.WHEEL_CIRCUMFERENCE) / 60

    async def continuous_position_tracking(self):
        """Continuously update position based on movement"""
        while True:
            current_time = time.time()
            dt = current_time - self.last_position_update

            if self.is_moving:
                # Calculate distance traveled in this time step
                distance = self.current_speed * dt

                if abs(self.current_steering_angle) > 1:  # If turning
                    # Calculate turn radius for current steering angle
                    # Use a minimum turn radius to prevent unrealistic sharp turns
                    turn_angle_rad = math.radians(abs(self.current_steering_angle))
                    # Limit minimum turn radius to wheelbase
                    turn_radius = max(
                        self.WHEELBASE,
                        self.WHEELBASE / math.tan(turn_angle_rad)
                    )

                    # Calculate angular velocity (radians per second)
                    angular_velocity = (self.current_speed / turn_radius)

                    # Calculate angle turned in this time step
                    angle_turned = math.degrees(angular_velocity * dt)
                    if self.current_steering_angle < 0:
                        angle_turned = -angle_turned

                    # Update heading (limit rate of turn)
                    max_turn_rate = 90 * dt  # max 90 degrees per second
                    angle_turned = max(min(angle_turned, max_turn_rate), -max_turn_rate)
                    self.heading = (self.heading + angle_turned) % 360

                    # Use average heading for arc movement
                    heading_rad = math.radians(self.heading - angle_turned / 2)

                    # Calculate arc movement
                    self.x += distance * math.cos(heading_rad)
                    self.y += distance * math.sin(heading_rad)

                else:  # Straight movement
                    heading_rad = math.radians(self.heading)
                    self.x += distance * math.cos(heading_rad)
                    self.y += distance * math.sin(heading_rad)

            self.last_position_update = current_time
            await asyncio.sleep(0.05)  # Update at 20Hz

    def forward(self, speed):
        """Move forward with speed tracking"""
        self.px.forward(speed)
        self.current_speed = self._speed_to_cm_per_sec(speed)
        self.is_moving = speed != 0

    def backward(self, speed):
        """Move backward with speed tracking"""
        self.px.backward(speed)
        self.current_speed = -self._speed_to_cm_per_sec(speed)
        self.is_moving = speed != 0

    def stop(self):
        """Stop movement"""
        self.px.forward(0)
        self.current_speed = 0
        self.is_moving = False

    def set_dir_servo_angle(self, angle):
        """Set steering angle"""
        # Clamp steering angle
        self.current_steering_angle = max(-self.MAX_STEERING_ANGLE,
                                          min(self.MAX_STEERING_ANGLE, angle))
        self.px.set_dir_servo_angle(self.current_steering_angle)

    async def turn_to_heading(self, target_heading, speed=30):
        """Turn to a specific heading more smoothly"""
        # Normalize target heading to 0-360
        target_heading = target_heading % 360

        while True:
            # Calculate angle difference
            angle_diff = target_heading - self.heading
            # Normalize to -180 to 180
            angle_diff = (angle_diff + 180) % 360 - 180

            # If we're close enough to target heading, stop
            if abs(angle_diff) < 5:  # 5 degree threshold
                self.stop()
                self.set_dir_servo_angle(0)
                return True

            # Calculate proportional steering angle
            # Use smaller turns for smaller angle differences
            turn_ratio = min(abs(angle_diff) / 90.0, 1.0)  # Scale based on how far we need to turn
            max_turn = self.MAX_STEERING_ANGLE * turn_ratio
            steering_angle = math.copysign(max_turn, angle_diff)

            # Set steering and move forward at reduced speed during turn
            self.set_dir_servo_angle(steering_angle)
            turn_speed = speed * (1 - abs(steering_angle) / self.MAX_STEERING_ANGLE)
            self.forward(turn_speed)

            await asyncio.sleep(0.1)

    def calculate_steering_angle(self, angle_diff):
        """Calculate appropriate steering angle based on angle difference"""
        # Use a proportional control for steering
        steering_angle = (angle_diff / 45.0) * self.MAX_STEERING_ANGLE
        return max(-self.MAX_STEERING_ANGLE,
                   min(self.MAX_STEERING_ANGLE, steering_angle))

    def get_min_turn_radius(self):
        """Get the minimum turning radius at the current speed"""
        return self.MIN_TURN_RADIUS

    def get_target_heading(self, target_x, target_y):
        """Calculate the heading needed to reach the target point"""
        dx = target_x - self.x
        dy = target_y - self.y
        return math.degrees(math.atan2(dy, dx)) % 360

    def get_position(self):
        """Get current position and heading"""
        return {
            'x': round(self.x, 2),
            'y': round(self.y, 2),
            'heading': round(self.heading, 2),
            'speed': round(self.current_speed, 2)
        }

    def reset_position(self):
        """Reset position tracking to origin"""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.current_steering_angle = 0
        self.last_position_update = time.time()

    # Delegate other Picarx methods to the base class
    def __getattr__(self, attr):
        """Delegate any other methods to the underlying Picarx instance"""
        return getattr(self.px, attr)
