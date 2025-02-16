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
        self.current_speed = 0  # Speed in cm/s
        self.is_moving = False

        # Physical constants for movement calculations
        self.WHEEL_DIAMETER = 6.5  # cm
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.WHEELBASE = 9.7  # Distance between wheels in cm
        self.MAX_STEERING_ANGLE = 28  # Maximum steering angle in degrees

        # Motor and turning constants
        self.MAX_MOTOR_SPEED = 100  # Maximum motor speed value
        self.MAX_RPM = 150  # Approximate max RPM at full speed
        self.TURNING_SPEED = 30  # Standard speed for turning
        self.MIN_TURN_RADIUS = self.WHEELBASE / math.tan(math.radians(self.MAX_STEERING_ANGLE))
        self.NINETY_DEG_TURN_TIME = 2.0  # Time to complete a 90-degree turn
        self.TURN_RATE = 90 / self.NINETY_DEG_TURN_TIME  # degrees per second at max steering

    def _speed_to_cm_per_sec(self, speed_value):
        """Convert motor speed value to cm/s"""
        # Convert speed value (0-100) to RPM
        rpm = (abs(speed_value) / self.MAX_MOTOR_SPEED) * self.MAX_RPM

        # Convert RPM to cm/s
        # RPM * circumference = distance per minute
        # Divide by 60 to get distance per second
        return (rpm * self.WHEEL_CIRCUMFERENCE) / 60

    def _update_position(self):
        """Update position based on time elapsed and current movement"""
        current_time = time.time()
        elapsed_time = current_time - self.last_movement_time

        if self.is_moving:
            # Calculate distance traveled
            distance = self.current_speed * elapsed_time

            # Convert heading to radians for calculations
            heading_rad = math.radians(self.heading)

            # Update position based on heading
            self.x += distance * math.cos(heading_rad)
            self.y += distance * math.sin(heading_rad)

        self.last_movement_time = current_time

    def forward(self, speed):
        """Move forward with speed tracking"""
        self._update_position()
        self.px.forward(speed)

        # Update movement tracking using wheel diameter-based calculation
        self.current_speed = self._speed_to_cm_per_sec(speed)
        self.is_moving = speed != 0

    def backward(self, speed):
        """Move backward with speed tracking"""
        self._update_position()
        self.px.backward(speed)

        # Update movement tracking using wheel diameter-based calculation
        self.current_speed = -self._speed_to_cm_per_sec(speed)
        self.is_moving = speed != 0

    def stop(self):
        """Stop movement"""
        self._update_position()
        self.px.forward(0)

        # Update movement tracking
        self.current_speed = 0
        self.is_moving = False

    def set_dir_servo_angle(self, angle):
        """Set steering angle and update heading calculations"""
        self._update_position()

        # Clamp steering angle
        clamped_angle = max(-self.MAX_STEERING_ANGLE,
                            min(self.MAX_STEERING_ANGLE, angle))

        self.px.set_dir_servo_angle(clamped_angle)

        if self.is_moving:
            # Calculate heading change based on steering geometry
            # Using Ackermann steering geometry for better accuracy
            # turn_radius = wheelbase / tan(steering_angle)
            if clamped_angle != 0:
                turn_radius = self.WHEELBASE / math.tan(math.radians(abs(clamped_angle)))
                # Calculate angular velocity based on current speed and turn radius
                angular_velocity = (self.current_speed / turn_radius)  # radians per second

                # Update heading (convert to degrees)
                heading_change = math.degrees(angular_velocity * 0.1)  # Small time step
                self.heading += math.copysign(heading_change, clamped_angle)

                # Normalize heading to 0-360 degrees
                self.heading = self.heading % 360

    async def navigate_to_point(self, target_x, target_y, speed=30):
        """Navigate to a target point while accounting for turning radius"""
        while True:
            # Update current position
            self._update_position()

            # Calculate distance and angle to target
            dx = target_x - self.x
            dy = target_y - self.y
            print(f"Final position: x={self.x}, y={self.y}, heading={self.heading}")
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

            # If we're close enough to target, stop
            if distance_to_target < 5:  # 5cm threshold
                self.stop()
                return True

            # Calculate target angle in degrees
            target_angle = math.degrees(math.atan2(dy, dx))
            # Normalize target angle to 0-360
            target_angle = target_angle % 360

            # Calculate angle difference
            angle_diff = target_angle - self.heading
            # Normalize to -180 to 180
            angle_diff = (angle_diff + 180) % 360 - 180

            # Calculate minimum turning radius at current speed
            current_min_radius = self.MIN_TURN_RADIUS

            # If we need to turn more than our minimum turning radius allows
            if abs(angle_diff) > 10:  # 10 degree threshold
                await self._execute_turn(angle_diff)
                continue

            # Check if we can reach the target with our turning radius
            if distance_to_target < current_min_radius:
                # Need to approach from a different angle
                await self._adjust_approach(target_x, target_y)
                continue

            # Move forward while continuously adjusting heading
            self.forward(speed)
            steering_angle = self._calculate_steering_angle(angle_diff)
            self.set_dir_servo_angle(steering_angle)

            await asyncio.sleep(0.1)  # Small delay for control loop

    async def _execute_turn(self, angle_diff):
        """Execute a turn of the specified angle"""
        # Calculate turn duration based on angle
        turn_duration = abs(angle_diff) / self.TURN_RATE

        # Set maximum steering in appropriate direction
        steering_angle = math.copysign(self.MAX_STEERING_ANGLE, angle_diff)
        self.set_dir_servo_angle(steering_angle)

        # Move at turning speed
        if angle_diff > 0:
            self.forward(self.TURNING_SPEED)
        else:
            self.backward(self.TURNING_SPEED)

        # Wait for turn to complete
        await asyncio.sleep(turn_duration)

        # Straighten wheels and stop
        self.set_dir_servo_angle(0)
        self.stop()

    async def _adjust_approach(self, target_x, target_y):
        """Adjust approach when target is within minimum turning radius"""
        # Calculate an intermediate point that's further away
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Move back to get more room
        angle_to_target = math.atan2(dy, dx)
        backup_x = self.x - (self.MIN_TURN_RADIUS * math.cos(angle_to_target))
        backup_y = self.y - (self.MIN_TURN_RADIUS * math.sin(angle_to_target))

        # Back up to the intermediate point
        self.backward(self.TURNING_SPEED)
        await asyncio.sleep(1.0)  # Back up for 1 second
        self.stop()

    def _calculate_steering_angle(self, angle_diff):
        """Calculate appropriate steering angle based on angle difference"""
        # Use a proportional control for steering
        # Scale the angle difference to our maximum steering angle
        steering_angle = (angle_diff / 45.0) * self.MAX_STEERING_ANGLE
        # Clamp to our maximum steering angle
        return max(-self.MAX_STEERING_ANGLE,
                   min(self.MAX_STEERING_ANGLE, steering_angle))

    def get_position(self):
        """Get current position and heading"""
        self._update_position()
        return {
            'x': round(self.x, 2),
            'y': round(self.y, 2),
            'heading': round(self.heading, 2),
            'speed': round(self.current_speed, 2)
        }

    def get_min_turn_radius(self):
        """Get the minimum turning radius at the current speed"""
        return self.MIN_TURN_RADIUS

    def get_target_heading(self, target_x, target_y):
        """Calculate the heading needed to reach the target point"""
        dx = target_x - self.x
        dy = target_y - self.y
        return math.degrees(math.atan2(dy, dx)) % 360

    def reset_position(self):
        """Reset position tracking to origin"""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.last_movement_time = time.time()

        # Delegate other Picarx methods to the base class

    def __getattr__(self, attr):
        """Delegate any other methods to the underlying Picarx instance"""
        return getattr(self.px, attr)
