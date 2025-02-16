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

        # Motor constants
        self.MAX_MOTOR_SPEED = 100  # Maximum motor speed value
        self.MAX_RPM = 150  # Approximate max RPM at full speed

        # Navigation parameters
        self.POSITION_TOLERANCE = 5.0  # cm
        self.HEADING_TOLERANCE = 2.0  # degrees
        self.DEFAULT_SPEED = 30  # Default movement speed
        self.TURN_SPEED = 20  # Slower speed for turning

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

    def get_position(self):
        """Get current position and heading"""
        self._update_position()
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
        self.last_movement_time = time.time()

        # Delegate other Picarx methods to the base class

    def __getattr__(self, attr):
        """Delegate any other methods to the underlying Picarx instance"""
        return getattr(self.px, attr)
