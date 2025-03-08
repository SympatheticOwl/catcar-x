import asyncio
import math
import time
from picarx import Picarx

from state_handler import State

class PicarXWrapper:
    def __init__(self, state: State):
        self.px = Picarx()
        self.__state = state

    def _speed_to_cm_per_sec(self, speed_value):
        """Convert motor speed value to cm/s"""
        rpm = (abs(speed_value) / self.__state.MAX_MOTOR_SPEED) * self.__state.MAX_RPM
        return (rpm * self.__state.WHEEL_CIRCUMFERENCE) / 60

    async def continuous_position_tracking(self):
        """Continuously update position based on movement"""
        while True:
            current_time = time.time()
            dt = current_time - self.__state.last_position_update

            if self.__state.is_moving:
                # Calculate distance traveled in this time step
                distance = self.__state.current_speed * dt

                if abs(self.__state.current_steering_angle) > 1:  # If turning
                    # Calculate turn radius for current steering angle
                    turn_radius = self.__state.WHEELBASE / math.tan(math.radians(abs(self.__state.current_steering_angle)))

                    # Calculate angular velocity (radians per second)
                    angular_velocity = self.__state.current_speed / turn_radius

                    # Calculate angle turned in this time step
                    angle_turned = math.degrees(angular_velocity * dt)
                    if self.__state.current_steering_angle < 0:
                        angle_turned = -angle_turned

                    # Update heading
                    self.__state.heading = (self.__state.heading + angle_turned) % 360
                    heading_rad = math.radians(self.__state.heading - angle_turned / 2)  # Use average heading for arc

                    # Calculate arc movement
                    self.__state.x += distance * math.cos(heading_rad)
                    self.__state.y += distance * math.sin(heading_rad)

                else:  # Straight movement
                    heading_rad = math.radians(self.__state.heading)
                    self.__state.x += distance * math.cos(heading_rad)
                    self.__state.y += distance * math.sin(heading_rad)

            self.__state.last_position_update = current_time
            await asyncio.sleep(0.05)

    def forward(self, speed = 30):
        """Move forward with speed tracking"""
        # self.px.forward(speed)
        self.px.set_motor_speed(1, speed)
        self.px.set_motor_speed(2, -1 * (speed+10))


        self.__state.current_speed = self._speed_to_cm_per_sec(speed)
        self.__state.is_moving = speed != 0
        self.__state.is_backing_up = False

    def backward(self, speed = 30):
        """Move backward with speed tracking"""
        # self.px.backward(speed)
        # print(f'motor speed 1: {self.px.motor_speed_pins[1]}')
        # print(f'motor speed 2: {self.px.motor_speed_pins[2]}')

        self.px.set_motor_speed(1, -1 * speed)
        self.px.set_motor_speed(2, speed)
        self.__state.current_speed = -self._speed_to_cm_per_sec(speed)
        self.__state.is_moving = speed != 0
        self.__state.is_backing_up = True

    def stop(self):
        """Stop movement"""
        self.px.forward(0)
        self.__state.current_speed = 0
        self.__state.is_moving = False

    def set_dir_servo_angle(self, angle):
        """Set steering angle"""
        # Clamp steering angle
        if 30 >= self.__state.current_steering_angle + angle >= -30:
            self.__state.current_steering_angle += angle
        self.px.set_dir_servo_angle(self.__state.current_steering_angle)

    async def turn_to_heading(self, target_heading, speed=30):
        """Turn to a specific heading"""
        # Normalize target heading to 0-360
        target_heading = target_heading % 360

        while True:
            # Calculate angle difference
            angle_diff = target_heading - self.__state.heading
            # Normalize to -180 to 180
            angle_diff = (angle_diff + 180) % 360 - 180

            # If we're close enough to target heading, stop
            if abs(angle_diff) < 5:  # 5 degree threshold
                self.stop()
                self.set_dir_servo_angle(0)
                return True

            # Set maximum steering in appropriate direction
            steering_angle = math.copysign(self.__state.MAX_STEERING_ANGLE, angle_diff)
            self.set_dir_servo_angle(steering_angle)

            # Move at turning speed
            if angle_diff > 0:
                self.forward(speed)
            else:
                self.backward(speed)

            await asyncio.sleep(0.1)

    def calculate_steering_angle(self, angle_diff):
        """Calculate appropriate steering angle based on angle difference"""
        # Use a proportional control for steering
        steering_angle = (angle_diff / 45.0) * self.__state.MAX_STEERING_ANGLE
        return max(-self.__state.MAX_STEERING_ANGLE,
                   min(self.__state.MAX_STEERING_ANGLE, steering_angle))

    def get_min_turn_radius(self):
        """Get the minimum turning radius at the current speed"""
        return self.__state.MIN_TURN_RADIUS

    def get_target_heading(self, target_x, target_y):
        """Calculate the heading needed to reach the target point"""
        dx = target_x - self.__state.x
        dy = target_y - self.__state.y
        return math.degrees(math.atan2(dy, dx)) % 360

    def get_position(self):
        """Get current position and heading"""
        return {
            'x': round(self.__state.x, 2),
            'y': round(self.__state.y, 2),
            'heading': round(self.__state.heading, 2),
            'speed': round(self.__state.current_speed, 2)
        }

    def reset_position(self):
        """Reset position tracking to origin"""
        self.__state.x = 0.0
        self.__state.y = 0.0
        self.__state.heading = 0.0
        self.__state.current_steering_angle = 0
        self.__state.last_position_update = time.time()

    # Delegate other Picarx methods to the base class
    def __getattr__(self, attr):
        return getattr(self.px, attr)
