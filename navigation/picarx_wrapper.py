import asyncio
import math
import time
import asyncio
from picarx import Picarx

from navigation.pathfinder import Pathfinder


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
        self.MAX_STEERING_ANGLE = 28  # Maximum steering angle in degrees

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
                    turn_radius = self.WHEELBASE / math.tan(math.radians(abs(self.current_steering_angle)))

                    # Calculate angular velocity (radians per second)
                    angular_velocity = self.current_speed / turn_radius

                    # Calculate angle turned in this time step
                    angle_turned = math.degrees(angular_velocity * dt)
                    if self.current_steering_angle < 0:
                        angle_turned = -angle_turned

                    # Update heading
                    self.heading = (self.heading + angle_turned) % 360
                    heading_rad = math.radians(self.heading - angle_turned / 2)  # Use average heading for arc

                    # Calculate arc movement
                    self.x += distance * math.cos(heading_rad)
                    self.y += distance * math.sin(heading_rad)

                else:  # Straight movement
                    heading_rad = math.radians(self.heading)
                    self.x += distance * math.cos(heading_rad)
                    self.y += distance * math.sin(heading_rad)

            self.last_position_update = current_time
            # await asyncio.sleep(0.05)  # Update at 20Hz
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

    async def navigate_to_point(self, target_x, target_y, speed=30):
        """Navigate to a target point while avoiding obstacles"""
        # Initialize path planner if not already done
        if not hasattr(self, 'path_planner'):
            self.pathfinder = Pathfinder(self.world_map)

        while True:
            # Get current position
            current_pos = self.get_position()
            print(
                f'Current position: x={current_pos["x"]:.1f}, y={current_pos["y"]:.1f}, heading={current_pos["heading"]:.1f}°')

            # Calculate distance to target
            dx = target_x - current_pos['x']
            dy = target_y - current_pos['y']
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

            # If we're close enough to target, stop
            if distance_to_target < 5:  # 5cm threshold
                self.stop()
                return True

            # Check for obstacles and update world map
            if self.vision_enabled:
                objects = self.vision.get_obstacle_info()
                if objects:
                    self._update_vision_detections(objects)

            # Get ultrasonic reading and update map
            distances = await self.scan_avg()
            if distances:
                self.current_distance = sum(distances) / len(distances)
                if self.current_distance < 300:  # Valid reading threshold
                    self._update_ultrasonic_detection(self.current_distance)

            # Find path to target using A*
            path = self.pathfinder.find_path(
                current_pos['x'],
                current_pos['y'],
                target_x,
                target_y
            )

            if not path:
                print("No valid path found! Performing scan...")
                # Perform a full environment scan
                await self.scan_environment()
                # Try path planning again
                path = self.pathfinder.find_path(
                    current_pos['x'],
                    current_pos['y'],
                    target_x,
                    target_y
                )
                if not path:
                    print("Still no valid path found. Stopping.")
                    self.stop()
                    return False

            # Get next waypoint from path
            next_x, next_y = path[1] if len(path) > 1 else path[0]

            # Calculate angle to next waypoint
            dx = next_x - current_pos['x']
            dy = next_y - current_pos['y']
            target_angle = math.degrees(math.atan2(dy, dx))

            # Calculate angle difference
            angle_diff = target_angle - current_pos['heading']
            # Normalize to -180 to 180
            angle_diff = (angle_diff + 180) % 360 - 180

            # If we need to turn more than 45 degrees, stop and turn first
            if abs(angle_diff) > 45:
                self.stop()
                await self.turn_to_heading(target_angle)
                continue

            # Calculate steering angle based on angle difference
            steering_angle = self._calculate_steering_angle(angle_diff)
            self.set_dir_servo_angle(steering_angle)

            # Adjust speed based on turn sharpness and proximity to obstacles
            speed_factor = 1.0
            if self.current_distance < 50:  # Reduce speed when obstacles are near
                speed_factor *= self.current_distance / 50

            turn_factor = 1.0 - abs(steering_angle) / (2 * self.MAX_STEERING_ANGLE)
            adjusted_speed = speed * speed_factor * turn_factor

            # Move forward with adjusted speed
            self.forward(adjusted_speed)

            # Visualize current state periodically
            if time.time() % 5 < 0.1:  # Every 5 seconds
                print("\nCurrent World Map:")
                self.world_map.visualize_map()
                print(f"Next waypoint: ({next_x:.1f}, {next_y:.1f})")
                print(f"Steering angle: {steering_angle:.1f}°")
                print(f"Speed: {adjusted_speed:.1f}")

            await asyncio.sleep(0.1)

    async def turn_to_heading(self, target_heading, speed=30):
        """Turn to a specific heading"""
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

            # Set maximum steering in appropriate direction
            steering_angle = math.copysign(self.MAX_STEERING_ANGLE, angle_diff)
            self.set_dir_servo_angle(steering_angle)

            # Move at turning speed
            if angle_diff > 0:
                self.forward(speed)
            else:
                self.backward(speed)

            await asyncio.sleep(0.1)

    def _calculate_steering_angle(self, angle_diff):
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
