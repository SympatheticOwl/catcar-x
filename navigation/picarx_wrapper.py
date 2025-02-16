import math
import time
import asyncio
from typing import Tuple
from picarx import Picarx

class PicarXWrapper:
    def __init__(self):
        self.px = Picarx()

        # Position state
        self.x = 0.0  # Position in feet
        self.y = 0.0
        self.heading = 0.0  # Heading in degrees, 0 is North, positive is clockwise

        # Movement calibration constants
        self.SPEED = 30  # Base speed
        self.TURNING_SPEED = 25
        self.FEET_PER_SECOND = 0.5  # Estimated speed in feet/second at base speed
        self.DEGREES_PER_SECOND = 60  # Estimated turning rate in degrees/second
        self.TURN_RADIUS = 1.0  # Estimated turning radius in feet

        # Initialize ultrasonic
        self.min_safe_distance = 15  # cm

    def get_position(self) -> Tuple[float, float, float]:
        """Returns current position (x, y) in feet and heading in degrees"""
        return (self.x, self.y, self.heading)

    def reset_position(self):
        """Reset position tracking to origin"""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

    def _update_position(self, distance: float):
        """Update position based on movement distance and current heading"""
        # Convert heading to radians
        heading_rad = math.radians(self.heading)

        # Update position using trigonometry
        self.x += distance * math.sin(heading_rad)  # East is positive X
        self.y += distance * math.cos(heading_rad)  # North is positive Y

    def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            time.sleep(0.01)
        return distances

    def check_path_clear(self, distance: float) -> bool:
        """Check if path is clear for given distance"""
        distances = self.scan_avg()
        if distances is None:
            return False

        # Convert intended movement distance to cm for comparison
        distance_cm = distance * 30.48  # feet to cm
        return (sum(distances)/len(distances)) > min(distance_cm, self.min_safe_distance)

    async def move_distance(self, distance: float) -> bool:
        """
        Move specified distance in feet along current heading
        Returns True if movement completed, False if blocked
        """
        if not self.check_path_clear(distance):
            return False

        # Calculate movement time based on calibrated speed
        movement_time = abs(distance) / self.FEET_PER_SECOND

        # Set direction based on positive/negative distance
        if distance > 0:
            self.px.forward(self.SPEED)
        else:
            self.px.backward(self.SPEED)

        # Move for calculated duration
        time.sleep(movement_time)
        self.px.forward(0)  # Stop

        # Update position tracking
        self._update_position(distance)
        return True

    def turn_to_heading(self, target_heading: float):
        """Turn to specified absolute heading in degrees"""
        # Normalize target heading to 0-360
        target_heading = target_heading % 360

        # Calculate shortest turning direction
        current_heading = self.heading % 360
        angle_diff = target_heading - current_heading

        # Normalize angle difference to -180 to +180
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # Calculate turn time
        turn_time = abs(angle_diff) / self.DEGREES_PER_SECOND

        # Execute turn
        if angle_diff > 0:
            self.px.set_dir_servo_angle(30)  # Turn right
        else:
            self.px.set_dir_servo_angle(-30)  # Turn left

        self.px.forward(self.TURNING_SPEED)
        time.sleep(turn_time)

        # Stop and straighten wheels
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)

        # Update heading
        self.heading = target_heading

    async def move_to_position(self, target_x: float, target_y: float) -> bool:
        """
        Move to specified position relative to start point
        Returns True if position reached, False if blocked
        """
        # Calculate required heading and distance
        dx = target_x - self.x
        dy = target_y - self.y

        distance = math.sqrt(dx * dx + dy * dy)
        target_heading = math.degrees(math.atan2(dx, dy)) % 360

        # Turn to face target
        self.turn_to_heading(target_heading)

        # Move to target
        return await self.move_distance(distance)

    def scan_surroundings(self) -> list[Tuple[float, float]]:
        """
        Scan surroundings using ultrasonic sensor
        Returns list of (angle, distance) tuples
        """
        scan_data = []

        # Scan from -60 to +60 degrees
        for angle in range(-60, 61, 10):
            self.px.set_cam_pan_angle(angle)
            time.sleep(0.1)  # Let sensor settle

            # Take multiple readings and average
            distances = []
            for _ in range(3):
                dist = self.px.ultrasonic.read()
                if dist and 0 < dist < 300:  # Filter invalid readings
                    distances.append(dist)
                time.sleep(0.01)

            if distances:
                avg_dist = sum(distances) / len(distances)
                scan_data.append((angle, avg_dist))

        # Reset camera position
        self.px.set_cam_pan_angle(0)
        return scan_data
