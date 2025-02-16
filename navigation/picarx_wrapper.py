import asyncio
import math
import time
from picarx import Picarx

class PicarXWrapper:
    def __init__(self):
        self.px = Picarx()

        # PicarX Physical Constants (in cm)
        self.WHEEL_DIAMETER = 6.5  # Diameter of PicarX wheels
        self.WHEEL_BASE = 11.7  # Distance between wheels
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER

        # Position tracking
        self.x = 0.0  # cm
        self.y = 0.0  # cm
        self.heading = 0.0  # radians

        # Movement state
        self.is_moving = False
        self.movement_interrupted = False
        self.current_speed = 0
        self.current_angle = 0

        # Speed conversion (motor speed to cm/s)
        self.MAX_MOTOR_SPEED = 50  # PicarX max motor speed value
        self.MAX_SPEED_CMS = 30.0  # Approximate max speed in cm/s at max motor speed

        # Obstacle tracking
        self.obstacles = []  # List of (x, y) coordinates of detected obstacles
        self.min_obstacle_distance = 15  # cm

        # Navigation parameters
        self.position_tolerance = 5.0  # cm
        self.heading_tolerance = math.radians(5)  # radians

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:
                distances.append(dist)
            else:
                distances.append(300)
            await asyncio.sleep(0.01)
        return distances

    def _motor_speed_to_cms(self, motor_speed):
        """Convert motor speed value to approximate cm/s"""
        return (motor_speed / self.MAX_MOTOR_SPEED) * self.MAX_SPEED_CMS

    def _update_position(self, distance, angle):
        """Update position based on distance traveled and steering angle"""
        # Convert steering angle to radians
        angle_rad = math.radians(angle)

        # Update heading (approximating arc motion)
        if abs(angle) > 1:  # If turning
            # Approximate turning radius
            turning_radius = self.WHEEL_BASE / math.tan(angle_rad)
            # Calculate heading change
            delta_heading = distance / turning_radius
            self.heading += delta_heading

        # Update position
        self.x += distance * math.cos(self.heading)
        self.y += distance * math.sin(self.heading)

        # Normalize heading to [-π, π]
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

    def add_obstacle(self, distance, angle):
        """Add detected obstacle to tracking"""
        # Convert polar coordinates to cartesian relative to car
        dx = distance * math.cos(self.heading + math.radians(angle))
        dy = distance * math.sin(self.heading + math.radians(angle))

        # Convert to global coordinates
        obstacle_x = self.x + dx
        obstacle_y = self.y + dy

        self.obstacles.append((obstacle_x, obstacle_y))

    def clear_obstacles(self):
        """Clear tracked obstacles"""
        self.obstacles = []

    def get_distance_to_goal(self, goal_x, goal_y):
        """Calculate distance to goal coordinates"""
        dx = goal_x - self.x
        dy = goal_y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def get_angle_to_goal(self, goal_x, goal_y):
        """Calculate angle to goal coordinates relative to current heading"""
        dx = goal_x - self.x
        dy = goal_y - self.y
        goal_heading = math.atan2(dy, dx)

        # Calculate smallest angle difference
        angle_diff = goal_heading - self.heading
        return math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    async def move_distance(self, distance, speed=30):
        """Move a specific distance in cm"""
        if distance == 0:
            return

        self.movement_interrupted = False
        self.is_moving = True
        self.current_speed = speed

        # Calculate time needed for movement based on speed
        speed_cms = self._motor_speed_to_cms(speed)
        duration = abs(distance) / speed_cms

        # Set direction based on sign of distance
        actual_speed = speed if distance > 0 else -speed
        self.px.forward(actual_speed)

        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                if self.movement_interrupted:
                    break
                # Update position based on time elapsed and speed
                elapsed = time.time() - start_time
                distance_moved = speed_cms * elapsed
                self._update_position(distance_moved, self.current_angle)
                await asyncio.sleep(0.1)
        finally:
            self.px.forward(0)
            self.is_moving = False
            self.current_speed = 0

    async def turn_to_angle(self, target_angle, speed=30):
        """Turn to a specific angle relative to current heading"""
        angle_diff = target_angle - math.degrees(self.heading)
        # Normalize to [-180, 180]
        angle_diff = ((angle_diff + 180) % 360) - 180

        self.current_angle = angle_diff
        self.px.set_dir_servo_angle(angle_diff)
        await asyncio.sleep(0.5)  # Allow time for servo to move

    async def navigate_to_goal(self, goal_x, goal_y, speed=30):
        """Navigate to goal coordinates with obstacle avoidance"""
        while True:
            curr_pos = self.get_position()
            print(f'Current position: {curr_pos}')
            distance = self.get_distance_to_goal(goal_x, goal_y)
            if distance < self.position_tolerance:
                print(f"Reached goal: ({self.x:.1f}, {self.y:.1f})")
                return True

            # Calculate angle to goal
            angle_to_goal = math.degrees(self.get_angle_to_goal(goal_x, goal_y))

            # Turn towards goal
            await self.turn_to_angle(angle_to_goal)

            # Check for obstacles before moving
            distances = await self.scan_avg()
            if distances and sum(distances) / len(distances) < self.min_obstacle_distance:
                print(f"Obstacle detected at {distances}cm")
                self.add_obstacle(distances, self.current_angle)
                self.movement_interrupted = True
                return False

            # Move towards goal
            movement_distance = min(distance, 20)  # Move in smaller increments
            await self.move_distance(movement_distance, speed)

            if self.movement_interrupted:
                print("Movement interrupted by obstacle")
                return False

            await asyncio.sleep(0.1)

    def get_position(self):
        """Get current position and heading"""
        return {
            'x': self.x,
            'y': self.y,
            'heading': math.degrees(self.heading),
            'obstacles': self.obstacles
        }

    def stop(self):
        """Stop all movement"""
        self.movement_interrupted = True
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        self.is_moving = False
        self.current_speed = 0
        self.current_angle = 0