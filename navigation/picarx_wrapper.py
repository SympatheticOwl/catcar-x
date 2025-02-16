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

        # Servo Limitations
        self.MAX_STEERING_ANGLE = 30  # Maximum steering angle in degrees
        self.MIN_TURN_RADIUS = self.WHEEL_BASE / math.tan(math.radians(self.MAX_STEERING_ANGLE))

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
        # Convert steering angle to radians (constrained by max steering angle)
        angle = max(min(angle, self.MAX_STEERING_ANGLE), -self.MAX_STEERING_ANGLE)
        angle_rad = math.radians(angle)

        # Update heading (approximating arc motion)
        if abs(angle) > 1:  # If turning
            # Calculate actual turning radius based on steering angle
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

    async def execute_turn(self, angle, speed=30):
        """Execute a turn of the specified angle using a series of small movements"""
        # Constrain angle to maximum steering angle
        turn_angle = max(min(angle, self.MAX_STEERING_ANGLE), -self.MAX_STEERING_ANGLE)

        # Set steering angle
        self.current_angle = turn_angle
        self.px.set_dir_servo_angle(turn_angle)
        await asyncio.sleep(0.3)  # Allow servo to settle

        # Calculate arc length needed for this turn
        # Using approximate turning radius based on steering angle
        turning_radius = self.WHEEL_BASE / math.tan(math.radians(abs(turn_angle)))
        angle_rad = math.radians(abs(angle))
        arc_length = angle_rad * turning_radius

        # Move along the arc
        await self.move_distance(arc_length, speed)

        # Reset steering
        self.px.set_dir_servo_angle(0)
        self.current_angle = 0
        await asyncio.sleep(0.3)

    async def turn_to_heading(self, target_heading, speed=30):
        """Turn to a specific heading using multiple small turns if necessary"""
        while True:
            # Calculate angle difference
            current_diff = target_heading - math.degrees(self.heading)
            # Normalize to [-180, 180]
            current_diff = ((current_diff + 180) % 360) - 180

            # If we're close enough to target heading, we're done
            if abs(current_diff) < 5:  # 5 degree tolerance
                break

            # Determine turn angle for this iteration
            turn_angle = max(min(current_diff, self.MAX_STEERING_ANGLE), -self.MAX_STEERING_ANGLE)

            # Execute the turn
            await self.execute_turn(turn_angle, speed)

            # Break if movement was interrupted
            if self.movement_interrupted:
                break

            await asyncio.sleep(0.1)

    async def navigate_to_goal(self, goal_x, goal_y, speed=30):
        """Navigate to goal coordinates with obstacle avoidance"""
        while True:
            distance = self.get_distance_to_goal(goal_x, goal_y)
            if distance < self.position_tolerance:
                print(f"Reached goal: ({self.x:.1f}, {self.y:.1f})")
                return True

            # Calculate angle to goal
            angle_to_goal = math.degrees(self.get_angle_to_goal(goal_x, goal_y))

            # First, turn towards goal
            await self.turn_to_heading(angle_to_goal, speed)
            if self.movement_interrupted:
                return False

            # Check for obstacles before moving
            distances = await self.scan_avg()
            if distances and sum(distances) / len(distances) < self.min_obstacle_distance:
                print(f"Obstacle detected at {distances}cm")
                self.add_obstacle(distances, self.current_angle)
                self.movement_interrupted = True
                return False

            # Move towards goal in smaller increments
            # Calculate movement distance based on turn radius constraints
            max_movement = min(distance, 20)  # Don't move more than 20cm at a time
            await self.move_distance(max_movement, speed)

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