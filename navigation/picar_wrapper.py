import math
import time
from picarx import Picarx

class PicarXMovement:
    def __init__(self):
        self.px = Picarx()
        
        # Physical properties of Picarx
        self.wheel_diameter = 6.5  # cm (diameter of Picarx wheels)
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.wheel_base = 11.5  # cm (distance between wheels)
        
        # Motor properties
        self.max_speed = 50  # Maximum motor speed value
        self.min_speed = 0
        
        # Calibration factors
        self.ticks_per_revolution = 20  # Number of encoder ticks per wheel revolution
        self.cm_per_tick = self.wheel_circumference / self.ticks_per_revolution
        
        # Speed to actual movement calibration
        self.speed_to_cm_per_sec = {
            10: 4.2,   # At speed 10, moves 4.2 cm/sec
            20: 8.5,   # At speed 20, moves 8.5 cm/sec
            30: 12.8,  # At speed 30, moves 12.8 cm/sec
            40: 17.1,  # At speed 40, moves 17.1 cm/sec
            50: 21.4   # At speed 50, moves 21.4 cm/sec
        }
        
        # Initialize movement tracking
        self.reset_movement()

    def reset_movement(self):
        """Reset movement tracking variables"""
        self.current_x = 0
        self.current_y = 0
        self.current_heading = 0  # degrees, 0 is forward, positive is clockwise
        
    def _speed_to_cm_per_sec(self, speed):
        """Convert motor speed value to cm/sec based on calibration data"""
        # Find nearest calibrated speeds
        speeds = sorted(self.speed_to_cm_per_sec.keys())
        
        if speed <= speeds[0]:
            return self.speed_to_cm_per_sec[speeds[0]]
        elif speed >= speeds[-1]:
            return self.speed_to_cm_per_sec[speeds[-1]]
            
        # Linear interpolation between calibrated points
        for i in range(len(speeds)-1):
            if speeds[i] <= speed <= speeds[i+1]:
                speed1, speed2 = speeds[i], speeds[i+1]
                cm1 = self.speed_to_cm_per_sec[speed1]
                cm2 = self.speed_to_cm_per_sec[speed2]
                
                # Linear interpolation
                return cm1 + (cm2 - cm1) * (speed - speed1) / (speed2 - speed1)
    
    async def move_distance(self, distance_cm, speed=30, direction=0):
        """
        Move the Picarx a specific distance in centimeters.
        
        Args:
            distance_cm (float): Distance to move in centimeters (positive for forward, negative for backward)
            speed (int): Motor speed value (0-100)
            direction (float): Direction in degrees (-40 to 40, 0 is straight)
        
        Returns:
            tuple: (success, actual_distance_moved)
        """
        # Validate inputs
        speed = max(self.min_speed, min(abs(speed), self.max_speed))
        direction = max(-40, min(40, direction))
        
        # Calculate movement time based on speed calibration
        cm_per_sec = self._speed_to_cm_per_sec(speed)
        movement_time = abs(distance_cm) / cm_per_sec
        
        # Set direction
        self.px.set_dir_servo_angle(direction)
        time.sleep(0.1)  # Allow servo to settle
        
        # Set movement direction
        if distance_cm >= 0:
            self.px.forward(speed)
        else:
            self.px.backward(speed)
        
        # Wait for movement to complete
        time.sleep(movement_time)
        
        # Stop movement
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        
        # Update position tracking
        angle_rad = math.radians(direction)
        dx = distance_cm * math.cos(angle_rad)
        dy = distance_cm * math.sin(angle_rad)
        
        self.current_x += dx
        self.current_y += dy
        
        return True, distance_cm
    
    async def turn_angle(self, angle_degrees, speed=30):
        """
        Turn the Picarx by a specific angle.
        
        Args:
            angle_degrees (float): Angle to turn in degrees (positive for right, negative for left)
            speed (int): Motor speed value (0-100)
        
        Returns:
            tuple: (success, actual_angle_turned)
        """
        # Validate inputs
        speed = max(self.min_speed, min(abs(speed), self.max_speed))
        
        # Calculate turn radius and circumference
        turn_radius = self.wheel_base / 2  # cm
        turn_circumference = 2 * math.pi * turn_radius
        
        # Calculate distance each wheel needs to travel
        angle_ratio = abs(angle_degrees) / 360
        arc_length = turn_circumference * angle_ratio
        
        # Calculate movement time based on speed calibration
        cm_per_sec = self._speed_to_cm_per_sec(speed)
        movement_time = arc_length / cm_per_sec
        
        # Set wheel speeds for turning
        if angle_degrees > 0:  # Turn right
            self.px.set_motor_speed(1, speed)   # Left wheel forward
            self.px.set_motor_speed(2, -speed)  # Right wheel backward
        else:  # Turn left
            self.px.set_motor_speed(1, -speed)  # Left wheel backward
            self.px.set_motor_speed(2, speed)   # Right wheel forward
        
        # Wait for turn to complete
        time.sleep(movement_time)
        
        # Stop movement
        self.px.forward(0)
        
        # Update heading
        self.current_heading = (self.current_heading + angle_degrees) % 360
        
        return True, angle_degrees
    
    def get_position(self):
        """
        Get the current position and heading of the Picarx.
        
        Returns:
            tuple: (x_cm, y_cm, heading_degrees)
        """
        return (self.current_x, self.current_y, self.current_heading)
    
    async def move_to_position(self, target_x, target_y, speed=30):
        """
        Move the Picarx to a specific (x,y) position.
        
        Args:
            target_x (float): Target X coordinate in centimeters
            target_y (float): Target Y coordinate in centimeters
            speed (int): Motor speed value (0-100)
        
        Returns:
            tuple: (success, distance_moved)
        """
        # Calculate distance and angle to target
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.degrees(math.atan2(dy, dx))
        
        # Calculate required turn
        turn_amount = angle - self.current_heading
        
        # Normalize turn amount to -180 to 180 degrees
        if turn_amount > 180:
            turn_amount -= 360
        elif turn_amount < -180:
            turn_amount += 360
            
        # Execute turn if needed
        if abs(turn_amount) > 2:  # 2 degree threshold
            await self.turn_angle(turn_amount, speed)
            
        # Move to position
        await self.move_distance(distance, speed)
        
        return True, distance