import math
import time
from typing import Final

class State:
    def __init__(self):
        self.current_distance = 300
        self.is_moving = False
        self.is_backing_up = False
        self.is_cliff = False
        self.emergency_stop_flag = False
        self.vision_clear = True
        self.speed = 45

        # Position tracking
        self.x = 0.0  # cm
        self.y = 0.0  # cm
        self.heading = 0.0  # degrees, 0 is forward, positive is clockwise

        # Movement tracking
        self.last_movement_time = time.time()
        self.last_position_update = time.time()
        self.current_speed = 0  # Speed in cm/s
        self.current_steering_angle = 0

        # configuration parameters
        self.min_distance: Final[int] = 25
        self.backup_time: Final[float] = 1.0
        self.long_backup_time: Final[float] = 2.0
        self.turn_time: Final[float] = 1.5
        self.sensor_read_freq: Final[float] = 0.05
        self.scan_range: Final[int, int] = (-60, 60)
        self.scan_step: Final[int] = 5
        self.scan_frequency: Final[float] = 0.05

        # Sensor offsets from center
        self.ULTRASONIC_OFFSET_X: Final[float] = 5.0  # cm forward
        self.ULTRASONIC_OFFSET_Y: Final[float] = 0.0  # cm sideways
        self.CAMERA_OFFSET_X: Final[float] = 5.0  # cm forward
        self.CAMERA_OFFSET_Y: Final[float] = 0.0  # cm sideways

        # Physical constants
        self.WHEEL_DIAMETER: Final[float] = 6.5  # cm
        self.WHEEL_CIRCUMFERENCE: Final[float] = math.pi * self.WHEEL_DIAMETER
        self.WHEELBASE: Final[float] = 9.7  # Distance between wheels in cm
        self.MAX_STEERING_ANGLE: Final[int] = 30  # Maximum steering angle in degrees

        # Movement constants
        self.MAX_MOTOR_SPEED: Final[int] = 100  # Maximum motor speed value
        self.MAX_RPM: Final[int] = 150  # Approximate max RPM at full speed
        self.TURNING_SPEED: Final[int] = 30  # Standard speed for turning
        self.MIN_TURN_RADIUS: Final[float] = self.WHEELBASE / math.tan(math.radians(self.MAX_STEERING_ANGLE))
        self.NINETY_DEG_TURN_TIME: Final[float] = 2.5  # Time to complete a 90-degree turn
        self.TURN_RATE: Final[float] = 90 / self.NINETY_DEG_TURN_TIME  # degrees per second at max steering

        # task states
        self.ultrasonic_task = None
        self.cliff_task = None
        self.pos_track_task = None
        self.vision_task = None
        self.movement_task = None