import asyncio
import math
import numpy as np
from typing import Tuple, List, Optional, Dict
import time
from picarx import Picarx
from scipy import ndimage
import cv2
import tflite_runtime.interpreter as tflite
import libcamera
from picamera2 import Picamera2


class WorldMap:
    def __init__(self, map_size: int = 400, resolution: float = 1.0):
        """
        Initialize world map for obstacle tracking

        Args:
            map_size: Size of the map in cm
            resolution: cm per grid cell (1.0 = 1cm per cell)
        """
        self.resolution = resolution
        self.grid_size = int(map_size / resolution)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.origin = np.array([self.grid_size // 2, self.grid_size // 2])

        # Obstacle tracking
        self.obstacles: Dict[str, Dict] = {}  # Track individual obstacles
        self.obstacle_id_counter = 0

        self.padding_size = 2  # Number of cells to pad around obstacles
        self.padding_structure = np.ones((2, 2))  # 3x3 structuring element for dilation

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (cm) to grid coordinates"""
        grid_x = int(x / self.resolution) + self.origin[0]
        grid_y = int(y / self.resolution) + self.origin[1]
        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates (cm)"""
        x = (grid_x - self.origin[0]) * self.resolution
        y = (grid_y - self.origin[1]) * self.resolution
        return x, y

    def add_obstacle(self, x: float, y: float, radius: float = 10.0, confidence: float = 1.0,
                     label: str = "unknown") -> str:
        """
        Add an obstacle to the map

        Args:
            x, y: World coordinates of obstacle center (cm)
            radius: Radius of obstacle (cm)
            confidence: Detection confidence (0-1)
            label: Object label from vision system

        Returns:
            obstacle_id: Unique ID for tracking this obstacle
        """
        obstacle_id = f"obs_{self.obstacle_id_counter}"
        self.obstacle_id_counter += 1

        # Store obstacle metadata
        self.obstacles[obstacle_id] = {
            'x': x,
            'y': y,
            'radius': radius,
            'confidence': confidence,
            'label': label,
            'last_seen': time.time()
        }

        # Update grid
        self._update_grid_for_obstacle(x, y, radius)

        return obstacle_id

    def _update_grid_for_obstacle(self, x: float, y: float, radius: float):
        """Update grid cells for an obstacle"""
        grid_x, grid_y = self.world_to_grid(x, y)
        grid_radius = int(radius / self.resolution)

        # Create a circle mask
        y_indices, x_indices = np.ogrid[-grid_radius:grid_radius + 1, -grid_radius:grid_radius + 1]
        mask = x_indices ** 2 + y_indices ** 2 <= grid_radius ** 2

        # Calculate grid boundaries
        x_min = max(0, grid_x - grid_radius)
        x_max = min(self.grid_size, grid_x + grid_radius + 1)
        y_min = max(0, grid_y - grid_radius)
        y_max = min(self.grid_size, grid_y + grid_radius + 1)

        # Apply mask to grid
        mask_height, mask_width = mask.shape
        grid_height = y_max - y_min
        grid_width = x_max - x_min

        # Trim mask if needed
        mask = mask[:grid_height, :grid_width]

        # Update grid
        self.grid[y_min:y_max, x_min:x_max][mask] = 1

    def update_obstacle_position(self, obstacle_id: str, new_x: float, new_y: float):
        """Update position of a tracked obstacle"""
        if obstacle_id in self.obstacles:
            # Clear old position
            old_x = self.obstacles[obstacle_id]['x']
            old_y = self.obstacles[obstacle_id]['y']
            radius = self.obstacles[obstacle_id]['radius']
            self._clear_grid_area(old_x, old_y, radius)

            # Update position
            self.obstacles[obstacle_id]['x'] = new_x
            self.obstacles[obstacle_id]['y'] = new_y
            self.obstacles[obstacle_id]['last_seen'] = time.time()

            # Add to new position
            self._update_grid_for_obstacle(new_x, new_y, radius)

    def _clear_grid_area(self, x: float, y: float, radius: float):
        """Clear grid cells in an area"""
        grid_x, grid_y = self.world_to_grid(x, y)
        grid_radius = int(radius / self.resolution)

        x_min = max(0, grid_x - grid_radius)
        x_max = min(self.grid_size, grid_x + grid_radius + 1)
        y_min = max(0, grid_y - grid_radius)
        y_max = min(self.grid_size, grid_y + grid_radius + 1)

        self.grid[y_min:y_max, x_min:x_max] = 0

    def add_padding(self):
        # Apply padding
        self.grid = ndimage.binary_dilation(
            self.grid,
            structure=self.padding_structure,
            iterations=self.padding_size
        )

        print("\nCurrent Map (with padding):")
        self.visualize_map()

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.grid:
            print(''.join(['1' if cell else '0' for cell in row]))

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
        """Navigate to a target point while accounting for turning radius"""
        while True:
            # Calculate distance and angle to target
            dx = target_x - self.x
            dy = target_y - self.y
            print(f'self.x: {self.x}, self.y: {self.y}')
            print(f'target.x: {self.x}, target.y: {self.y}')
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

            # If we're close enough to target, stop
            if distance_to_target < 5:  # 5cm threshold
                self.stop()
                return True

            # Calculate target angle in degrees
            target_angle = math.degrees(math.atan2(dy, dx))

            # Calculate angle difference
            angle_diff = target_angle - self.heading
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

            # Adjust speed based on turn sharpness
            adjusted_speed = speed * (1 - abs(steering_angle) / (2 * self.MAX_STEERING_ANGLE))
            self.forward(adjusted_speed)

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

class VisionSystem:
    def __init__(self, model_path='efficientdet_lite0.tflite', labels_path='coco_labels.txt'):
        # Initialize camera
        self.camera = Picamera2()
        # Configure camera for video mode with higher framerate
        config = self.camera.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"FrameDurationLimits": (33333, 33333)}  # ~30fps (1/30 sec = 33333 Î¼s)
        )
        self.camera.set_controls({
            "AwbEnable": True,
            "AwbMode": libcamera.controls.AwbModeEnum.Tungsten,  # Try different modes
            "ExposureTime": 20000,  # Adjust as needed
            "ColourGains": (1.5, 1.5)  # (red, blue) gains - adjust these values
        })
        self.camera.configure(config)
        self.camera.start(show_preview=False)

        # Add buffer for frame timing
        self.last_process_time = time.time()
        self.process_interval = 1.0  # Process every 1 second, but capture at full speed

        # Load TFLite model
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        # Load labels
        with open(labels_path, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        # Initialize detection results
        self.detected_objects = []
        self.frame = None

    async def capture_and_detect(self):
        """Continuously capture frames and perform object detection"""
        while True:
            current_time = time.time()

            # Always capture the latest frame at full camera speed
            self.frame = self.camera.capture_array()

            # Only process frame if enough time has elapsed
            if current_time - self.last_process_time >= self.process_interval:
                # Preprocess the image
                input_data = self._preprocess_image(self.frame)

                # Run inference
                self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
                self.interpreter.invoke()

                # Get detection results
                boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
                classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
                scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]

                # Process results
                self.detected_objects = []
                for i in range(len(scores)):
                    if scores[i] > 0.5:  # Confidence threshold
                        class_id = int(classes[i])
                        label = self.labels[class_id]
                        box = boxes[i]

                        # Convert box coordinates to pixels
                        ymin, xmin, ymax, xmax = box
                        xmin = int(xmin * self.frame.shape[1])
                        xmax = int(xmax * self.frame.shape[1])
                        ymin = int(ymin * self.frame.shape[0])
                        ymax = int(ymax * self.frame.shape[0])

                        self.detected_objects.append({
                            'label': label,
                            'confidence': scores[i],
                            'box': (xmin, ymin, xmax, ymax)
                        })

                # Optional: Draw bounding boxes for debugging
                self._draw_boxes()
                self.last_process_time = current_time

            # Small sleep to prevent CPU overload while maintaining high capture rate
            await asyncio.sleep(0.01)  # 10ms sleep

    def _draw_boxes(self):
        """Draw bounding boxes on the frame for visualization"""
        if self.frame is None:
            return

        debug_frame = self.frame.copy()
        for obj in self.detected_objects:
            xmin, ymin, xmax, ymax = obj['box']
            label = f"{obj['label']} {obj['confidence']:.2f}"

            cv2.rectangle(debug_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(debug_frame, label, (xmin, ymin - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the frame (optional, for debugging)
        cv2.imshow('Object Detection', debug_frame)
        cv2.waitKey(1)

    def _preprocess_image(self, image):
        """Preprocess image for EfficientDet model which expects uint8 input"""
        image = cv2.resize(image, (self.width, self.height))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        image = np.expand_dims(image, axis=0)
        return image.astype(np.uint8)

    def get_obstacle_info(self):
        """Return information about detected obstacles"""
        # Focus on objects in the center third of the frame
        if self.frame is None or not self.detected_objects:
            return None

        frame_width = self.frame.shape[1]
        center_third_left = frame_width // 3
        center_third_right = (frame_width * 2) // 3

        center_objects = []
        for obj in self.detected_objects:
            xmin, _, xmax, _ = obj['box']
            center = (xmin + xmax) // 2
            if center_third_left <= center <= center_third_right:
                center_objects.append(obj)

        return center_objects

    def cleanup(self):
        """Cleanup camera resources"""
        self.camera.stop()
        cv2.destroyAllWindows()

class AsyncObstacleAvoidance:
    def __init__(self):
        self.px = PicarXWrapper()

        # World mapping
        self.world_map = WorldMap(map_size=400, resolution=1.0)  # 4m x 4m map, 1cm resolution

        # Sensor offsets from center
        self.ULTRASONIC_OFFSET_X = 5.0  # cm forward
        self.ULTRASONIC_OFFSET_Y = 0.0  # cm sideways
        self.CAMERA_OFFSET_X = 5.0  # cm forward
        self.CAMERA_OFFSET_Y = 0.0  # cm sideways

        # configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05

        # state management
        self.current_distance = 100
        self.is_moving = False
        self.current_maneuver = None
        self.emergency_stop_flag = False
        self.is_backing_up = False  # since this is async we don't want to interrupt evasive backup maneuvers while obejcts are still too close
        self.is_cliff = False

        # vision
        self.vision = VisionSystem()
        self.vision_enabled = True
        self.vision_clear = True

        # scanning parameters
        map_size = 100
        # self.map_size = map_size
        # self.map = np.zeros((map_size, map_size))
        self.scan_range = (-60, 60)
        self.scan_step = 5

    # def visualize_map(self):
    #     """Print ASCII visualization of the map"""
    #     for row in self.map:
    #         print(''.join(['1' if cell else '0' for cell in row]))

    def _update_ultrasonic_detection(self, distance: float):
        """Update map with obstacle detected by ultrasonic sensor"""
        if not (0 < distance < 300):  # Ignore invalid readings
            return

        # Calculate obstacle position in world coordinates
        sensor_angle_rad = math.radians(self.px.heading)
        sensor_x = self.px.x + self.ULTRASONIC_OFFSET_X * math.cos(sensor_angle_rad)
        sensor_y = self.px.y + self.ULTRASONIC_OFFSET_X * math.sin(sensor_angle_rad)

        obstacle_x = sensor_x + distance * math.cos(sensor_angle_rad)
        obstacle_y = sensor_y + distance * math.sin(sensor_angle_rad)

        # Add to world map
        self.world_map.add_obstacle(
            x=obstacle_x,
            y=obstacle_y,
            radius=5.0,  # Assume 5cm radius for ultrasonic detections
            confidence=0.8,
            label="ultrasonic_detection"
        )

    def _update_vision_detections(self, detected_objects: List[Dict]):
        """Update map with obstacles detected by vision system"""
        if not detected_objects:
            return

        for obj in detected_objects:
            # Calculate relative position from bounding box
            box = obj['box']  # (xmin, ymin, xmax, ymax)
            box_center_x = (box[0] + box[2]) / 2
            box_width = box[2] - box[0]

            # Estimate distance based on box width
            estimated_distance = 100 * (100 / box_width)  # Simplified example

            # Calculate object position in world coordinates
            camera_angle_rad = math.radians(self.px.heading)
            camera_x = (self.px.x + self.CAMERA_OFFSET_X * math.cos(camera_angle_rad))
            camera_y = (self.px.y + self.CAMERA_OFFSET_X * math.sin(camera_angle_rad))

            obstacle_x = camera_x + estimated_distance * math.cos(camera_angle_rad)
            obstacle_y = camera_y + estimated_distance * math.sin(camera_angle_rad)

            # Add to world map
            self.world_map.add_obstacle(
                x=obstacle_x,
                y=obstacle_y,
                radius=10.0,  # Adjust based on object type/size
                confidence=obj['confidence'],
                label=obj['label']
            )

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            await asyncio.sleep(0.01)

        if distances:
            self.current_distance = sum(distances) / len(distances)
            self._update_ultrasonic_detection(self.current_distance)
        return distances

    async def scan_environment(self):
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)
            await self.scan_avg()

        self.px.set_cam_pan_angle(0)
        return scan_data

    def _polar_to_cartesian(self, angle_deg, distance):
        """Convert polar coordinates to cartesian"""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return np.array([x, y])

    async def ultrasonic_monitoring(self):
        while True:
            distances = await self.scan_avg()
            if distances:
                print(f"Distance: {self.current_distance:.1f} cm")

                # emergency stop if too close during forward movement only
                if (self.current_distance < self.min_distance and
                        self.is_moving and
                        not self.emergency_stop_flag and
                        not self.is_backing_up):  # Don't interrupt backup
                    print(f"Emergency stop! Object detected at {self.current_distance:.1f}cm")
                    await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)

    async def cliff_monitoring(self):
        while True:
            self.is_cliff = self.px.get_cliff_status(self.px.get_grayscale_data())

            if (self.is_cliff and
                    self.is_moving and
                    not self.emergency_stop_flag and
                    not self.is_backing_up):
                print(f"Emergency stop, cliff detected!")
                await self.emergency_stop()

            await asyncio.sleep(self.sensor_read_freq)


    async def emergency_stop(self):
        self.emergency_stop_flag = True
        # cancel any ongoing maneuver except backup
        if self.current_maneuver:
            self.current_maneuver.cancel()

        self.is_moving = False
        self.px.forward(0)
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.5)
        self.emergency_stop_flag = False
        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

    def find_best_direction(self):
        """Analyze scan data to find the best direction to move"""
        max_distance = 0
        best_angle = 0

        for angle, distance in self.world_map.grid:
            if distance > max_distance:
                max_distance = distance
                best_angle = angle

        return best_angle, max_distance

    async def evasive_maneuver(self):
        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            await self.scan_environment()
            # add padding once scanning is done
            self.world_map.add_padding()

            best_angle, max_distance = self.find_best_direction()

            # normal evasive maneuver
            print("Backing up...")
            self.is_moving = True
            self.is_backing_up = True
            self.px.backward(self.speed)
            await asyncio.sleep(self.backup_time)
            self.is_backing_up = False

            print(f"Turning to {best_angle}° (clearest path: {max_distance:.1f}cm)")
            self.px.set_dir_servo_angle(best_angle)
            self.px.forward(self.speed)
            await asyncio.sleep(self.turn_time)

            if not self.emergency_stop_flag:
                self.px.set_dir_servo_angle(0)
                self.is_moving = True
                self.px.forward(self.speed)

            self.px.set_dir_servo_angle(0)

        except asyncio.CancelledError:
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            self.is_moving = False
            raise

        finally:
            self.current_maneuver = None

    async def forward_movement(self):
        while True:
            position = self.px.get_position()
            print(f'{position}')
            if not self.emergency_stop_flag and not self.current_maneuver:
                # Check both ultrasonic and vision systems
                if self.vision_enabled:
                    objects = self.vision.get_obstacle_info()
                    if objects:
                        self._update_vision_detections(objects)
                        for obj in objects:
                            print(f"Vision system detected: {obj['label']}")
                            if obj['label'] == "stop sign":
                                print("STOP!!!!")
                                self.vision_clear = False

                if (self.current_distance >= self.min_distance and not self.is_cliff):
                    if not self.is_moving:
                        print("Moving forward...")
                        self.is_moving = True
                        self.px.forward(self.speed)
                else:
                    if self.is_moving:
                        if self.is_cliff:
                            print("Cliff detected!")
                        elif not self.vision_clear:
                            print("Vision system detected obstacle!")
                        else:
                            print(f"Ultrasonic detected obstacle at {self.current_distance:.1f}cm")
                        self.is_moving = False
                        self.px.forward(0)
                        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

            # Periodically visualize the map (for debugging)
            if time.time() % 5 < 0.1:  # Every 5 seconds
                print("\nCurrent World Map:")
                self.world_map.visualize_map()
                pos = self.px.get_position()
                print(f"Position: x={pos['x']:.1f}, y={pos['y']:.1f}, heading={pos['heading']:.1f}°")

            await asyncio.sleep(0.1)

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            # Add vision task to existing tasks
            pos_trak_task = asyncio.create_task(self.px.continuous_position_tracking())
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            # movement_task = asyncio.create_task(self.forward_movement())
            movement_task = asyncio.create_task(self.px.navigate_to_point(50, 50))
            tasks = [pos_trak_task, vision_task, ultrasonic_task, cliff_task, movement_task]
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            print("\nShutting down gracefully...")
        finally:
            for task in tasks:
                task.cancel()
            try:
                await asyncio.gather(*tasks, return_exceptions=True)
            except asyncio.CancelledError:
                pass

            self.vision.cleanup()
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            print("Shutdown complete")

def main():
    avoider = AsyncObstacleAvoidance()
    try:
        loop = asyncio.get_event_loop()
        runner = loop.create_task(avoider.run())
        loop.run_until_complete(runner)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
        runner.cancel()
        loop.run_until_complete(runner)
    finally:
        loop.close()

if __name__ == "__main__":
    main()