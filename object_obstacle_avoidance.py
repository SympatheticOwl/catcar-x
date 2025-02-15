import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import libcamera
import asyncio
import time
import random
import math
from picamera2 import Picamera2
from picarx import Picarx
from robot_hat import TTS
from collections import deque
from datetime import datetime, timedelta

class AsyncObstacleAvoidance:
    def __init__(self):
        self.px = Picarx()
        self.tts = TTS()
        self.tts.lang("en-US")

        # configuration parameters
        self.min_distance = 15
        self.backup_time = 1.0
        self.long_backup_time = 2.0
        self.turn_time = 1.5
        self.speed = 30
        self.sensor_read_freq = 0.05

        # turn history tracking
        self.turn_history = deque(maxlen=10)
        self.turn_timestamps = deque(maxlen=10)
        self.stuck_threshold = timedelta(seconds=5)
        self.pattern_threshold = 5

        # state management
        self.current_distance = 100
        self.is_moving = False
        self.current_maneuver = None
        self.emergency_stop_flag = False
        self.is_backing_up = False  # since this is async we don't want to interrupt evasive backup maneuvers while obejcts are still too close
        self.is_cliff = False

        # scanning parameters
        map_size = 100
        self.map_size = map_size
        self.map = np.zeros((map_size, map_size))
        # self.car_pos = np.array([map_size // 2, map_size // 2])  # Start at center
        self.car_pos = np.array([0, map_size // 2])  # Start at lower center
        self.car_angle = 0  # Facing positive x-axis
        self.scan_range = (-60, 60)  # degrees
        self.scan_step = 5  # degrees

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.map:
            print(''.join(['1' if cell else '0' for cell in row]))

    async def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            await asyncio.sleep(0.01)
        return distances

    async def scan_environment(self):
        """Perform a sensor sweep and return scan data"""
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.set_cam_pan_angle(angle)
            await asyncio.sleep(0.1)

            distances = await self.scan_avg()

            if distances:
                avg_dist = sum(distances) / len(distances)
                scan_data.append((angle, avg_dist))

        self.px.set_cam_pan_angle(0)
        return scan_data

    def update_map(self, scan_data):
        """Update internal map with new scan data"""
        # Clear previous readings
        self.map = np.zeros((self.map_size, self.map_size))

        for i in range(len(scan_data) - 1):
            angle1, dist1 = scan_data[i]
            angle2, dist2 = scan_data[i + 1]

            # Convert to cartesian coordinates
            point1 = self._polar_to_cartesian(angle1, dist1)
            point2 = self._polar_to_cartesian(angle2, dist2)

            # Interpolate between points
            num_points = max(
                abs(int(point2[0] - point1[0])),
                abs(int(point2[1] - point1[1]))
            ) + 1

            if num_points > 1:
                x_points = np.linspace(point1[0], point2[0], num_points)
                y_points = np.linspace(point1[1], point2[1], num_points)

                # Mark interpolated points
                for x, y in zip(x_points, y_points):
                    map_x = int(x + self.map_size // 2)
                    map_y = int(y + self.map_size // 2)

                    if (0 <= map_x < self.map_size and
                            0 <= map_y < self.map_size):
                        self.map[map_y, map_x] = 1

        print("\nCurrent Map:")
        self.visualize_map()

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
                self.current_distance = sum(distances) / len(distances)

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

    def is_stuck_in_pattern(self):
        print(f"Checking stuck pattern...")
        print(f"Stuck pattern history length: {self.turn_history}...")
        print(len(self.turn_history))
        if len(self.turn_history) < self.pattern_threshold:
            return False

        # if we have too many turns in a certain amount of time we'll assume we're stuck
        # going back and forth in somewhere like a corner
        time_window = datetime.now() - self.stuck_threshold
        recent_turns = sum(1 for timestamp in self.turn_timestamps
                           if timestamp > time_window)
        print(f"Recent turns within time window: {recent_turns}...")

        return recent_turns < self.pattern_threshold

    async def spin_turn_180(self):
        print("Executing signature spin turn...")
        self.is_moving = True

        self.px.forward(0)
        await asyncio.sleep(0.2)

        if not self.emergency_stop_flag:
            # spins wheels in opposite direction
            # depending on friction of terrain will usually achieve
            # somewhere between 90-180 degrees
            spin_direction = self.choose_turn_direction()
            self.px.set_motor_speed(1, spin_direction * self.speed)
            self.px.set_motor_speed(2, spin_direction * self.speed)
            await asyncio.sleep(5)

            self.px.forward(0)
            await asyncio.sleep(0.2)

        self.px.set_dir_servo_angle(0)
        self.is_moving = False

    def choose_turn_direction(self):
        return random.choice([-1, 1])

    def find_best_direction(self, scan_data):
        """Analyze scan data to find the best direction to move"""
        max_distance = 0
        best_angle = 0

        for angle, distance in scan_data:
            if distance > max_distance:
                max_distance = distance
                best_angle = angle

        return best_angle, max_distance

    async def evasive_maneuver(self):

        try:
            self.is_moving = False
            self.px.forward(0)
            await asyncio.sleep(0.5)

            scan_data = await self.scan_environment()
            self.update_map(scan_data)

            best_angle, max_distance = self.find_best_direction(scan_data)

            # normal evasive maneuver
            print("Backing up...")
            self.is_moving = True
            self.is_backing_up = True
            self.px.backward(self.speed)
            await asyncio.sleep(self.backup_time)
            self.is_backing_up = False

            print(f"Turning to {best_angle}ÃÂ° (clearest path: {max_distance:.1f}cm)")
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
            if not self.emergency_stop_flag and not self.current_maneuver:
                if self.current_distance >= self.min_distance and not self.is_cliff:
                    if not self.is_moving:
                        print("Moving forward...")
                        self.is_moving = True
                        self.px.forward(self.speed)
                else:
                    if self.is_moving:
                        if self.is_cliff:
                            print("Cliff detected!")
                        else:
                            print(f"Obstacle detected at {self.current_distance:.1f}cm")
                        self.is_moving = False
                        self.px.forward(0)
                        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

            await asyncio.sleep(0.1)

    async def run(self):
        print("Starting obstacle avoidance program...")
        tasks = []
        try:
            # allow ultrasonic and cliff scanning to run concurrently with movement
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            movement_task = asyncio.create_task(self.forward_movement())
            tasks = [ultrasonic_task, cliff_task, movement_task]
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

            # reset position on shutdown
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)
            print("Shutdown complete")


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


class EnhancedObstacleAvoidance(AsyncObstacleAvoidance):
    def __init__(self):
        super().__init__()
        self.vision = VisionSystem()
        self.vision_enabled = True

    async def run(self):
        print("Starting enhanced obstacle avoidance program...")
        tasks = []
        try:
            # Add vision task to existing tasks
            vision_task = asyncio.create_task(self.vision.capture_and_detect())
            ultrasonic_task = asyncio.create_task(self.ultrasonic_monitoring())
            cliff_task = asyncio.create_task(self.cliff_monitoring())
            movement_task = asyncio.create_task(self.forward_movement())
            tasks = [vision_task, ultrasonic_task, cliff_task, movement_task]
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

    async def forward_movement(self):
        while True:
            if not self.emergency_stop_flag and not self.current_maneuver:
                # Check both ultrasonic and vision systems
                vision_clear = True
                if self.vision_enabled:
                    objects = self.vision.get_obstacle_info()
                    if objects:
                        print(f"Vision system detected: {[obj['label'] for obj in objects]}")
                        vision_clear = False

                if (self.current_distance >= self.min_distance and
                        not self.is_cliff and
                        vision_clear):
                    if not self.is_moving:
                        print("Moving forward...")
                        self.is_moving = True
                        self.px.forward(self.speed)
                else:
                    if self.is_moving:
                        if self.is_cliff:
                            print("Cliff detected!")
                        elif not vision_clear:
                            print("Vision system detected obstacle!")
                        else:
                            print(f"Ultrasonic detected obstacle at {self.current_distance:.1f}cm")
                        self.is_moving = False
                        self.px.forward(0)
                        self.current_maneuver = asyncio.create_task(self.evasive_maneuver())

            await asyncio.sleep(0.1)


def main():
    avoider = EnhancedObstacleAvoidance()
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
