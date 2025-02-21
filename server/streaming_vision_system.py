import threading
from typing import Optional
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import libcamera
import asyncio
import time
from picamera2 import Picamera2
from state_handler import State


class VisionSystem:
    def __init__(self, state: State, model_path='../efficientdet_lite0.tflite', labels_path='../coco_labels.txt'):
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
        self._frame_lock = threading.Lock()
        self._latest_jpeg = None

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

                # Draw boxes and update jpeg frame
                debug_frame = self._draw_boxes()
                self._update_jpeg_frame(debug_frame)

                self.last_process_time = current_time

            # Small sleep to prevent CPU overload
            await asyncio.sleep(0.01)

    def _draw_boxes(self):
        """Draw bounding boxes on the frame and return the annotated frame"""
        if self.frame is None:
            return None

        debug_frame = self.frame.copy()
        for obj in self.detected_objects:
            xmin, ymin, xmax, ymax = obj['box']
            label = f"{obj['label']} {obj['confidence']:.2f}"

            cv2.rectangle(debug_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(debug_frame, label, (xmin, ymin - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return debug_frame

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

    def _update_jpeg_frame(self, frame: np.ndarray) -> None:
        """Update the latest JPEG frame with thread safety"""
        if frame is None:
            return

        # Encode frame to JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if ret:
            with self._frame_lock:
                self._latest_jpeg = jpeg.tobytes()

    def get_latest_frame_jpeg(self) -> Optional[bytes]:
        """Get the latest JPEG frame with thread safety"""
        with self._frame_lock:
            return self._latest_jpeg

    def cleanup(self):
        """Cleanup camera resources"""
        self.camera.stop()
        cv2.destroyAllWindows()
