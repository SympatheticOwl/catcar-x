#!/usr/bin/env python3
"""
Minimal Raspberry Pi Bluetooth Server
------------------------------------
A minimal implementation that avoids using problematic callbacks.
"""

import os
import json
import logging
import asyncio
import threading
import time
import base64
from bluedot.btcomm import BluetoothServer

# Import existing car components
from commands import Commands

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("BT-Server")

# Global variable to track client connections
bt_client = None


def data_received(data):
    """
    Global callback function for received data.

    Args:
        data: Data received from the client
    """
    global bt_client

    # If this is the first message, set the client
    if bt_client is None and hasattr(data, 'client'):
        bt_client = data.client
        logger.info(f"Client connected")

    try:
        # Process the data
        car_server.process_command(data)
    except Exception as e:
        logger.error(f"Error processing data: {str(e)}")


class CarBluetoothServer:
    """Minimalist Bluetooth server for the PicarX car."""

    def __init__(self):
        """Initialize the server and car control system."""
        # Create commands interface
        self.commands = Commands()

        # Initialize Bluetooth server
        self.server = None
        self.video_enabled = False
        self.video_thread = None

        # Set up event loop for async operations
        self.loop = asyncio.get_event_loop()

        # Start monitoring tasks
        self._start_monitoring_tasks()

    def _start_monitoring_tasks(self):
        """Start basic monitoring tasks in the event loop."""
        # Create monitoring tasks
        self.commands.state.ultrasonic_task = self.loop.create_task(
            self.commands.object_system.ultrasonic_monitoring())
        self.commands.state.cliff_task = self.loop.create_task(
            self.commands.object_system.cliff_monitoring())
        self.commands.state.pos_track_task = self.loop.create_task(
            self.commands.object_system.px.continuous_position_tracking())

    def start(self):
        """Start the Bluetooth server."""
        try:
            logger.info("Starting Bluetooth server...")

            # Create Bluetooth server with only the data received callback
            self.server = BluetoothServer(data_received, port=2)

            logger.info(f"Bluetooth server started")
            return True

        except Exception as e:
            logger.error(f"Failed to start Bluetooth server: {str(e)}")
            return False

    def stop(self):
        """Stop the server and clean up resources."""
        logger.info("Stopping Bluetooth server...")

        # Stop video if running
        self.video_enabled = False
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join(timeout=2.0)

        # Stop server
        if self.server:
            self.server.stop()

        # Cancel asyncio tasks
        for task in asyncio.all_tasks(self.loop):
            task.cancel()

        logger.info("Bluetooth server stopped")
        return True

    def process_command(self, data):
        """
        Process command received from the client.

        Args:
            data: The received data
        """
        try:
            # Parse JSON data
            cmd_data = json.loads(data)
            command = cmd_data.get('command')
            params = cmd_data.get('params', {})

            logger.info(f"Command received: {command}, params: {params}")

            # Process command with appropriate handler
            if command == 'forward':
                success = self.commands.forward()
                self._send_response({
                    "status": "success" if success else "error",
                    "message": "Moving forward" if success else "Cannot move forward due to hazard"
                })

            elif command == 'backward':
                self.commands.backward()
                self._send_response({
                    "status": "success",
                    "message": "Moving backward"
                })

            elif command == 'left' or command == 'right':
                angle = params.get('angle', 30 if command == 'right' else -30)
                success = self.commands.turn(angle)
                self._send_response({
                    "status": "success" if success else "error",
                    "message": f"Turning with angle {angle}" if success else "Cannot turn due to hazard"
                })

            elif command == 'stop':
                self.commands.cancel_movement()
                self._send_response({
                    "status": "success",
                    "message": "Stopped movement"
                })

            elif command == 'scan':
                # Create future for async operation
                future = asyncio.run_coroutine_threadsafe(self._handle_scan(), self.loop)
                # Wait for result (optional, but ensures we get the response)
                try:
                    future.result(timeout=10)
                except Exception as e:
                    logger.error(f"Error in scan operation: {str(e)}")

            elif command == 'see':
                self._handle_see()

            elif command == 'blind':
                self._handle_blind()

            elif command == 'visualization':
                future = asyncio.run_coroutine_threadsafe(self._handle_visualization(), self.loop)
                try:
                    future.result(timeout=5)
                except Exception as e:
                    logger.error(f"Error in visualization operation: {str(e)}")

            else:
                self._send_error(f"Unknown command: {command}")

        except json.JSONDecodeError:
            logger.error(f"Invalid JSON data received: {data}")
            self._send_error("Invalid JSON data")

        except Exception as e:
            logger.error(f"Error handling command: {str(e)}")
            self._send_error(f"Error: {str(e)}")

    async def _handle_scan(self):
        """Handle scan command."""
        try:
            logger.info("Starting environment scan...")

            # Perform the scan
            await self.commands.object_system.scan_environment()
            logger.info("Environment scan completed")

            # Generate visualization data
            visualization_data = self.commands.object_system.world_map.visualize()

            # Get world state
            world_data = self._get_world_state()

            # Send response
            self._send_response({
                "status": "success",
                "data": {
                    "grid_data": visualization_data['grid_data'],
                    "plot_image": visualization_data['visualization'],
                    "world_state": world_data
                }
            })

        except Exception as e:
            logger.error(f"Error during scan: {str(e)}")
            self._send_error(f"Scan error: {str(e)}")

    async def _handle_visualization(self):
        """Handle visualization command."""
        try:
            # Generate visualization
            visualization_data = self.commands.object_system.world_map.visualize()

            # Send response
            self._send_response({
                "status": "success",
                "data": {
                    "grid_data": visualization_data['grid_data'],
                    "plot_image": visualization_data['visualization']
                }
            })

        except Exception as e:
            logger.error(f"Error generating visualization: {str(e)}")
            self._send_error(f"Visualization error: {str(e)}")

    def _handle_see(self):
        """Handle see command to start video streaming."""
        try:
            # Start vision system if not already running
            if not self.commands.state.vision_task or self.commands.state.vision_task.done():
                self.commands.state.vision_task = self.loop.create_task(
                    self.commands.vision.capture_and_detect())

            # Start video streaming thread if not already running
            self.video_enabled = True
            if not self.video_thread or not self.video_thread.is_alive():
                self.video_thread = threading.Thread(target=self._video_stream_loop)
                self.video_thread.daemon = True
                self.video_thread.start()

            # Get current detected objects
            objects = self.commands.get_objects()

            # Send response
            self._send_response({
                "status": "success",
                "message": "Vision system started",
                "objects": objects
            })

        except Exception as e:
            logger.error(f"Error starting vision system: {str(e)}")
            self._send_error(f"Vision error: {str(e)}")

    def _handle_blind(self):
        """Handle blind command to stop video streaming."""
        try:
            # Stop video streaming
            self.video_enabled = False

            # Stop vision system
            self.commands.stop_vision()

            # Send response
            self._send_response({
                "status": "success",
                "message": "Vision system stopped"
            })

        except Exception as e:
            logger.error(f"Error stopping vision system: {str(e)}")
            self._send_error(f"Vision error: {str(e)}")

    def _video_stream_loop(self):
        """Background thread for streaming video frames."""
        logger.info("Video streaming started")
        global bt_client

        while self.video_enabled and bt_client:
            try:
                # Get latest frame
                frame = self.commands.vision.get_latest_frame_jpeg()

                if frame:
                    # Encode frame as base64
                    frame_b64 = base64.b64encode(frame).decode('utf-8')

                    # Send frame with "FRAME:" prefix to identify it as a video frame
                    bt_client.send(f"FRAME:{frame_b64}")

                # Add a small delay to control frame rate
                time.sleep(0.1)  # Reduce to ~10 fps for Bluetooth

            except Exception as e:
                logger.error(f"Error streaming video frame: {str(e)}")
                break

        logger.info("Video streaming stopped")

    def _get_world_state(self):
        """Get the current world state."""
        # Get position data
        position = self.commands.object_system.px.get_position()

        # Get ultrasonic data
        distance = self.commands.object_system.get_current_distance()

        # Get detected objects if vision system is active
        objects = []
        if hasattr(self.commands,
                   'vision') and self.commands.state.vision_task and not self.commands.state.vision_task.done():
            objects = self.commands.vision.detected_objects

        return {
            "position": position,
            "distance": distance,
            "objects": objects,
            "emergency_stop": self.commands.state.emergency_stop_flag,
            "is_moving": self.commands.state.is_moving,
            "is_cliff": self.commands.state.is_cliff,
            'ascii_map': self.commands.object_system.world_map.get_ascii_map(),
            'visualization': self.commands.object_system.world_map.visualize(return_image=True)
        }

    def _send_response(self, data):
        """Send a JSON response to the client."""
        global bt_client
        if bt_client:
            try:
                bt_client.send(json.dumps(data))
            except Exception as e:
                logger.error(f"Error sending response: {str(e)}")

    def _send_error(self, message):
        """Send an error message to the client."""
        self._send_response({
            "status": "error",
            "message": message
        })


# Create a global instance
car_server = None


def main():
    """Main function."""
    global car_server

    try:
        # Create and start the Bluetooth server
        car_server = CarBluetoothServer()
        if car_server.start():
            # Run the event loop to keep the program alive
            car_server.loop.run_forever()
        else:
            logger.error("Failed to start Bluetooth server")
            return 1

    except KeyboardInterrupt:
        logger.info("Shutting down due to keyboard interrupt...")

    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
        return 1

    finally:
        # Clean up resources
        if car_server:
            car_server.stop()

    return 0


if __name__ == "__main__":
    exit(main())