#!/usr/bin/env python3
"""
Raspberry Pi Bluetooth Server
----------------------------
This script runs on the Raspberry Pi car and exposes the car's functionality
over Bluetooth using the BlueDot library. It receives commands from the
Bluetooth client and forwards them to the existing car control system.
"""

import os
import sys
import json
import logging
import asyncio
import threading
import time
import base64
from typing import Dict, Any
from io import BytesIO
from bluedot.btcomm import BluetoothServer

# Add the current directory to the path so we can import the existing modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the existing car control system
from commands import Commands
from state_handler import State

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("BT-Server")


class CarBluetoothServer:
    """
    Bluetooth server that exposes the car's functionality over Bluetooth.
    """

    def __init__(self):
        """Initialize the Bluetooth server and car control system."""
        # Initialize the event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()

        # Initialize the server
        self.server = None
        self.client = None
        self.video_thread = None
        self.video_enabled = False

    def _run_event_loop(self):
        """Run the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)

        # Initialize the car control system
        self.commands = Commands()

        # Initialize monitoring tasks
        self._initialize_tasks()

        # Run the event loop
        self.loop.run_forever()

    def _initialize_tasks(self):
        """Initialize all monitoring tasks."""
        # Run in the event loop thread
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
            self.server = BluetoothServer(
                self._data_received,
                auto_start=False,
                when_client_connects=self._client_connected,
                when_client_disconnects=self._client_disconnected,
                port=2
            )
            self.server.start()
            logger.info(f"Bluetooth server started on {self.server.server_address}")
            return True
        except Exception as e:
            logger.error(f"Failed to start Bluetooth server: {str(e)}")
            return False

    def stop(self):
        """Stop the Bluetooth server and clean up resources."""
        try:
            logger.info("Stopping Bluetooth server...")

            # Stop video streaming if active
            self.video_enabled = False
            if self.video_thread and self.video_thread.is_alive():
                self.video_thread.join(timeout=2.0)

            # Disconnect client
            if self.client:
                try:
                    self.client.disconnect()
                except:
                    pass
                self.client = None

            # Stop server
            if self.server:
                self.server.stop()

            # Cancel all tasks
            self._cancel_tasks()

            # Stop event loop
            if self.loop and self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)

            # Wait for thread to finish
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=2.0)

            logger.info("Bluetooth server stopped")
            return True
        except Exception as e:
            logger.error(f"Error stopping Bluetooth server: {str(e)}")
            return False

    def _cancel_tasks(self):
        """Cancel all tasks in the event loop."""
        # Get all tasks from the event loop
        tasks = asyncio.all_tasks(self.loop)

        # Cancel all tasks
        for task in tasks:
            task.cancel()

    def _client_connected(self, client):
        """
        Callback for when a client connects.

        Args:
            client: The connected client
        """
        self.client = client
        client_info = client.client_address if hasattr(client, 'client_address') else "Unknown"
        logger.info(f"Client connected: {client_info}")

    def _client_disconnected(self, client):
        """
        Callback for when a client disconnects.

        Args:
            client: The disconnected client
        """
        client_info = client.client_address if hasattr(client, 'client_address') else "Unknown"
        logger.info(f"Client disconnected: {client_info}")

        # Stop video streaming if active
        self.video_enabled = False

        # Reset client reference
        self.client = None

    def _data_received(self, data):
        """
        Callback for when data is received from a client.

        Args:
            data: The received data
        """
        try:
            # Parse JSON data
            cmd_data = json.loads(data)

            # Extract command and parameters
            command = cmd_data.get('command')
            params = cmd_data.get('params', {})

            logger.debug(f"Command received: {command}, Params: {params}")

            # Handle command
            if command == 'forward':
                self._run_coroutine(self._handle_forward())
            elif command == 'backward':
                self._run_coroutine(self._handle_backward())
            elif command == 'left' or command == 'right':
                angle = params.get('angle', 30 if command == 'right' else -30)
                self._run_coroutine(self._handle_turn(angle))
            elif command == 'stop':
                self._run_coroutine(self._handle_stop())
            elif command == 'scan':
                self._run_coroutine(self._handle_scan())
            elif command == 'see':
                self._handle_see()
            elif command == 'blind':
                self._handle_blind()
            elif command == 'visualization':
                self._run_coroutine(self._handle_visualization())
            else:
                self._send_error(f"Unknown command: {command}")
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON data received: {data}")
            self._send_error("Invalid JSON data")
        except Exception as e:
            logger.error(f"Error handling command: {str(e)}")
            self._send_error(f"Error: {str(e)}")

    def _run_coroutine(self, coro):
        """
        Run a coroutine in the event loop.

        Args:
            coro: The coroutine to run
        """
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        try:
            return future.result(timeout=10.0)
        except asyncio.TimeoutError:
            logger.error("Coroutine execution timed out")
            return None
        except Exception as e:
            logger.error(f"Error running coroutine: {str(e)}")
            return None

    async def _handle_forward(self):
        """Handle forward command."""
        success = self.commands.forward()
        if success:
            self._send_response({
                "status": "success",
                "message": "Moving forward"
            })
        else:
            self._send_error("Cannot move forward due to hazard")

    async def _handle_backward(self):
        """Handle backward command."""
        self.commands.backward()
        self._send_response({
            "status": "success",
            "message": "Moving backward"
        })

    async def _handle_turn(self, angle):
        """
        Handle turn command.

        Args:
            angle: The angle to turn
        """
        success = self.commands.turn(angle)
        if success:
            self._send_response({
                "status": "success",
                "message": f"Turning with angle {angle}"
            })
        else:
            self._send_error("Cannot turn due to hazard")

    async def _handle_stop(self):
        """Handle stop command."""
        self.commands.cancel_movement()
        self._send_response({
            "status": "success",
            "message": "Stopped movement"
        })

    async def _handle_scan(self):
        """Handle scan command."""
        try:
            logger.info("Starting environment scan...")
            await self.commands.object_system.scan_environment()
            logger.info("Environment scan completed")

            # Generate visualization right after scan completes
            visualization_data = self.commands.object_system.world_map.visualize()

            # Get the world state
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

        while self.video_enabled and self.client:
            try:
                # Get latest frame
                frame = self.commands.vision.get_latest_frame_jpeg()

                if frame:
                    # Encode frame as base64
                    frame_b64 = base64.b64encode(frame).decode('utf-8')

                    # Send frame with "FRAME:" prefix to identify it as a video frame
                    self.client.send(f"FRAME:{frame_b64}")

                # Add a small delay to control frame rate
                time.sleep(0.1)  # Reduce to ~10 fps for Bluetooth
            except Exception as e:
                logger.error(f"Error streaming video frame: {str(e)}")
                break

        logger.info("Video streaming stopped")

    def _get_world_state(self):
        """
        Get the current world state.

        Returns:
            Dictionary with world state data
        """
        # Get position data
        position = self.commands.object_system.px.get_position()

        # Get ultrasonic data
        distance = self.commands.object_system.get_current_distance()

        # Get detected objects if vision system is active
        objects = []
        if self.commands.state.vision_task and not self.commands.state.vision_task.done():
            objects = self.commands.vision.detected_objects

        return {
            "position": position,
            "distance": distance,
            "objects": objects,
            "emergency_stop": self.commands.state.emergency_stop_flag,
            "is_moving": self.commands.state.is_moving,
            "is_cliff": self.commands.state.is_cliff
        }

    def _send_response(self, data):
        """
        Send a JSON response to the client.

        Args:
            data: The data to send
        """
        if self.client:
            try:
                self.client.send(json.dumps(data))
            except Exception as e:
                logger.error(f"Error sending response: {str(e)}")

    def _send_error(self, message):
        """
        Send an error message to the client.

        Args:
            message: The error message
        """
        self._send_response({
            "status": "error",
            "message": message
        })


def main():
    """Main function."""
    try:
        # Create and start the Bluetooth server
        server = CarBluetoothServer()
        if server.start():
            # Keep the main thread running
            while True:
                time.sleep(1)
        else:
            logger.error("Failed to start Bluetooth server")
            sys.exit(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        # Clean up resources
        if 'server' in locals():
            server.stop()


if __name__ == "__main__":
    main()