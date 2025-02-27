import os
import time
import asyncio
import threading
import json
import logging
from typing import Dict, Optional
import base64

# Set environment variables
os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # Tell Qt to not use GUI
os.environ['OPENCV_VIDEOIO_PRIORITY_BACKEND'] = 'v4l2'  # Use V4L2 backend for OpenCV
os.environ['MPLBACKEND'] = 'Agg'  # Force matplotlib to use Agg backend

# Bluedot library
from bluedot.btcomm import BluetoothServer

# Import your existing commands module
from commands import Commands

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("PicarX_Bluedot_Server")


class AsyncCommandManager:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.commands = None
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()
        self._last_visualization = None  # Cache for visualization data
        self._visualization_lock = threading.Lock()

    def _run_event_loop(self):
        """Runs the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.commands = Commands()

        # Initialize monitoring tasks in the event loop
        self.loop.run_until_complete(self._initialize_commands())
        self.loop.run_forever()

    async def _initialize_commands(self):
        """Initialize all monitoring tasks"""
        logger.info("Initializing monitoring tasks...")
        self.commands.state.ultrasonic_task = self.loop.create_task(
            self.commands.object_system.ultrasonic_monitoring())
        self.commands.state.cliff_task = self.loop.create_task(
            self.commands.object_system.cliff_monitoring())
        self.commands.state.pos_track_task = self.loop.create_task(
            self.commands.object_system.px.continuous_position_tracking())
        logger.info("Monitoring tasks initialized")

    def run_coroutine(self, coro):
        """Run a coroutine in the event loop and return its result"""
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result(10)  # Wait up to 10 seconds for result

    def get_cached_visualization(self):
        """Get the cached visualization data"""
        with self._visualization_lock:
            return self._last_visualization

    def update_cached_visualization(self, viz_data):
        """Update the cached visualization data"""
        with self._visualization_lock:
            self._last_visualization = viz_data


class BluedotServer:
    def __init__(self, device_name="PicarXServer"):
        """Initialize the Bluedot server

        Args:
            device_name: Device name for Bluetooth advertisements
        """
        self.manager = AsyncCommandManager()
        self.device_name = device_name
        self.server = None
        self.running = False

        # Set up connection state
        self.connected_clients = set()

    def handle_data(self, data, client_addr):
        """Handle data received from a client

        Args:
            data: The data received as string
            client_addr: The client address tuple (address, port)

        Returns:
            Response string or None
        """
        try:
            # Parse the command
            command_data = json.loads(data)
            cmd = command_data.get('cmd')
            params = command_data.get('params', {})

            logger.debug(f"Received command: {cmd}, params: {params} from {client_addr}")

            if not self.manager.commands:
                return json.dumps({
                    "status": "error",
                    "message": "Server still initializing"
                })

            # Process commands
            if cmd == "forward":
                success = self.manager.commands.forward()
                if not success:
                    return json.dumps({
                        "status": "error",
                        "message": "Cannot move forward due to hazard"
                    })
                return json.dumps({
                    "status": "success",
                    "message": "Moving forward"
                })

            elif cmd == "backward":
                self.manager.commands.backward()
                return json.dumps({
                    "status": "success",
                    "message": "Moving backward"
                })

            elif cmd in ["left", "right"]:
                angle = params.get('angle', 30 if cmd == "right" else -30)
                success = self.manager.commands.turn(angle)
                if not success:
                    return json.dumps({
                        "status": "error",
                        "message": "Cannot turn due to hazard"
                    })
                return json.dumps({
                    "status": "success",
                    "message": f"Turning {cmd} with angle {angle}"
                })

            elif cmd == "stop":
                self.manager.commands.cancel_movement()
                return json.dumps({
                    "status": "success",
                    "message": "Stopped movement"
                })

            elif cmd == "scan":
                # Run the scan operation and get results
                try:
                    scan_result = self.manager.run_coroutine(self.manager.commands.scan_env())

                    # Cache visualization for later requests
                    if scan_result.get('status') == 'success':
                        self.manager.update_cached_visualization(scan_result.get('data', {}))

                    # Since Bluetooth has limited bandwidth, we'll just inform that scan is complete
                    # The client can request visualization separately if needed
                    return json.dumps({
                        "status": "success",
                        "message": "Scan complete",
                        "world_state": scan_result.get('data', {}).get('world_state', {})
                    })
                except asyncio.TimeoutError:
                    return json.dumps({
                        "status": "error",
                        "message": "Scan operation timed out"
                    })

            elif cmd == "get_visualization":
                # Get the cached visualization
                viz_data = self.manager.get_cached_visualization()
                if not viz_data:
                    return json.dumps({
                        "status": "error",
                        "message": "No visualization data available. Run a scan first."
                    })

                # Return the visualization data
                return json.dumps({
                    "status": "success",
                    "data": {
                        "grid_data": viz_data.get('grid_data', {}),
                        # Don't include the plot image in the initial response as it could be large
                        "has_image": "plot_image" in viz_data
                    }
                })

            elif cmd == "get_visualization_image":
                # Get the cached visualization
                viz_data = self.manager.get_cached_visualization()
                if not viz_data or "plot_image" not in viz_data:
                    return json.dumps({
                        "status": "error",
                        "message": "No visualization image available. Run a scan first."
                    })

                # Return just the image part
                return json.dumps({
                    "status": "success",
                    "data": {
                        "plot_image": viz_data.get("plot_image", "")
                    }
                })

            elif cmd == "status":
                return json.dumps({
                    "status": "success",
                    "data": {
                        "object_distance": self.manager.commands.get_object_distance(),
                        "emergency_stop": self.manager.commands.state.emergency_stop_flag,
                        "position": self.manager.commands.object_system.px.get_position(),
                    }
                })

            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown command: {cmd}"
                })

        except Exception as e:
            logger.error(f"Error handling command: {str(e)}")
            return json.dumps({
                "status": "error",
                "message": f"Server error: {str(e)}"
            })

    def client_connected(self, client_addr):
        """Handle client connection"""
        logger.info(f"Client connected: {client_addr}")
        self.connected_clients.add(client_addr)

    def client_disconnected(self, client_addr):
        """Handle client disconnection"""
        logger.info(f"Client disconnected: {client_addr}")
        if client_addr in self.connected_clients:
            self.connected_clients.remove(client_addr)

    def start(self):
        """Start the Bluetooth server"""
        try:
            logger.info(f"Starting Bluetooth server with device name: {self.device_name}")

            # Create the Bluetooth server
            self.server = BluetoothServer(
                self.handle_data,
                when_client_connects=self.client_connected,
                when_client_disconnects=self.client_disconnected,
                device=self.device_name,
                auto_start=False
            )

            # Start the server
            self.server.start()
            self.running = True

            logger.info("Bluetooth server started. Waiting for connections...")

            # Keep the main thread alive
            try:
                while self.running:
                    time.sleep(1)
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received. Shutting down...")
                self.stop()

        except Exception as e:
            logger.error(f"Error starting Bluetooth server: {str(e)}")
            self.stop()

    def stop(self):
        """Stop the Bluetooth server"""
        logger.info("Stopping Bluetooth server...")
        self.running = False

        if self.server:
            self.server.stop()
            self.server = None

        # Clean up command manager
        if self.manager.commands:
            if hasattr(self.manager.commands, 'state') and hasattr(self.manager.commands.state,
                                                                   'vision_task') and self.manager.commands.state.vision_task:
                self.manager.commands.stop_vision()
            self.manager.commands.cancel_movement()

        # Stop the event loop
        self.manager.loop.call_soon_threadsafe(self.manager.loop.stop)
        self.manager.thread.join(2)  # Wait up to 2 seconds


def main():
    """Main function"""
    try:
        # Create and start the server
        server = BluedotServer()
        server.start()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        logger.error(f"Error in main: {str(e)}")
    finally:
        if 'server' in locals():
            server.stop()


if __name__ == "__main__":
    main()