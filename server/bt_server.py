import os
import time
import asyncio
import threading
import json
import logging
from typing import Dict, Optional

# Bluetooth libraries
import bluetooth
from bluetooth.bluez import BluetoothSocket
from bluetooth.bluez import advertise_service
import uuid

# Set environment variables
os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # Tell Qt to not use GUI
os.environ['OPENCV_VIDEOIO_PRIORITY_BACKEND'] = 'v4l2'  # Use V4L2 backend for OpenCV
os.environ['MPLBACKEND'] = 'Agg'  # Force matplotlib to use Agg backend

# Import your existing commands module
from commands import Commands

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("PicarX_BT_Server")

# Define Bluetooth service
SERVICE_NAME = "PicarXBTServer"
SERVICE_UUID = str(uuid.uuid4())


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
        return future.result()

    def get_cached_visualization(self):
        """Get the cached visualization data"""
        with self._visualization_lock:
            return self._last_visualization

    def update_cached_visualization(self, viz_data):
        """Update the cached visualization data"""
        with self._visualization_lock:
            self._last_visualization = viz_data


class BluetoothServer:
    def __init__(self, port=1):
        self.manager = AsyncCommandManager()
        self.port = port
        self.server_sock = None
        self.client_sock = None
        self.client_info = None
        self.running = False
        self.current_frame = None
        self.frame_lock = threading.Lock()

    def setup_server(self):
        """Set up the Bluetooth server socket"""
        try:
            # Create a Bluetooth server socket
            self.server_sock = BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("", self.port))
            self.server_sock.listen(1)  # Only allow 1 connection at a time

            # Advertise the service
            advertise_service(
                self.server_sock,
                SERVICE_NAME,
                service_id=SERVICE_UUID,
                service_classes=[SERVICE_UUID, bluetooth.SERIAL_PORT_CLASS],
                profiles=[bluetooth.SERIAL_PORT_PROFILE]
            )

            logger.info(f"Bluetooth server started on port {self.port}")
            logger.info(f"Service Name: {SERVICE_NAME}")
            logger.info(f"Service UUID: {SERVICE_UUID}")

            return True
        except Exception as e:
            logger.error(f"Error setting up Bluetooth server: {str(e)}")
            return False

    def wait_for_connection(self):
        """Wait for a client to connect"""
        logger.info("Waiting for Bluetooth connection...")
        self.client_sock, self.client_info = self.server_sock.accept()
        logger.info(f"Accepted connection from {self.client_info}")
        return True

    def handle_command(self, command_str):
        """Handle a command received from the client"""
        try:
            # Parse the command
            command_data = json.loads(command_str)
            cmd = command_data.get('cmd')
            params = command_data.get('params', {})

            logger.debug(f"Received command: {cmd}, params: {params}")

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

    def run(self):
        """Run the Bluetooth server"""
        if not self.setup_server():
            return

        self.running = True

        while self.running:
            try:
                # Wait for a connection
                self.wait_for_connection()

                # Handle client communication
                while True:
                    try:
                        # Receive data
                        data = self.client_sock.recv(1024)
                        if not data:
                            break

                        # Process the command
                        command_str = data.decode('utf-8').strip()
                        response = self.handle_command(command_str)

                        # Send response
                        self.client_sock.send(response.encode('utf-8') + b'\n')

                    except bluetooth.btcommon.BluetoothError as e:
                        logger.error(f"Bluetooth error: {str(e)}")
                        break

                # Close the client socket
                if self.client_sock:
                    self.client_sock.close()
                    self.client_sock = None

            except KeyboardInterrupt:
                logger.info("Server shutdown requested...")
                self.running = False
                break
            except Exception as e:
                logger.error(f"Error in server loop: {str(e)}")
                time.sleep(1)  # Prevent tight loop on error

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up resources...")

        # Stop the command manager
        if self.manager.commands:
            if self.manager.commands.state.vision_task:
                self.manager.commands.stop_vision()
            self.manager.commands.cancel_movement()

        # Close sockets
        if self.client_sock:
            self.client_sock.close()
        if self.server_sock:
            self.server_sock.close()

        # Stop the event loop
        self.manager.loop.call_soon_threadsafe(self.manager.loop.stop)
        self.manager.thread.join()

        logger.info("Cleanup complete")


def main():
    """Main function"""
    try:
        # Create and run the server
        server = BluetoothServer()
        server.run()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        logger.error(f"Error in main: {str(e)}")
    finally:
        if 'server' in locals():
            server.cleanup()


if __name__ == "__main__":
    main()