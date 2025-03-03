import os
import json
import base64
import threading
import asyncio
import numpy as np
import time
from bluedot.btcomm import BluetoothServer as BlueDotServer
from commands import Commands


def numpy_json_encoder(obj):
    """Custom JSON encoder for NumPy objects"""
    if isinstance(obj, (np.integer, np.int32, np.int64)):
        return int(obj)
    elif isinstance(obj, (np.floating, np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.bool_):
        return bool(obj)
    else:
        try:
            return obj.tolist()  # Try to convert any other numpy objects
        except:
            pass
    return None  # Let default encoder handle it


# Set environment variables (same as in the Flask app)
os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # Tell Qt to not use GUI
os.environ['OPENCV_VIDEOIO_PRIORITY_BACKEND'] = 'v4l2'  # Use V4L2 backend for OpenCV
os.environ['MPLBACKEND'] = 'Agg'  # Force matplotlib to use Agg backend


class AsyncCommandManager:
    def __init__(self, commands: Commands):
        self.loop = asyncio.new_event_loop()
        self.commands = commands
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()

    def _run_event_loop(self):
        """Runs the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        # self.commands = Commands()

        # Initialize monitoring tasks in the event loop
        self.loop.run_until_complete(self._initialize_commands())
        self.loop.run_forever()

    async def _initialize_commands(self):
        """Initialize all monitoring tasks"""
        self.commands.state.ultrasonic_task = self.loop.create_task(
            self.commands.object_system.ultrasonic_monitoring())
        self.commands.state.cliff_task = self.loop.create_task(
            self.commands.object_system.cliff_monitoring())
        self.commands.state.pos_track_task = self.loop.create_task(
            self.commands.object_system.px.continuous_position_tracking())

    def run_coroutine(self, coro):
        """Run a coroutine in the event loop"""
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result(timeout=30)  # Add timeout to prevent hanging

    def cleanup(self):
        """Cleanup function to stop all tasks and the event loop"""
        if self.commands:
            if self.commands.state.vision_task:
                self.commands.stop_vision()
            self.commands.cancel_movement()

        # Stop the event loop
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=5)  # Add timeout to prevent hanging


class BTServer:
    def __init__(self, commands: Commands):
        print("Initializing Bluetooth server for PicarX...")
        self.manager = AsyncCommandManager(commands)
        self.connected_clients = {}  # Store client addresses and sockets
        self.client_lock = threading.Lock()  # Lock for thread-safe client access

        self.server = BlueDotServer(self.data_received, when_client_connects=self.client_connected,
                                    when_client_disconnects=self.client_disconnected,
                                    port=2,
                                    auto_start=False)

        # Wait for manager to initialize
        print("Waiting for AsyncCommandManager to initialize...")
        time.sleep(3)  # Give time for the event loop to start and commands to initialize

        # Start the server
        self.server.start()
        print(f"Bluetooth server started. Device name: {self.server.server_address}")
        print(f"The server is discoverable as '{self.server.server_address}'")

    def client_connected(self, client_address=None):
        """Called when a client connects"""
        if client_address:
            print(f"Client connected: {client_address}")
        else:
            print("Client connected (address unknown)")

    def client_disconnected(self, client_address=None):
        """Called when a client disconnects"""
        if client_address:
            print(f"Client disconnected: {client_address}")
        else:
            print("Client disconnected (address unknown)")

        # Stop any active movement when client disconnects
        if self.manager.commands:
            self.manager.commands.cancel_movement()

    def data_received(self, data):
        """Handle incoming Bluetooth commands"""
        try:
            # Convert data to string if it's bytes
            if isinstance(data, bytes):
                data = data.decode('utf-8')

            # Parse the JSON data
            request = json.loads(data)
            endpoint = request.get('endpoint', '')
            cmd = request.get('cmd', '')
            params = request.get('params', {})
            command_id = request.get('command_id', None)  # Extract the command_id

            print(f"Received command: {endpoint}/{cmd} with params: {params}, ID: {command_id}")

            # Handle different command types
            response = None
            if endpoint == 'command':
                response = self.handle_command(cmd, params)
            elif endpoint == 'status':
                response = self.handle_status()
            elif endpoint == 'world-state':
                response = self.handle_world_state()
            elif endpoint == 'visualization':
                response = self.handle_visualization()
            elif endpoint == 'telemetry':
                # Pass the entire request to include command_id
                response = self.handle_telemetry(cmd, request)
            elif endpoint == 'ping':
                # Handle ping command for connection testing
                response = json.dumps({
                    "status": "success",
                    "message": "pong",
                    "command_id": command_id
                })
            else:
                response = json.dumps({
                    "status": "error",
                    "message": f"Unknown endpoint: {endpoint}",
                    "command_id": command_id  # Include command_id in error response
                })

            # Ensure command_id is included in response if it's a dict
            if isinstance(response, str):
                try:
                    response_dict = json.loads(response)
                    if isinstance(response_dict, dict) and 'command_id' not in response_dict and command_id:
                        response_dict['command_id'] = command_id
                        response = json.dumps(response_dict)
                except json.JSONDecodeError:
                    # Not JSON, leave as is
                    pass

            # Use the new send_response method
            sent = self.send_response(response)
            if not sent:
                print("Warning: Failed to send response")

            return None  # Don't return response directly as we're manually sending it
        except json.JSONDecodeError:
            error_response = json.dumps({
                "status": "error",
                "message": "Invalid JSON data"
            })
            self.send_response(error_response)
            return None
        except Exception as e:
            import traceback
            traceback.print_exc()
            error_response = json.dumps({
                "status": "error",
                "message": str(e)
            })
            self.send_response(error_response)
            return None

    def send_response(self, response, client_address=None):
        """
        Send a response to a client

        Args:
            response (str): The response to send
            client_address (str, optional): The client address to send to (not used with BlueDot)
        """
        try:
            # Log what we're sending
            print(f"Sending response: {response[:100]}...")

            # BlueDotServer.send() only takes the response parameter, not the client address
            # It sends to all connected clients or the last connected client
            self.server.send(response)

            return True
        except Exception as e:
            print(f"Error sending response: {e}")
            import traceback
            traceback.print_exc()
            return False

    def send_chunked_response(self, data, command_id=None, client_address=None, chunk_size=1024):
        """
        Send large responses in chunks to avoid Bluetooth transmission issues

        Args:
            data (str): JSON string to be sent
            command_id (str, optional): Command ID to track the response
            client_address (str, optional): Not used with BlueDot but kept for API compatibility
            chunk_size (int, optional): Size of each chunk
        """
        if not command_id:
            command_id = "server_" + str(int(time.time()))

        # Calculate number of chunks
        total_size = len(data)
        chunks = (total_size + chunk_size - 1) // chunk_size

        # Send start marker
        start_marker = json.dumps({
            "type": "chunked_start",
            "command_id": command_id,
            "total_size": total_size,
            "chunks": chunks
        })
        print(f"Sending chunked response start: {start_marker}")
        self.send_response(start_marker)

        # Add a small delay between chunks to avoid overwhelming the Bluetooth connection
        time.sleep(0.1)

        # Send the actual data in chunks
        for i in range(chunks):
            start_pos = i * chunk_size
            end_pos = min(start_pos + chunk_size, total_size)
            chunk_data = data[start_pos:end_pos]

            chunk = json.dumps({
                "type": "chunk",
                "command_id": command_id,
                "chunk_index": i,
                "data": chunk_data
            })
            print(f"Sending chunk {i + 1}/{chunks}, size: {len(chunk_data)}")
            self.send_response(chunk)

            # Small delay between chunks
            time.sleep(0.05)

        # Send end marker
        end_marker = json.dumps({
            "type": "chunked_end",
            "command_id": command_id
        })
        print(f"Sending chunked response end: {end_marker}")
        self.send_response(end_marker)

        return True

    def handle_command(self, cmd, params):
        """Handle robot command endpoints"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        try:
            if cmd == "forward":
                # Check hazard flag without trying to create a task
                if self.manager.commands.state.emergency_stop_flag:
                    return json.dumps({
                        "status": "error",
                        "message": "Cannot move forward due to hazard"
                    })

                # Do the forward movement synchronously
                try:
                    # Direct call to the motor functions instead of creating a task
                    self.manager.commands.px.set_dir_servo_angle(0)  # Straighten wheels
                    self.manager.commands.px.forward(self.manager.commands.state.speed)
                    self.manager.commands.state.is_moving = True
                except Exception as e:
                    print(f"Forward movement error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Moving forward"
                })

            elif cmd == "backward":
                # Reset emergency stop flag
                self.manager.commands.state.emergency_stop_flag = False

                # Do the backward movement synchronously
                try:
                    # Direct call to the motor functions
                    self.manager.commands.px.set_dir_servo_angle(0)  # Straighten wheels
                    self.manager.commands.px.backward(self.manager.commands.state.speed)
                    self.manager.commands.state.is_moving = True
                except Exception as e:
                    print(f"Backward movement error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Moving backward"
                })

            elif cmd in ["left", "right"]:
                # Get angle from params, default to ±30
                angle = int(params.get('angle', 30 if cmd == "right" else -30))

                # This is a synchronous call, no need for event loop
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
                # This is a synchronous call, no need for event loop
                try:
                    self.manager.commands.cancel_movement()
                    self.manager.commands.px.stop()
                    self.manager.commands.state.is_moving = False

                    # Additionally, make sure any active task is cancelled in the event loop
                    if self.manager.commands.state.movement_task:
                        self.manager.loop.call_soon_threadsafe(self.manager.commands.state.movement_task.cancel)
                        self.manager.commands.state.movement_task = None
                except Exception as e:
                    print(f"Stop movement error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Stopped movement"
                })

            elif cmd == "set_speed":
                # Set speed from params
                speed = float(params.get('speed', 0.5))

                # Clamp speed between 0.1 and 1.0
                speed = max(0.1, min(1.0, speed))

                # Set speed in state
                self.manager.commands.state.speed = speed

                return json.dumps({
                    "status": "success",
                    "message": f"Speed set to {speed}"
                })

            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown command: {cmd}. Available commands: forward, backward, left, right, stop, set_speed"
                })

        except Exception as e:
            import traceback
            traceback.print_exc()
            return json.dumps({
                "status": "error",
                "message": str(e)
            })

    def handle_status(self):
        """Handle status endpoint"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        return json.dumps({
            "object_distance": float(self.manager.commands.get_object_distance()),
            "emergency_stop": bool(self.manager.commands.state.emergency_stop_flag),
            "is_moving": bool(self.manager.commands.state.is_moving),
            "speed": float(self.manager.commands.state.speed)
        }, default=numpy_json_encoder)

    def handle_world_state(self):
        """Handle world-state endpoint"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        return json.dumps({
            "state": self.manager.commands.world_state()
        }, default=numpy_json_encoder)

    def handle_visualization(self):
        """Handle visualization endpoint"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        try:
            # Get visualization data directly from the world map
            visualization_data = self.manager.commands.object_system.world_map.visualize()

            if visualization_data.get('visualization') is None:
                return json.dumps({
                    "status": "error",
                    "message": "Failed to generate visualization"
                })

            return json.dumps({
                "status": "success",
                "data": {
                    "grid_data": visualization_data['grid_data'],
                    "plot_image": visualization_data['visualization']
                }
            }, default=numpy_json_encoder)
        except Exception as e:
            import traceback
            traceback.print_exc()
            return json.dumps({
                "status": "error",
                "message": str(e)
            })

    def handle_telemetry(self, cmd, request=None):
        """Handle telemetry endpoint requests"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        command_id = None
        if request and isinstance(request, dict):
            command_id = request.get('command_id')

        try:
            if cmd == "all":
                # Get all telemetry data
                telemetry_data = self.manager.commands.get_telemetry()

                # Prepare the response
                response = {
                    "status": "success",
                    "telemetry": telemetry_data,
                    "command_id": command_id
                }

                # Convert to JSON string
                json_response = json.dumps(response, default=numpy_json_encoder)

                # Check if the response is large and needs chunking (e.g., over 1KB)
                if len(json_response) > 1024:
                    print(f"Telemetry response is large ({len(json_response)} bytes), sending chunked")
                    self.send_chunked_response(json_response, command_id)
                    # Return a placeholder to indicate chunked response is being sent
                    return json.dumps({
                        "status": "processing",
                        "message": "Sending telemetry data in chunks",
                        "command_id": command_id
                    })

                # If small enough, send directly
                return json_response

            elif cmd == "battery":
                # Get just battery information
                battery_data = self.manager.commands.get_battery_level()
                return json.dumps({
                    "status": "success",
                    "battery": battery_data,
                    "command_id": command_id
                }, default=numpy_json_encoder)

            elif cmd == "temperature":
                # Get just CPU temperature
                temp = self.manager.commands.get_cpu_temperature()
                return json.dumps({
                    "status": "success",
                    "temperature": temp,
                    "command_id": command_id
                }, default=numpy_json_encoder)

            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown telemetry command: {cmd}. Available commands: all, battery, temperature",
                    "command_id": command_id
                })

        except Exception as e:
            import traceback
            traceback.print_exc()
            return json.dumps({
                "status": "error",
                "message": str(e),
                "command_id": command_id
            })

    def cleanup(self):
        """Cleanup resources"""
        print("Cleaning up resources...")
        self.manager.cleanup()
        self.server.stop()


# This is used if this file is run directly
if __name__ == "__main__":
    try:
        # Create a Commands instance
        commands = Commands()

        # Create and start the server
        server = BTServer(commands)
        print("Server running. Press Ctrl+C to exit.")

        # Keep the main thread alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'server' in locals():
            server.cleanup()