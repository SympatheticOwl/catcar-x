import os
import json
import base64
import threading
import asyncio
import numpy as np
import time
from bluedot.btcomm import BluetoothServer as BlueDotServer
from commands import Commands  # Assuming commands.py contains the Commands class


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
        self.server = BlueDotServer(self.data_received, when_client_connects=self.client_connected,
                                    when_client_disconnects=self.client_disconnected,
                                    port=2,
                                    auto_start=False)
        self.video_active = False
        self.video_thread = None

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

        # Stop video streaming if active
        if self.video_active:
            self.video_active = False
            if self.video_thread and self.video_thread.is_alive():
                self.video_thread.join(timeout=5)

    def data_received(self, data):
        """Handle incoming Bluetooth commands"""
        try:
            # Parse the JSON data
            request = json.loads(data)
            endpoint = request.get('endpoint', '')
            cmd = request.get('cmd', '')
            params = request.get('params', {})

            print(f"Received command: {endpoint}/{cmd} with params: {params}")

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
            elif endpoint == 'video-feed':
                response = self.handle_video_feed(params)
            else:
                response = json.dumps({
                    "status": "error",
                    "message": f"Unknown endpoint: {endpoint}"
                })

            return response
        except json.JSONDecodeError:
            return json.dumps({
                "status": "error",
                "message": "Invalid JSON data"
            })
        except Exception as e:
            import traceback
            traceback.print_exc()
            return json.dumps({
                "status": "error",
                "message": str(e)
            })

    def handle_command(self, cmd, params):
        """Handle robot command endpoints"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        try:
            if cmd == 'scan':
                # Show scanning message
                print("Starting environment scan...")

                # Handle scan command differently as it's a coroutine
                # We need to run this in the event loop
                future = asyncio.run_coroutine_threadsafe(
                    self.manager.commands.scan_env(),
                    self.manager.loop
                )
                try:
                    # Wait for the scan to complete
                    result = future.result(timeout=30)
                    print("Scan completed, response size:", len(str(result)))

                    # Log the structure of the result
                    if isinstance(result, dict):
                        print(f"Result structure: {list(result.keys())}")
                        if 'data' in result:
                            print(f"Data structure: {list(result['data'].keys())}")

                    # Ensure we have grid data
                    if 'data' not in result or 'grid_data' not in result['data']:
                        print("Warning: Missing grid_data in scan result")

                    # Ensure we have visualization image
                    if 'data' not in result or 'plot_image' not in result['data']:
                        print("Warning: Missing plot_image in scan result")

                    # Return the result with NumPy handling
                    result_json = json.dumps(result, default=numpy_json_encoder)

                    # If result is large, chunk it (e.g., 4KB chunks)
                    if len(result_json) > 4096:
                        # Send a header with total size and command ID
                        header = json.dumps({
                            "type": "chunked_start",
                            "command_id": params.get("command_id"),
                            "total_size": len(result_json),
                            "chunks": (len(result_json) + 4095) // 4096
                        })
                        self.server.send(header)

                        # Send chunks with sequential IDs
                        for i in range(0, len(result_json), 4096):
                            chunk = json.dumps({
                                "type": "chunk",
                                "chunk_id": i // 4096,
                                "command_id": params.get("command_id"),
                                "data": result_json[i:i + 4096]
                            })
                            self.server.send(chunk)
                            time.sleep(0.1)  # Give time for transmission

                        # Send completion marker
                        footer = json.dumps({
                            "type": "chunked_end",
                            "command_id": params.get("command_id")
                        })
                        self.server.send(footer)
                        return None  # Don't return anything, we sent the data in chunks

                    return result_json
                except asyncio.TimeoutError:
                    print("Scan operation timed out")
                    return json.dumps({
                        "status": "error",
                        "message": "Scan operation timed out"
                    })
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    print(f"Scan error: {e}")
                    return json.dumps({
                        "status": "error",
                        "message": f"Scan error: {str(e)}"
                    })

            elif cmd == "forward":
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
                        self.manager.loop.call_soon_threadsafe(
                            self.manager.commands.state.movement_task.cancel
                        )
                        self.manager.commands.state.movement_task = None
                except Exception as e:
                    print(f"Stop movement error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Stopped movement"
                })
            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown command: {cmd}. Available commands: forward, backward, left, right, stop, scan, see, blind"
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
            "emergency_stop": bool(self.manager.commands.state.emergency_stop_flag)
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

    def stream_video(self):
        """Stream video frames over Bluetooth"""
        try:
            frame_count = 0
            last_frame_time = time.time()

            while self.video_active and self.manager.commands:
                current_time = time.time()

                # Get frame if enough time has passed (limit to about 5 fps for Bluetooth)
                if current_time - last_frame_time >= 0.2:  # 200ms between frames = 5fps
                    frame = self.manager.commands.vision.get_latest_frame_jpeg()
                    if frame is not None:
                        # Compress frame more aggressively for Bluetooth transmission
                        try:
                            import cv2
                            import numpy as np
                            # Decode JPEG to image
                            nparr = np.frombuffer(frame, np.uint8)
                            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                            # Resize to smaller dimensions for Bluetooth
                            img_small = cv2.resize(img, (320, 240))

                            # Encode with higher compression
                            _, compressed_frame = cv2.imencode('.jpg', img_small, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            frame = compressed_frame.tobytes()
                        except Exception as e:
                            print(f"Frame compression error: {e}")
                            # Continue with original frame if compression fails

                        # Encode frame as base64 and send with a type indicator
                        frame_data = {
                            "type": "video_frame",
                            "frame": base64.b64encode(frame).decode('utf-8')
                        }

                        try:
                            self.server.send(json.dumps(frame_data))
                            frame_count += 1

                            # Print stats occasionally
                            if frame_count % 20 == 0:
                                print(f"Sent {frame_count} frames, latest size: {len(frame)} bytes")
                        except Exception as e:
                            print(f"Frame send error: {e}")

                        last_frame_time = current_time

                # Short sleep to prevent CPU hogging
                time.sleep(0.05)

        except Exception as e:
            print(f"Video streaming error: {e}")
            self.video_active = False

    def handle_video_feed(self, params):
        """Handle one-time request for a video frame"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        if not self.manager.commands.state.vision_task:
            return json.dumps({
                "status": "error",
                "message": "Vision system not active"
            })

        frame = self.manager.commands.vision.get_latest_frame_jpeg()
        if frame is not None:
            return json.dumps({
                "status": "success",
                "frame": base64.b64encode(frame).decode('utf-8')
            })
        else:
            return json.dumps({
                "status": "error",
                "message": "No frame available"
            })

    def cleanup(self):
        """Cleanup resources"""
        print("Cleaning up resources...")
        self.video_active = False
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join(timeout=5)
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