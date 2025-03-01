import os
import json
import base64
import threading
import asyncio
from bluedot.btcomm import BluetoothServer
from commands import Commands  # Assuming commands.py contains the Commands class

# Set environment variables (same as in the Flask app)
os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # Tell Qt to not use GUI
os.environ['OPENCV_VIDEOIO_PRIORITY_BACKEND'] = 'v4l2'  # Use V4L2 backend for OpenCV
os.environ['MPLBACKEND'] = 'Agg'  # Force matplotlib to use Agg backend

class AsyncCommandManager:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.commands = None
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()

    def _run_event_loop(self):
        """Runs the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.commands = Commands()

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


class PicarXBluetoothServer:
    def __init__(self):
        print("Initializing Bluetooth server for PicarX...")
        self.manager = AsyncCommandManager()
        self.server = BluetoothServer(self.data_received,
                                      when_client_connects=self.client_connected,
                                      when_client_disconnects=self.client_disconnected,
                                      port=2)
        self.video_active = False
        self.video_thread = None
        print(f"Bluetooth server started. Device name: {self.server.server_address}")
        print(f"The server is discoverable as '{self.server.server_address}'")

    def client_connected(self, client_address):
        print(f"Client connected: {client_address}")

    def client_disconnected(self, client_address):
        print(f"Client disconnected: {client_address}")
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
            if endpoint == 'command':
                return self.handle_command(cmd, params)
            elif endpoint == 'status':
                return self.handle_status()
            elif endpoint == 'world-state':
                return self.handle_world_state()
            elif endpoint == 'visualization':
                return self.handle_visualization()
            elif endpoint == 'video-feed':
                return self.handle_video_feed(params)
            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown endpoint: {endpoint}"
                })
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
                # Handle scan command differently as it's a coroutine
                result = self.manager.run_coroutine(self.manager.commands.scan_env())
                return json.dumps(result)

            elif cmd == "forward":
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
                # Get angle from params, default to ±30
                angle = int(params.get('angle', 30 if cmd == "right" else -30))
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

            elif cmd == "see":
                # Create vision task in the event loop
                self.manager.loop.call_soon_threadsafe(
                    lambda: setattr(
                        self.manager.commands.state,
                        'vision_task',
                        self.manager.loop.create_task(self.manager.commands.vision.capture_and_detect())
                    )
                )
                objects = self.manager.commands.get_objects()

                # Start video streaming in a separate thread
                self.video_active = True
                self.video_thread = threading.Thread(target=self.stream_video)
                self.video_thread.daemon = True
                self.video_thread.start()

                return json.dumps({
                    "status": "success",
                    "message": "Vision system started",
                    "objects": objects,
                })

            elif cmd == "blind":
                # Stop vision task
                self.manager.commands.stop_vision()
                self.video_active = False
                return json.dumps({
                    "status": "success",
                    "message": "Vision system stopped"
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
            "object_distance": self.manager.commands.get_object_distance(),
            "emergency_stop": self.manager.commands.state.emergency_stop_flag
        })

    def handle_world_state(self):
        """Handle world-state endpoint"""
        if not self.manager.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        return json.dumps({
            "state": self.manager.commands.world_state()
        })

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
            })
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
            while self.video_active and self.manager.commands:
                frame = self.manager.commands.vision.get_latest_frame_jpeg()
                if frame is not None:
                    # Encode frame as base64 and send with a type indicator
                    frame_data = {
                        "type": "video_frame",
                        "frame": base64.b64encode(frame).decode('utf-8')
                    }
                    self.server.send(json.dumps(frame_data))

                # Add a delay to control frame rate (adjust as needed)
                import time
                time.sleep(0.1)  # Lower frame rate for Bluetooth bandwidth
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


if __name__ == "__main__":
    try:
        server = PicarXBluetoothServer()
        print("Server running. Press Ctrl+C to exit.")

        # Keep the main thread alive
        while True:
            import time

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'server' in locals():
            server.cleanup()