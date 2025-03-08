import os

os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # Tell Qt to not use GUI
os.environ['OPENCV_VIDEOIO_PRIORITY_BACKEND'] = 'v4l2'  # Use V4L2 backend for OpenCV

import time
from flask import Flask, jsonify, Response, request
import asyncio
import threading
from commands import Commands

class AsyncCommandManager:
    def __init__(self, commands: Commands):
        self.loop = asyncio.new_event_loop()
        self.commands = commands
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()

    def _run_event_loop(self):
        """Runs the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)

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
        return future.result()

app = Flask(__name__)

class WifiServer:
    def __init__(self, commands: Commands):
        self.commands = commands
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()

        self.app = app
        self.register_routes()

    def _run_event_loop(self):
        """Runs the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def register_routes(self):
        """Register all Flask routes"""
        # Attach class methods to app routes
        self.app.after_request(self.after_request)
        self.app.route('/video_feed')(self.video_feed)
        self.app.route("/command/scan", methods=['POST'])(self.scan_command)
        self.app.route("/command/<cmd>", methods=['POST'])(self.execute_command)
        self.app.route("/status", methods=['GET'])(self.get_status)
        self.app.route("/world-state", methods=['GET'])(self.get_world_state)
        self.app.route("/visualization", methods=['GET', 'OPTIONS'])(self.get_visualization)

    def after_request(self, response):
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
        response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
        response.headers.add('Access-Control-Allow-Credentials', 'true')
        response.headers.add('Access-Control-Expose-Headers', 'Content-Type,Content-Length')
        return response

    def generate_frames(self):
        """Generator function for video streaming"""
        while True:
            if not self.commands:
                yield b''
                continue

            frame = self.commands.vision.get_latest_frame_jpeg()
            if frame is not None:
                yield b'--frame\r\n'
                yield b'Content-Type: image/jpeg\r\n\r\n'
                yield frame
                yield b'\r\n'

            # Add a small delay to control frame rate
            time.sleep(0.033)  # ~30 fps

    def video_feed(self):
        """Video streaming route"""
        return Response(
            self.generate_frames(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    def scan_command(self):
        """Start a scan and return results immediately"""
        if not self.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        try:
            future = asyncio.run_coroutine_threadsafe(
                self.commands.scan_env(),
                self.loop
            )

            result = future.result(timeout=30)

            return jsonify(result)
        except asyncio.TimeoutError:
            return jsonify({
                "status": "error",
                "message": "Scan operation timed out"
            }), 504
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": str(e)
            }), 500

    def execute_command(self, cmd: str):
        """Execute a command on the PicarX"""
        try:
            if not self.commands:
                return jsonify({
                    "status": "error",
                    "message": "Server still initializing"
                }), 503

            if cmd == "forward":
                success = self.commands.forward()
                if not success:
                    return jsonify({
                        "status": "error",
                        "message": "Cannot move forward due to hazard"
                    }), 400
                return jsonify({
                    "status": "success",
                    "message": "Moving forward"
                })

            elif cmd == "backward":
                self.commands.backward()
                return jsonify({
                    "status": "success",
                    "message": "Moving backward"
                })

            elif cmd in ["left", "right"]:
                angle = int(30 if cmd == "right" else -30)
                success = self.commands.turn(angle)
                if not success:
                    return jsonify({
                        "status": "error",
                        "message": "Cannot turn due to hazard"
                    }), 400
                return jsonify({
                    "status": "success",
                    "message": f"Turning {cmd} with angle {angle}"
                })

            elif cmd == "stop":
                self.commands.cancel_movement()
                return jsonify({
                    "status": "success",
                    "message": "Stopped movement"
                })

            elif cmd == "see":
                # Create vision task in the event loop
                self.loop.call_soon_threadsafe(
                    lambda: setattr(
                        self.commands.state,
                        'vision_task',
                        self.loop.create_task(self.commands.vision.capture_and_detect())
                    )
                )
                objects = self.commands.get_objects()
                return jsonify({
                    "status": "success",
                    "message": "Vision system started. Access video stream at /video_feed",
                    "stream_url": "/video_feed",
                    "objects": objects,
                })

            elif cmd == "blind":
                self.commands.stop_vision()
                return jsonify({
                    "status": "success",
                    "message": "Vision system stopped"
                })


            elif cmd == "reset":
                try:
                    self.commands.reset()
                except Exception as e:
                    print(f"Reset env error: {e}")

                return jsonify({
                    "status": "success",
                    "message": "Environment reset"
                })

            else:
                return jsonify({
                    "status": "error",
                    "message": f"Unknown command: {cmd}"
                }), 400

        except Exception as e:
            return jsonify({
                "status": "error",
                "message": str(e)
            }), 500

    def get_status(self):
        """Get the current status of the PicarX"""
        if not self.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        return jsonify({
            "object_distance": self.commands.get_object_distance(),
            "emergency_stop": self.commands.state.emergency_stop_flag
        })

    def get_world_state(self):
        """Get the current status of the PicarX"""
        if not self.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        return jsonify({
            "state": self.commands.world_state()
        })

    def get_visualization(self):
        """Get the matplotlib visualization of the world map"""
        if request.method == 'OPTIONS':
            return jsonify({'status': 'ok'})

        if not self.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        try:
            # Get visualization data directly from the world map
            visualization_data = self.commands.object_system.world_map.visualize()

            if visualization_data.get('visualization') is None:
                return jsonify({
                    "status": "error",
                    "message": "Failed to generate visualization"
                }), 500

            return jsonify({
                "status": "success",
                "data": {
                    "grid_data": visualization_data['grid_data'],
                    "plot_image": visualization_data['visualization']
                }
            })
        except Exception as e:
            import traceback
            traceback.print_exc()
            return jsonify({
                "status": "error",
                "message": str(e)
            }), 500

    def cleanup(self):
        """Cleanup function to stop all tasks and the event loop"""
        if self.commands:
            if self.commands.state.vision_task:
                self.commands.stop_vision()
            self.commands.cancel_movement()

        # Stop the event loop
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=5)  # Add timeout to prevent hanging


# for running directly
if __name__ == "__main__":
    try:
        commands = Commands()

        server = WifiServer(commands)
        print("Server running. Press Ctrl+C to exit.")

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'server' in locals():
            server.cleanup()