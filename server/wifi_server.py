import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # Tell Qt to not use GUI
os.environ['OPENCV_VIDEOIO_PRIORITY_BACKEND'] = 'v4l2'  # Use V4L2 backend for OpenCV
os.environ['MPLBACKEND'] = 'Agg'  # Force matplotlib to use Agg backend

import time
from flask import Flask, jsonify, Response, request
from typing import Dict, Optional
import asyncio
import threading
from commands import Commands  # Assuming commands.py contains the Commands class

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
        return future.result()

    pass

app = Flask(__name__)

class WifiServer:
    def __init__(self, commands: Commands):
        self.manager = AsyncCommandManager(commands)
        # app.run(host="192.168.0.163", port=8000)
        app.run(host="10.0.0.219", port=8000)


    @app.after_request
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
            if not self.manager.commands:
                yield b''
                continue

            frame = self.manager.commands.vision.get_latest_frame_jpeg()
            if frame is not None:
                yield b'--frame\r\n'
                yield b'Content-Type: image/jpeg\r\n\r\n'
                yield frame
                yield b'\r\n'

            # Add a small delay to control frame rate
            time.sleep(0.033)  # ~30 fps

    @app.route('/video_feed')
    def video_feed(self):
        """Video streaming route"""
        return Response(
            self.generate_frames(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    @app.route("/command/scan", methods=['POST'])
    def scan_command(self):
        """Start a scan and return results immediately"""
        if not self.manager.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        try:
            # Create async task to perform scan
            future = asyncio.run_coroutine_threadsafe(
                self.manager.commands.scan_env(),
                self.manager.loop
            )

            # Get the result with timeout
            result = future.result(timeout=30)

            # Return scan result which includes visualization data
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

    @app.route("/command/<cmd>", methods=['POST'])
    async def execute_command(self, cmd: str) -> Dict:
        """Execute a command on the PicarX"""
        try:
            if not self.manager.commands:
                return jsonify({
                    "status": "error",
                    "message": "Server still initializing"
                }), 503

            if cmd == "forward":
                success = self.manager.commands.forward()
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
                self.manager.commands.backward()
                return jsonify({
                    "status": "success",
                    "message": "Moving backward"
                })

            elif cmd in ["left", "right"]:
                # Get angle from query params, default to ±30
                angle = int(request.args.get('angle', 30 if cmd == "right" else -30))
                success = self.manager.commands.turn(angle)
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
                self.manager.commands.cancel_movement()
                return jsonify({
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
                return jsonify({
                    "status": "success",
                    "message": "Vision system started. Access video stream at /video_feed",
                    "stream_url": "/video_feed",
                    "objects": objects,
                })

            elif cmd == "blind":
                # Create vision task in the event loop
                self.manager.commands.stop_vision()
                return jsonify({
                    "status": "success",
                    "message": "Vision system stopped"
                })

            else:
                return jsonify({
                    "status": "error",
                    "message": f"Unknown command: {cmd}. Available commands: forward, stop, scan, see"
                }), 400

        except Exception as e:
            return jsonify({
                "status": "error",
                "message": str(e)
            }), 500

    @app.route("/status", methods=['GET'])
    def get_status(self) -> Dict:
        """Get the current status of the PicarX"""
        if not self.manager.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        return jsonify({
            "object_distance": self.manager.commands.get_object_distance(),
            "emergency_stop": self.manager.commands.state.emergency_stop_flag
        })

    @app.route("/world-state", methods=['GET'])
    def get_world_state(self) -> Dict:
        """Get the current status of the PicarX"""
        if not self.manager.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        return jsonify({
            "state": self.manager.commands.world_state()
        })

    @app.route("/grid-data", methods=['GET'])
    def get_grid_data(self) -> Dict:
        """Get the visual grid data of the world map"""
        if not self.manager.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        try:
            world_state = self.manager.commands.world_state()
            return jsonify({
                "status": "success",
                "data": world_state['grid_data']
            })
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": str(e)
            }), 500

    @app.route("/visualization", methods=['GET', 'OPTIONS'])
    def get_visualization(self):
        """Get the matplotlib visualization of the world map"""
        if request.method == 'OPTIONS':
            return jsonify({'status': 'ok'})

        if not self.manager.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        try:
            # Get visualization data directly from the world map
            visualization_data = self.manager.commands.object_system.world_map.visualize()

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
        if self.manager.commands:
            if self.manager.commands.state.vision_task:
                self.manager.commands.stop_vision()
            self.manager.commands.cancel_movement()

        # Stop the event loop
        self.manager.loop.call_soon_threadsafe(self.manager.loop.stop)
        self.manager.thread.join()
