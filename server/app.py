import time
from flask import Flask, jsonify, Response, request
from typing import Dict, Optional
import asyncio
import threading
from commands import Commands  # Assuming commands.py contains the Commands class

app = Flask(__name__)

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    return response


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
        return future.result()

    pass


# Create a single instance of the AsyncCommandManager
manager = AsyncCommandManager()


def generate_frames():
    """Generator function for video streaming"""
    while True:
        if not manager.commands:
            yield b''
            continue

        frame = manager.commands.vision.get_latest_frame_jpeg()
        if frame is not None:
            yield b'--frame\r\n'
            yield b'Content-Type: image/jpeg\r\n\r\n'
            yield frame
            yield b'\r\n'

        # Add a small delay to control frame rate
        time.sleep(0.033)  # ~30 fps

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route("/command/<cmd>", methods=['POST'])
async def execute_command(cmd: str) -> Dict:
    """Execute a command on the PicarX"""
    try:
        if not manager.commands:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        if cmd == "forward":
            success = manager.commands.forward()
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
            manager.commands.backward()
            return jsonify({
                "status": "success",
                "message": "Moving backward"
            })

        elif cmd in ["left", "right"]:
            # Get angle from query params, default to ±30
            angle = int(request.args.get('angle', 30 if cmd == "right" else -30))
            success = manager.commands.turn(angle)
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
            manager.commands.cancel_movement()
            return jsonify({
                "status": "success",
                "message": "Stopped movement"
            })

        elif cmd == "scan":
            map = await manager.commands.scan_env()
            return jsonify({
                "state": map
            })

            # Create scan task in the event loop
            # manager.loop.call_soon_threadsafe(
            #     lambda: setattr(
            #         manager.commands.state,
            #         'scan_task',
            #         manager.loop.create_task(manager.commands.object_system.scan_environment())
            #     )
            # )
            # return jsonify({
            #     "status": "success",
            #     "message": "Scanning environment"
            # })

        elif cmd == "see":
            # Create vision task in the event loop
            manager.loop.call_soon_threadsafe(
                lambda: setattr(
                    manager.commands.state,
                    'vision_task',
                    manager.loop.create_task(manager.commands.vision.capture_and_detect())
                )
            )
            objects = manager.commands.get_objects()
            return jsonify({
                "status": "success",
                "message": "Vision system started. Access video stream at /video_feed",
                "stream_url": "/video_feed",
                "objects": objects,
            })

        elif cmd == "blind":
            # Create vision task in the event loop
            manager.commands.stop_vision()
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
def get_status() -> Dict:
    """Get the current status of the PicarX"""
    if not manager.commands:
        return jsonify({
            "status": "error",
            "message": "Server still initializing"
        }), 503

    return jsonify({
        "object_distance": manager.commands.get_object_distance(),
        "emergency_stop": manager.commands.state.emergency_stop_flag
    })

@app.route("/world-state", methods=['GET'])
def get_world_state() -> Dict:
    """Get the current status of the PicarX"""
    if not manager.commands:
        return jsonify({
            "status": "error",
            "message": "Server still initializing"
        }), 503

    return jsonify({
        "state": manager.commands.world_state()
    })

@app.route("/grid-data", methods=['GET'])
def get_grid_data() -> Dict:
    """Get the visual grid data of the world map"""
    if not manager.commands:
        return jsonify({
            "status": "error",
            "message": "Server still initializing"
        }), 503

    try:
        world_state = manager.commands.world_state()
        return jsonify({
            "status": "success",
            "data": world_state['grid_data']
        })
    except Exception as e:
        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500


@app.route("/visualization", methods=['GET'])
def get_visualization() -> Dict:
    """Get the matplotlib visualization of the world map"""
    if not manager.commands:
        return jsonify({
            "status": "error",
            "message": "Server still initializing"
        }), 503

    try:
        # Get the visualization from the world map
        visualization_data = manager.commands.object_system.world_map.get_visualization_data()

        return jsonify({
            "status": "success",
            "data": {
                "grid_data": visualization_data['grid_data'],
                "plot_image": visualization_data['visualization']
            }
        })
    except Exception as e:
        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

def cleanup():
    """Cleanup function to stop all tasks and the event loop"""
    if manager.commands:
        if manager.commands.state.vision_task:
            manager.commands.stop_vision()
        manager.commands.cancel_movement()

    # Stop the event loop
    manager.loop.call_soon_threadsafe(manager.loop.stop)
    manager.thread.join()


def main():
    """Run the server"""
    try:
        app.run(host="10.0.0.219", port=8000)
        # app.run(host="192.168.0.163", port=8000)
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        cleanup()


if __name__ == "__main__":
    main()