import time
from flask import Flask, jsonify, Response, request
from typing import Dict, Optional
import asyncio
import threading
from commands import Commands  # Assuming commands.py contains the Commands class

app = Flask(__name__)


class AsyncCommandManager:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.command_instance = None
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()

    def _run_event_loop(self):
        """Runs the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.command_instance = Commands()

        # Initialize monitoring tasks in the event loop
        self.loop.run_until_complete(self._initialize_commands())
        self.loop.run_forever()

    async def _initialize_commands(self):
        """Initialize all monitoring tasks"""
        self.command_instance.state.ultrasonic_task = self.loop.create_task(
            self.command_instance.object_system.ultrasonic_monitoring())
        self.command_instance.state.cliff_task = self.loop.create_task(
            self.command_instance.object_system.cliff_monitoring())
        self.command_instance.state.pos_track_task = self.loop.create_task(
            self.command_instance.object_system.px.continuous_position_tracking())

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
        if not manager.command_instance:
            yield b''
            continue

        frame = manager.command_instance.vision.get_latest_frame_jpeg()
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
def execute_command(cmd: str) -> Dict:
    """
    Execute a command on the PicarX.
    Available commands: forward, stop, scan, see
    """
    try:
        if not manager.command_instance:
            return jsonify({
                "status": "error",
                "message": "Server still initializing"
            }), 503

        if cmd == "forward":
            success = manager.command_instance.forward()
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
            manager.command_instance.backward()
            return jsonify({
                "status": "success",
                "message": "Moving backwards"
            })

        elif cmd == "turn":
            angle: int = request.args.get('angle')
            manager.command_instance.turn(angle)
            return jsonify({
                "status": "success",
                "message": "turning"
            })

        elif cmd == "stop":
            manager.command_instance.cancel_movement()
            return jsonify({
                "status": "success",
                "message": "Stopped movement"
            })

        elif cmd == "scan":
            # Create scan task in the event loop
            manager.loop.call_soon_threadsafe(
                lambda: setattr(
                    manager.command_instance.state,
                    'scan_task',
                    manager.loop.create_task(manager.command_instance.object_system.scan_environment())
                )
            )
            return jsonify({
                "status": "success",
                "message": "Scanning environment"
            })

        elif cmd == "see":
            # Create vision task in the event loop
            manager.loop.call_soon_threadsafe(
                lambda: setattr(
                    manager.command_instance.state,
                    'vision_task',
                    manager.loop.create_task(manager.command_instance.vision.capture_and_detect())
                )
            )
            objects = manager.command_instance.get_objects()
            return jsonify({
                "status": "success",
                "message": "Vision system started. Access video stream at /video_feed",
                "stream_url": "/video_feed",
                "objects": objects,
            })

        elif cmd == "blind":
            # Create vision task in the event loop
            manager.command_instance.stop_vision()
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
    if not manager.command_instance:
        return jsonify({
            "status": "error",
            "message": "Server still initializing"
        }), 503

    return jsonify({
        "object_distance": manager.command_instance.get_object_distance(),
        "emergency_stop": manager.command_instance.state.emergency_stop_flag
    })


def cleanup():
    """Cleanup function to stop all tasks and the event loop"""
    if manager.command_instance:
        if manager.command_instance.state.vision_task:
            manager.command_instance.stop_vision()
        manager.command_instance.cancel_movement()

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