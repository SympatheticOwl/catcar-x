from flask import Flask, jsonify
from typing import Dict, Optional
import asyncio
from commands import Commands  # Assuming commands.py contains the Commands class

app = Flask(__name__)

# Create a single instance of Commands to be used across all requests
command_instance = Commands()
command_instance.__post_init__()  # Initialize the monitoring tasks


@app.route("/command/<cmd>", methods=['POST'])
def execute_command(cmd: str) -> Dict:
    """
    Execute a command on the PicarX.
    Available commands: forward, stop, scan, see
    """
    try:
        if cmd == "forward":
            success = command_instance.forward()
            if not success:
                return jsonify({
                    "status": "error",
                    "message": "Cannot move forward due to hazard"
                }), 400
            return jsonify({
                "status": "success",
                "message": "Moving forward"
            })

        elif cmd == "stop":
            command_instance.cancel_movement()
            return jsonify({
                "status": "success",
                "message": "Stopped movement"
            })

        elif cmd == "scan":
            command_instance.scan_env()
            return jsonify({
                "status": "success",
                "message": "Scanning environment"
            })

        elif cmd == "see":
            command_instance.start_vision()
            # Note: Flask doesn't support async/await directly
            # so we'll return immediately after starting vision
            objects = command_instance.get_objects()
            return jsonify({
                "status": "success",
                "message": "Vision system started",
                "objects": objects
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
    return jsonify({
        "object_distance": command_instance.get_object_distance(),
        "emergency_stop": command_instance.state.emergency_stop_flag
    })


def main():
    """Run the server"""
    try:
        app.run(host="10.0.0.219", port=8000)
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        # Cleanup any running tasks
        if command_instance.state.vision_task:
            command_instance.stop_vision()
        command_instance.cancel_movement()


if __name__ == "__main__":
    main()