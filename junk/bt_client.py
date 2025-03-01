#!/usr/bin/env python3
"""
Debugged Bluetooth Bridge Server
-------------------------------
Improved Flask route handling to fix 400 Bad Request errors.
"""

import os
import sys
import json
import asyncio
import threading
import logging
import time
import base64
import traceback
from typing import Dict, Any, Optional
from queue import Queue, Empty
from bluedot.btcomm import BluetoothClient

from flask import Flask, jsonify, Response, request, send_from_directory
from flask_cors import CORS

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,  # Set to DEBUG for more detailed logs
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("BT-Bridge")

# Create Flask app
app = Flask(__name__, static_folder='static')
CORS(app)  # Enable Cross-Origin Resource Sharing

# Command queue for sending commands to the Pi
cmd_queue = Queue()
# Response queue for receiving responses from the Pi
resp_queue = Queue()
# Keep track of the latest frame for video streaming
latest_frame = None
# Flag to indicate if the Bluetooth connection is active
bt_connected = False


class BTCarClient:
    """
    Bluetooth client to communicate with the Raspberry Pi car.
    """

    def __init__(self, server_mac: str):
        """
        Initialize Bluetooth client.

        Args:
            server_mac: MAC address of the Raspberry Pi Bluetooth adapter
        """
        self.server_mac = server_mac
        self.client = None
        self.connected = False
        self.receive_thread = None
        self.send_thread = None

    def connect(self):
        """Connect to the Raspberry Pi via Bluetooth."""
        try:
            logger.info(f"Connecting to Raspberry Pi at {self.server_mac}...")

            # Initialize the client with a callback
            self.client = BluetoothClient(
                self.server_mac,
                data_received_callback=self._data_received,
                port=2
            )

            # The client is now connected
            self.connected = True
            global bt_connected
            bt_connected = True
            logger.info("Connected to Raspberry Pi")

            # Start send and receive threads
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()

            self.send_thread = threading.Thread(target=self._send_loop)
            self.send_thread.daemon = True
            self.send_thread.start()

            # Send a test message to verify connection
            self.send_command("ping")

            return True
        except Exception as e:
            logger.error(f"Failed to connect to Raspberry Pi: {str(e)}")
            self.connected = False
            bt_connected = False
            return False

    def disconnect(self):
        """Disconnect from the Raspberry Pi."""
        if self.client:
            logger.info("Disconnecting from Raspberry Pi...")
            self.connected = False
            global bt_connected
            bt_connected = False

            try:
                self.client.disconnect()
            except Exception as e:
                logger.error(f"Error during disconnect: {str(e)}")

            logger.info("Disconnected from Raspberry Pi")

    def _data_received(self, data):
        """
        Callback for received data from Bluetooth.

        Args:
            data: Data received from the Raspberry Pi
        """
        try:
            # Log raw data for debugging
            logger.info(f"Raw data received: {repr(data)}")

            # Skip empty data
            if not data or data.isspace():
                logger.warning("Received empty data, ignoring")
                return

            # Check if it's a video frame (starts with "FRAME:")
            if isinstance(data, str) and data.startswith("FRAME:"):
                global latest_frame
                latest_frame = data[6:]  # Remove the "FRAME:" prefix
                logger.debug("Received video frame")
            else:
                # Try to parse as JSON response
                try:
                    # Make sure we have a string
                    if not isinstance(data, str):
                        data = str(data)

                    # Clean the data - remove any leading/trailing whitespace
                    data = data.strip()

                    # Only try to parse if it looks like JSON
                    if data and (data.startswith('{') or data.startswith('[')):
                        response = json.loads(data)
                        resp_queue.put(response)
                        logger.info(f"Received JSON response: {response}")
                    else:
                        logger.warning(f"Received non-JSON data: {repr(data)}")
                        # Put a simple acknowledgment in the response queue
                        resp_queue.put({"status": "ack", "raw_data": data})
                except json.JSONDecodeError as e:
                    logger.warning(f"JSON decode error: {e}, Data: {repr(data)}")
                    # Put an error response in the queue
                    resp_queue.put({"status": "error", "message": f"Invalid JSON: {str(e)}"})
        except Exception as e:
            logger.error(f"Error processing received data: {str(e)}")

    def _send_loop(self):
        """Background thread to send commands from the queue."""
        while self.connected:
            try:
                # Get command from queue with timeout
                cmd = cmd_queue.get(timeout=1)

                # Serialize the command to JSON if it's not already a string
                if not isinstance(cmd, str):
                    cmd = json.dumps(cmd)

                # Send command to Pi
                logger.info(f"Sending command: {cmd}")
                self.client.send(cmd)

                # Mark task as done
                cmd_queue.task_done()
            except Empty:
                # Queue is empty, continue waiting
                pass
            except Exception as e:
                logger.error(f"Error sending command: {str(e)}")

    def _receive_loop(self):
        """Background thread to process incoming data."""
        # This is mostly handled by the callback, but we keep the thread
        # running to maintain the connection
        while self.connected:
            time.sleep(0.1)

    def send_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> bool:
        """
        Send a command to the Raspberry Pi.

        Args:
            command: Command to send
            params: Optional parameters for the command

        Returns:
            True if command was queued successfully, False otherwise
        """
        if not self.connected:
            logger.error(f"Cannot send command '{command}': Not connected to Raspberry Pi")
            return False

        cmd = {
            "command": command,
            "params": params or {}
        }

        logger.info(f"Queueing command: {cmd}")
        cmd_queue.put(cmd)
        return True

    def get_response(self, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        """
        Get a response from the Raspberry Pi.

        Args:
            timeout: Time to wait for a response in seconds

        Returns:
            Response from the Pi or None if timed out
        """
        try:
            return resp_queue.get(timeout=timeout)
        except Empty:
            logger.warning(f"No response received within {timeout} seconds")
            return None


# Initialize Bluetooth client (will be set in main function)
bt_client = None


@app.route('/')
def index():
    """Serve the main HTML interface."""
    return send_from_directory('../client/static', 'index.html')


@app.route('/status')
def get_status():
    """Get the current connection status."""
    if not bt_client or not bt_client.connected:
        return jsonify({
            "status": "error",
            "message": "Not connected to Raspberry Pi"
        }), 503

    return jsonify({
        "status": "success",
        "connected": bt_client.connected
    })


@app.route('/connect', methods=['POST'])
def connect():
    """Connect to the Raspberry Pi via Bluetooth."""
    global bt_client

    if bt_client and bt_client.connected:
        return jsonify({
            "status": "success",
            "message": "Already connected to Raspberry Pi"
        })

    # Get MAC address from request
    try:
        mac_address = None

        # Try to get MAC from JSON body first
        if request.is_json:
            data = request.get_json()
            mac_address = data.get('mac')
            logger.debug(f"Got MAC from JSON: {mac_address}")

        # If no MAC in JSON, try form data
        if not mac_address and request.form:
            mac_address = request.form.get('mac')
            logger.debug(f"Got MAC from form: {mac_address}")

        # If no MAC in form, check if we have an existing client
        if not mac_address and bt_client:
            mac_address = bt_client.server_mac
            logger.debug(f"Using existing MAC: {mac_address}")

        # If still no MAC, return error
        if not mac_address:
            logger.error("No MAC address provided in request")
            return jsonify({
                "status": "error",
                "message": "No MAC address provided"
            }), 400

        # Create new client if needed
        if not bt_client:
            bt_client = BTCarClient(mac_address)

        # Connect to Pi
        success = bt_client.connect()

        if success:
            return jsonify({
                "status": "success",
                "message": "Connected to Raspberry Pi"
            })
        else:
            return jsonify({
                "status": "error",
                "message": "Failed to connect to Raspberry Pi"
            }), 500

    except Exception as e:
        error_details = traceback.format_exc()
        logger.error(f"Error connecting to Raspberry Pi: {str(e)}\n{error_details}")
        return jsonify({
            "status": "error",
            "message": f"Error: {str(e)}"
        }), 500


@app.route('/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from the Raspberry Pi."""
    global bt_client

    if not bt_client or not bt_client.connected:
        return jsonify({
            "status": "error",
            "message": "Not connected to Raspberry Pi"
        }), 400

    bt_client.disconnect()

    return jsonify({
        "status": "success",
        "message": "Disconnected from Raspberry Pi"
    })


@app.route('/command/<cmd>', methods=['POST', 'GET'])
def execute_command(cmd):
    """
    Execute a command on the Raspberry Pi.

    Args:
        cmd: Command to execute
    """
    global bt_client

    # Log full request details for debugging
    logger.debug(f"Received command request: {cmd}")
    logger.debug(f"Request method: {request.method}")
    logger.debug(f"Request headers: {dict(request.headers)}")
    logger.debug(f"Request args: {dict(request.args)}")

    if request.is_json:
        logger.debug(f"Request JSON: {request.get_json()}")
    elif request.form:
        logger.debug(f"Request form: {dict(request.form)}")

    # Check connection
    if not bt_client or not bt_client.connected:
        return jsonify({
            "status": "error",
            "message": "Not connected to Raspberry Pi"
        }), 503

    # Parse request data
    try:
        # Initialize params dictionary
        params = {}

        # Get JSON data if provided
        if request.is_json:
            try:
                json_data = request.get_json()
                if json_data and isinstance(json_data, dict):
                    params.update(json_data)
            except Exception as e:
                logger.warning(f"Error parsing JSON data: {str(e)}")

        # Add form data if present
        if request.form:
            for key, value in request.form.items():
                try:
                    # Try to convert values to appropriate types
                    if value.isdigit():
                        params[key] = int(value)
                    elif value.replace('.', '', 1).isdigit():
                        params[key] = float(value)
                    else:
                        params[key] = value
                except:
                    params[key] = value

        # Add query parameters (for angle, etc.)
        for key, value in request.args.items():
            try:
                # Try to convert values to appropriate types
                if value.isdigit():
                    params[key] = int(value)
                elif value.replace('.', '', 1).isdigit():
                    params[key] = float(value)
                else:
                    params[key] = value
            except:
                params[key] = value

        logger.info(f"Processed command: {cmd}, Params: {params}")

        # Send command to Pi
        bt_client.send_command(cmd, params)

        # For 'scan' command, wait for response
        if cmd == 'scan':
            response = bt_client.get_response(timeout=15.0)
            if response:
                return jsonify(response)
            else:
                return jsonify({
                    "status": "error",
                    "message": "Scan timed out"
                }), 504

        # For most commands, return immediate success
        return jsonify({
            "status": "success",
            "message": f"Command '{cmd}' sent to Raspberry Pi"
        })

    except Exception as e:
        error_details = traceback.format_exc()
        logger.error(f"Error executing command: {str(e)}\n{error_details}")
        return jsonify({
            "status": "error",
            "message": f"Error: {str(e)}"
        }), 500


def generate_frames():
    """Generator function for video streaming."""
    while True:
        global latest_frame

        if not bt_connected:
            # Not connected, yield empty frame
            yield b'--frame\r\n'
            yield b'Content-Type: image/jpeg\r\n\r\n'
            yield b'\r\n'
        elif latest_frame:
            # Got a frame, decode and yield it
            try:
                frame_data = base64.b64decode(latest_frame)
                yield b'--frame\r\n'
                yield b'Content-Type: image/jpeg\r\n\r\n'
                yield frame_data
                yield b'\r\n'
            except Exception as e:
                logger.error(f"Error processing video frame: {str(e)}")

        # Add a small delay to control frame rate
        time.sleep(0.033)  # ~30 fps


@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/visualization')
def get_visualization():
    """Get the visualization data from the Pi."""
    global bt_client

    if not bt_client or not bt_client.connected:
        return jsonify({
            "status": "error",
            "message": "Not connected to Raspberry Pi"
        }), 503

    # Send visualization command
    bt_client.send_command('visualization')

    # Wait for response
    response = bt_client.get_response(timeout=5.0)

    if response:
        return jsonify(response)
    else:
        return jsonify({
            "status": "error",
            "message": "Visualization request timed out"
        }), 504


def main():
    """Main function."""
    import argparse

    parser = argparse.ArgumentParser(description='Bluetooth Bridge Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to run the web server on')
    parser.add_argument('--port', type=int, default=8080, help='Port to run the web server on')
    parser.add_argument('--mac', default='D8:3A:DD:6D:E3:54', help='MAC address of the Raspberry Pi Bluetooth adapter')
    parser.add_argument('--debug', action='store_true', help='Run Flask in debug mode')

    args = parser.parse_args()

    # Connect to Pi if MAC address provided
    if args.mac:
        global bt_client
        bt_client = BTCarClient(args.mac)
        bt_client.connect()

    try:
        logger.info(f"Starting web server on {args.host}:{args.port}")
        app.run(host=args.host, port=args.port, threaded=True, debug=args.debug)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        if bt_client and bt_client.connected:
            bt_client.disconnect()


if __name__ == '__main__':
    main()