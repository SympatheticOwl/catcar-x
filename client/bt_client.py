#!/usr/bin/env python3
"""
Bluetooth Bridge Server
-----------------------
This script creates a bridge between the web client and the Raspberry Pi car.
It connects to the car via Bluetooth using the BlueDot library and
serves the web client interface on a local HTTP server.
"""

import os
import sys
import json
import asyncio
import threading
import logging
import time
import base64
from typing import Dict, Any, Optional
from queue import Queue, Empty
from bluedot.btcomm import BluetoothClient

from flask import Flask, jsonify, Response, request, send_from_directory
from flask_cors import CORS

# Configure logging
logging.basicConfig(
    level=logging.INFO,
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
            self.client = BluetoothClient(
                self.server_mac,
                data_received_callback=self._data_received,
                port=2
            )
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

            return True
        except Exception as e:
            logger.error(f"Failed to connect to Raspberry Pi: {str(e)}")
            self.connected = False
            bt_connected = False
            return False

    def disconnect(self):
        """Disconnect from the Raspberry Pi."""
        if self.client:
            self.connected = False
            global bt_connected
            bt_connected = False
            self.client.disconnect()
            logger.info("Disconnected from Raspberry Pi")

    def _data_received(self, data):
        """
        Callback for received data from Bluetooth.

        Args:
            data: Data received from the Raspberry Pi
        """
        try:
            # Check if it's a video frame (starts with "FRAME:")
            if data.startswith("FRAME:"):
                global latest_frame
                latest_frame = data[6:]  # Remove the "FRAME:" prefix
            else:
                # It's a JSON response
                response = json.loads(data)
                resp_queue.put(response)
                logger.debug(f"Received response: {response}")
        except Exception as e:
            logger.error(f"Error processing received data: {str(e)}")

    def _send_loop(self):
        """Background thread to send commands from the queue."""
        while self.connected:
            try:
                # Get command from queue with timeout
                cmd = cmd_queue.get(timeout=1)
                logger.debug(f"running command: {cmd}")

                # Send command to Pi
                self.client.send(json.dumps(cmd))
                logger.debug(f"Sent command: {cmd}")

                # Mark task as done
                cmd_queue.task_done()
            except Empty:
                # Queue is empty, continue waiting
                pass
            except Exception as e:
                logger.error(f"Error sending command: {str(e)}")

    def _receive_loop(self):
        """Background thread to process incoming data."""
        # Note: This is not actually used since data is processed in the callback,
        # but keeping it in case we need additional processing
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
            logger.error("Cannot send command: Not connected to Raspberry Pi")
            return False

        cmd = {
            "command": command,
            "params": params or {}
        }

        print(f'adding command {cmd}')
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
    return send_from_directory('static', 'index.html')


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

    # Get MAC address from request or use default
    mac_address = request.json.get('mac') if request.json else None

    if not mac_address and not bt_client:
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


@app.route('/command/<cmd>', methods=['POST'])
def execute_command(cmd):
    """
    Execute a command on the Raspberry Pi.

    Args:
        cmd: Command to execute
    """
    if not bt_client or not bt_client.connected:
        return jsonify({
            "status": "error",
            "message": "Not connected to Raspberry Pi"
        }), 503

    # Get parameters from request
    params = request.json if request.json else {}

    # Add angle parameter for turn commands
    if cmd in ['left', 'right'] and request.args.get('angle'):
        params['angle'] = int(request.args.get('angle'))

    # Send command to Pi
    bt_client.send_command(cmd, params)

    # For 'scan' command, wait for response
    if cmd == 'scan':
        response = bt_client.get_response(timeout=10.0)
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

    args = parser.parse_args()

    # Connect to Pi if MAC address provided
    if args.mac:
        global bt_client
        bt_client = BTCarClient(args.mac)
        bt_client.connect()

    try:
        logger.info(f"Starting web server on {args.host}:{args.port}")
        app.run(host=args.host, port=args.port, threaded=True)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        if bt_client:
            bt_client.disconnect()


if __name__ == '__main__':
    main()