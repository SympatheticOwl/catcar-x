import os
import json
import time
import threading
import queue
from flask import Flask, jsonify, Response, request, send_from_directory
from bluedot.btcomm import BluetoothClient

app = Flask(__name__, static_folder='static')


# CORS headers
@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    response.headers.add('Access-Control-Allow-Credentials', 'true')
    response.headers.add('Access-Control-Expose-Headers', 'Content-Type,Content-Length')
    return response


class PicarXBridge:
    def __init__(self, pi_bluetooth_address=None):
        self.bt_address = pi_bluetooth_address
        self.client = None
        self.connected = False
        self.response_queue = queue.Queue()
        self.frame_queue = queue.Queue(maxsize=10)  # Limit queue size for video frames

        # Start connection thread if address is provided
        if self.bt_address:
            self.connect_to_pi()

    def connect_to_pi(self, bt_address='D8:3A:DD:6D:E3:54'):
        """Connect to the Raspberry Pi via Bluetooth"""
        if bt_address:
            self.bt_address = bt_address

        if not self.bt_address:
            print("No Bluetooth address provided")
            return False

        try:
            print(f"Connecting to Raspberry Pi at {self.bt_address}...")
            self.client = BluetoothClient(self.bt_address,
                                          data_received_callback=self.data_received,
                                          port=2)
            self.connected = True
            print("Connected to Raspberry Pi")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from the Raspberry Pi"""
        if self.client:
            try:
                self.client.disconnect()
            except:
                pass
            self.client = None
        self.connected = False
        print("Disconnected from Raspberry Pi")

    def data_received(self, data):
        """Handle data received from the Raspberry Pi"""
        try:
            # Parse the received JSON data
            response = json.loads(data)

            # Check if it's a video frame
            if isinstance(response, dict) and response.get('type') == 'video_frame':
                # Handle video frame differently - put in frame queue
                try:
                    # If queue is full, remove oldest frame
                    if self.frame_queue.full():
                        self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait(response['frame'])
                except:
                    pass
            else:
                # Put other responses in the response queue
                self.response_queue.put(response)

        except json.JSONDecodeError:
            print(f"Invalid JSON received: {data[:100]}...")
        except Exception as e:
            print(f"Error processing received data: {e}")

    def send_command(self, endpoint, cmd, params=None):
        """Send a command to the Raspberry Pi and wait for response"""
        if not self.connected or not self.client:
            return {"status": "error", "message": "Not connected to Raspberry Pi"}

        if params is None:
            params = {}

        try:
            # Prepare the command
            command = {
                "endpoint": endpoint,
                "cmd": cmd,
                "params": params
            }

            # Send the command
            self.client.send(json.dumps(command))

            # Wait for response with timeout
            try:
                response = self.response_queue.get(timeout=30)
                return response
            except queue.Empty:
                return {"status": "error", "message": "Timeout waiting for response"}

        except Exception as e:
            print(f"Error sending command: {e}")
            return {"status": "error", "message": str(e)}


# Create a global bridge instance
bridge = PicarXBridge()


@app.route('/')
def index():
    """Serve the main HTML page"""
    return send_from_directory('static', 'index_2.html')


@app.route('/connect', methods=['POST'])
def connect():
    """Connect to the Raspberry Pi"""
    data = request.json
    bt_address = data.get('bt_address')

    if not bt_address:
        return jsonify({"status": "error", "message": "No Bluetooth address provided"}), 400

    success = bridge.connect_to_pi(bt_address)

    if success:
        return jsonify({"status": "success", "message": "Connected to Raspberry Pi"})
    else:
        return jsonify({"status": "error", "message": "Failed to connect"}), 500


@app.route('/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from the Raspberry Pi"""
    bridge.disconnect()
    return jsonify({"status": "success", "message": "Disconnected from Raspberry Pi"})


@app.route('/command/<cmd>', methods=['POST'])
def execute_command(cmd):
    """Forward commands to the Raspberry Pi"""
    if not bridge.connected:
        return jsonify({"status": "error", "message": "Not connected to Raspberry Pi"}), 400

    # Get angle from query params for turn commands
    params = {}
    if cmd in ['left', 'right']:
        angle = request.args.get('angle')
        if angle:
            params['angle'] = angle

    # Send the command to the Pi
    response = bridge.send_command('command', cmd, params)
    return jsonify(response)


@app.route('/status', methods=['GET'])
def get_status():
    """Get the current status of the PicarX"""
    if not bridge.connected:
        return jsonify({"status": "error", "message": "Not connected to Raspberry Pi"}), 400

    response = bridge.send_command('status', '')
    return jsonify(response)


@app.route('/world-state', methods=['GET'])
def get_world_state():
    """Get the current world state of the PicarX"""
    if not bridge.connected:
        return jsonify({"status": "error", "message": "Not connected to Raspberry Pi"}), 400

    response = bridge.send_command('world-state', '')
    return jsonify(response)


@app.route('/visualization', methods=['GET'])
def get_visualization():
    """Get the visualization of the world map"""
    if not bridge.connected:
        return jsonify({"status": "error", "message": "Not connected to Raspberry Pi"}), 400

    response = bridge.send_command('visualization', '')
    return jsonify(response)


def generate_frames():
    """Generator function for video streaming"""
    while True:
        if not bridge.connected:
            yield b'--frame\r\n'
            yield b'Content-Type: text/plain\r\n\r\n'
            yield b'Not connected to Raspberry Pi\r\n'
            time.sleep(1)
            continue

        try:
            # Try to get a frame from the queue
            frame_data = bridge.frame_queue.get(timeout=0.5)
            frame = frame_data.encode('utf-8')

            yield b'--frame\r\n'
            yield b'Content-Type: image/jpeg\r\n\r\n'
            yield frame
            yield b'\r\n'
        except queue.Empty:
            # If no frame is available, send a placeholder
            yield b'--frame\r\n'
            yield b'Content-Type: text/plain\r\n\r\n'
            yield b'Waiting for video...\r\n'
            time.sleep(0.1)
        except Exception as e:
            print(f"Error in frame generation: {e}")
            time.sleep(0.1)


@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


# Run the server if executed directly
if __name__ == '__main__':
    try:
        port = 8080
        print(f"Starting client bridge on port {port}")
        app.run(host="0.0.0.0", port=port, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down client bridge...")
    finally:
        bridge.disconnect()