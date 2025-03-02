import base64
import os
import json
import time
import threading
import queue
from flask import Flask, jsonify, Response, request, send_from_directory, make_response
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
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.frame_queue.put_nowait(response['frame'])
                except Exception as e:
                    print(f"Error adding frame to queue: {e}")
            else:
                # Put other responses in the response queue
                try:
                    self.response_queue.put(response, block=False)
                except queue.Full:
                    # If queue is full, clear it and add the new response
                    while not self.response_queue.empty():
                        try:
                            self.response_queue.get_nowait()
                        except queue.Empty:
                            break
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

            # Clear the queue to prevent stale responses
            while not self.response_queue.empty():
                try:
                    self.response_queue.get_nowait()
                except queue.Empty:
                    break

            # Send the command
            self.client.send(json.dumps(command))

            # For movement commands, don't wait for response
            if endpoint == 'command' and cmd in ['forward', 'backward', 'left', 'right', 'stop']:
                # Return a default response immediately for movement commands
                return {"status": "success", "message": f"Sent {cmd} command"}

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
    """Serve the composite HTML page that includes both controllers"""
    return send_from_directory('static', 'composite_index.html')


@app.route('/bt')
def bt_index():
    """Serve the Bluetooth HTML page"""
    if request.headers.get('HX-Request'):
        # For HTMX requests, return just the content without layout
        with open(os.path.join(app.static_folder, 'bt_index.html'), 'r') as f:
            content = f.read()
            # Extract the body content for HTMX
            import re
            body_match = re.search(r'<body[^>]*>([\s\S]*?)<\/body>', content, re.IGNORECASE)
            if body_match:
                body_content = body_match.group(1)

                # Make modifications to avoid ID conflicts
                # Add a prefix to Bluetooth-specific IDs to avoid conflicts with WiFi controller
                body_content = body_content.replace('id="status"', 'id="bt-status" class="status"')

                # Add a div wrapper to ensure proper context for scripts
                response = make_response(
                    f'<div id="bt-controller-content" class="bt-controller-content">{body_content}</div>')
                # Tell HTMX to trigger JavaScript initialization
                response.headers['HX-Trigger'] = 'bt-controller-loaded'
                return response
    # For regular requests, return the full page
    return send_from_directory('static', 'bt_index.html')


@app.route('/wifi')
def wifi_index():
    """Serve the WiFi HTML page"""
    if request.headers.get('HX-Request'):
        # For HTMX requests, return just the content without layout
        with open(os.path.join(app.static_folder, 'wifi_index.html'), 'r') as f:
            content = f.read()
            # Extract the body content for HTMX
            import re
            body_match = re.search(r'<body[^>]*>([\s\S]*?)<\/body>', content, re.IGNORECASE)
            if body_match:
                body_content = body_match.group(1)

                # Make modifications to avoid ID conflicts
                # Add a prefix to WiFi-specific IDs to avoid conflicts with Bluetooth controller
                body_content = body_content.replace('id="status"', 'id="wifi-status" class="status"')

                # Add a div wrapper to ensure proper context for scripts
                response = make_response(
                    f'<div id="wifi-controller-content" class="wifi-controller-content">{body_content}</div>')
                # Tell HTMX to trigger JavaScript initialization
                response.headers['HX-Trigger'] = 'wifi-controller-loaded'
                return response
    # For regular requests, return the full page
    return send_from_directory('static', 'wifi_index.html')


# Add this route to serve our initialization script
@app.route('/scripts/initialize-controllers.js')
def initialize_controllers():
    """Serve the script to initialize the controllers after HTMX load"""
    return send_from_directory('static/js', 'initialize-controllers.js')

@app.route('/stacks.min.css')
def serve_stacks_css():
    """Serve the Stacks CSS file if locally hosted"""
    return send_from_directory('static/css', 'stacks.min.css')


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

    # Special handling for scan command
    if cmd == 'scan':
        print("Executing scan command...")

    # Send the command to the Pi
    response = bridge.send_command('command', cmd, params)

    # Debug logging for scan command
    if cmd == 'scan':
        print(f"Scan response received, status: {response.get('status')}")
        if response.get('status') == 'success' and 'data' in response:
            data_keys = list(response['data'].keys()) if isinstance(response['data'], dict) else 'Not a dict'
            print(f"Scan data contains keys: {data_keys}")

            # Check if visualization data exists
            if isinstance(response['data'], dict):
                if 'plot_image' in response['data']:
                    img_length = len(response['data']['plot_image']) if response['data']['plot_image'] else 0
                    print(f"plot_image length: {img_length}")
                else:
                    print("plot_image is missing")

                if 'grid_data' in response['data']:
                    if isinstance(response['data']['grid_data'], dict) and 'grid' in response['data']['grid_data']:
                        grid_size = len(response['data']['grid_data']['grid']) if response['data']['grid_data'][
                            'grid'] else 0
                        print(
                            f"grid size: {grid_size}x{len(response['data']['grid_data']['grid'][0]) if grid_size > 0 else 0}")
                    else:
                        print("grid is missing or malformed in grid_data")
                else:
                    print("grid_data is missing")

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
    last_frame_time = time.time()
    no_frame_count = 0

    while True:
        if not bridge.connected:
            yield b'--frame\r\n'
            yield b'Content-Type: text/plain\r\n\r\n'
            yield b'Not connected to Raspberry Pi\r\n'
            time.sleep(1)
            continue

        try:
            # Try to get a frame from the queue with a short timeout
            try:
                frame_base64 = bridge.frame_queue.get(timeout=0.2)
                no_frame_count = 0  # Reset counter when we get a frame

                # Decode base64 frame
                try:
                    frame_bytes = base64.b64decode(frame_base64)

                    # Send the frame
                    yield b'--frame\r\n'
                    yield b'Content-Type: image/jpeg\r\n\r\n'
                    yield frame_bytes
                    yield b'\r\n'

                    # Rate limit frame delivery (max 10 fps to client)
                    current_time = time.time()
                    if current_time - last_frame_time < 0.1:  # 100ms = 10fps
                        time.sleep(0.1 - (current_time - last_frame_time))
                    last_frame_time = time.time()

                except Exception as e:
                    print(f"Frame decoding error: {e}")
                    yield b'--frame\r\n'
                    yield b'Content-Type: text/plain\r\n\r\n'
                    yield f'Frame error: {str(e)[:50]}...\r\n'.encode('utf-8')
                    time.sleep(0.5)

            except queue.Empty:
                # If no frame is available after timeout
                no_frame_count += 1

                if no_frame_count > 5:  # After ~1 second of no frames
                    yield b'--frame\r\n'
                    yield b'Content-Type: text/plain\r\n\r\n'
                    yield b'Waiting for video...\r\n'

                time.sleep(0.2)

        except Exception as e:
            print(f"Error in frame generation: {e}")
            yield b'--frame\r\n'
            yield b'Content-Type: text/plain\r\n\r\n'
            yield f'Stream error: {str(e)[:50]}...\r\n'.encode('utf-8')
            time.sleep(0.5)


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