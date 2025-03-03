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
        self.pending_commands = {}  # Track commands waiting for responses
        self.command_lock = threading.Lock()  # Lock for thread safety

        # Buffer for accumulating large responses
        self.data_buffer = ""
        self.buffer_lock = threading.Lock()
        self.expecting_large_response = False
        self.last_command_id = None

        # Cache for telemetry data to reduce Bluetooth traffic
        self.telemetry_cache = {}
        self.telemetry_cache_time = 0
        self.telemetry_cache_ttl = 5  # Cache TTL in seconds

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
                                          port=2,
                                          auto_connect=True,
                                          device="hci0",  # Explicitly set Bluetooth device
                                          encoding=None)  # Important: use binary mode for large responses
            self.connected = True
            print("Connected to Raspberry Pi")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            import traceback
            traceback.print_exc()
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

        # Clear any buffered data
        with self.buffer_lock:
            self.data_buffer = ""
            self.expecting_large_response = False
            self.last_command_id = None

    def process_data_chunk(self, data_chunk):
        """Process a chunk of binary data received from the Pi"""
        try:
            # Convert binary data to string if needed
            if isinstance(data_chunk, bytes):
                data_chunk = data_chunk.decode('utf-8', errors='replace')

            print(f"Processing data chunk of length: {len(data_chunk)}")

            # Check if it looks like a JSON object
            if data_chunk.strip().startswith('{'):
                try:
                    response = json.loads(data_chunk)

                    # Handle chunked data protocol
                    if isinstance(response, dict) and 'type' in response:
                        if response['type'] == 'chunked_start':
                            # Initialize a new chunked response
                            cmd_id = response.get('command_id')
                            with self.buffer_lock:
                                self.chunked_buffers = getattr(self, 'chunked_buffers', {})
                                self.chunked_buffers[cmd_id] = {
                                    'data': '',
                                    'total_size': response.get('total_size', 0),
                                    'received_chunks': 0,
                                    'total_chunks': response.get('chunks', 0)
                                }
                            return

                        elif response['type'] == 'chunk':
                            # Add chunk to the buffer
                            cmd_id = response.get('command_id')
                            with self.buffer_lock:
                                if hasattr(self, 'chunked_buffers') and cmd_id in self.chunked_buffers:
                                    self.chunked_buffers[cmd_id]['data'] += response.get('data', '')
                                    self.chunked_buffers[cmd_id]['received_chunks'] += 1
                            return

                        elif response['type'] == 'chunked_end':
                            # Process the complete chunked response
                            cmd_id = response.get('command_id')
                            with self.buffer_lock:
                                if hasattr(self, 'chunked_buffers') and cmd_id in self.chunked_buffers:
                                    complete_data = self.chunked_buffers[cmd_id]['data']
                                    try:
                                        complete_response = json.loads(complete_data)
                                        # Handle the complete response
                                        if cmd_id in self.pending_commands:
                                            self.pending_commands[cmd_id].put(complete_response)
                                        else:
                                            self.response_queue.put(complete_response)
                                    except json.JSONDecodeError:
                                        print(f"Error parsing chunked response JSON")
                                    # Clean up
                                    del self.chunked_buffers[cmd_id]
                            return

                    # For non-chunked responses, handle normally
                    self.handle_response(response)

                except json.JSONDecodeError:
                    # Append to buffer for incomplete JSON
                    with self.buffer_lock:
                        self.data_buffer += data_chunk

                        # Try to find complete JSON objects in the buffer
                        if self.data_buffer.startswith('{'):
                            try:
                                # Look for matching pairs of { and }
                                # This could be improved with a more robust JSON streaming parser
                                open_count = 0
                                close_count = 0
                                for i, char in enumerate(self.data_buffer):
                                    if char == '{':
                                        open_count += 1
                                    elif char == '}':
                                        close_count += 1
                                        if open_count == close_count:
                                            # We found a complete JSON object
                                            try:
                                                complete_json = self.data_buffer[:i + 1]
                                                response = json.loads(complete_json)
                                                self.data_buffer = self.data_buffer[i + 1:]
                                                self.handle_response(response)
                                                break
                                            except json.JSONDecodeError:
                                                # Not a valid JSON, continue
                                                pass
                            except Exception as e:
                                print(f"Error processing buffer: {e}")
            else:
                # Not JSON, append to buffer
                with self.buffer_lock:
                    self.data_buffer += data_chunk

        except Exception as e:
            print(f"Error processing data chunk: {e}")
            import traceback
            traceback.print_exc()

    def data_received(self, data):
        """Handle data received from the Raspberry Pi"""
        print(f"DATA RECEIVED: {len(data)} bytes")
        if isinstance(data, bytes):
            first_bytes = data[:20].hex()
            print(f"First bytes: {first_bytes}")
        # Then call process_data_chunk
        self.process_data_chunk(data)

    def handle_response(self, response):
        """Process a complete response object"""
        try:
            print(f"Handling response of type: {type(response)}")

            # Check if it's a video frame
            if isinstance(response, dict) and response.get('type') == 'video_frame':
                # Handle video frame
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
                # Handle other responses
                try:
                    # Check if this is a response to a pending command
                    with self.command_lock:
                        # If we have command_id in the response, use it to match with pending commands
                        if isinstance(response, dict) and 'command_id' in response:
                            cmd_id = response.get('command_id')
                            if cmd_id in self.pending_commands:
                                print(f"Matched response to command ID: {cmd_id}")
                                # Use a dedicated result queue for this command
                                self.pending_commands[cmd_id].put(response)
                                return

                        # If we're expecting a large response for a specific command
                        if self.expecting_large_response and self.last_command_id:
                            if self.last_command_id in self.pending_commands:
                                print(f"Matched large response to last command ID: {self.last_command_id}")
                                self.pending_commands[self.last_command_id].put(response)
                                self.expecting_large_response = False
                                self.last_command_id = None
                                return

                    # If no match or no command_id, put in general queue
                    self.response_queue.put(response, block=False)
                    print(f"Added response to general queue: {type(response)}")
                except queue.Full:
                    # If queue is full, clear it and add the new response
                    while not self.response_queue.empty():
                        try:
                            self.response_queue.get_nowait()
                        except queue.Empty:
                            break
                    self.response_queue.put(response)
                    print("Cleared queue and added new response")

        except Exception as e:
            print(f"Error handling response: {e}")
            import traceback
            traceback.print_exc()

    def send_command(self, endpoint, cmd, params=None, timeout=None, use_cache=False):
        """Send a command to the Raspberry Pi and wait for response"""
        if not self.connected or not self.client:
            return {"status": "error", "message": "Not connected to Raspberry Pi"}

        if params is None:
            params = {}

        # For telemetry endpoints, check if we can use cached data
        if use_cache and endpoint == 'telemetry':
            current_time = time.time()
            if (cmd in self.telemetry_cache and
                    current_time - self.telemetry_cache_time < self.telemetry_cache_ttl):
                print(f"Using cached telemetry data for {cmd}")
                return self.telemetry_cache[cmd]

        try:
            # Generate a unique command ID
            import uuid
            cmd_id = str(uuid.uuid4())

            # Prepare the command with ID
            command = {
                "endpoint": endpoint,
                "cmd": cmd,
                "params": params,
                "command_id": cmd_id  # Add command ID to match response
            }

            # Create a dedicated queue for this command's response
            cmd_queue = queue.Queue()

            # Register this command as pending
            with self.command_lock:
                self.pending_commands[cmd_id] = cmd_queue

            # Send the command - CONVERT STRING TO BYTES HERE
            command_json = json.dumps(command)
            print(f"Sending command: {command_json}")

            # Encode the string to bytes before sending
            self.client.send(command_json.encode('utf-8'))
            print(f"Command sent with ID: {cmd_id}")

            # For movement commands, don't wait for response
            if endpoint == 'command' and cmd in ['forward', 'backward', 'left', 'right', 'stop']:
                # Remove from pending commands
                with self.command_lock:
                    if cmd_id in self.pending_commands:
                        del self.pending_commands[cmd_id]
                # Return a default response immediately for movement commands
                return {"status": "success", "message": f"Sent {cmd} command"}

            # Wait for response with timeout
            try:
                # Wait for response on this command's queue
                response = cmd_queue.get(timeout=timeout)
                print(f"Received response for command {cmd_id}")

                # Clean up
                with self.command_lock:
                    if cmd_id in self.pending_commands:
                        del self.pending_commands[cmd_id]

                # Cache telemetry responses
                if endpoint == 'telemetry':
                    self.telemetry_cache[cmd] = response
                    self.telemetry_cache_time = time.time()

                return response
            except queue.Empty:
                print(f"Timeout waiting for response to command {cmd_id}")
                # Clean up on timeout
                with self.command_lock:
                    if cmd_id in self.pending_commands:
                        del self.pending_commands[cmd_id]

                return {"status": "error", "message": f"Timeout waiting for response to {cmd}"}

        except Exception as e:
            print(f"Error sending command: {e}")
            import traceback
            traceback.print_exc()
            return {"status": "error", "message": str(e)}


# Create a global bridge instance
bridge = PicarXBridge()


# Rest of the code remains the same...
@app.route('/')
def index():
    """Serve the composite HTML page that includes both controllers"""
    return send_from_directory('static', 'index.html')


@app.route('/bt')
def bt_index():
    try:
        with open(os.path.join(app.static_folder, 'bt_fragment.html'), 'r') as f:
            content = f.read()
            content = content.replace('id="status"', 'id="bt-status" class="status"')
            response = make_response(f'<div id="bt-controller-content" class="bt-controller-content">{content}</div>')
            response.headers['HX-Trigger'] = 'bt-controller-loaded'
            return response
    except Exception as e:
        print(f"Error reading bt fragment: {e}")
        return "<div>bt fragment not found</div>"


@app.route('/wifi')
def wifi_index():
    try:
        with open(os.path.join(app.static_folder, 'wifi_fragment.html'), 'r') as f:
            content = f.read()
            content = content.replace('id="status"', 'id="wifi-status" class="status"')
            response = make_response(
                f'<div id="wifi-controller-content" class="wifi-controller-content">{content}</div>')
            response.headers['HX-Trigger'] = 'wifi-controller-loaded'
            return response
    except Exception as e:
        print(f"Error reading wifi fragment: {e}")
        return "<div>wifi fragment not found</div>"

@app.route('/telemetry-fragment')
def telemetry_fragment():
    """Serve the telemetry fragment HTML"""
    try:
        with open(os.path.join(app.static_folder, 'telemetry_fragment.html'), 'r') as f:
            content = f.read()
            response = make_response(content)
            return response
    except Exception as e:
        print(f"Error reading telemetry fragment: {e}")
        return "<div>Telemetry fragment not found</div>"


# Add this route to serve our initialization script
@app.route('/scripts/initialize-controllers.js')
def initialize_controllers():
    """Serve the script to initialize the controllers after HTMX load"""
    return send_from_directory('static/scripts', 'initialize-controllers.js')

@app.route('/scripts/telemetry.js')
def serve_telemetry_js():
    """Serve the script to initialize the controllers after HTMX load"""
    return send_from_directory('static/scripts', 'telemetry.js')


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

    # Get parameters from query string or JSON body
    params = {}
    if request.is_json:
        params = request.json
    else:
        # Get angle from query params for turn commands
        if cmd in ['left', 'right']:
            angle = request.args.get('angle')
            if angle:
                params['angle'] = angle
        # Get speed for speed command
        elif cmd == 'set_speed':
            speed = request.args.get('speed')
            if speed:
                params['speed'] = float(speed)

    # Send the command to the Pi with default timeout
    response = bridge.send_command('command', cmd, params)

    print(f"Command {cmd} response: {response}")
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


@app.route('/telemetry', methods=['GET'])
def get_telemetry():
    """Get all telemetry data from the PicarX"""
    if not bridge.connected:
        return jsonify({"status": "error", "message": "Not connected to Raspberry Pi"}), 400

    # Use cached data if available and not expired
    response = bridge.send_command('telemetry', 'all', use_cache=True, timeout=5)
    print(f"Telemetry response: {response}")
    return jsonify(response)


@app.route('/telemetry/battery', methods=['GET'])
def get_battery():
    """Get battery information from the PicarX"""
    if not bridge.connected:
        return jsonify({"status": "error", "message": "Not connected to Raspberry Pi"}), 400

    # Use cached data if available and not expired
    response = bridge.send_command('telemetry', 'battery', use_cache=True, timeout=3)
    return jsonify(response)


@app.route('/telemetry/temperature', methods=['GET'])
def get_temperature():
    """Get CPU temperature from the PicarX"""
    if not bridge.connected:
        return jsonify({"status": "error", "message": "Not connected to Raspberry Pi"}), 400

    # Use cached data if available and not expired
    response = bridge.send_command('telemetry', 'temperature', use_cache=True, timeout=2)
    return jsonify(response)


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