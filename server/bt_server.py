import os
import json
import base64
import threading
import asyncio
import numpy as np
import time
from bluedot.btcomm import BluetoothServer as BlueDotServer
from commands import Commands


def numpy_json_encoder(obj):
    """Custom JSON encoder for NumPy objects"""
    if isinstance(obj, (np.integer, np.int32, np.int64)):
        return int(obj)
    elif isinstance(obj, (np.floating, np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.bool_):
        return bool(obj)
    else:
        try:
            return obj.tolist()
        except:
            pass
    return None


class BTServer:
    def __init__(self, commands: Commands):
        print("Initializing Bluetooth server...")
        self.commands = commands
        self.connected_clients = {}
        self.client_lock = threading.Lock()

        self.server = BlueDotServer(data_received_callback=self.data_received,
                                    when_client_connects=self.client_connected,
                                    when_client_disconnects=self.client_disconnected,
                                    port=2,
                                    auto_start=False)

        print("Waiting for CommandManager to initialize...")
        time.sleep(3)

        self.server.start()
        print(f"Bluetooth server started. Device name: {self.server.server_address}")
        print(f"The server is discoverable as '{self.server.server_address}'")

    def client_connected(self, client_address=None):
        if client_address:
            print(f"Client connected: {client_address}")
        else:
            print("Client connected (address unknown)")

    def client_disconnected(self, client_address=None):
        if client_address:
            print(f"Client disconnected: {client_address}")
        else:
            print("Client disconnected (address unknown)")

        # Stop any active movement when client disconnects
        if self.commands:
            self.commands.cancel_movement()

    def data_received(self, data):
        try:
            if isinstance(data, bytes):
                data = data.decode('utf-8')

            # JSON is easy to work with but probably too verbose for a low bandwidth bluetooth connection
            request = json.loads(data)
            endpoint = request.get('endpoint', '')
            cmd = request.get('cmd', '')
            params = request.get('params', {})
            command_id = request.get('command_id', None)

            print(f"Received command: {endpoint}/{cmd} with params: {params}, ID: {command_id}")

            if endpoint == 'command':
                response = self.handle_command(cmd, params)
            elif endpoint == 'status':
                response = self.handle_status()
            elif endpoint == 'telemetry':
                response = self.handle_telemetry(cmd, command_id)
            elif endpoint == 'ping':
                response = json.dumps({
                    "status": "success",
                    "message": "pong",
                    "command_id": command_id
                })
            else:
                response = json.dumps({
                    "status": "error",
                    "message": f"Unknown endpoint: {endpoint}",
                    "command_id": command_id  # Include command_id in error response
                })

            # Ensure command_id is included in response if it's a dict
            if isinstance(response, str):
                try:
                    response_dict = json.loads(response)
                    if isinstance(response_dict, dict) and 'command_id' not in response_dict and command_id:
                        response_dict['command_id'] = command_id
                        response = json.dumps(response_dict)
                except json.JSONDecodeError:
                    # Not JSON, leave as is
                    pass

            sent = self.send_response(response)
            if not sent:
                print("Warning: Failed to send response")

            return None  # Don't return response directly as we're manually sending it
        except json.JSONDecodeError:
            error_response = json.dumps({
                "status": "error",
                "message": "Invalid JSON data"
            })
            self.send_response(error_response)
            return None
        except Exception as e:
            import traceback
            traceback.print_exc()
            error_response = json.dumps({
                "status": "error",
                "message": str(e)
            })
            self.send_response(error_response)
            return None

    def send_response(self, response):
        try:
            print(f"Sending response: {response}...")

            self.server.send(response)

            return True
        except Exception as e:
            print(f"Error sending response: {e}")
            import traceback
            traceback.print_exc()
            return False

    def send_chunked_response(self, data, command_id=None, chunk_size=250000):
        if not command_id:
            command_id = "server_" + str(int(time.time()))

        # Calculate number of chunks
        total_size = len(data)
        chunks = (total_size + chunk_size - 1) // chunk_size

        # Send start marker
        start_marker = json.dumps({
            "type": "chunked_start",
            "command_id": command_id,
            "total_size": total_size,
            "chunks": chunks
        })
        print(f"Sending chunked response start: {start_marker}")
        self.send_response(start_marker)

        time.sleep(0.1)

        for i in range(chunks):
            start_pos = i * chunk_size
            end_pos = min(start_pos + chunk_size, total_size)
            chunk_data = data[start_pos:end_pos]

            chunk = json.dumps({
                "type": "chunk",
                "command_id": command_id,
                "chunk_index": i,
                "data": chunk_data
            })
            print(f"Sending chunk {i + 1}/{chunks}, size: {len(chunk_data)}")
            self.send_response(chunk)

            # small delay between chunks
            time.sleep(0.05)

        # send end marker
        end_marker = json.dumps({
            "type": "chunked_end",
            "command_id": command_id
        })
        print(f"Sending chunked response end: {end_marker}")
        self.send_response(end_marker)

        return True

    def handle_command(self, cmd, params):
        if not self.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        try:
            if cmd == "forward":
                if self.commands.state.emergency_stop_flag:
                    return json.dumps({
                        "status": "error",
                        "message": "Cannot move forward due to hazard"
                    })

                try:
                    self.commands.forward()
                except Exception as e:
                    print(f"Forward movement error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Moving forward"
                })

            elif cmd == "backward":
                self.commands.state.emergency_stop_flag = False

                try:
                    self.commands.backward()
                except Exception as e:
                    print(f"Backward movement error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Moving backward"
                })

            elif cmd in ["left", "right"]:
                angle = int(30 if cmd == "right" else -30)
                print(f'setting andle: ${angle}')

                success = self.commands.turn(angle)
                if not success:
                    return json.dumps({
                        "status": "error",
                        "message": "Cannot turn due to hazard"
                    })
                return json.dumps({
                    "status": "success",
                    "message": f"Turning {cmd} with angle {angle}"
                })

            elif cmd == "stop":
                try:
                    self.commands.cancel_movement()

                    if self.commands.state.movement_task:
                        self.commands.state.movement_task.cancel()
                        self.commands.state.movement_task = None
                except Exception as e:
                    print(f"Stop movement error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Stopped movement"
                })

            elif cmd == "reset":
                try:
                    self.commands.reset()
                except Exception as e:
                    print(f"Reset env error: {e}")

                return json.dumps({
                    "status": "success",
                    "message": "Environment reset"
                })

            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown command: {cmd}. Available commands: forward, backward, left, right, stop, set_speed"
                })

        except Exception as e:
            import traceback
            traceback.print_exc()
            return json.dumps({
                "status": "error",
                "message": str(e)
            })

    def handle_status(self):
        """Handle status endpoint"""
        if not self.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        return json.dumps({
            "object_distance": float(self.commands.get_object_distance()),
            "emergency_stop": bool(self.commands.state.emergency_stop_flag),
            "is_moving": bool(self.commands.state.is_moving),
            "speed": float(self.commands.state.speed)
        }, default=numpy_json_encoder)

    def handle_telemetry(self, cmd, command_id):
        """Handle telemetry endpoint requests"""
        if not self.commands:
            return json.dumps({
                "status": "error",
                "message": "Server still initializing"
            })

        try:
            if cmd == "all":
                telemetry_data = self.commands.get_telemetry()

                response = {
                    "status": "success",
                    "telemetry": telemetry_data,
                    "command_id": command_id
                }

                # Convert to JSON string
                json_response = json.dumps(response, default=numpy_json_encoder)

                # should be able to handle up to 2 megabits for BT 5.0
                if len(json_response) > 250000:
                    print(f"Telemetry response is large ({len(json_response)} bytes), sending chunked")
                    self.send_chunked_response(json_response, command_id)
                    # Return a placeholder to indicate chunked response is being sent
                    return json.dumps({
                        "status": "processing",
                        "message": "Sending telemetry data in chunks",
                        "command_id": command_id
                    })

                # If small enough, send directly
                return json_response

            elif cmd == "battery":
                battery_data = self.commands.get_battery_level()
                return json.dumps({
                    "status": "success",
                    "battery": battery_data,
                    "command_id": command_id
                }, default=numpy_json_encoder)

            elif cmd == "temperature":
                temp = self.commands.get_cpu_temperature()
                return json.dumps({
                    "status": "success",
                    "temperature": temp,
                    "command_id": command_id
                }, default=numpy_json_encoder)

            else:
                return json.dumps({
                    "status": "error",
                    "message": f"Unknown telemetry command: {cmd}. Available commands: all, battery, temperature",
                    "command_id": command_id
                })

        except Exception as e:
            import traceback
            traceback.print_exc()
            return json.dumps({
                "status": "error",
                "message": str(e),
                "command_id": command_id
            })

    def cleanup(self):
        print("Cleaning up resources...")
        if self.commands:
            if self.commands.state.vision_task:
                self.commands.stop_vision()
            self.commands.cancel_movement()

        self.server.stop()


# for running directly
if __name__ == "__main__":
    try:
        commands = Commands()

        server = BTServer(commands)
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