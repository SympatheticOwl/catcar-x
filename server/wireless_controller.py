import time
import threading
from commands import Commands
from bt_server import BluetoothServer
from wifi_server import WifiServer
from flask import Flask

if __name__ == "__main__":
    try:
        # Create shared Commands instance
        commands = Commands()

        # Create both servers
        wifi_server = WifiServer(commands)
        bt_server = BluetoothServer(commands)

        # Start Flask server in a separate thread
        flask_thread = threading.Thread(target=lambda: Flask.run(
            app=wifi_server.app,
            host="10.0.0.219",  # Use your desired IP address
            port=8000,
            use_reloader=False  # Important: disable reloader to avoid creating duplicate threads
        ), daemon=True)

        # Start the Flask thread
        flask_thread.start()
        print("Wifi Server Running in background thread...")

        # Now we can proceed to the Bluetooth server which is already running
        print("Bluetooth Server Running...")
        print("Press Ctrl+C to exit.")

        # Keep the main thread alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down servers...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Clean up resources
        if 'bt_server' in locals():
            bt_server.cleanup()
        if 'wifi_server' in locals():
            wifi_server.cleanup()