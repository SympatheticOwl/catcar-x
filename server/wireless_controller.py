import time
import threading

from commands import Commands
from bt_server import BTServer
from wifi_server import WifiServer

if __name__ == "__main__":
    try:
        # Create shared Commands instance
        commands = Commands()

        # Create both servers
        wifi_server = WifiServer(commands)
        bt_server = BTServer(commands)

        # Start Flask server in a separate thread
        # Use the app instance's run method, not the Flask class method
        flask_thread = threading.Thread(
            target=wifi_server.app.run,
            kwargs={
                'host': "10.0.0.219",  # Use your desired IP address
                'port': 8000,
                'use_reloader': False,  # Important: disable reloader to avoid creating duplicate threads
                'debug': False,  # Disable debug mode to prevent auto-restarting
                'threaded': True  # Enable Flask's internal threading
            },
            daemon=True
        )

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