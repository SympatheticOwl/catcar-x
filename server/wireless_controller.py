import time
from bt_server import BluetoothServer
from wifi_server import WifiServer

if __name__ == "__main__":
    try:
        wifi_server = WifiServer()
        bt_server = BluetoothServer()
        print("Servers running. Press Ctrl+C to exit.")

        # Keep the main thread alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down servers...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'bt_server' in locals():
            bt_server.cleanup()
        if 'wifi_server' in locals():
            wifi_server.cleanup()