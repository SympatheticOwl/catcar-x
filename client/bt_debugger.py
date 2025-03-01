#!/usr/bin/env python3
"""
Bluetooth Debug Tool
------------------
A simple tool to test Bluetooth communication with the Raspberry Pi.
"""

import sys
import json
import logging
import time
from bluedot.btcomm import BluetoothClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("BT-Debug")


class BTDebugClient:
    def __init__(self, mac_address):
        self.mac_address = mac_address
        self.client = None

    def connect(self):
        """Connect to the Raspberry Pi."""
        try:
            logger.info(f"Connecting to {self.mac_address}...")
            self.client = BluetoothClient(
                self.mac_address,
                data_received_callback=self.data_received
            )
            logger.info("Connected!")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {str(e)}")
            return False

    def disconnect(self):
        """Disconnect from the Raspberry Pi."""
        if self.client:
            logger.info("Disconnecting...")
            self.client.disconnect()
            logger.info("Disconnected")

    def data_received(self, data):
        """Handle received data."""
        logger.info(f"Received data: {repr(data)}")

        # Try to parse as JSON
        try:
            if isinstance(data, str) and data.strip() and (
                    data.strip().startswith('{') or data.strip().startswith('[')):
                json_data = json.loads(data)
                logger.info(f"Parsed JSON: {json.dumps(json_data, indent=2)}")
        except Exception as e:
            logger.warning(f"Not valid JSON: {str(e)}")

    def send_command(self, command, params=None):
        """Send a command to the Pi."""
        if not self.client:
            logger.error("Not connected")
            return

        cmd = {
            "command": command,
            "params": params or {}
        }

        json_str = json.dumps(cmd)
        logger.info(f"Sending: {json_str}")

        try:
            self.client.send(json_str)
            logger.info("Command sent")
        except Exception as e:
            logger.error(f"Send failed: {str(e)}")

    def run_tests(self):
        """Run a series of communication tests."""
        if not self.client:
            logger.error("Not connected")
            return

        # Test 1: Send ping
        logger.info("Test 1: Sending ping...")
        self.send_command("ping")
        time.sleep(2)

        # Test 2: Send forward command
        logger.info("Test 2: Sending forward command...")
        self.send_command("forward")
        time.sleep(2)

        # Test 3: Send stop command
        logger.info("Test 3: Sending stop command...")
        self.send_command("stop")
        time.sleep(2)

        # Test 4: Send scan command
        logger.info("Test 4: Sending scan command...")
        self.send_command("scan")
        time.sleep(10)  # Wait longer for scan to complete

        logger.info("All tests completed")


def main():
    if len(sys.argv) < 2:
        print("Usage: python bt_debug.py <MAC_ADDRESS> [command] [param=value ...]")
        sys.exit(1)

    mac_address = sys.argv[1]

    client = BTDebugClient(mac_address)
    if not client.connect():
        sys.exit(1)

    try:
        if len(sys.argv) == 2:
            # No command specified, run tests
            client.run_tests()
        else:
            # Command specified
            command = sys.argv[2]

            # Parse parameters if provided
            params = {}
            for arg in sys.argv[3:]:
                if '=' in arg:
                    key, value = arg.split('=', 1)
                    # Try to convert numeric values
                    if value.isdigit():
                        params[key] = int(value)
                    elif value.replace('.', '', 1).isdigit():
                        params[key] = float(value)
                    else:
                        params[key] = value

            # Send the command
            client.send_command(command, params)

            # Wait for response
            logger.info("Waiting for response...")
            time.sleep(5)

    finally:
        client.disconnect()


if __name__ == "__main__":
    main()