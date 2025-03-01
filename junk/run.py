#!/usr/bin/env python3
"""
Run Bridge Server
----------------
This script sets up the necessary directory structure and launches the Bluetooth Bridge Server.
"""

import os
import sys
import shutil
import argparse
import subprocess
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("Setup")


def ensure_directory_structure():
    """Ensure the necessary directory structure exists."""
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Create static directory if it doesn't exist
    static_dir = os.path.join(script_dir, 'static')
    if not os.path.exists(static_dir):
        os.makedirs(static_dir)
        logger.info(f"Created static directory: {static_dir}")

    # Check if wifi_index.html exists in static directory
    index_path = os.path.join(static_dir, 'wifi_index.html')
    bt_client_path = os.path.join(script_dir, 'bt_index.html')

    if os.path.exists(bt_client_path) and not os.path.exists(index_path):
        # Copy bluetooth_client.html to static/wifi_index.html
        shutil.copy2(bt_client_path, index_path)
        logger.info(f"Copied {bt_client_path} to {index_path}")

    # Check for stacks.min.css
    css_path = os.path.join(static_dir, 'stacks.min.css')
    if not os.path.exists(css_path):
        # Download stacks.min.css if it doesn't exist
        try:
            import requests
            css_url = "https://cdn.jsdelivr.net/npm/@stackoverflow/stacks/dist/css/stacks.min.css"
            response = requests.get(css_url)

            if response.status_code == 200:
                with open(css_path, 'wb') as f:
                    f.write(response.content)
                logger.info(f"Downloaded stacks.min.css to {css_path}")
            else:
                logger.warning(f"Failed to download stacks.min.css: HTTP {response.status_code}")
        except Exception as e:
            logger.warning(f"Error downloading stacks.min.css: {str(e)}")
            logger.warning("You may need to manually download the CSS file")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Run Bluetooth Bridge Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to run the web server on')
    parser.add_argument('--port', type=int, default=8080, help='Port to run the web server on')
    parser.add_argument('--mac', default='D8:3A:DD:6D:E3:54', help='MAC address of the Raspberry Pi Bluetooth adapter')

    args = parser.parse_args()

    # Ensure directory structure
    ensure_directory_structure()

    # Check if bluetooth_bridge.py exists
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bridge_script = os.path.join(script_dir, 'bt_client.py')

    if not os.path.exists(bridge_script):
        logger.error(f"Bridge script not found: {bridge_script}")
        sys.exit(1)

    # Build command
    cmd = [sys.executable, bridge_script, '--host', args.host, '--port', str(args.port)]

    if args.mac:
        cmd.extend(['--mac', args.mac])

    # Run bridge server
    logger.info(f"Starting Bluetooth Bridge Server: {' '.join(cmd)}")

    try:
        # Run the bridge server
        subprocess.run(cmd)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.error(f"Error running bridge server: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()