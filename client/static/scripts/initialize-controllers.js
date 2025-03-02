/**
 * This script handles the initialization of controllers after they're loaded via HTMX
 */

// Listen for HTMX events
document.addEventListener('htmx:afterSwap', function(event) {
    console.log('HTMX swap completed for:', event.target.id);

    // Initialize Bluetooth controller if it was just loaded
    if (event.target.id === 'bt-content' || event.detail.target.id === 'bt-content') {
        initializeBluetoothController();
    }

    // Initialize WiFi controller if it was just loaded
    if (event.target.id === 'wifi-content' || event.detail.target.id === 'wifi-content') {
        initializeWifiController();
    }
});

// Handle custom events from server
document.addEventListener('bt-controller-loaded', function() {
    console.log('bt-controller-loaded event triggered');
    initializeBluetoothController();
});

document.addEventListener('wifi-controller-loaded', function() {
    console.log('wifi-controller-loaded event triggered');
    initializeWifiController();
});

// Initialize Bluetooth controller
function initializeBluetoothController() {
    console.log('Initializing Bluetooth controller...');

    const btContainer = document.getElementById('bt-content');
    if (!btContainer) return;

    // Re-initialize all event listeners and components
    setupButtonHandlers(btContainer, true);

    // Re-initialize any additional Bluetooth-specific functionality
    const btStatus = btContainer.querySelector('#status');
    if (btStatus) {
        btStatus.textContent = 'Status: Bluetooth controller ready';
    }

    console.log('Bluetooth controller initialized');
}

// Initialize WiFi controller
function initializeWifiController() {
    console.log('Initializing WiFi controller...');

    const wifiContainer = document.getElementById('wifi-content');
    if (!wifiContainer) return;

    // Re-initialize all event listeners and components
    setupButtonHandlers(wifiContainer, false);

    // Re-initialize any additional WiFi-specific functionality
    const wifiStatus = wifiContainer.querySelector('#status');
    if (wifiStatus) {
        wifiStatus.textContent = 'Status: WiFi controller ready';
    }

    console.log('WiFi controller initialized');
}

// Setup button event handlers for a controller
function setupButtonHandlers(container, isBluetooth) {
    if (!container) return;

    // Get API base URL based on controller type
    // IMPORTANT: For WiFi controller, we must use the hardcoded URL from the original controller
    const apiUrl = isBluetooth ? window.location.origin : 'http://10.0.0.219:8000';
    console.log(`Using API URL for ${isBluetooth ? 'Bluetooth' : 'WiFi'} controller: ${apiUrl}`);

    // Helper function for making API calls with timeout support
    async function sendCommand(cmd, options = {}) {
        try {
            console.log(`Sending ${cmd} command to ${apiUrl}`);

            // Handle different API formats between Bluetooth and WiFi controllers
            let url;
            let fetchOptions = { method: 'POST' };

            // Add AbortController for timeout handling
            const controller = new AbortController();
            const timeoutMs = options.timeout || (cmd === 'scan' ? 90000 : 30000); // Use longer timeout for scan
            const timeoutId = setTimeout(() => controller.abort(), timeoutMs);
            fetchOptions.signal = controller.signal;

            if (isBluetooth) {
                // Bluetooth controller uses query params for angles
                const queryString = options.angle ? `?angle=${options.angle}` : '';
                url = `${apiUrl}/command/${cmd}${queryString}`;
            } else {
                // WiFi controller might not use query params the same way
                url = `${apiUrl}/command/${cmd}`;

                // If we need to pass parameters in body instead
                if (options.angle) {
                    fetchOptions.headers = {
                        'Content-Type': 'application/json'
                    };
                    fetchOptions.body = JSON.stringify({ angle: options.angle });
                }
            }

            const response = await fetch(url, fetchOptions);
            clearTimeout(timeoutId);

            const data = await response.json();

            const statusEl = container.querySelector('#status');
            if (statusEl) {
                statusEl.textContent = `Status: ${data.message || data.status}`;
            }

            return data;
        } catch (error) {
            if (error.name === 'AbortError') {
                console.log(`Request timed out for command: ${cmd}`);
                const statusEl = container.querySelector('#status');
                if (statusEl) {
                    statusEl.textContent = `Error: Request timed out. The server may still be processing the command.`;
                }
            } else {
                console.error(`Error sending command ${cmd} to ${apiUrl}:`, error);
                const statusEl = container.querySelector('#status');
                if (statusEl) {
                    statusEl.textContent = `Error: ${error.message}`;
                }
            }
            throw error;
        }
    }

    // Setup control buttons
    const setupButton = (id, pressCommand, angle = null) => {
        const button = container.querySelector(`#${id}`);
        if (!button) return;

        // Clear existing listeners by cloning and replacing
        const newButton = button.cloneNode(true);
        button.parentNode.replaceChild(newButton, button);

        let isPressed = false;

        // Touch events
        newButton.addEventListener('touchstart', async (e) => {
            e.preventDefault();
            if (!isPressed) {
                isPressed = true;
                newButton.classList.add('active');
                await sendCommand(pressCommand, { angle });
            }
        });

        newButton.addEventListener('touchend', async (e) => {
            e.preventDefault();
            if (isPressed) {
                isPressed = false;
                newButton.classList.remove('active');
                await sendCommand('stop');
            }
        });

        // Mouse events
        newButton.addEventListener('mousedown', async () => {
            if (!isPressed) {
                isPressed = true;
                newButton.classList.add('active');
                await sendCommand(pressCommand, { angle });
            }
        });

        newButton.addEventListener('mouseup', async () => {
            if (isPressed) {
                isPressed = false;
                newButton.classList.remove('active');
                await sendCommand('stop');
            }
        });

        // Prevent stuck movement if mouse leaves button while pressed
        newButton.addEventListener('mouseleave', async () => {
            if (isPressed) {
                isPressed = false;
                newButton.classList.remove('active');
                await sendCommand('stop');
            }
        });
    };

    // Setup directional buttons
    setupButton('up', 'forward');
    setupButton('down', 'backward');
    setupButton('left', 'left', -30);
    setupButton('right', 'right', 30);

    // Setup scan environment button
    const scanEnvBtn = container.querySelector('#scanEnv');
    if (scanEnvBtn) {
        const newScanBtn = scanEnvBtn.cloneNode(true);
        scanEnvBtn.parentNode.replaceChild(newScanBtn, scanEnvBtn);

        // Find or create the scan progress indicator
        let scanProgress = container.querySelector('#scanProgress');
        if (!scanProgress) {
            scanProgress = document.createElement('span');
            scanProgress.id = 'scanProgress';
            scanProgress.style.display = 'none';
            scanProgress.style.marginLeft = '10px';
            scanProgress.innerHTML = '<span class="spinner"></span> Scanning... This may take up to 30 seconds';
            newScanBtn.parentNode.appendChild(scanProgress);

            // Add spinner style if needed
            if (!document.querySelector('style#spinnerStyle')) {
                const style = document.createElement('style');
                style.id = 'spinnerStyle';
                style.textContent = `
                    .spinner {
                        display: inline-block;
                        width: 15px;
                        height: 15px;
                        border: 2px solid rgba(0, 0, 0, 0.1);
                        border-top-color: #4a90e2;
                        border-radius: 50%;
                        animation: spin 1s linear infinite;
                    }
                    
                    @keyframes spin {
                        to { transform: rotate(360deg); }
                    }
                `;
                document.head.appendChild(style);
            }
        }

        newScanBtn.addEventListener('click', async function() {
            try {
                this.disabled = true;
                scanProgress.style.display = 'inline-block';

                const statusEl = container.querySelector('#status');
                if (statusEl) {
                    statusEl.textContent = 'Status: Scanning environment... (this may take up to 30 seconds)';
                }

                // Clear previous visualization
                const mapImg = container.querySelector('#mapVisualization');
                if (mapImg) mapImg.style.display = 'none';

                const asciiMap = container.querySelector('#asciiMap');
                if (asciiMap) asciiMap.style.display = 'none';

                // Get the scan data with long timeout
                const response = await sendCommand('scan', { timeout: 90000 }); // 90 second timeout

                // Hide progress indicator
                scanProgress.style.display = 'none';

                if (response && response.status === 'success') {
                    // Display the matplotlib visualization
                    if (mapImg && response.data && response.data.plot_image) {
                        mapImg.src = `data:image/png;base64,${response.data.plot_image}`;
                        mapImg.style.display = 'block';
                    }

                    // Display the ASCII map
                    if (asciiMap && response.data && response.data.grid_data && response.data.grid_data.grid) {
                        // Create ASCII representation
                        const grid = response.data.grid_data.grid;
                        const asciiRepresentation = grid.map(row =>
                            row.map(cell => cell === 2 ? 'C' : cell === 1 ? '#' : '.').join('')
                        ).join('\n');
                        asciiMap.textContent = asciiRepresentation;
                        asciiMap.style.display = 'block';
                    }

                    if (statusEl) {
                        statusEl.textContent = 'Status: Scan complete';
                    }
                }
            } catch (error) {
                console.error("Scan error:", error);
                scanProgress.style.display = 'none';
            } finally {
                this.disabled = false;
            }
        });
    }

    // Setup video feed button
    const toggleVideoBtn = container.querySelector('#toggleVideo');
    const videoFeed = container.querySelector('#videoFeed');

    if (toggleVideoBtn && videoFeed) {
        const newToggleBtn = toggleVideoBtn.cloneNode(true);
        toggleVideoBtn.parentNode.replaceChild(newToggleBtn, toggleVideoBtn);

        let isVideoActive = false;
        let videoInterval;

        newToggleBtn.addEventListener('click', async function() {
            if (!isVideoActive) {
                // Start video
                await sendCommand('see');
                videoFeed.style.display = 'block';
                this.textContent = 'Stop Video Feed';

                if (isBluetooth) {
                    // For Bluetooth, set source directly to streaming endpoint
                    const timestamp = new Date().getTime();
                    videoFeed.src = `${apiUrl}/video_feed?t=${timestamp}`;
                } else {
                    // For WiFi, poll for new frames
                    videoInterval = setInterval(() => {
                        const timestamp = new Date().getTime();
                        videoFeed.src = `${apiUrl}/video_feed?t=${timestamp}`;
                    }, 100);
                }

                isVideoActive = true;
            } else {
                // Stop video
                await sendCommand('blind');

                if (!isBluetooth && videoInterval) {
                    clearInterval(videoInterval);
                }

                videoFeed.style.display = 'none';
                videoFeed.src = '';
                this.textContent = 'Start Video Feed';
                isVideoActive = false;
            }
        });
    }

    // Setup Bluetooth-specific features
    if (isBluetooth) {
        const connectBtn = container.querySelector('#connectButton');
        const disconnectBtn = container.querySelector('#disconnectButton');
        const btAddressInput = container.querySelector('#btAddress');
        const connectionStatus = container.querySelector('#connectionStatus');

        if (connectBtn && disconnectBtn && btAddressInput && connectionStatus) {
            // Clear existing listeners
            const newConnectBtn = connectBtn.cloneNode(true);
            const newDisconnectBtn = disconnectBtn.cloneNode(true);

            connectBtn.parentNode.replaceChild(newConnectBtn, connectBtn);
            disconnectBtn.parentNode.replaceChild(newDisconnectBtn, disconnectBtn);

            // Reconnect event listeners
            newConnectBtn.addEventListener('click', async function() {
                const address = btAddressInput.value.trim();
                if (!address) {
                    if (container.querySelector('#status')) {
                        container.querySelector('#status').textContent = 'Status: Please enter a Bluetooth address';
                    }
                    return;
                }

                try {
                    if (container.querySelector('#status')) {
                        container.querySelector('#status').textContent = 'Status: Connecting...';
                    }

                    const response = await fetch(`${apiUrl}/connect`, {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({ bt_address: address })
                    });

                    const data = await response.json();

                    if (data.status === 'success') {
                        connectionStatus.textContent = 'Status: Connected';
                        connectionStatus.className = 'connected';

                        if (container.querySelector('#status')) {
                            container.querySelector('#status').textContent = `Status: ${data.message}`;
                        }

                        // Enable control buttons
                        container.querySelectorAll('.controls button, #scanEnv, #toggleVideo').forEach(button => {
                            button.disabled = false;
                        });

                        newConnectBtn.disabled = true;
                        newDisconnectBtn.disabled = false;

                        // Save address for future use
                        localStorage.setItem('picarx_bt_address', address);
                    } else {
                        if (container.querySelector('#status')) {
                            container.querySelector('#status').textContent = `Status: ${data.message}`;
                        }
                    }
                } catch (error) {
                    if (container.querySelector('#status')) {
                        container.querySelector('#status').textContent = `Error: ${error.message}`;
                    }
                }
            });

            newDisconnectBtn.addEventListener('click', async function() {
                try {
                    const response = await fetch(`${apiUrl}/disconnect`, {
                        method: 'POST'
                    });

                    const data = await response.json();

                    connectionStatus.textContent = 'Status: Disconnected';
                    connectionStatus.className = 'disconnected';

                    if (container.querySelector('#status')) {
                        container.querySelector('#status').textContent = `Status: ${data.message}`;
                    }

                    // Disable control buttons
                    container.querySelectorAll('.controls button, #scanEnv, #toggleVideo').forEach(button => {
                        button.disabled = true;
                    });

                    newConnectBtn.disabled = false;
                    newDisconnectBtn.disabled = true;

                    // Handle video feed cleanup
                    const toggleVideoBtn = container.querySelector('#toggleVideo');
                    const videoFeed = container.querySelector('#videoFeed');

                    if (toggleVideoBtn && videoFeed && videoFeed.style.display !== 'none') {
                        videoFeed.style.display = 'none';
                        videoFeed.src = '';
                        toggleVideoBtn.textContent = 'Start Video Feed';
                    }
                } catch (error) {
                    if (container.querySelector('#status')) {
                        container.querySelector('#status').textContent = `Error: ${error.message}`;
                    }
                }
            });
        }
    }
}