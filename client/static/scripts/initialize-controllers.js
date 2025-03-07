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

// Global telemetry variables
let telemetryUpdateInterval = null;
const telemetryUpdateIntervalMs = 5000; // Update every 5 seconds

// Initialize Bluetooth controller
function initializeBluetoothController() {
    console.log('Initializing Bluetooth controller...');

    const btContainer = document.getElementById('bt-content') || document.getElementById('bt-controller-content');
    if (!btContainer) {
        console.warn('Bluetooth container not found');
        return;
    }

    // Re-initialize all event listeners and components
    setupButtonHandlers(btContainer, true);

    // Initialize telemetry components
    initializeTelemetry(btContainer, true);

    // Re-initialize any additional Bluetooth-specific functionality
    const btStatus = btContainer.querySelector('#status') || btContainer.querySelector('#bt-status');
    if (btStatus) {
        btStatus.textContent = 'Status: Bluetooth controller ready';
    }

    console.log('Bluetooth controller initialized');
}

// Initialize WiFi controller
function initializeWifiController() {
    console.log('Initializing WiFi controller...');

    const wifiContainer = document.getElementById('wifi-content') || document.getElementById('wifi-controller-content');
    if (!wifiContainer) {
        console.warn('WiFi container not found');
        return;
    }

    // Re-initialize all event listeners and components
    setupButtonHandlers(wifiContainer, false);

    // Initialize telemetry components for WiFi if needed
    initializeTelemetry(wifiContainer, false);

    // Re-initialize any additional WiFi-specific functionality
    const wifiStatus = wifiContainer.querySelector('#status') || wifiContainer.querySelector('#wifi-status');
    if (wifiStatus) {
        wifiStatus.textContent = 'Status: WiFi controller ready';
    }

    console.log('WiFi controller initialized');
}

// Initialize telemetry components and functionality
function initializeTelemetry(container, isBluetooth) {
    if (!container) return;

    // Get API base URL based on controller type
    // const apiUrl = isBluetooth ? window.location.origin : 'http://10.0.0.219:8000';
    const apiUrl = isBluetooth ? window.location.origin : 'http://192.168.0.163:8000';

    // Find telemetry elements
    const elements = {
        cpuTemp: container.querySelector('#cpu-temp'),
        cpuTempBar: container.querySelector('#cpu-temp-bar .s-progress--bar'),
        batteryLevel: container.querySelector('#battery-level'),
        batteryLevelBar: container.querySelector('#battery-level-bar .s-progress--bar'),
        batteryVoltage: container.querySelector('#battery-voltage'),
        batteryStatus: container.querySelector('#battery-status'),
        cpuUsage: container.querySelector('#cpu-usage'),
        cpuUsageBar: container.querySelector('#cpu-usage-bar .s-progress--bar'),
        memoryUsage: container.querySelector('#memory-usage'),
        memoryUsageBar: container.querySelector('#memory-usage-bar .s-progress--bar'),
        loadAverage: container.querySelector('#load-average'),
        timestamp: container.querySelector('#telemetry-timestamp'),
        refreshButton: container.querySelector('#refresh-telemetry')
    };

    // If telemetry elements don't exist, log and return
    if (!elements.cpuTemp) {
        console.log('Telemetry elements not found in this container');
        return;
    }

    console.log('Initializing telemetry components');

    // Add click event for refresh button
    if (elements.refreshButton) {
        elements.refreshButton.addEventListener('click', () => {
            fetchTelemetry(apiUrl, elements);
        });
    }

    // Listen for connection/disconnection events specific to this controller
    const connectButton = container.querySelector('#connectButton');
    const disconnectButton = container.querySelector('#disconnectButton');

    if (connectButton) {
        // Add event to start telemetry updates on successful connection
        connectButton.addEventListener('click', function() {
            // We need to check if connection was successful
            setTimeout(() => {
                const connectionStatus = container.querySelector('#connectionStatus');
                if (connectionStatus && connectionStatus.classList.contains('connected')) {
                    startTelemetryUpdates(apiUrl, elements);
                }
            }, 1000); // Small delay to allow connection status to update
        });
    }

    if (disconnectButton) {
        // Stop telemetry updates on disconnect
        disconnectButton.addEventListener('click', function() {
            stopTelemetryUpdates();
            resetTelemetryDisplay(elements);
        });
    }

    // Check if already connected and start telemetry if needed
    const connectionStatus = container.querySelector('#connectionStatus');
    if (connectionStatus && connectionStatus.classList.contains('connected')) {
        startTelemetryUpdates(apiUrl, elements);
    }
}

// Function to start periodic telemetry updates
function startTelemetryUpdates(apiUrl, elements) {
    // Clear any existing interval
    stopTelemetryUpdates();

    // Fetch telemetry immediately
    fetchTelemetry(apiUrl, elements);

    // Start periodic updates
    telemetryUpdateInterval = setInterval(() => {
        fetchTelemetry(apiUrl, elements);
    }, telemetryUpdateIntervalMs);

    console.log('Telemetry updates started');
}

// Function to stop telemetry updates
function stopTelemetryUpdates() {
    if (telemetryUpdateInterval) {
        clearInterval(telemetryUpdateInterval);
        telemetryUpdateInterval = null;
        console.log('Telemetry updates stopped');
    }
}

// Function to fetch telemetry data
function fetchTelemetry(apiUrl, elements) {
    console.log('Fetching telemetry data');

    // Use AbortController for timeout control
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 3000); // 3s timeout

    fetch(`${apiUrl}/telemetry`, {
        signal: controller.signal
    })
    .then(response => response.json())
    .then(data => {
        clearTimeout(timeoutId);
        if (data.status === 'success' && data.telemetry) {
            updateTelemetryDisplay(elements, data.telemetry);
        } else {
            console.error('Error fetching telemetry:', data.message || 'Unknown error');
        }
    })
    .catch(error => {
        clearTimeout(timeoutId);
        if (error.name === 'AbortError') {
            console.log('Telemetry request timed out');
        } else {
            console.error('Error fetching telemetry:', error);
        }
    });
}

// Function to update telemetry display with new data
function updateTelemetryDisplay(elements, telemetry) {
    // Update CPU temperature
    if (telemetry.cpu_temperature !== undefined && elements.cpuTemp && elements.cpuTempBar) {
        elements.cpuTemp.textContent = telemetry.cpu_temperature.toFixed(1);

        // Update progress bar (0-100°C range)
        const percentage = Math.min(100, Math.max(0, telemetry.cpu_temperature / 100 * 100));
        elements.cpuTempBar.style.width = `${percentage}%`;

        // Change color based on temperature
        const progressElement = elements.cpuTempBar.closest('.s-progress');
        progressElement.className = 's-progress';

        if (telemetry.cpu_temperature > 80) {
            progressElement.classList.add('s-progress__danger');
        } else if (telemetry.cpu_temperature > 65) {
            progressElement.classList.add('s-progress__warning');
        } else {
            progressElement.classList.add('s-progress__success');
        }
    }

    // Update battery information if available
    if (telemetry.battery) {
        const battery = telemetry.battery;

        // Update battery level
        if (battery.percentage !== null && elements.batteryLevel && elements.batteryLevelBar) {
            elements.batteryLevel.textContent = battery.percentage.toFixed(1);

            // Update progress bar
            const percentage = Math.min(100, Math.max(0, battery.percentage));
            elements.batteryLevelBar.style.width = `${percentage}%`;

            // Change color based on level
            const progressElement = elements.batteryLevelBar.closest('.s-progress');
            progressElement.className = 's-progress';

            if (battery.percentage < 20) {
                progressElement.classList.add('s-progress__danger');
            } else if (battery.percentage < 40) {
                progressElement.classList.add('s-progress__warning');
            } else {
                progressElement.classList.add('s-progress__success');
            }
        } else if (elements.batteryLevel) {
            elements.batteryLevel.textContent = 'N/A';
        }

        // Update voltage
        if (battery.voltage !== null && elements.batteryVoltage) {
            elements.batteryVoltage.textContent = battery.voltage.toFixed(2);
        } else if (elements.batteryVoltage) {
            elements.batteryVoltage.textContent = 'N/A';
        }

        // Update charging status
        if (battery.is_charging !== null && elements.batteryStatus) {
            elements.batteryStatus.textContent = battery.is_charging ? 'Charging' : 'Discharging';
        } else if (elements.batteryStatus) {
            elements.batteryStatus.textContent = battery.available ? 'Unknown' : 'Not Available';
        }
    }

    // Update system information
    if (telemetry.system) {
        const system = telemetry.system;

        // Update CPU usage
        if (system.cpu_usage !== undefined && elements.cpuUsage && elements.cpuUsageBar) {
            elements.cpuUsage.textContent = system.cpu_usage.toFixed(1);

            // Update progress bar
            const percentage = Math.min(100, Math.max(0, system.cpu_usage));
            elements.cpuUsageBar.style.width = `${percentage}%`;

            // Change color based on usage
            const progressElement = elements.cpuUsageBar.closest('.s-progress');
            progressElement.className = 's-progress';

            if (system.cpu_usage > 90) {
                progressElement.classList.add('s-progress__danger');
            } else if (system.cpu_usage > 70) {
                progressElement.classList.add('s-progress__warning');
            } else {
                progressElement.classList.add('s-progress__success');
            }
        }

        // Update memory usage
        if (system.memory_usage !== undefined && elements.memoryUsage && elements.memoryUsageBar) {
            elements.memoryUsage.textContent = system.memory_usage.toFixed(1);

            // Update progress bar
            const percentage = Math.min(100, Math.max(0, system.memory_usage));
            elements.memoryUsageBar.style.width = `${percentage}%`;

            // Change color based on usage
            const progressElement = elements.memoryUsageBar.closest('.s-progress');
            progressElement.className = 's-progress';

            if (system.memory_usage > 90) {
                progressElement.classList.add('s-progress__danger');
            } else if (system.memory_usage > 70) {
                progressElement.classList.add('s-progress__warning');
            } else {
                progressElement.classList.add('s-progress__success');
            }
        }

        // Update load average
        if (system.load_average && elements.loadAverage) {
            elements.loadAverage.textContent = system.load_average.map(val => val.toFixed(2)).join(', ');
        }
    }

    // Update timestamp
    if (telemetry.timestamp && elements.timestamp) {
        const date = new Date(telemetry.timestamp * 1000);
        elements.timestamp.textContent = date.toLocaleTimeString();
    }
}

// Function to reset telemetry display to default values
function resetTelemetryDisplay(elements) {
    if (!elements) return;

    // Reset all display elements to their default values
    if (elements.cpuTemp) elements.cpuTemp.textContent = '--';
    if (elements.cpuTempBar) elements.cpuTempBar.style.width = '0%';
    if (elements.batteryLevel) elements.batteryLevel.textContent = '--';
    if (elements.batteryLevelBar) elements.batteryLevelBar.style.width = '0%';
    if (elements.batteryVoltage) elements.batteryVoltage.textContent = '--';
    if (elements.batteryStatus) elements.batteryStatus.textContent = 'Unknown';
    if (elements.cpuUsage) elements.cpuUsage.textContent = '--';
    if (elements.cpuUsageBar) elements.cpuUsageBar.style.width = '0%';
    if (elements.memoryUsage) elements.memoryUsage.textContent = '--';
    if (elements.memoryUsageBar) elements.memoryUsageBar.style.width = '0%';
    if (elements.loadAverage) elements.loadAverage.textContent = '--';
    if (elements.timestamp) elements.timestamp.textContent = '--';
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

    // Setup speed control if present
    const speedControl = container.querySelector('#speedControl');
    if (speedControl) {
        const speedValue = container.querySelector('#speedValue');

        // Clone to remove existing listeners
        const newSpeedControl = speedControl.cloneNode(true);
        speedControl.parentNode.replaceChild(newSpeedControl, speedControl);

        // Add event listener for speed changes
        newSpeedControl.addEventListener('change', async function() {
            const speed = parseFloat(this.value);
            if (speedValue) {
                speedValue.textContent = speed.toFixed(2);
            }

            // Send the speed command
            await sendCommand('set_speed', {
                speed: speed,
                // Use content type application/json
                contentType: 'application/json'
            });
        });

        // Update speed value display if present
        newSpeedControl.addEventListener('input', function() {
            if (speedValue) {
                speedValue.textContent = parseFloat(this.value).toFixed(2);
            }
        });
    }

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

                        // Enable speed control if present
                        if (container.querySelector('#speedControl')) {
                            container.querySelector('#speedControl').disabled = false;
                        }

                        newConnectBtn.disabled = true;
                        newDisconnectBtn.disabled = false;

                        // Save address for future use
                        localStorage.setItem('picarx_bt_address', address);

                        // Trigger a custom event that telemetry can listen for
                        document.dispatchEvent(new CustomEvent('picarx-connected'));

                        // Start telemetry updates by finding the telemetry elements again
                        const telemetryElements = {
                            cpuTemp: container.querySelector('#cpu-temp'),
                            cpuTempBar: container.querySelector('#cpu-temp-bar .s-progress--bar'),
                            batteryLevel: container.querySelector('#battery-level'),
                            batteryLevelBar: container.querySelector('#battery-level-bar .s-progress--bar'),
                            batteryVoltage: container.querySelector('#battery-voltage'),
                            batteryStatus: container.querySelector('#battery-status'),
                            cpuUsage: container.querySelector('#cpu-usage'),
                            cpuUsageBar: container.querySelector('#cpu-usage-bar .s-progress--bar'),
                            memoryUsage: container.querySelector('#memory-usage'),
                            memoryUsageBar: container.querySelector('#memory-usage-bar .s-progress--bar'),
                            loadAverage: container.querySelector('#load-average'),
                            timestamp: container.querySelector('#telemetry-timestamp')
                        };

                        // Start telemetry updates if elements exist
                        if (telemetryElements.cpuTemp) {
                            startTelemetryUpdates(apiUrl, telemetryElements);
                        }
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

                    // Disable speed control if present
                    if (container.querySelector('#speedControl')) {
                        container.querySelector('#speedControl').disabled = true;
                    }

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

                    // Trigger a custom event that telemetry can listen for
                    document.dispatchEvent(new CustomEvent('picarx-disconnected'));

                    // Stop telemetry updates
                    stopTelemetryUpdates();

                    // Reset telemetry display
                    const telemetryElements = {
                        cpuTemp: container.querySelector('#cpu-temp'),
                        cpuTempBar: container.querySelector('#cpu-temp-bar .s-progress--bar'),
                        batteryLevel: container.querySelector('#battery-level'),
                        batteryLevelBar: container.querySelector('#battery-level-bar .s-progress--bar'),
                        batteryVoltage: container.querySelector('#battery-voltage'),
                        batteryStatus: container.querySelector('#battery-status'),
                        cpuUsage: container.querySelector('#cpu-usage'),
                        cpuUsageBar: container.querySelector('#cpu-usage-bar .s-progress--bar'),
                        memoryUsage: container.querySelector('#memory-usage'),
                        memoryUsageBar: container.querySelector('#memory-usage-bar .s-progress--bar'),
                        loadAverage: container.querySelector('#load-average'),
                        timestamp: container.querySelector('#telemetry-timestamp')
                    };

                    resetTelemetryDisplay(telemetryElements);
                } catch (error) {
                    if (container.querySelector('#status')) {
                        container.querySelector('#status').textContent = `Error: ${error.message}`;
                    }
                }
            });
        }
    }

    // Add keyboard controls
    document.addEventListener('keydown', function(event) {
        // Only handle keyboard events if this controller is active
        if (!container.contains(document.activeElement) &&
            !container.matches(':focus-within') &&
            document.activeElement !== document.body) {
            return;
        }

        const validKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'];
        if (!validKeys.includes(event.key)) return;

        event.preventDefault(); // Prevent scrolling

        // Find the corresponding button
        let buttonId;
        switch (event.key) {
            case 'ArrowUp':
                buttonId = 'up';
                break;
            case 'ArrowDown':
                buttonId = 'down';
                break;
            case 'ArrowLeft':
                buttonId = 'left';
                break;
            case 'ArrowRight':
                buttonId = 'right';
                break;
            case 'Space':
                // Space should trigger stop
                sendCommand('stop');
                return;
        }

        const button = container.querySelector(`#${buttonId}`);
        if (button && !button.disabled) {
            // Trigger the mousedown event on the button
            button.dispatchEvent(new MouseEvent('mousedown', {
                bubbles: true,
                cancelable: true,
                view: window
            }));
        }
    });

    document.addEventListener('keyup', function(event) {
        // Only handle keyboard events if this controller is active
        if (!container.contains(document.activeElement) &&
            !container.matches(':focus-within') &&
            document.activeElement !== document.body) {
            return;
        }

        const validKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
        if (!validKeys.includes(event.key)) return;

        event.preventDefault();

        // Find the corresponding button
        let buttonId;
        switch (event.key) {
            case 'ArrowUp':
                buttonId = 'up';
                break;
            case 'ArrowDown':
                buttonId = 'down';
                break;
            case 'ArrowLeft':
                buttonId = 'left';
                break;
            case 'ArrowRight':
                buttonId = 'right';
                break;
        }

        const button = container.querySelector(`#${buttonId}`);
        if (button) {
            // Trigger the mouseup event on the button
            button.dispatchEvent(new MouseEvent('mouseup', {
                bubbles: true,
                cancelable: true,
                view: window
            }));
        }
    });
}