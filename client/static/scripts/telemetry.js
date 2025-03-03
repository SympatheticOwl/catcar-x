// telemetry.js - Handles the telemetry data fetching and display

class TelemetryManager {
    constructor() {
        this.telemetryUpdateInterval = null;
        this.updateIntervalMs = 5000; // Update every 5 seconds by default
        this.lastUpdate = 0;

        // Initialize the UI elements
        this.initElements();

        // Add event listeners
        this.addEventListeners();
    }

    initElements() {
        // Get all UI elements
        this.elements = {
            cpuTemp: document.getElementById('cpu-temp'),
            cpuTempBar: document.querySelector('#cpu-temp-bar .s-progress--bar'),
            batteryLevel: document.getElementById('battery-level'),
            batteryLevelBar: document.querySelector('#battery-level-bar .s-progress--bar'),
            batteryVoltage: document.getElementById('battery-voltage'),
            batteryStatus: document.getElementById('battery-status'),
            cpuUsage: document.getElementById('cpu-usage'),
            cpuUsageBar: document.querySelector('#cpu-usage-bar .s-progress--bar'),
            memoryUsage: document.getElementById('memory-usage'),
            memoryUsageBar: document.querySelector('#memory-usage-bar .s-progress--bar'),
            loadAverage: document.getElementById('load-average'),
            timestamp: document.getElementById('telemetry-timestamp'),
            refreshButton: document.getElementById('refresh-telemetry')
        };
    }

    addEventListeners() {
        // Add click event for refresh button
        if (this.elements.refreshButton) {
            this.elements.refreshButton.addEventListener('click', () => {
                this.fetchTelemetry();
            });
        }

        // Listen for connection/disconnection events
        document.addEventListener('picarx-connected', () => {
            this.startTelemetryUpdates();
        });

        document.addEventListener('picarx-disconnected', () => {
            this.stopTelemetryUpdates();
            this.resetDisplay();
        });
    }

    startTelemetryUpdates() {
        // Fetch telemetry immediately
        this.fetchTelemetry();

        // Start periodic updates
        this.telemetryUpdateInterval = setInterval(() => {
            this.fetchTelemetry();
        }, this.updateIntervalMs);

        console.log('Telemetry updates started');
    }

    stopTelemetryUpdates() {
        if (this.telemetryUpdateInterval) {
            clearInterval(this.telemetryUpdateInterval);
            this.telemetryUpdateInterval = null;
            console.log('Telemetry updates stopped');
        }
    }

    fetchTelemetry() {
        // Check if enough time has passed since last update
        const now = Date.now();
        if (now - this.lastUpdate < 1000) {
            console.log('Throttling telemetry update');
            return;
        }

        this.lastUpdate = now;

        // Fetch the telemetry data
        fetch('/telemetry')
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success' && data.telemetry) {
                    this.updateDisplay(data.telemetry);
                } else {
                    console.error('Error fetching telemetry:', data.message || 'Unknown error');
                }
            })
            .catch(error => {
                console.error('Error fetching telemetry:', error);
            });
    }

    updateDisplay(telemetry) {
        // Update CPU temperature
        if (telemetry.cpu_temperature !== undefined) {
            this.updateCpuTemperature(telemetry.cpu_temperature);
        }

        // Update battery information if available
        if (telemetry.battery) {
            this.updateBatteryInfo(telemetry.battery);
        }

        // Update system information
        if (telemetry.system) {
            this.updateSystemInfo(telemetry.system);
        }

        // Update timestamp
        this.updateTimestamp(telemetry.timestamp);
    }

    updateCpuTemperature(temp) {
        if (!this.elements.cpuTemp || !this.elements.cpuTempBar) return;

        this.elements.cpuTemp.textContent = temp.toFixed(1);

        // Update progress bar (0-100°C range)
        const percentage = Math.min(100, Math.max(0, temp / 100 * 100));
        this.elements.cpuTempBar.style.width = `${percentage}%`;

        // Change color based on temperature
        const progressElement = this.elements.cpuTempBar.closest('.s-progress');
        progressElement.className = 's-progress';

        if (temp > 80) {
            progressElement.classList.add('s-progress__danger');
        } else if (temp > 65) {
            progressElement.classList.add('s-progress__warning');
        } else {
            progressElement.classList.add('s-progress__success');
        }
    }

    updateBatteryInfo(battery) {
        // Update battery level if available
        if (battery.percentage !== null && this.elements.batteryLevel && this.elements.batteryLevelBar) {
            this.elements.batteryLevel.textContent = battery.percentage.toFixed(1);

            // Update progress bar
            const percentage = Math.min(100, Math.max(0, battery.percentage));
            this.elements.batteryLevelBar.style.width = `${percentage}%`;

            // Change color based on level
            const progressElement = this.elements.batteryLevelBar.closest('.s-progress');
            progressElement.className = 's-progress';

            if (battery.percentage < 20) {
                progressElement.classList.add('s-progress__danger');
            } else if (battery.percentage < 40) {
                progressElement.classList.add('s-progress__warning');
            } else {
                progressElement.classList.add('s-progress__success');
            }
        } else if (this.elements.batteryLevel) {
            this.elements.batteryLevel.textContent = 'N/A';
        }

        // Update voltage if available
        if (battery.voltage !== null && this.elements.batteryVoltage) {
            this.elements.batteryVoltage.textContent = battery.voltage.toFixed(2);
        } else if (this.elements.batteryVoltage) {
            this.elements.batteryVoltage.textContent = 'N/A';
        }

        // Update charging status if available
        if (battery.is_charging !== null && this.elements.batteryStatus) {
            this.elements.batteryStatus.textContent = battery.is_charging ? 'Charging' : 'Discharging';
        } else if (this.elements.batteryStatus) {
            this.elements.batteryStatus.textContent = battery.available ? 'Unknown' : 'Not Available';
        }
    }

    updateSystemInfo(system) {
        // Update CPU usage
        if (system.cpu_usage !== undefined && this.elements.cpuUsage && this.elements.cpuUsageBar) {
            this.elements.cpuUsage.textContent = system.cpu_usage.toFixed(1);

            // Update progress bar
            const percentage = Math.min(100, Math.max(0, system.cpu_usage));
            this.elements.cpuUsageBar.style.width = `${percentage}%`;

            // Change color based on usage
            const progressElement = this.elements.cpuUsageBar.closest('.s-progress');
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
        if (system.memory_usage !== undefined && this.elements.memoryUsage && this.elements.memoryUsageBar) {
            this.elements.memoryUsage.textContent = system.memory_usage.toFixed(1);

            // Update progress bar
            const percentage = Math.min(100, Math.max(0, system.memory_usage));
            this.elements.memoryUsageBar.style.width = `${percentage}%`;

            // Change color based on usage
            const progressElement = this.elements.memoryUsageBar.closest('.s-progress');
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
        if (system.load_average && this.elements.loadAverage) {
            this.elements.loadAverage.textContent = system.load_average.map(val => val.toFixed(2)).join(', ');
        }
    }

    updateTimestamp(timestamp) {
        if (!this.elements.timestamp) return;

        if (timestamp) {
            const date = new Date(timestamp * 1000);
            this.elements.timestamp.textContent = date.toLocaleTimeString();
        } else {
            this.elements.timestamp.textContent = 'Unknown';
        }
    }

    resetDisplay() {
        // Reset all display elements to their default values
        if (this.elements.cpuTemp) this.elements.cpuTemp.textContent = '--';
        if (this.elements.cpuTempBar) this.elements.cpuTempBar.style.width = '0%';
        if (this.elements.batteryLevel) this.elements.batteryLevel.textContent = '--';
        if (this.elements.batteryLevelBar) this.elements.batteryLevelBar.style.width = '0%';
        if (this.elements.batteryVoltage) this.elements.batteryVoltage.textContent = '--';
        if (this.elements.batteryStatus) this.elements.batteryStatus.textContent = 'Unknown';
        if (this.elements.cpuUsage) this.elements.cpuUsage.textContent = '--';
        if (this.elements.cpuUsageBar) this.elements.cpuUsageBar.style.width = '0%';
        if (this.elements.memoryUsage) this.elements.memoryUsage.textContent = '--';
        if (this.elements.memoryUsageBar) this.elements.memoryUsageBar.style.width = '0%';
        if (this.elements.loadAverage) this.elements.loadAverage.textContent = '--';
        if (this.elements.timestamp) this.elements.timestamp.textContent = '--';
    }
}

// Initialize telemetry when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.telemetryManager = new TelemetryManager();

    // If we're already connected, start updates
    if (window.bridge && window.bridge.isConnected) {
        window.telemetryManager.startTelemetryUpdates();
    }
});

// Add telemetry tab to the existing page
function initializeTelemetryTab() {
    // Check if the tabs container exists
    const tabsContainer = document.querySelector('.s-tabs');
    if (!tabsContainer) return;

    // Add the telemetry tab
    const telemetryTab = document.createElement('li');
    telemetryTab.className = 's-tab';
    telemetryTab.innerHTML = '<a href="#telemetry">Telemetry</a>';
    tabsContainer.appendChild(telemetryTab);

    // Add the telemetry content area if needed
    let contentArea = document.getElementById('telemetry');
    if (!contentArea) {
        contentArea = document.createElement('div');
        contentArea.id = 'telemetry';
        contentArea.className = 'tab-content';
        contentArea.style.display = 'none';
        document.querySelector('.tab-content-container').appendChild(contentArea);

        // Load the telemetry content
        fetch('/telemetry-fragment')
            .then(response => response.text())
            .then(html => {
                contentArea.innerHTML = html;
                // Re-initialize telemetry manager to find new elements
                window.telemetryManager = new TelemetryManager();
            })
            .catch(error => {
                console.error('Error loading telemetry fragment:', error);
                contentArea.innerHTML = '<p class="p-4">Error loading telemetry data.</p>';
            });
    }

    // Add tab click handler
    telemetryTab.querySelector('a').addEventListener('click', (e) => {
        e.preventDefault();

        // Hide all content areas
        document.querySelectorAll('.tab-content').forEach(el => el.style.display = 'none');

        // Show the telemetry content
        contentArea.style.display = 'block';

        // Update active tab
        document.querySelectorAll('.s-tab').forEach(tab => tab.classList.remove('is-selected'));
        telemetryTab.classList.add('is-selected');

        // Refresh telemetry data
        if (window.telemetryManager) {
            window.telemetryManager.fetchTelemetry();
        }
    });
}

// Call the initialization function when the page is ready
document.addEventListener('page-initialized', initializeTelemetryTab);