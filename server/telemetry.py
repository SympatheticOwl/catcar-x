import collections
import os
import re
import subprocess
import time
from robot_hat import ADC


class Telemetry:
    """
    Telemetry class for monitoring Raspberry Pi system metrics like CPU temperature
    and battery level (if available through connected hardware).
    """

    def __init__(self, battery_path=None):
        """
        Initialize the telemetry system.

        Args:
            battery_path (str, optional): Path to battery information if using a specific
                                         battery module/HAT with its own monitoring interface.
        """
        self.battery_adc = ADC(4)
        self.battery_path = battery_path
        self.last_cpu_temp = 0
        self.last_battery_level = None
        self.last_update_time = 0
        self.update_interval = 5  # Update readings every 5 seconds

        self.MAX_VOLTAGE = 8.4  # Fully charged 2S Li-ion
        self.MIN_VOLTAGE = 6.0  # Over-discharge protection at 6.0V
        self.BATTERY_CAPACITY_MAH = 2000  # 2000mAh as specified
        self.SCALING_FACTOR = 3  # May need calibration
        # For time remaining estimation
        self.voltage_history = collections.deque(maxlen=60)  # Store last minute of readings
        self.current_draw_estimates = {
            'idle': 100,  # mA - estimate when robot is stationary
            'moving': 500,  # mA - estimate when moving (no load)
            'heavy_load': 1000  # mA - estimate under heavy load (climbing, carrying)
        }

        # Default current usage pattern - can be updated based on robot state
        self.current_usage_mode = 'idle'

    def _should_update(self):
        """Check if values should be updated based on time interval"""
        current_time = time.time()
        if current_time - self.last_update_time >= self.update_interval:
            self.last_update_time = current_time
            return True
        return False

    def get_cpu_temperature(self):
        """
        Get the current CPU temperature.

        Returns:
            float: CPU temperature in Celsius
        """
        if not self._should_update():
            return self.last_cpu_temp

        try:
            # Get CPU temperature using the thermal zone interface
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
                self.last_cpu_temp = round(temp, 1)
                return self.last_cpu_temp
        except Exception as e:
            print(f"Error getting CPU temperature: {e}")
            # Fall back to vcgencmd if thermal zone interface isn't available
            try:
                temp_output = subprocess.check_output(['vcgencmd', 'measure_temp']).decode('utf-8')
                temp = float(re.search(r'temp=(\d+\.\d+)', temp_output).group(1))
                self.last_cpu_temp = round(temp, 1)
                return self.last_cpu_temp
            except Exception as e:
                print(f"Error getting CPU temperature via vcgencmd: {e}")
                return self.last_cpu_temp or 0

    def get_system_load(self):
        """
        Get system load and memory usage.

        Returns:
            dict: Dictionary containing system metrics:
                - cpu_usage (float): CPU usage percentage
                - memory_usage (float): Memory usage percentage
                - load_average (list): System load average for 1, 5, and 15 minutes
        """
        system_info = {
            "cpu_usage": 0,
            "memory_usage": 0,
            "load_average": [0, 0, 0]
        }

        try:
            # Get CPU usage using top
            cpu_output = subprocess.check_output(
                ["top", "-bn1"],
                universal_newlines=True
            )
            cpu_line = [line for line in cpu_output.split('\n') if 'Cpu(s)' in line][0]
            cpu_usage = 100.0 - float(re.search(r'(\d+\.\d+)(?=\s*id)', cpu_line).group(1))
            system_info["cpu_usage"] = round(cpu_usage, 1)

            # Get memory usage
            mem_output = subprocess.check_output(
                ["free", "-m"],
                universal_newlines=True
            )
            mem_line = mem_output.split('\n')[1]
            mem_values = mem_line.split()
            total_mem = float(mem_values[1])
            used_mem = float(mem_values[2])
            memory_percentage = (used_mem / total_mem) * 100
            system_info["memory_usage"] = round(memory_percentage, 1)

            # Get load average
            with open('/proc/loadavg', 'r') as f:
                load_avg = f.read().strip().split()
                system_info["load_average"] = [float(load_avg[0]), float(load_avg[1]), float(load_avg[2])]
        except Exception as e:
            print(f"Error getting system info: {e}")

        return system_info

    def get_battery_voltage(self):
        raw_voltage = self.battery_adc.read_voltage()
        scaling_factor = 3  # Typical for 2S LiPo battery monitoring, may need adjustment
        battery_voltage = raw_voltage * scaling_factor
        return battery_voltage

    def estimate_battery_percentage(self, voltage):
        if voltage >= self.MAX_VOLTAGE:
            return 100
        elif voltage <= self.MIN_VOLTAGE:
            return 0
        else:
            percentage = ((voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE)) * 100
            return round(percentage, 1)

    def set_usage_mode(self, mode):
        """Update the current usage mode to improve time estimation"""
        if mode in self.current_draw_estimates:
            self.current_usage_mode = mode
            return True
        return False

    def estimate_time_remaining(self, voltage, percentage):
        # Add current voltage to history
        self.voltage_history.append((time.time(), voltage))

        # Method 1: Based on voltage discharge rate
        if len(self.voltage_history) >= 10:  # Need some history for discharge rate
            oldest = self.voltage_history[0]
            newest = self.voltage_history[-1]
            time_diff_hours = (newest[0] - oldest[0]) / 3600

            if time_diff_hours > 0.01:  # Ensure we have enough time difference
                voltage_diff = oldest[1] - newest[1]

                if voltage_diff > 0:  # Discharging
                    discharge_rate = voltage_diff / time_diff_hours  # V/hour
                    voltage_remaining = voltage - self.MIN_VOLTAGE
                    hours_remaining_voltage = voltage_remaining / discharge_rate

                    # Method 2: Based on battery percentage and current draw
                    current_draw = self.current_draw_estimates[self.current_usage_mode]
                    capacity_remaining_mah = (percentage / 100) * self.BATTERY_CAPACITY_MAH
                    hours_remaining_current = capacity_remaining_mah / current_draw

                    # Weighted average of both methods (favor the voltage-based one)
                    hours_remaining = (hours_remaining_voltage * 0.7) + (hours_remaining_current * 0.3)

                    hours = int(hours_remaining)
                    minutes = int((hours_remaining - hours) * 60)

                    return f"{hours}h {minutes}m ({self.current_usage_mode} mode)"

                return "Charging or stable"

        # Fallback method when discharge rate can't be calculated
        current_draw = self.current_draw_estimates[self.current_usage_mode]
        capacity_remaining_mah = (percentage / 100) * self.BATTERY_CAPACITY_MAH
        hours_remaining = capacity_remaining_mah / current_draw

        hours = int(hours_remaining)
        minutes = int((hours_remaining - hours) * 60)

        return f"{hours}h {minutes}m (estimated based on {self.current_usage_mode} mode)"

    def get_battery_level(self):
        """
        Get the current battery level if available.

        Returns:
            dict: Dictionary containing battery metrics:
                - percentage (float): Battery level percentage (0-100)
                - voltage (float): Battery voltage
                - is_charging (bool): Whether the battery is currently charging
                - time_remaining (float): Estimated time remaining in hours (if available)
        """
        if not self._should_update() and self.last_battery_level is not None:
            return self.last_battery_level

        # Default response if no battery monitoring is available
        voltage = self.get_battery_voltage()
        percentage = self.estimate_battery_percentage(voltage)
        battery_info = {
            "percentage": percentage,
            "voltage": voltage,
            "is_charging": None,
            "time_remaining": self.estimate_time_remaining(voltage, percentage),
        }

        self.last_battery_level = battery_info
        return battery_info

    def get_all_telemetry(self):
        """
        Get all telemetry information in a single call.

        Returns:
            dict: Dictionary containing all telemetry metrics
        """
        data = {
            "cpu_temperature": self.get_cpu_temperature(),
            "battery": self.get_battery_level(),
            "system": self.get_system_load(),
            "timestamp": time.time()
        }

        print(f"Telemetry data: {data}")

        return data