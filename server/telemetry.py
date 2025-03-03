import os
import re
import subprocess
import time


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
        self.battery_path = battery_path
        self.last_cpu_temp = 0
        self.last_battery_level = None
        self.last_update_time = 0
        self.update_interval = 5  # Update readings every 5 seconds

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
        battery_info = {
            "percentage": None,
            "voltage": None,
            "is_charging": None,
            "time_remaining": None,
            "available": False
        }

        # Try to read battery info based on what's available on the system
        try:
            # If using a UPS HAT or battery module with direct path
            if self.battery_path and os.path.exists(self.battery_path):
                with open(self.battery_path, 'r') as f:
                    data = f.read().strip()
                    # Parse data based on the specific battery module format
                    # This will need to be customized for specific hardware
                    # Example assuming a CSV format: percentage,voltage,is_charging
                    parts = data.split(',')
                    if len(parts) >= 3:
                        battery_info["percentage"] = float(parts[0])
                        battery_info["voltage"] = float(parts[1])
                        battery_info["is_charging"] = parts[2].lower() == 'true'
                        battery_info["available"] = True
            else:
                # Try to use i2c interface for common battery modules like INA219
                # This requires additional setup and may need to be adapted
                try:
                    import board
                    import adafruit_ina219

                    i2c = board.I2C()
                    ina219 = adafruit_ina219.INA219(i2c)

                    voltage = ina219.bus_voltage + ina219.shunt_voltage
                    # Simple estimation - will need calibration for actual battery
                    # Assuming Li-ion battery with voltage range 3.2V (0%) to 4.2V (100%)
                    percentage = max(0, min(100, (voltage - 3.2) / (4.2 - 3.2) * 100))

                    battery_info["voltage"] = round(voltage, 2)
                    battery_info["percentage"] = round(percentage, 1)
                    battery_info["is_charging"] = ina219.current > 0  # Positive current = charging
                    battery_info["available"] = True

                except Exception as e:
                    # If INA219 is not available, try other methods
                    print(f"INA219 not available: {e}")
                    pass
        except Exception as e:
            print(f"Error getting battery info: {e}")

        self.last_battery_level = battery_info
        return battery_info

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