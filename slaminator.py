import numpy as np
from picarx import Picarx
import time
import math

class SmartNavigator:
    def __init__(self, map_size=100):
        self.px = Picarx()
        self.map_size = map_size
        self.map = np.zeros((map_size, map_size))
        self.car_pos = np.array([map_size // 2, map_size // 2])  # Start at center
        self.car_angle = 0  # Facing positive x-axis

        # Navigation parameters
        self.min_distance = 15  # cm
        self.backup_distance = 20  # cm
        self.speed = 30
        self.backup_time = 1.0
        self.turn_time = 1.2

        # Scanning parameters
        self.scan_range = (-60, 60)  # degrees
        self.scan_step = 5  # degrees

    def scan_avg(self):
        distances = []
        for _ in range(3):
            dist = self.px.ultrasonic.read()
            if dist and 0 < dist < 300:  # Filter invalid readings
                distances.append(dist)
            time.sleep(0.01)
        return distances

    def scan_environment(self):
        """Perform a sensor sweep and return scan data"""
        scan_data = []
        start_angle, end_angle = self.scan_range

        for angle in range(start_angle, end_angle + 1, self.scan_step):
            self.px.set_cam_pan_angle(angle)
            time.sleep(0.1)  # Allow servo to settle

            # Multiple readings for stability
            distances = self.scan_avg()
            # for _ in range(3):
            # dist = self.px.ultrasonic.read()
            # if dist and 0 < dist < 300:  # Filter invalid readings
            # distances.append(dist)
            # time.sleep(0.01)

            if distances:
                avg_dist = sum(distances) / len(distances)
                scan_data.append((angle, avg_dist))

        self.px.set_cam_pan_angle(0)
        return scan_data

    def update_map(self, scan_data):
        """Update internal map with new scan data"""
        # Clear previous readings
        self.map = np.zeros((self.map_size, self.map_size))

        for i in range(len(scan_data) - 1):
            angle1, dist1 = scan_data[i]
            angle2, dist2 = scan_data[i + 1]

            # Convert to cartesian coordinates
            point1 = self._polar_to_cartesian(angle1, dist1)
            point2 = self._polar_to_cartesian(angle2, dist2)

            # Interpolate between points
            num_points = max(
                abs(int(point2[0] - point1[0])),
                abs(int(point2[1] - point1[1]))
            ) + 1

            if num_points > 1:
                x_points = np.linspace(point1[0], point2[0], num_points)
                y_points = np.linspace(point1[1], point2[1], num_points)

                # Mark interpolated points
                for x, y in zip(x_points, y_points):
                    map_x = int(x + self.map_size // 2)
                    map_y = int(y + self.map_size // 2)

                    if (0 <= map_x < self.map_size and
                            0 <= map_y < self.map_size):
                        self.map[map_y, map_x] = 1

    def _polar_to_cartesian(self, angle_deg, distance):
        """Convert polar coordinates to cartesian"""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return np.array([x, y])

    def find_best_direction(self, scan_data):
        """Analyze scan data to find the best direction to move"""
        max_distance = 0
        best_angle = 0

        for angle, distance in scan_data:
            if distance > max_distance:
                max_distance = distance
                best_angle = angle

        return best_angle, max_distance

    def check_immediate_surroundings(self):
        """Check if any obstacles are too close"""
        self.px.set_dir_servo_angle(0)  # Look forward
        time.sleep(0.1)

        # distance = self.px.ultrasonic.read(10)
        distances = self.scan_avg()
        if distances:
            avg_dist = sum(distances) / len(distances)
            # scan_data.append((angle, avg_dist))
            return avg_dist

        return 100

    def evasive_maneuver(self):
        """Execute intelligent evasive maneuver based on map"""
        # Stop first
        self.px.forward(0)
        time.sleep(0.5)

        # Scan surroundings
        scan_data = self.scan_environment()
        self.update_map(scan_data)

        # Find best direction to turn
        best_angle, max_distance = self.find_best_direction(scan_data)

        # Back up
        print(f"Backing up... ({self.backup_distance}cm)")
        self.px.backward(self.speed)
        time.sleep(self.backup_time)
        self.px.forward(0)
        time.sleep(0.2)

        # Turn toward best direction
        print(f"Turning to {best_angle}Â° (clearest path: {max_distance:.1f}cm)")
        self.px.set_dir_servo_angle(best_angle)
        self.px.forward(self.speed)
        time.sleep(self.turn_time)

        # Straighten wheels gradually
        self.px.set_dir_servo_angle(0)

    def visualize_map(self):
        """Print ASCII visualization of the map"""
        for row in self.map:
            print(''.join(['â' if cell else '.' for cell in row]))

    def run(self):
        """Main navigation loop"""
        print("Starting smart navigation...")

        try:
            while True:
                # Quick check of immediate surroundings
                distance = self.check_immediate_surroundings()

                if distance < self.min_distance:
                    print(f"\nObstacle detected at {distance:.1f}cm!")
                    self.evasive_maneuver()

                    # After evasion, show updated map
                    print("\nCurrent Map:")
                    self.visualize_map()
                else:
                    # Move forward if path is clear
                    self.px.forward(self.speed)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nNavigation stopped by user")
            self.px.forward(0)
            self.px.set_dir_servo_angle(0)


def main():
    navigator = SmartNavigator()
    navigator.run()


if __name__ == "__main__":
    main()