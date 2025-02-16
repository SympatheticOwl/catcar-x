import asyncio
import math
from picarx_wrapper import PicarXWrapper
from object_sensors import AsyncObstacleAvoidance
from enum import Enum


class NavigationState(Enum):
    IDLE = "idle"
    TURNING = "turning"
    MOVING = "moving"
    ARRIVED = "arrived"


class PicarXNavigator:
    def __init__(self, tracking_picarx):
        self.px = tracking_picarx

        # Navigation constants
        self.TURN_SPEED = 30  # Speed during turns
        self.FORWARD_SPEED = 30  # Speed during forward movement
        self.MIN_TURN_RADIUS = 25  # cm - minimum turning radius at max steering
        self.MAX_STEERING_ANGLE = 30  # degrees
        self.POSITION_TOLERANCE = 5  # cm - how close we need to be to target
        self.HEADING_TOLERANCE = 5  # degrees - acceptable heading error
        self.TURNING_TIME_90 = 4.0  # seconds for 90-degree turn
        self.DEGREES_PER_SECOND = 90 / self.TURNING_TIME_90  # degrees/second when turning

        # Navigation state
        self.target_x = 0
        self.target_y = 0
        self.state = NavigationState.IDLE
        self.path_segments = []
        self.current_segment = 0

    def set_target(self, x, y):
        """Set new target coordinates and plan path"""
        self.target_x = x
        self.target_y = y
        self._plan_path()

    def _plan_path(self):
        """Plan path to target considering turning constraints"""
        current_pos = self.px.get_position()
        self.path_segments = []

        # Calculate angle to target
        dx = self.target_x - current_pos['x']
        dy = self.target_y - current_pos['y']
        target_angle = math.degrees(math.atan2(dy, dx))

        # Normalize target angle to 0-360
        target_angle = target_angle % 360
        if target_angle < 0:
            target_angle += 360

        # Calculate required turn
        current_heading = current_pos['heading'] % 360
        angle_diff = target_angle - current_heading

        # Normalize angle difference to -180 to 180
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # Calculate distance to target
        distance = math.sqrt(dx * dx + dy * dy)

        # If we need to turn more than our max steering angle
        # we'll need to do a point turn first
        if abs(angle_diff) > self.MAX_STEERING_ANGLE:
            self.path_segments.append({
                'type': 'turn',
                'angle': angle_diff
            })

        # Add forward movement to target
        if distance > self.POSITION_TOLERANCE:
            self.path_segments.append({
                'type': 'forward',
                'distance': distance
            })

        self.current_segment = 0

    async def execute_path(self):
        """Execute the planned path"""
        self.state = NavigationState.IDLE

        while self.current_segment < len(self.path_segments):
            segment = self.path_segments[self.current_segment]

            if segment['type'] == 'turn':
                await self._execute_turn(segment['angle'])
            elif segment['type'] == 'forward':
                await self._execute_forward(segment['distance'])

            self.current_segment += 1

        # Final position check
        current_pos = self.px.get_position()
        dx = self.target_x - current_pos['x']
        dy = self.target_y - current_pos['y']
        final_distance = math.sqrt(dx * dx + dy * dy)

        if final_distance <= self.POSITION_TOLERANCE:
            self.state = NavigationState.ARRIVED
        else:
            # If we're not close enough, replan and try again
            self._plan_path()
            if len(self.path_segments) > 0:
                await self.execute_path()

    async def _execute_turn(self, angle):
        """Execute a point turn of the specified angle"""
        self.state = NavigationState.TURNING

        # Calculate turn time based on angle
        turn_time = abs(angle) / self.DEGREES_PER_SECOND

        # Set maximum steering angle in appropriate direction
        steering_angle = math.copysign(self.MAX_STEERING_ANGLE, angle)
        self.px.set_dir_servo_angle(steering_angle)

        # Start turn
        if angle > 0:
            self.px.forward(self.TURN_SPEED)
        else:
            self.px.backward(self.TURN_SPEED)

        # Wait for turn to complete
        await asyncio.sleep(turn_time)

        # Stop and center steering
        self.px.stop()
        self.px.set_dir_servo_angle(0)
        await asyncio.sleep(0.1)  # Short pause to stabilize

    async def _execute_forward(self, distance):
        """Execute forward movement for the specified distance"""
        self.state = NavigationState.MOVING

        # Calculate time needed based on speed
        estimated_speed = self.px._speed_to_cm_per_sec(self.FORWARD_SPEED)
        move_time = distance / estimated_speed

        # Move forward
        self.px.forward(self.FORWARD_SPEED)
        await asyncio.sleep(move_time)

        self.px.stop()
        await asyncio.sleep(0.1)  # Short pause to stabilize

    def get_state(self):
        """Get current navigation state"""
        return {
            'state': self.state.value,
            'target': (self.target_x, self.target_y),
            'current_segment': self.current_segment,
            'total_segments': len(self.path_segments),
            'position': self.px.get_position()
        }


# def main():
#     avoider = AsyncObstacleAvoidance()
#     try:
#         loop = asyncio.get_event_loop()
#         runner = loop.create_task(avoider.run())
#         loop.run_until_complete(runner)
#     except KeyboardInterrupt:
#         print("\nKeyboard interrupt received")
#         runner.cancel()
#         loop.run_until_complete(runner)
#     finally:
#         loop.close()

async def main():
    # Initialize tracking-enabled Picarx
    px = PicarXWrapper()
    navigator = PicarXNavigator(px)

    # Navigate to point (100, 100)
    navigator.set_target(100, 100)
    await navigator.execute_path()

    # Check final state
    state = navigator.get_state()
    print(f"Final position: {state['position']}")

if __name__ == "__main__":
    main()
