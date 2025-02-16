import asyncio
from object_sensors import AsyncObstacleAvoidance

async def main():
    """Main function to start goal-based navigation"""
    robot = AsyncObstacleAvoidance()

    # Start the background tasks
    control_task = asyncio.create_task(robot.run())

    try:
        # Define waypoints
        waypoints = [
            (50, 0),  # Move 50cm forward
            (50, 50),  # Turn right and move 50cm
            (0, 50),  # Return to starting y but at x=0
            (0, 0)  # Return to start
        ]

        # Navigate through waypoints
        for goal_x, goal_y in waypoints:
            print(f"\nNavigating to goal: ({goal_x}, {goal_y})")
            await robot.navigate_to_goal(goal_x, goal_y)

            # Get current position after reaching waypoint
            position = robot.px.get_position()
            print(f"Current position: ({position['x']:.1f}, {position['y']:.1f})")
            print(f"Current heading: {position['heading']:.1f}°")

            # Optional pause between waypoints
            await asyncio.sleep(1.0)

    except KeyboardInterrupt:
        print("\nNavigation interrupted by user")
    finally:
        # Cleanup
        control_task.cancel()
        try:
            await control_task
        except asyncio.CancelledError:
            pass


if __name__ == "__main__":
    asyncio.run(main())

