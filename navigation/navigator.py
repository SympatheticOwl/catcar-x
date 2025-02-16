import asyncio
from object_sensors import AsyncObstacleAvoidance
from picarx_wrapper import PicarXWrapper

async def main():
    # Initialize the wrapper and obstacle avoidance system
    px_wrapper = PicarXWrapper()
    obstacle_system = AsyncObstacleAvoidance(px_wrapper)

    # Start the background tasks
    system_task = asyncio.create_task(obstacle_system.run())

    try:
        # Example navigation sequence
        goals = [
            (50, 0),  # Move 50cm forward
            (50, 50),  # Turn right and move to point
            (0, 50),  # Move left
            (0, 0)  # Return to start
        ]

        for goal_x, goal_y in goals:
            print(f"\nNavigating to goal: ({goal_x}, {goal_y})")
            await obstacle_system.navigate_to_goal(goal_x, goal_y)
            current_pos = px_wrapper.get_position()
            print(f"Current position: ({current_pos['x']:.1f}, {current_pos['y']:.1f})")
            await asyncio.sleep(1)  # Pause between goals

    except KeyboardInterrupt:
        print("\nNavigation interrupted by user")
    finally:
        # Cleanup
        system_task.cancel()
        try:
            await system_task
        except asyncio.CancelledError:
            pass


if __name__ == "__main__":
    asyncio.run(main())
