import asyncio
from object_sensors import AsyncObstacleAvoidance

async def main():
    # Create and run the obstacle avoidance system with a goal
    controller = AsyncObstacleAvoidance()

    # Set goal coordinates (in cm)
    goal_x = 100  # 1 meter forward
    goal_y = 50  # 0.5 meters right

    try:
        # Run the system with goal coordinates
        await controller.run(goal_x, goal_y)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        # Ensure cleanup
        controller.px.stop()
        controller.vision.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
