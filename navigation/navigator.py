import asyncio

from picarx_wrapper import PicarXWrapper
from object_sensors import AsyncObstacleAvoidance

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
#
#
# if __name__ == "__main__":
#     main()


async def main():
    robot = PicarXWrapper()

    # Move in a square pattern
    for _ in range(4):
        await robot.move_distance(2.0)  # Move forward 2 feet
        robot.turn_to_heading(robot.heading + 90)  # Turn 90 degrees right

    # Return to start
    await robot.move_to_position(0, 0)
    robot.turn_to_heading(0)

    print(f"Final position: {robot.get_position()}")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
