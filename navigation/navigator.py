import asyncio
from object_sensors import AsyncObstacleAvoidance
from enum import Enum

def main():
    avoider = AsyncObstacleAvoidance()
    try:
        loop = asyncio.get_event_loop()
        runner = loop.create_task(avoider.run())
        loop.run_until_complete(runner)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
        runner.cancel()
        loop.run_until_complete(runner)
    finally:
        loop.close()

# async def main():
#     px = PicarXWrapper()
#     await px.navigate_to_point(50, 0)  # Forward 50cm
#     await px.navigate_to_point(50, 50)  # Right 50cm
#     await px.navigate_to_point(0, 50)  # Back 100cm
#     await px.navigate_to_point(0, 0)  # Left 100cm
#     pos = px.get_position()
#     print(f"Final position: x={pos['x']}, y={pos['y']}, heading={pos['heading']}")
#
# if __name__ == "__main__":
#     asyncio.run(main())
