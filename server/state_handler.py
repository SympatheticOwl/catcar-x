import asyncio

from object_system import ObjectSystem
from enum import Enum

def main():
    avoider = ObjectSystem()
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

if __name__ == "__main__":
    main()

