from fastapi import FastAPI, HTTPException
from typing import Dict, Optional
import uvicorn
import asyncio
from commands import Commands  # Assuming commands.py contains the Commands class

app = FastAPI()

# Create a single instance of Commands to be used across all requests
command_instance = Commands()
command_instance.__post_init__()  # Initialize the monitoring tasks


@app.post("/command/{cmd}")
async def execute_command(cmd: str) -> Dict[str, str]:
    """
    Execute a command on the PicarX.
    Available commands: forward, stop, scan, see
    """
    try:
        if cmd == "forward":
            success = command_instance.forward()
            if not success:
                raise HTTPException(status_code=400, detail="Cannot move forward due to hazard")
            return {"status": "success", "message": "Moving forward"}

        elif cmd == "stop":
            command_instance.cancel_movement()
            return {"status": "success", "message": "Stopped movement"}

        elif cmd == "scan":
            command_instance.scan_env()
            return {"status": "success", "message": "Scanning environment"}

        elif cmd == "see":
            command_instance.start_vision()
            # Wait briefly for initial vision processing
            await asyncio.sleep(0.5)
            objects = command_instance.get_objects()
            return {
                "status": "success",
                "message": "Vision system started",
                "objects": objects
            }

        else:
            raise HTTPException(
                status_code=400,
                detail=f"Unknown command: {cmd}. Available commands: forward, stop, scan, see"
            )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/status")
async def get_status() -> Dict[str, Optional[float]]:
    """Get the current status of the PicarX"""
    return {
        "object_distance": command_instance.get_object_distance(),
        "emergency_stop": command_instance.state.emergency_stop_flag
    }


def main():
    """Run the server"""
    try:
        uvicorn.run(app, host="10.0.0.219", port=8000)
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        # Cleanup any running tasks
        if command_instance.state.vision_task:
            command_instance.stop_vision()
        command_instance.cancel_movement()


if __name__ == "__main__":
    main()