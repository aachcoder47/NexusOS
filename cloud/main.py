"""
NexusOS Cloud Fleet Management FastAPI Server
Provides REST APIs, WebSocket streaming endpoints for Edge Agents and Fleet Dashboard.
"""

import asyncio
import json
import logging
import os
from typing import Dict, Any, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from cloud.state import FleetStateManager
from cloud.licensing_api import router as licensing_router
from hal.base import RobotTelemetryData

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("NexusOS.CloudServer")

app = FastAPI(
    title="NexusOS Fleet Management Middleware",
    description="Hardware-agnostic, real-time robotics fleet orchestration & telemetry platform.",
    version="1.0.0"
)

# Include licensing API router
app.include_router(licensing_router)

# Enable CORS for cross-origin dashboard access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Singleton state manager instance
fleet_state = FleetStateManager()

# Static files path for Dashboard UI
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


# Pydantic schema for REST Command Request
class Vector3Schema(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class TwistSchema(BaseModel):
    linear: Vector3Schema = Vector3Schema()
    angular: Vector3Schema = Vector3Schema()


class JointTargetSchema(BaseModel):
    names: list[str] = []
    positions: list[float] = []


class CommandRequestSchema(BaseModel):
    requested_mode: Optional[str] = None
    twist: Optional[TwistSchema] = None
    joints: Optional[JointTargetSchema] = None
    parameters: Optional[Dict[str, Any]] = None


# --- Web Page Route ---

@app.get("/", response_class=HTMLResponse)
async def serve_dashboard():
    index_file = os.path.join(STATIC_DIR, "index.html")
    if os.path.exists(index_file):
        with open(index_file, "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    return HTMLResponse("<h2>NexusOS Server Running. Static dashboard missing.</h2>")


# --- REST API Endpoints ---

@app.get("/api/v1/health")
async def health_check():
    return {
        "status": "HEALTHY",
        "service": "NexusOS Cloud Fleet Server",
        "version": "1.0.0"
    }


@app.get("/api/v1/robots")
async def get_robots_summary():
    """Retrieve current list of active robots and their telemetry status."""
    return fleet_state.get_fleet_summary()


@app.get("/api/v1/robots/{robot_id}")
async def get_robot_telemetry(robot_id: str):
    """Retrieve detailed telemetry snapshot for a specific robot."""
    telemetry = fleet_state._fleet_telemetry.get(robot_id)
    if not telemetry:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot '{robot_id}' not found or no telemetry reported."
        )
    return telemetry


@app.post("/api/v1/robots/{robot_id}/command")
async def send_robot_command(robot_id: str, cmd: CommandRequestSchema):
    """Dispatch control command to a specific edge robot."""
    command_dict = cmd.model_dump(exclude_unset=True)
    success = await fleet_state.dispatch_command(robot_id, command_dict)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Could not send command to robot '{robot_id}'. Is it connected?"
        )
    return {"status": "DISPATCHED", "robot_id": robot_id, "command": command_dict}


class NavGoalSchema(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class ObstacleSchema(BaseModel):
    obstacle_id: str
    x: float
    y: float
    z: float = 0.5
    width: float = 1.0
    length: float = 1.0
    height: float = 1.0
    is_dynamic: bool = False


# In-memory shared physics environment state for fleet
PHYSICS_OBSTACLES: Dict[str, Dict[str, Any]] = {
    "crate_1": {"obstacle_id": "crate_1", "x": 2.0, "y": 1.5, "z": 0.5, "width": 1.0, "length": 1.0, "height": 1.0, "is_dynamic": False},
    "crate_2": {"obstacle_id": "crate_2", "x": -2.5, "y": 3.0, "z": 0.5, "width": 1.2, "length": 0.8, "height": 1.0, "is_dynamic": False},
    "pillar_1": {"obstacle_id": "pillar_1", "x": 0.0, "y": 4.0, "z": 1.0, "width": 0.6, "length": 0.6, "height": 2.0, "is_dynamic": False},
}


@app.post("/api/v1/robots/{robot_id}/navigate")
async def send_robot_nav_goal(robot_id: str, goal: NavGoalSchema):
    """Dispatch autonomous navigation target waypoint to edge robot."""
    command_dict = {
        "mode": "AUTONOMOUS",
        "parameters": {
            "nav_goal": {"x": goal.x, "y": goal.y, "yaw": goal.yaw}
        }
    }
    success = await fleet_state.dispatch_command(robot_id, command_dict)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Could not send navigation goal to robot '{robot_id}'. Is it connected?"
        )
    return {"status": "NAVIGATION_DISPATCHED", "robot_id": robot_id, "goal": goal.model_dump()}


@app.post("/api/v1/robots/{robot_id}/cancel_nav")
async def cancel_robot_navigation(robot_id: str):
    """Cancel active navigation mission on edge robot."""
    command_dict = {
        "parameters": {
            "cancel_nav": True
        }
    }
    success = await fleet_state.dispatch_command(robot_id, command_dict)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Could not cancel navigation for robot '{robot_id}'."
        )
    return {"status": "NAVIGATION_CANCELLED", "robot_id": robot_id}


@app.get("/api/v1/physics/world")
async def get_physics_world():
    """Retrieve current physical world obstacles and gravity configuration."""
    return {
        "gravity": {"x": 0.0, "y": 0.0, "z": -9.81},
        "obstacles": list(PHYSICS_OBSTACLES.values())
    }


@app.post("/api/v1/physics/obstacle")
async def add_physics_obstacle(obs: ObstacleSchema):
    """Add a new physical obstacle to the shared simulation environment."""
    PHYSICS_OBSTACLES[obs.obstacle_id] = obs.model_dump()
    return {"status": "OBSTACLE_ADDED", "obstacle": obs.model_dump()}


@app.post("/api/v1/fleet/estop")
async def emergency_stop_fleet():
    """Trigger Global Emergency Stop across all connected robots in fleet."""
    stopped_count = await fleet_state.emergency_stop_all()
    return {
        "status": "GLOBAL_EMERGENCY_STOP_EXECUTED",
        "robots_notified_count": stopped_count
    }


# --- WebSockets Endpoint for Edge Agents ---

@app.websocket("/ws/telemetry/{robot_id}")
async def websocket_edge_agent_telemetry(websocket: WebSocket, robot_id: str):
    """
    WebSocket endpoint for Edge Agent daemons to stream telemetry
    and receive control commands.
    """
    await websocket.accept()
    await fleet_state.register_robot(robot_id, websocket)

    try:
        while True:
            message = await websocket.receive()
            if "bytes" in message and message["bytes"] is not None:
                try:
                    telemetry_obj = RobotTelemetryData.from_msgpack(message["bytes"])
                    await fleet_state.update_robot_telemetry(robot_id, telemetry_obj.to_dict())
                except Exception as e:
                    logger.warning(f"Error parsing binary telemetry from '{robot_id}': {e}")
            elif "text" in message and message["text"] is not None:
                try:
                    telemetry = json.loads(message["text"])
                    await fleet_state.update_robot_telemetry(robot_id, telemetry)
                except json.JSONDecodeError:
                    logger.warning(f"Malformed JSON telemetry received from '{robot_id}'")
            elif message.get("type") == "websocket.disconnect":
                break
    except (WebSocketDisconnect, asyncio.CancelledError):
        logger.info(f"Edge agent WebSocket disconnected: '{robot_id}'")
    except Exception as e:
        logger.error(f"Error handling Edge WebSocket for '{robot_id}': {e}")
    finally:
        await fleet_state.unregister_robot(robot_id)


# --- WebSockets Endpoint for Front-end Dashboards ---

@app.websocket("/ws/dashboard")
async def websocket_dashboard_subscriber(websocket: WebSocket):
    """
    WebSocket endpoint for front-end visualizer dashboards to receive
    live multi-robot telemetry broadcasts.
    """
    await websocket.accept()
    await fleet_state.register_dashboard(websocket)

    try:
        while True:
            # Listen for optional incoming commands from UI dashboard over socket
            message = await websocket.receive()
            if message.get("type") == "websocket.disconnect":
                break
            if "text" in message and message["text"] is not None:
                try:
                    msg = json.loads(message["text"])
                    if msg.get("action") == "COMMAND" and "robot_id" in msg:
                        target_robot = msg["robot_id"]
                        cmd_payload = msg.get("command", {})
                        await fleet_state.dispatch_command(target_robot, cmd_payload)
                    elif msg.get("action") == "GLOBAL_ESTOP":
                        await fleet_state.emergency_stop_all()
                except json.JSONDecodeError:
                    pass

    except (WebSocketDisconnect, asyncio.CancelledError):
        logger.info("Dashboard WebSocket client disconnected.")
    except Exception as e:
        logger.error(f"Dashboard WebSocket error: {e}")
    finally:
        await fleet_state.unregister_dashboard(websocket)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("cloud.main:app", host="0.0.0.0", port=8000, reload=True)
