"""
NexusOS Cloud Fleet Control Backend.
Production-grade FastAPI server managing:
  - Multi-tenant robot registration and session management
  - Real-time WebSocket telemetry fan-out for dashboards and digital twins
  - Inbound edge agent WebSocket connections with license validation
  - RESTful command dispatching and mission waypoint API
  - In-memory fleet state with ring-buffer telemetry history
  - Agent Context Memory proxying
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
import uuid
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from nexusos.core.types import (
    TelemetryFrame,
    CommandFrame,
    DeviceHandshake,
    HandshakeResponse,
    RobotMode,
    Twist3D,
    Vector3D,
    LicenseTier,
    AgentContextBlock
)
from nexusos.licensing.verifier import LicenseVerifier, LicenseStatus

logger = logging.getLogger("NexusOS.CloudServer")

# ─────────────────────────────────────────────────────────────────────────────
# Application Bootstrap
# ─────────────────────────────────────────────────────────────────────────────

app = FastAPI(
    title="NexusOS Cloud Fleet Control",
    description="Universal Robotics OS — Cloud Fleet Orchestration, Telemetry Hub, & Agent Memory Gateway",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"]
)

# ─────────────────────────────────────────────────────────────────────────────
# Global Fleet State (Production: replace with Redis/PostgreSQL)
# ─────────────────────────────────────────────────────────────────────────────

# robot_id -> session dict
fleet_sessions: Dict[str, Dict[str, Any]] = {}

# robot_id -> last TelemetryFrame
fleet_state: Dict[str, TelemetryFrame] = {}

# robot_id -> deque of TelemetryFrames (ring buffer)
telemetry_history: Dict[str, Deque[TelemetryFrame]] = {}

# robot_id -> deque of AgentContextBlocks
context_history: Dict[str, Deque[AgentContextBlock]] = {}

# Active dashboard WebSocket clients
dashboard_clients: Set[WebSocket] = set()

# Active edge agent WebSocket connections (robot_id -> WebSocket)
edge_connections: Dict[str, WebSocket] = {}

# License verifier
verifier = LicenseVerifier()

# Server stats
server_start_time = time.time()
total_telemetry_frames: int = 0

TELEMETRY_HISTORY_DEPTH = 500


# ─────────────────────────────────────────────────────────────────────────────
# REST: Root & Health
# ─────────────────────────────────────────────────────────────────────────────

@app.get("/", include_in_schema=False)
async def root():
    return JSONResponse({
        "system": "NexusOS Cloud Fleet Control",
        "version": "1.0.0",
        "uptime_sec": round(time.time() - server_start_time, 1),
        "active_robots": len(fleet_state),
        "total_telemetry_frames": total_telemetry_frames,
        "docs": "/api/docs"
    })


@app.get("/api/v1/health")
async def health():
    return {
        "status": "OK",
        "timestamp": time.time(),
        "active_robots": len(fleet_state),
        "dashboard_clients": len(dashboard_clients)
    }


# ─────────────────────────────────────────────────────────────────────────────
# REST: Fleet Management
# ─────────────────────────────────────────────────────────────────────────────

@app.get("/api/v1/fleet")
async def list_fleet():
    """Return current state summary of all connected robots."""
    return {
        "active_robots_count": len(fleet_state),
        "robots": {
            rid: {
                "robot_id": rid,
                "robot_type": state.robot_type,
                "mode": state.mode,
                "status": state.status_message,
                "battery_pct": state.battery.percentage,
                "last_seen": state.timestamp_sec,
                "license_tier": fleet_sessions.get(rid, {}).get("license_tier", "COMMUNITY")
            }
            for rid, state in fleet_state.items()
        }
    }


@app.get("/api/v1/fleet/{robot_id}")
async def get_robot_state(robot_id: str):
    """Return full telemetry state for a specific robot."""
    if robot_id not in fleet_state:
        raise HTTPException(404, f"Robot '{robot_id}' not found in fleet.")
    return fleet_state[robot_id].model_dump()


@app.get("/api/v1/fleet/{robot_id}/telemetry/history")
async def get_telemetry_history(robot_id: str, limit: int = 50):
    """Return recent telemetry frame history for a robot."""
    if robot_id not in telemetry_history:
        raise HTTPException(404, f"No telemetry history for '{robot_id}'.")
    frames = list(telemetry_history[robot_id])[-limit:]
    return {"robot_id": robot_id, "count": len(frames), "frames": [f.model_dump() for f in frames]}


# ─────────────────────────────────────────────────────────────────────────────
# REST: Command Dispatch
# ─────────────────────────────────────────────────────────────────────────────

class NavGoalRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


class TwistCommandRequest(BaseModel):
    linear_x: float = 0.0
    angular_z: float = 0.0


@app.post("/api/v1/robots/{robot_id}/navigate")
async def navigate_robot(robot_id: str, goal: NavGoalRequest):
    """Send a navigation waypoint goal to a robot."""
    if robot_id not in edge_connections:
        raise HTTPException(503, f"Robot '{robot_id}' is not connected.")

    cmd = CommandFrame(
        command_id=f"nav_{uuid.uuid4().hex[:8]}",
        robot_id=robot_id,
        requested_mode=RobotMode.AUTONOMOUS,
        parameters={"nav_goal": {"x": goal.x, "y": goal.y, "z": goal.z, "yaw": goal.yaw}}
    )
    await _dispatch_command(robot_id, cmd)
    return {"status": "dispatched", "command_id": cmd.command_id, "goal": goal.model_dump()}


@app.post("/api/v1/robots/{robot_id}/twist")
async def send_twist(robot_id: str, twist: TwistCommandRequest):
    """Send manual velocity command to a robot."""
    if robot_id not in edge_connections:
        raise HTTPException(503, f"Robot '{robot_id}' is not connected.")

    cmd = CommandFrame(
        command_id=f"twist_{uuid.uuid4().hex[:8]}",
        robot_id=robot_id,
        requested_mode=RobotMode.MANUAL,
        target_twist=Twist3D(
            linear=Vector3D(x=twist.linear_x, y=0.0, z=0.0),
            angular=Vector3D(x=0.0, y=0.0, z=twist.angular_z)
        )
    )
    await _dispatch_command(robot_id, cmd)
    return {"status": "dispatched", "command_id": cmd.command_id}


@app.post("/api/v1/robots/{robot_id}/estop")
async def emergency_stop(robot_id: str):
    """Trigger emergency stop on a robot."""
    cmd = CommandFrame(
        command_id=f"estop_{uuid.uuid4().hex[:8]}",
        robot_id=robot_id,
        emergency_stop=True,
        requested_mode=RobotMode.EMERGENCY_STOP
    )
    await _dispatch_command(robot_id, cmd)
    return {"status": "EMERGENCY_STOP_DISPATCHED", "robot_id": robot_id}


@app.post("/api/v1/fleet/estop/global")
async def global_estop():
    """Trigger emergency stop on all connected robots simultaneously."""
    for robot_id in list(edge_connections.keys()):
        cmd = CommandFrame(
            command_id=f"gestop_{uuid.uuid4().hex[:8]}",
            robot_id=robot_id,
            emergency_stop=True,
            requested_mode=RobotMode.EMERGENCY_STOP
        )
        await _dispatch_command(robot_id, cmd)
    return {"status": "GLOBAL_ESTOP_DISPATCHED", "affected_robots": list(edge_connections.keys())}


async def _dispatch_command(robot_id: str, cmd: CommandFrame) -> None:
    """Internal: send a CommandFrame to the appropriate edge agent WebSocket."""
    ws = edge_connections.get(robot_id)
    if ws:
        try:
            await ws.send_text(cmd.model_dump_json())
        except Exception as exc:
            logger.warning(f"Failed to dispatch command to '{robot_id}': {exc}")
    # Always queue for dashboard fan-out too
    await _broadcast_to_dashboards({
        "type": "COMMAND_DISPATCHED",
        "robot_id": robot_id,
        "command_id": cmd.command_id
    })


# ─────────────────────────────────────────────────────────────────────────────
# REST: Licensing
# ─────────────────────────────────────────────────────────────────────────────

class LicenseIssueRequest(BaseModel):
    robot_id: str
    org_name: str
    tier: str = "COMMUNITY"
    annual_revenue_usd: float = 0.0
    valid_days: int = 365


@app.post("/api/v1/licensing/issue")
async def issue_license(req: LicenseIssueRequest):
    """Issue a signed NexusOS license token (admin endpoint)."""
    try:
        tier = LicenseTier(req.tier.upper())
    except ValueError:
        raise HTTPException(400, f"Invalid license tier: {req.tier}")

    token = verifier.issue_token(
        robot_id=req.robot_id,
        org_name=req.org_name,
        tier=tier,
        annual_revenue_usd=req.annual_revenue_usd,
        valid_days=req.valid_days
    )
    return {"token": token, "robot_id": req.robot_id, "tier": tier.value}


@app.post("/api/v1/licensing/verify")
async def verify_license(payload: Dict[str, str]):
    """Verify a NexusOS license token and return its status."""
    token = payload.get("token", "")
    info = verifier.verify(token)
    return {
        "status": info.status.value,
        "tier": info.tier.value,
        "grace_mode": info.grace_mode,
        "message": info.message,
        "robot_id": info.robot_id,
        "org_name": info.org_name
    }


# ─────────────────────────────────────────────────────────────────────────────
# REST: Agent Context Memory
# ─────────────────────────────────────────────────────────────────────────────

@app.get("/api/v1/fleet/{robot_id}/context")
async def get_latest_context(robot_id: str):
    """Return the latest AgentContextBlock for a robot."""
    history = context_history.get(robot_id)
    if not history:
        raise HTTPException(404, f"No context found for '{robot_id}'.")
    return history[-1].model_dump()


# ─────────────────────────────────────────────────────────────────────────────
# WebSocket: Edge Agent Connections (bi-directional streaming)
# ─────────────────────────────────────────────────────────────────────────────

@app.websocket("/ws/edge")
async def edge_websocket(websocket: WebSocket):
    """
    WebSocket endpoint for edge robot agents.
    Receives TelemetryFrames and sends CommandFrames.
    """
    global total_telemetry_frames

    await websocket.accept()
    robot_id: Optional[str] = None

    try:
        # Expect handshake as first message
        raw = await asyncio.wait_for(websocket.receive_text(), timeout=10.0)
        handshake = DeviceHandshake.model_validate_json(raw)
        robot_id = handshake.robot_id

        # Validate license
        license_info = verifier.verify(handshake.license_token)
        session_id = uuid.uuid4().hex
        fleet_sessions[robot_id] = {
            "session_id": session_id,
            "connected_at": time.time(),
            "license_tier": license_info.tier.value,
            "grace_mode": license_info.grace_mode
        }
        edge_connections[robot_id] = websocket
        telemetry_history[robot_id] = deque(maxlen=TELEMETRY_HISTORY_DEPTH)
        context_history[robot_id] = deque(maxlen=100)

        response = HandshakeResponse(
            accepted=True,
            session_id=session_id,
            active_tier=license_info.tier,
            grace_mode_active=license_info.grace_mode,
            error_reason="" if not license_info.grace_mode else license_info.message
        )
        await websocket.send_text(response.model_dump_json())

        logger.info(
            f"[{robot_id}] Edge agent connected. "
            f"Session={session_id[:8]}, Tier={license_info.tier.value}, "
            f"GraceMode={license_info.grace_mode}"
        )

        await _broadcast_to_dashboards({"type": "ROBOT_CONNECTED", "robot_id": robot_id})

        # Main telemetry loop
        while True:
            raw = await websocket.receive_text()
            try:
                frame = TelemetryFrame.model_validate_json(raw)
                fleet_state[robot_id] = frame
                telemetry_history[robot_id].append(frame)
                total_telemetry_frames += 1

                # Fan-out telemetry to all dashboard clients
                await _broadcast_fleet_update()

            except Exception:
                # Try parsing as context block
                try:
                    block = AgentContextBlock.model_validate_json(raw)
                    context_history[robot_id].append(block)
                except Exception:
                    logger.debug(f"[{robot_id}] Unrecognized message format.")

    except WebSocketDisconnect:
        pass
    except asyncio.TimeoutError:
        logger.warning("Edge agent handshake timed out.")
    except Exception as exc:
        logger.exception(f"Edge WebSocket error: {exc}")
    finally:
        if robot_id:
            edge_connections.pop(robot_id, None)
            fleet_state.pop(robot_id, None)
            fleet_sessions.pop(robot_id, None)
            logger.info(f"[{robot_id}] Edge agent disconnected.")
            await _broadcast_to_dashboards({"type": "ROBOT_DISCONNECTED", "robot_id": robot_id})


# ─────────────────────────────────────────────────────────────────────────────
# WebSocket: Dashboard Clients (real-time fleet state fan-out)
# ─────────────────────────────────────────────────────────────────────────────

@app.websocket("/ws/dashboard")
async def dashboard_websocket(websocket: WebSocket):
    """WebSocket endpoint for dashboard/digital-twin clients."""
    await websocket.accept()
    dashboard_clients.add(websocket)
    logger.info(f"Dashboard client connected. Total: {len(dashboard_clients)}")

    try:
        # Send initial fleet snapshot
        await websocket.send_text(json.dumps({
            "type": "INITIAL_STATE",
            "robots": {rid: f.model_dump() for rid, f in fleet_state.items()},
            "active_robots_count": len(fleet_state),
            "total_telemetry_frames": total_telemetry_frames,
            "server_time": time.time()
        }))

        # Handle inbound dashboard messages (commands, config changes, etc.)
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)
                action = msg.get("action")
                if action == "COMMAND":
                    cmd = CommandFrame.model_validate(msg.get("command", {}))
                    cmd = cmd.model_copy(update={"robot_id": msg.get("robot_id", cmd.robot_id)})
                    await _dispatch_command(cmd.robot_id, cmd)
                elif action == "GLOBAL_ESTOP":
                    for rid in list(edge_connections.keys()):
                        c = CommandFrame(
                            command_id=f"gestop_{uuid.uuid4().hex[:8]}",
                            robot_id=rid,
                            emergency_stop=True,
                            requested_mode=RobotMode.EMERGENCY_STOP
                        )
                        await _dispatch_command(rid, c)
            except Exception as exc:
                logger.debug(f"Dashboard message error: {exc}")

    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.exception(f"Dashboard WebSocket error: {exc}")
    finally:
        dashboard_clients.discard(websocket)
        logger.info(f"Dashboard client disconnected. Remaining: {len(dashboard_clients)}")


# ─────────────────────────────────────────────────────────────────────────────
# Internal Fan-out Utilities
# ─────────────────────────────────────────────────────────────────────────────

async def _broadcast_fleet_update() -> None:
    """Push full fleet state update to all dashboard WebSocket clients."""
    if not dashboard_clients:
        return
    payload = json.dumps({
        "type": "FLEET_UPDATE",
        "robots": {rid: f.model_dump() for rid, f in fleet_state.items()},
        "active_robots_count": len(fleet_state),
        "total_telemetry_frames": total_telemetry_frames,
        "server_time": time.time()
    })
    dead = set()
    for ws in dashboard_clients:
        try:
            await ws.send_text(payload)
        except Exception:
            dead.add(ws)
    dashboard_clients.difference_update(dead)


async def _broadcast_to_dashboards(event: Dict[str, Any]) -> None:
    """Push a single event dict to all dashboard clients."""
    if not dashboard_clients:
        return
    payload = json.dumps(event)
    dead = set()
    for ws in dashboard_clients:
        try:
            await ws.send_text(payload)
        except Exception:
            dead.add(ws)
    dashboard_clients.difference_update(dead)


# ─────────────────────────────────────────────────────────────────────────────
# Static Assets (3D Digital Twin UI)
# ─────────────────────────────────────────────────────────────────────────────

import os
_static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.isdir(_static_dir):
    app.mount("/", StaticFiles(directory=_static_dir, html=True), name="static")


# ─────────────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import uvicorn
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    uvicorn.run("nexusos.server.main:app", host="0.0.0.0", port=8000, reload=False, workers=1)
