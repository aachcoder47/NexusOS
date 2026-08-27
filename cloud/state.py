"""
NexusOS Cloud Fleet Management State & Broadcast Manager
Maintains in-memory live state of active robots, WebSocket connections, and real-time streaming.
"""

import asyncio
import json
import logging
import time
from typing import Dict, Set, Any, Optional

from fastapi import WebSocket

logger = logging.getLogger("NexusOS.CloudState")


class FleetStateManager:
    """
    Centralized thread-safe Fleet State Store.
    Manages active robot telemetry, edge agent sockets, and dashboard broadcast streams.
    """

    def __init__(self):
        # Active robot state map: robot_id -> latest telemetry dictionary
        self._fleet_telemetry: Dict[str, Dict[str, Any]] = {}

        # Connection timestamps: robot_id -> float
        self._last_seen: Dict[str, float] = {}

        # Edge Agent WebSockets: robot_id -> WebSocket
        self._robot_sockets: Dict[str, WebSocket] = {}

        # Front-end Dashboard WebSockets: set of WebSockets
        self._dashboard_sockets: Set[WebSocket] = set()

        # Telemetry message counter for metrics
        self.total_telemetry_count: int = 0
        self.start_time: float = time.time()

    # --- Edge Robot Socket Handlers ---

    async def register_robot(self, robot_id: str, websocket: WebSocket) -> None:
        """Register a connected Edge Agent robot socket."""
        self._robot_sockets[robot_id] = websocket
        self._last_seen[robot_id] = time.time()
        logger.info(f"Fleet State: Registered active Robot ID '{robot_id}'. Total active robots: {len(self._robot_sockets)}")

    async def unregister_robot(self, robot_id: str) -> None:
        """Remove a disconnected Edge Agent robot socket."""
        if robot_id in self._robot_sockets:
            del self._robot_sockets[robot_id]
        if robot_id in self._fleet_telemetry:
            self._fleet_telemetry[robot_id]["status_message"] = "DISCONNECTED"
            self._fleet_telemetry[robot_id]["mode"] = "FAULT"

        logger.info(f"Fleet State: Unregistered Robot ID '{robot_id}'. Remaining robots: {len(self._robot_sockets)}")

        # Notify dashboard subscriber clients of disconnection
        await self.broadcast_fleet_update()

    async def update_robot_telemetry(self, robot_id: str, telemetry: Dict[str, Any]) -> None:
        """Store incoming telemetry frame and broadcast to dashboard subscribers."""
        self._fleet_telemetry[robot_id] = telemetry
        self._last_seen[robot_id] = time.time()
        self.total_telemetry_count += 1

        # Broadcast update to connected dashboard subscribers
        await self.broadcast_fleet_update()

    # --- Dashboard Socket Handlers ---

    async def register_dashboard(self, websocket: WebSocket) -> None:
        """Register a front-end UI dashboard subscriber."""
        self._dashboard_sockets.add(websocket)
        logger.info(f"Fleet State: Registered Dashboard client. Total clients: {len(self._dashboard_sockets)}")
        # Immediately send current state on connect
        await self.send_full_state(websocket)

    async def unregister_dashboard(self, websocket: WebSocket) -> None:
        """Remove a front-end UI dashboard subscriber."""
        self._dashboard_sockets.discard(websocket)
        logger.info(f"Fleet State: Unregistered Dashboard client. Remaining: {len(self._dashboard_sockets)}")

    async def broadcast_fleet_update(self) -> None:
        """Broadcast latest fleet summary state to all connected UI dashboards."""
        if not self._dashboard_sockets:
            return

        payload = json.dumps({
            "type": "FLEET_UPDATE",
            "timestamp": time.time(),
            "active_robots_count": len(self._robot_sockets),
            "total_telemetry_frames": self.total_telemetry_count,
            "robots": self._fleet_telemetry
        })

        disconnected_clients = set()
        for ws in list(self._dashboard_sockets):
            try:
                await ws.send_text(payload)
            except Exception:
                disconnected_clients.add(ws)

        for ws in disconnected_clients:
            self._dashboard_sockets.discard(ws)

    async def send_full_state(self, websocket: WebSocket) -> None:
        payload = json.dumps({
            "type": "INITIAL_STATE",
            "timestamp": time.time(),
            "active_robots_count": len(self._robot_sockets),
            "robots": self._fleet_telemetry
        })
        try:
            await websocket.send_text(payload)
        except Exception as e:
            logger.warning(f"Failed to send initial state to dashboard: {e}")

    # --- Command Dispatch ---

    async def dispatch_command(self, robot_id: str, command: Dict[str, Any]) -> bool:
        """Send a JSON control command to a specific Edge Agent robot."""
        ws = self._robot_sockets.get(robot_id)
        if not ws:
            logger.warning(f"Cannot dispatch command: Robot '{robot_id}' is not connected!")
            return False

        try:
            payload = json.dumps(command)
            await ws.send_text(payload)
            logger.info(f"Dispatched command to Robot '{robot_id}': {command}")
            return True
        except Exception as e:
            logger.error(f"Failed to send command to Robot '{robot_id}': {e}")
            return False

    async def emergency_stop_all(self) -> int:
        """Broadcast emergency stop command to all connected edge robots."""
        e_stop_cmd = {
            "requested_mode": "EMERGENCY_STOP",
            "twist": {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}
        }
        count = 0
        for robot_id in list(self._robot_sockets.keys()):
            if await self.dispatch_command(robot_id, e_stop_cmd):
                count += 1
        return count

    def get_fleet_summary(self) -> Dict[str, Any]:
        """Return REST API snapshot summary."""
        now = time.time()
        robots_summary = {}
        for r_id, t_data in self._fleet_telemetry.items():
            is_alive = (now - self._last_seen.get(r_id, 0)) < 3.0
            robots_summary[r_id] = {
                "robot_id": r_id,
                "robot_type": t_data.get("robot_type"),
                "mode": t_data.get("mode"),
                "is_online": is_alive and (r_id in self._robot_sockets),
                "battery_pct": t_data.get("battery", {}).get("percentage", 0),
                "pose": t_data.get("pose"),
                "last_seen_sec_ago": round(now - self._last_seen.get(r_id, 0), 2)
            }

        return {
            "uptime_seconds": round(now - self.start_time, 1),
            "total_telemetry_frames": self.total_telemetry_count,
            "connected_robots_count": len(self._robot_sockets),
            "dashboard_clients_count": len(self._dashboard_sockets),
            "robots": robots_summary
        }
