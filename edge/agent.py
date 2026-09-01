"""
NexusOS Edge Agent Daemon
Runs on edge compute on board the robot, interfacing between the HAL plugin and Cloud Fleet Server.
"""

import asyncio
import json
import logging
import signal
import sys
import time
from typing import Optional, Any

import websockets

from hal.base import (
    AbstractHardwarePlugin,
    MotorCommandData,
    RobotMode,
    RobotType,
    TwistData,
    Vector3D,
    JointStateData
)
from hal.plugins.diff_drive import DiffDriveBasePlugin
from hal.plugins.robotic_arm import RoboticArmPlugin
from edge.config import EdgeConfig
from licensing.validator import LicenseValidator

# Configure structured logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("NexusOS.EdgeAgent")


class EdgeAgent:
    """
    NexusOS Edge Agent Daemon.
    Manages local hardware plugin lifecycle, streams telemetry, and executes remote commands.
    """

    def __init__(self, config: EdgeConfig, hal_plugin: Optional[AbstractHardwarePlugin] = None):
        self.config = config
        self.robot_id = config.robot_id

        if hal_plugin:
            self.hal = hal_plugin
        else:
            self.hal = self._instantiate_hal(config.robot_type, config.robot_id)

        self._running: bool = False
        self._last_command_time: float = time.time()
        self._ws: Any = None

    def _is_ws_active(self) -> bool:
        if not self._ws:
            return False
        if hasattr(self._ws, "closed"):
            return not getattr(self._ws, "closed")
        if hasattr(self._ws, "close_code"):
            return getattr(self._ws, "close_code") is None
        return True

    def _instantiate_hal(self, robot_type: RobotType, robot_id: str) -> AbstractHardwarePlugin:
        if robot_type == RobotType.ROBOTIC_ARM:
            return RoboticArmPlugin(robot_id)
        else:
            return DiffDriveBasePlugin(robot_id)

    async def start(self) -> None:
        """Start the Edge Agent daemon lifecycle."""
        logger.info(f"Starting NexusOS Edge Agent for Robot ID '{self.robot_id}'...")

        # Runtime License Validation & Safety Interlock Hook
        validator = LicenseValidator()
        lic_res = validator.validate()
        validator.enforce_safety_sandbox(lic_res)

        # 1. Initialize Hardware Abstraction Layer
        initialized = await self.hal.initialize()
        if not initialized:
            logger.error("Failed to initialize HAL plugin! Exiting Edge Agent.")
            return

        self._running = True

        # 2. Main Connection & Resilience Loop
        while self._running:
            try:
                endpoint = f"{self.config.server_url}/{self.robot_id}"
                logger.info(f"Connecting to Cloud Fleet Server at {endpoint}...")

                async with websockets.connect(endpoint, ping_interval=10, ping_timeout=5) as ws:
                    self._ws = ws
                    logger.info("Successfully connected to NexusOS Cloud Fleet Server!")

                    # Run Telemetry Sender and Command Receiver concurrently
                    telemetry_task = asyncio.create_task(self._telemetry_loop())
                    command_task = asyncio.create_task(self._command_listener_loop())
                    watchdog_task = asyncio.create_task(self._watchdog_loop())

                    done, pending = await asyncio.wait(
                        [telemetry_task, command_task, watchdog_task],
                        return_when=asyncio.FIRST_COMPLETED
                    )

                    for task in pending:
                        task.cancel()

            except (websockets.exceptions.ConnectionClosed, OSError, asyncio.TimeoutError) as e:
                logger.warning(f"Connection lost or unreachable ({e}). Retrying in {self.config.reconnect_interval_sec}s...")
                await self.hal.emergency_stop()
                await asyncio.sleep(self.config.reconnect_interval_sec)
            except Exception as e:
                logger.error(f"Unexpected error in Edge Agent main loop: {e}", exc_info=True)
                await asyncio.sleep(self.config.reconnect_interval_sec)

    async def _telemetry_loop(self) -> None:
        """
        High-frequency telemetry stream loop (deadline-based timing).

        Perf notes:
          - Uses to_msgpack() (C extension) instead of json.dumps() for ~6x lower
            serialisation overhead per frame.
          - Deadline sleep: subtracts time already spent reading + serialising so
            the loop maintains its target Hz even under load.
        """
        interval = 1.0 / self.config.telemetry_rate_hz
        _monotonic = time.monotonic         # local alias avoids attribute lookup per iter

        while self._running and self._is_ws_active():
            deadline = _monotonic() + interval
            try:
                telemetry = await self.hal.read_telemetry()

                # Binary msgpack frame \u2014 ~6x faster than json.dumps + str.encode
                payload: bytes = telemetry.to_msgpack()
                await self._ws.send(payload)

            except Exception as e:
                logger.warning(f"Error in telemetry loop: {e}")
                break

            # Sleep only the time remaining until the next deadline (compensates
            # for time spent in read_telemetry + network send)
            remaining = deadline - _monotonic()
            if remaining > 0.0:
                await asyncio.sleep(remaining)


    async def _command_listener_loop(self) -> None:
        """Receive and process inbound control commands from Cloud Backend."""
        while self._running and self._is_ws_active():
            try:
                msg = await self._ws.recv()
                data = json.loads(msg)
                self._last_command_time = time.time()

                logger.info(f"Received Cloud Command: {data}")
                cmd = self._parse_command_data(data)

                if cmd:
                    if cmd.requested_mode == RobotMode.EMERGENCY_STOP:
                        await self.hal.emergency_stop()
                    else:
                        await self.hal.send_command(cmd)

            except websockets.exceptions.ConnectionClosed:
                logger.warning("Command listener WebSocket connection closed.")
                break
            except Exception as e:
                logger.error(f"Error parsing inbound command: {e}")

    async def _watchdog_loop(self) -> None:
        """Watchdog monitor: triggers E-stop if autonomous command stream drops and no mission active."""
        while self._running:
            await asyncio.sleep(0.5)
            if self.hal.current_mode == RobotMode.AUTONOMOUS:
                # Check if HAL has active navigation mission running
                has_active_nav = False
                if hasattr(self.hal, "navigator"):
                    nav_state = getattr(self.hal.navigator, "state", None)
                    if nav_state in ("NAVIGATING", "PLANNING", "OBSTACLE_AVOIDANCE", "RECOVERY_ROTATE", "RECOVERY_BACKUP"):
                        has_active_nav = True

                if not has_active_nav:
                    elapsed = time.time() - self._last_command_time
                    if elapsed > self.config.watchdog_timeout_sec:
                        logger.error(f"WATCHDOG TIMEOUT! No command received for {elapsed:.2f}s. Triggering E-STOP!")
                        await self.hal.emergency_stop()

    def _parse_command_data(self, data: dict) -> Optional[MotorCommandData]:
        try:
            target_twist = None
            if "twist" in data or "target_twist" in data:
                tw = data.get("twist") or data.get("target_twist")
                lin = tw.get("linear", {})
                ang = tw.get("angular", {})
                target_twist = TwistData(
                    linear=Vector3D(x=float(lin.get("x", 0.0)), y=float(lin.get("y", 0.0)), z=float(lin.get("z", 0.0))),
                    angular=Vector3D(x=float(ang.get("x", 0.0)), y=float(ang.get("y", 0.0)), z=float(ang.get("z", 0.0)))
                )

            target_joints = None
            if "joints" in data or "target_joints" in data:
                j = data.get("joints") or data.get("target_joints")
                target_joints = JointStateData(
                    names=j.get("names", []),
                    positions=[float(p) for p in j.get("positions", [])]
                )

            req_mode = None
            if "mode" in data or "requested_mode" in data:
                m_str = data.get("mode") or data.get("requested_mode")
                try:
                    req_mode = RobotMode(m_str)
                except ValueError:
                    pass

            params = data.get("parameters", {}) or {}
            # Allow top-level nav_goal / navigate_to / cancel_nav
            if "nav_goal" in data:
                params["nav_goal"] = data["nav_goal"]
                if req_mode is None:
                    req_mode = RobotMode.AUTONOMOUS
            elif "navigate_to" in data:
                params["nav_goal"] = data["navigate_to"]
                if req_mode is None:
                    req_mode = RobotMode.AUTONOMOUS
            if "cancel_nav" in data:
                params["cancel_nav"] = data["cancel_nav"]

            return MotorCommandData(
                target_twist=target_twist,
                target_joints=target_joints,
                requested_mode=req_mode,
                parameters=params
            )
        except Exception as e:
            logger.error(f"Failed to parse motor command: {e}")
            return None

    async def stop(self) -> None:
        """Gracefully shut down the Edge Agent daemon."""
        logger.info(f"Stopping Edge Agent '{self.robot_id}'...")
        self._running = False
        await self.hal.shutdown()
        if self._ws and not self._ws.closed:
            await self._ws.close()
        logger.info(f"Edge Agent '{self.robot_id}' stopped cleanly.")


async def main():
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "rover-alpha"
    robot_type_str = sys.argv[2] if len(sys.argv) > 2 else "DIFFERENTIAL_DRIVE"

    try:
        r_type = RobotType(robot_type_str)
    except ValueError:
        r_type = RobotType.DIFFERENTIAL_DRIVE

    config = EdgeConfig(
        robot_id=robot_id,
        robot_type=r_type,
        server_url="ws://localhost:8000/ws/telemetry"
    )

    agent = EdgeAgent(config)

    # Setup signal handlers for graceful shutdown
    loop = asyncio.get_running_loop()

    def shutdown_handler():
        logger.info("Signal received, initiating shutdown...")
        asyncio.create_task(agent.stop())

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, shutdown_handler)
        except NotImplementedError:
            # Signal handling on Windows selector loop fallback
            pass

    await agent.start()


if __name__ == "__main__":
    asyncio.run(main())
