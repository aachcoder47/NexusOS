"""
NexusOS Asynchronous Edge Daemon.
High-frequency edge agent that:
  - Runs a microsecond-isolated HAL control loop at ~50Hz
  - Checks licensing state and enforces grace-mode speed caps
  - Streams telemetry to the cloud via WebSocket with automatic exponential backoff
  - Accepts and dispatches CommandFrames from cloud to HAL
  - Maintains local offline telemetry buffer when cloud connectivity is lost
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
import uuid
from collections import deque
from typing import Deque, Dict, Optional, Any

try:
    import websockets
    from websockets.exceptions import ConnectionClosed
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

from nexusos.core.hal import BaseRobotDriver
from nexusos.core.types import TelemetryFrame, CommandFrame, RobotMode
from nexusos.memory.engine import AgentContextEngine
from nexusos.licensing.verifier import LicenseVerifier, LicenseTier, LicenseStatus

logger = logging.getLogger("NexusOS.EdgeDaemon")


class EdgeDaemon:
    """
    Async Edge Daemon managing hardware control loops, license enforcement,
    and resilient cloud telemetry streaming.

    Architecture:
      - Task A: HAL polling loop at HAL_POLL_HZ (safety-isolated, always runs)
      - Task B: Cloud WebSocket streamer (best-effort, auto-reconnecting)
      - Task C: Inbound command consumer (dispatches cloud commands to HAL)
      - Task D: Local memory ingestion (feeds AgentContextEngine)
    """

    HAL_POLL_HZ: float = 50.0       # Frequency of hardware polling
    STREAM_HZ: float = 10.0         # Cloud telemetry push frequency
    OFFLINE_BUFFER_MAX: int = 500   # Max frames buffered offline

    def __init__(
        self,
        robot_id: str,
        driver: BaseRobotDriver,
        license_token: str = "",
        cloud_ws_url: str = "ws://localhost:8000/ws/edge",
        agent_context: Optional[AgentContextEngine] = None
    ):
        self.robot_id = robot_id
        self.driver = driver
        self.license_token = license_token
        self.cloud_ws_url = cloud_ws_url

        # Licensing
        self.license_verifier = LicenseVerifier()
        self._license_info = self.license_verifier.verify(self.license_token)
        self._grace_mode = self._license_info.grace_mode

        # Agent Context Memory
        self.context: AgentContextEngine = agent_context or AgentContextEngine(robot_id=robot_id)

        # Internal state
        self._running: bool = False
        self._ws: Optional[Any] = None
        self._ws_connected: bool = False
        self._offline_buffer: Deque[TelemetryFrame] = deque(maxlen=self.OFFLINE_BUFFER_MAX)
        self._command_queue: asyncio.Queue[CommandFrame] = asyncio.Queue(maxsize=64)
        self._latest_telemetry: Optional[TelemetryFrame] = None
        self._telemetry_count: int = 0
        self._reconnect_delay: float = 1.0  # Exponential backoff base
        self._max_reconnect_delay: float = 60.0

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    async def start(self) -> None:
        """Start all daemon tasks concurrently."""
        if not self.driver.is_initialized:
            ok = await self.driver.initialize()
            if not ok:
                logger.critical(f"[{self.robot_id}] Driver initialization failed. Aborting daemon start.")
                return

        self._running = True
        logger.info(
            f"[{self.robot_id}] EdgeDaemon starting. License={self._license_info.status.value}, "
            f"GraceMode={self._grace_mode}, CloudURL={self.cloud_ws_url}"
        )

        await asyncio.gather(
            self._hal_loop(),
            self._cloud_stream_loop(),
            self._command_dispatch_loop(),
            return_exceptions=True
        )

    async def stop(self) -> None:
        """Gracefully stop all loops and shut down the driver."""
        logger.info(f"[{self.robot_id}] EdgeDaemon stopping...")
        self._running = False
        await self.driver.shutdown()
        if self._ws and self._ws_connected:
            try:
                await self._ws.close()
            except Exception:
                pass

    # ─────────────────────────────────────────────────────────────────────────
    # Task A: HAL Polling Loop (Safety-Critical, 50 Hz)
    # ─────────────────────────────────────────────────────────────────────────

    async def _hal_loop(self) -> None:
        """
        Safety-critical hardware loop. Runs in isolation from cloud I/O.
        Enforces grace-mode speed limit if license is invalid/expired.
        """
        poll_interval = 1.0 / self.HAL_POLL_HZ
        logger.info(f"[{self.robot_id}] HAL loop started at {self.HAL_POLL_HZ} Hz.")

        while self._running:
            t0 = time.monotonic()
            try:
                frame = await self.driver.read_telemetry()

                # ── Grace Mode Speed Enforcement ────────────────────────────
                if self._grace_mode:
                    speed = abs(frame.twist.linear.x)
                    max_speed = self._license_info.max_speed_ms
                    if speed > max_speed:
                        scale = max_speed / speed
                        capped_cmd = CommandFrame(
                            command_id=f"grace_{uuid.uuid4().hex[:8]}",
                            robot_id=self.robot_id,
                            target_twist=frame.twist.model_copy(
                                update={"linear": frame.twist.linear.model_copy(
                                    update={"x": frame.twist.linear.x * scale}
                                )}
                            )
                        )
                        await self.driver.send_command(capped_cmd)

                    # Watermark telemetry
                    frame = frame.model_copy(
                        update={"custom_metrics": {
                            **frame.custom_metrics,
                            "_nexusos_grace_mode": True,
                            "_nexusos_notice": "License required for production speed."
                        }}
                    )

                self._latest_telemetry = frame
                self._telemetry_count += 1

                # Feed memory engine
                self.context.ingest_telemetry(frame)

            except Exception as exc:
                logger.exception(f"[{self.robot_id}] HAL loop exception: {exc}")

            # Deterministic sleep to maintain target frequency
            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.0, poll_interval - elapsed))

    # ─────────────────────────────────────────────────────────────────────────
    # Task B: Cloud Telemetry Streaming via WebSocket (Best-Effort, 10 Hz)
    # ─────────────────────────────────────────────────────────────────────────

    async def _cloud_stream_loop(self) -> None:
        """Connect to cloud WebSocket and stream telemetry. Auto-reconnects on failure."""
        if not HAS_WEBSOCKETS:
            logger.warning("websockets package not installed. Cloud streaming disabled.")
            return

        stream_interval = 1.0 / self.STREAM_HZ

        while self._running:
            try:
                logger.info(f"[{self.robot_id}] Connecting to cloud: {self.cloud_ws_url}")
                async with websockets.connect(
                    self.cloud_ws_url,
                    ping_interval=20,
                    ping_timeout=10,
                    extra_headers={"X-Robot-ID": self.robot_id, "X-License": self.license_token[:32]}
                ) as ws:
                    self._ws = ws
                    self._ws_connected = True
                    self._reconnect_delay = 1.0  # Reset backoff on success
                    logger.info(f"[{self.robot_id}] WebSocket connected to cloud.")

                    # Flush offline buffer first
                    await self._flush_offline_buffer(ws)

                    while self._running:
                        t0 = time.monotonic()
                        if self._latest_telemetry:
                            try:
                                payload = self._latest_telemetry.model_dump_json()
                                await ws.send(payload)
                            except Exception:
                                raise  # Bubble up to reconnect

                        # Non-blocking receive for inbound commands
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=0.001)
                            cmd = CommandFrame.model_validate_json(raw)
                            await self._command_queue.put(cmd)
                        except asyncio.TimeoutError:
                            pass
                        except Exception:
                            pass  # Malformed command — ignore

                        elapsed = time.monotonic() - t0
                        await asyncio.sleep(max(0.0, stream_interval - elapsed))

            except Exception as exc:
                self._ws_connected = False
                if self._latest_telemetry:
                    self._offline_buffer.append(self._latest_telemetry)

                logger.warning(
                    f"[{self.robot_id}] Cloud WebSocket disconnected: {exc}. "
                    f"Buffered {len(self._offline_buffer)} frames. "
                    f"Retry in {self._reconnect_delay:.1f}s."
                )
                await asyncio.sleep(self._reconnect_delay)
                self._reconnect_delay = min(self._reconnect_delay * 2, self._max_reconnect_delay)

    async def _flush_offline_buffer(self, ws: Any) -> None:
        """Drain the offline telemetry buffer to cloud on reconnection."""
        if not self._offline_buffer:
            return
        count = len(self._offline_buffer)
        logger.info(f"[{self.robot_id}] Flushing {count} buffered telemetry frames to cloud.")
        while self._offline_buffer:
            frame = self._offline_buffer.popleft()
            try:
                payload = frame.model_dump_json()
                await ws.send(payload)
            except Exception:
                self._offline_buffer.appendleft(frame)  # Re-queue on failure
                break

    # ─────────────────────────────────────────────────────────────────────────
    # Task C: Inbound Command Dispatch Loop
    # ─────────────────────────────────────────────────────────────────────────

    async def _command_dispatch_loop(self) -> None:
        """Consume commands from cloud queue and dispatch to hardware driver."""
        logger.info(f"[{self.robot_id}] Command dispatch loop started.")
        while self._running:
            try:
                cmd = await asyncio.wait_for(self._command_queue.get(), timeout=0.5)

                # Grace mode: block commands that exceed speed limit
                if self._grace_mode and cmd.target_twist:
                    max_speed = self._license_info.max_speed_ms
                    if abs(cmd.target_twist.linear.x) > max_speed:
                        logger.warning(
                            f"[{self.robot_id}] Grace Mode: Command speed "
                            f"{cmd.target_twist.linear.x:.2f} m/s capped to {max_speed} m/s."
                        )
                        cmd = cmd.model_copy(
                            update={"target_twist": cmd.target_twist.model_copy(
                                update={"linear": cmd.target_twist.linear.model_copy(
                                    update={"x": max_speed}
                                )}
                            )}
                        )

                ok = await self.driver.send_command(cmd)
                if not ok:
                    logger.warning(f"[{self.robot_id}] Command {cmd.command_id} was rejected by driver.")

            except asyncio.TimeoutError:
                pass
            except Exception as exc:
                logger.exception(f"[{self.robot_id}] Command dispatch error: {exc}")

    # ─────────────────────────────────────────────────────────────────────────
    # External Interface
    # ─────────────────────────────────────────────────────────────────────────

    async def dispatch_command(self, command: CommandFrame) -> None:
        """External API: inject a command directly (e.g., from local UI or test harness)."""
        await self._command_queue.put(command)

    def get_status(self) -> Dict[str, Any]:
        """Return daemon operational status dict."""
        return {
            "robot_id": self.robot_id,
            "running": self._running,
            "ws_connected": self._ws_connected,
            "telemetry_count": self._telemetry_count,
            "offline_buffer_size": len(self._offline_buffer),
            "license_status": self._license_info.status.value,
            "license_tier": self._license_info.tier.value,
            "grace_mode": self._grace_mode,
            "max_speed_ms": self._license_info.max_speed_ms,
        }
