"""
NexusOS Edge Agent Runner.
Entry point for running the edge daemon from Docker or CLI.
Usage: python -m nexusos.agent.run_edge
"""

from __future__ import annotations
import asyncio
import logging
import os
import signal
import sys

from nexusos.core.registry import DriverRegistry
from nexusos.agent.daemon import EdgeDaemon

logging.basicConfig(
    level=os.environ.get("NEXUSOS_LOG_LEVEL", "INFO").upper(),
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("NexusOS.EdgeRunner")


async def main() -> None:
    robot_id = os.environ.get("NEXUSOS_ROBOT_ID", "edge_robot_01")
    driver_type = os.environ.get("NEXUSOS_DRIVER_TYPE", "DIFFERENTIAL_DRIVE")
    cloud_url = os.environ.get("NEXUSOS_CLOUD_URL", "ws://localhost:8000/ws/edge")
    license_token = os.environ.get("NEXUSOS_LICENSE_TOKEN", "")

    logger.info(f"NexusOS Edge Daemon starting: robot_id={robot_id}, driver={driver_type}")

    driver = DriverRegistry.create_driver(driver_type, robot_id=robot_id)
    daemon = EdgeDaemon(
        robot_id=robot_id,
        driver=driver,
        license_token=license_token,
        cloud_ws_url=cloud_url
    )

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _signal_handler(*_):
        logger.info("Shutdown signal received.")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _signal_handler)
        except NotImplementedError:
            pass  # Windows doesn't support add_signal_handler

    daemon_task = asyncio.create_task(daemon.start())

    await stop_event.wait()
    await daemon.stop()
    daemon_task.cancel()
    try:
        await daemon_task
    except asyncio.CancelledError:
        pass
    logger.info("NexusOS Edge Daemon stopped cleanly.")


if __name__ == "__main__":
    asyncio.run(main())
