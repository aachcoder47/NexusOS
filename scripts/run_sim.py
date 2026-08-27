"""
NexusOS Local Simulation Test Runner
Launches the FastAPI Cloud Server and multiple Edge Agent simulators (rover-alpha and arm-beta) concurrently.
"""

import asyncio
import logging
import sys
from pathlib import Path

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import uvicorn

from cloud.main import app as fastapi_app
from edge.agent import EdgeAgent
from edge.config import EdgeConfig
from hal.base import RobotType

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("NexusOS.Runner")


async def run_server():
    config = uvicorn.Config(fastapi_app, host="127.0.0.1", port=8000, log_level="info")
    server = uvicorn.Server(config)
    await server.serve()


async def run_agent(robot_id: str, robot_type: RobotType):
    # Brief delay to allow server to start listening
    await asyncio.sleep(1.5)
    config = EdgeConfig(
        robot_id=robot_id,
        robot_type=robot_type,
        server_url="ws://127.0.0.1:8000/ws/telemetry"
    )
    agent = EdgeAgent(config)
    await agent.start()


async def main():
    logger.info("Starting NexusOS Local Complete Simulation Stack...")
    logger.info("Access Fleet Dashboard UI at: http://localhost:8000")

    server_task = asyncio.create_task(run_server())
    rover_task = asyncio.create_task(run_agent("rover-alpha", RobotType.DIFFERENTIAL_DRIVE))
    arm_task = asyncio.create_task(run_agent("arm-beta", RobotType.ROBOTIC_ARM))

    await asyncio.gather(server_task, rover_task, arm_task)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Local simulation stopped by user.")
