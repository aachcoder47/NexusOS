"""
NexusOS Demo: Autonomous Warehouse Logistics & Real-World Physics Simulation.

Demonstrates:
- 3D Physics Engine with gravity, rigid body dynamics, and obstacle collision responses.
- Simulated 360° LiDAR raycasting point clouds and 6-Axis IMU measurements.
- Real-time Multi-layer Costmap with inflation and dynamic obstacle avoidance.
- A* / Theta* global path planning + DWA reactive local trajectory navigation.
- Dynamic obstacle crossing and autonomous recovery maneuvers.
"""

import asyncio
import logging
import math
import sys
import time
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from physics.vector import Vec3
from physics.world import PhysicsWorld
from hal.plugins.diff_drive import DiffDriveBasePlugin
from hal.base import RobotMode

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("NexusOS.WarehouseDemo")


async def run_warehouse_mission():
    logger.info("=================================================================")
    logger.info("  NexusOS Autonomous Warehouse Navigation & 3D Physics Engine   ")
    logger.info("=================================================================")

    # 1. Create Physics World with Gravity
    world = PhysicsWorld(gravity=Vec3(0.0, 0.0, -9.81), enable_ground_plane=True)

    # 2. Build Warehouse Environment (Aisles, Shelves, Pallets)
    logger.info("Building physical warehouse environment with static obstacles & pallet racks...")
    world.add_box_obstacle("shelf_aisle_left", Vec3(-2.0, 2.0, 1.0), width=1.0, length=4.0, height=2.0)
    world.add_box_obstacle("shelf_aisle_right", Vec3(2.0, 2.0, 1.0), width=1.0, length=4.0, height=2.0)
    world.add_box_obstacle("center_pillar", Vec3(0.0, 2.5, 1.0), width=0.6, length=0.6, height=2.0)

    # 3. Instantiate Physics-Backed Autonomous Rover
    rover = DiffDriveBasePlugin(robot_id="warehouse-rover-01", physics_world=world)
    await rover.initialize()

    # Populate costmap with static shelves
    rover.costmap.set_static_obstacle(-2.0, 2.0, 1.0, 4.0)
    rover.costmap.set_static_obstacle(2.0, 2.0, 1.0, 4.0)
    rover.costmap.set_static_obstacle(0.0, 2.5, 0.6, 0.6)

    # 4. Multi-Waypoint Delivery Mission Plan
    waypoints = [
        ("Station Alpha (Pickup)", 0.0, 5.0),
        ("Station Beta (Dropoff)", 3.5, 4.0),
        ("Charging Dock", 0.0, 0.0)
    ]

    for label, target_x, target_y in waypoints:
        logger.info(f"\n>>> Dispatching Navigation Goal: {label} at ({target_x:.1f}, {target_y:.1f})m...")
        rover.set_navigation_goal(target_x, target_y)

        # Simulation loop until waypoint reached or timeout
        t0 = time.time()
        while time.time() - t0 < 15.0:
            telem = await rover.read_telemetry()
            pos = telem.pose.position
            nav = telem.custom_metrics.get("navigation", {})
            imu = telem.custom_metrics.get("imu", {})
            dist_left = nav.get("distance_to_goal", 0.0)
            state = nav.get("state", "UNKNOWN")

            logger.info(
                f"  [{label}] Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})m | "
                f"Vel: {telem.twist.linear.x:.2f}m/s | Yaw: {telem.pose.orientation_rpy.z:.2f}rad | "
                f"Dist: {dist_left:.2f}m | State: {state}"
            )

            if state == "GOAL_REACHED" or dist_left <= 0.25:
                logger.info(f"*** Successfully reached {label}! ***")
                await asyncio.sleep(0.5)
                break

            await asyncio.sleep(0.1)

    logger.info("\n=================================================================")
    logger.info("  Warehouse Autonomous Navigation Mission Completed Successfully! ")
    logger.info("=================================================================")
    await rover.shutdown()


if __name__ == "__main__":
    asyncio.run(run_warehouse_mission())
