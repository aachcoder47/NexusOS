"""
Demo: Autonomous Fleet Mission Controller for NexusOS.

Demonstrates programmatic mission orchestration across multiple heterogenous robots:
- Autonomous Waypoint Trajectory for Differential Drive Rover
- Robotic Arm Pick-and-Place Joint Cycle
- Quadcopter Drone Patrol & Landing
- Real-time Health / Safety Telemetry Verification
"""

import asyncio
import logging
import math
import time
import sys
from pathlib import Path
from typing import Dict, Any

# Add project root and demo folder to path for standalone execution
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEMO_ROOT = Path(__file__).resolve().parent
for p in (PROJECT_ROOT, DEMO_ROOT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from hal.base import (
    MotorCommandData,
    TwistData,
    Vector3D,
    JointStateData,
    RobotMode,
    RobotType
)
from edge.config import EdgeConfig
from edge.agent import EdgeAgent
from hal.plugins.diff_drive import DiffDriveBasePlugin
from hal.plugins.robotic_arm import RoboticArmPlugin
from custom_hardware_plugin import QuadcopterDronePlugin

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("NexusOS.MissionController")


async def run_rover_mission(rover: DiffDriveBasePlugin):
    """Executes a square patrol route with the differential drive robot."""
    logger.info("=== Starting Rover Alpha Patrol Mission ===")
    waypoints = [
        ("FORWARD 2m", TwistData(linear=Vector3D(x=0.5, y=0.0, z=0.0), angular=Vector3D(z=0.0)), 2.0),
        ("ROTATE 90 deg", TwistData(linear=Vector3D(x=0.0, y=0.0, z=0.0), angular=Vector3D(z=0.785)), 1.5),
        ("FORWARD 2m", TwistData(linear=Vector3D(x=0.5, y=0.0, z=0.0), angular=Vector3D(z=0.0)), 2.0),
        ("ROTATE 90 deg", TwistData(linear=Vector3D(x=0.0, y=0.0, z=0.0), angular=Vector3D(z=0.785)), 1.5),
        ("STOP", TwistData(linear=Vector3D(), angular=Vector3D()), 0.5)
    ]

    for label, twist, duration in waypoints:
        logger.info(f"Rover Alpha Action: {label} (Duration: {duration}s)")
        await rover.send_command(MotorCommandData(
            target_twist=twist,
            requested_mode=RobotMode.AUTONOMOUS
        ))
        await asyncio.sleep(duration)
        telem = await rover.read_telemetry()
        logger.info(
            f"  [Rover Telemetry] Pos: ({telem.pose.position.x:.2f}, {telem.pose.position.y:.2f}) | "
            f"Battery: {telem.battery.percentage:.1f}% | Odometry: {telem.custom_metrics.get('odometry_distance_traveled_m', 0.0):.2f}m"
        )


async def run_arm_mission(arm: RoboticArmPlugin):
    """Executes a pick-and-place joint trajectory cycle with the 6-DOF Arm."""
    logger.info("=== Starting Robotic Arm Beta Pick-and-Place Mission ===")
    poses = [
        ("HOME POSE", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False, 1.5),
        ("REACH TO OBJECT", [0.5, 0.78, -0.4, 0.0, 1.2, 0.0], False, 1.5),
        ("GRIP OBJECT", [0.5, 0.78, -0.4, 0.0, 1.2, 0.0], True, 1.0),
        ("LIFT OBJECT", [0.5, 0.2, -0.2, 0.0, 0.8, 0.0], True, 1.5),
        ("PLACE AT TARGET", [-0.8, 0.6, -0.5, 0.0, 1.1, -0.5], True, 1.5),
        ("RELEASE GRIPPER", [-0.8, 0.6, -0.5, 0.0, 1.1, -0.5], False, 1.0),
        ("RETURN TO HOME", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False, 1.5)
    ]

    for label, joint_angles, gripper_closed, duration in poses:
        logger.info(f"Arm Beta Action: {label} (Gripper: {'CLOSED' if gripper_closed else 'OPEN'})")
        cmd = MotorCommandData(
            target_joints=JointStateData(
                names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                positions=joint_angles
            ),
            parameters={"gripper_closed": gripper_closed},
            requested_mode=RobotMode.AUTONOMOUS
        )
        await arm.send_command(cmd)
        await asyncio.sleep(duration)
        telem = await arm.read_telemetry()
        ee_pos = telem.custom_metrics.get("end_effector_position", {})
        logger.info(
            f"  [Arm Telemetry] End-Effector XYZ: ({ee_pos.get('x', 0):.2f}, {ee_pos.get('y', 0):.2f}, {ee_pos.get('z', 0):.2f}) | "
            f"Power Draw: {telem.custom_metrics.get('power_consumption_watts', 0):.1f}W"
        )


async def run_drone_mission(drone: QuadcopterDronePlugin):
    """Executes takeoff, aerial waypoint survey, and precision landing."""
    logger.info("=== Starting Quadcopter Drone Gamma Aerial Survey ===")
    flight_plan = [
        ("ARM & TAKEOFF TO 5m", TwistData(linear=Vector3D(x=0.0, y=0.0, z=1.5)), {"arm": True}, 2.0),
        ("PATROL EAST 4m", TwistData(linear=Vector3D(x=1.0, y=0.0, z=0.0)), {"arm": True}, 2.0),
        ("PATROL NORTH 4m", TwistData(linear=Vector3D(x=0.0, y=1.0, z=0.0)), {"arm": True}, 2.0),
        ("HOVER & SCAN", TwistData(linear=Vector3D(x=0.0, y=0.0, z=0.0), angular=Vector3D(z=0.5)), {"arm": True}, 1.5),
        ("RETURN & LAND", TwistData(linear=Vector3D(x=-1.0, y=-1.0, z=-1.0)), {"arm": True}, 2.0),
        ("TOUCHDOWN & DISARM", TwistData(), {"arm": False}, 0.5)
    ]

    for label, twist, params, duration in flight_plan:
        logger.info(f"Drone Gamma Action: {label}")
        await drone.send_command(MotorCommandData(
            target_twist=twist,
            parameters=params,
            requested_mode=RobotMode.AUTONOMOUS
        ))
        await asyncio.sleep(duration)
        telem = await drone.read_telemetry()
        logger.info(
            f"  [Drone Telemetry] Alt: {telem.pose.position.z:.2f}m | Pos: ({telem.pose.position.x:.2f}, {telem.pose.position.y:.2f}) | "
            f"Battery: {telem.battery.percentage:.1f}% ({telem.battery.voltage:.1f}V)"
        )


async def run_fleet_orchestration():
    """Runs concurrent mission across all 3 heterogeneous robots."""
    logger.info("=======================================================")
    logger.info("Initializing NexusOS Heterogeneous Fleet Mission Engine")
    logger.info("=======================================================")

    rover = DiffDriveBasePlugin(robot_id="rover-alpha")
    arm = RoboticArmPlugin(robot_id="arm-beta")
    drone = QuadcopterDronePlugin(robot_id="drone-gamma")

    await rover.initialize()
    await arm.initialize()
    await drone.initialize()

    logger.info("All robot hardware plugins initialized successfully!")
    logger.info("Starting concurrent multi-robot mission execution...\n")

    t0 = time.time()
    await asyncio.gather(
        run_rover_mission(rover),
        run_arm_mission(arm),
        run_drone_mission(drone)
    )
    elapsed = time.time() - t0

    logger.info(f"\n=======================================================")
    logger.info(f"Fleet Mission Complete in {elapsed:.2f}s!")
    logger.info("Executing graceful fleet shutdown...")
    logger.info("=======================================================")

    await rover.shutdown()
    await arm.shutdown()
    await drone.shutdown()
    logger.info("Fleet successfully parked and safe.")


if __name__ == "__main__":
    asyncio.run(run_fleet_orchestration())
