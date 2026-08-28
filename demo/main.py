"""
NexusOS Fleet - Interactive Demo Suite Launcher (v1.0.1)

Provides an all-in-one interactive launcher showcasing the capabilities of the
`nexusos-fleet` package:
1. Autonomous Multi-Robot Mission (Rover + Arm + Drone)
2. Full Cloud Fleet Server + Simulated Edge Daemons + Web Dashboard
3. Custom Hardware Plugin Extensibility (Autonomous Quadcopter UAV)
4. Ultra-Low-Latency Telemetry Benchmark (MsgPack vs JSON)
"""

import asyncio
import logging
import os
import sys
import time
from pathlib import Path

# Add project root and demo dir if executing locally
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEMO_ROOT = Path(__file__).resolve().parent
for p in (PROJECT_ROOT, DEMO_ROOT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from hal.base import (
    RobotType,
    RobotMode,
    MotorCommandData,
    TwistData,
    Vector3D,
    JointStateData
)
from hal.plugins.diff_drive import DiffDriveBasePlugin
from hal.plugins.robotic_arm import RoboticArmPlugin
from edge.config import EdgeConfig
from edge.agent import EdgeAgent
from custom_hardware_plugin import QuadcopterDronePlugin


def print_banner():
    banner = r"""
========================================================================
     _   _                       ___  ____  
    | \ | | _____  ___   _ ___  / _ \/ ___| 
    |  \| |/ _ \ \/ / | | / __|| | | \___ \ 
    | |\  |  __/>  <| |_| \__ \| |_| |___) |
    |_| \_|\___/_/\_\\__,_|___/ \___/|____/ 
                                            
    Universal Robotics Abstraction & Cloud Fleet Middleware (v1.0.1)
========================================================================
    """
    print(banner)


async def run_standalone_mission():
    """Option 1: Execute concurrent autonomous mission for Rover, Arm, and Drone."""
    from mission_controller import run_fleet_orchestration
    print("\n>>> Launching Autonomous Multi-Robot Fleet Mission...\n")
    await run_fleet_orchestration()


async def run_full_cloud_stack():
    """Option 2: Launch FastAPI Cloud Server + 2 Edge Agents."""
    import uvicorn
    from cloud.main import app as fastapi_app

    print("\n>>> Launching NexusOS Cloud Fleet Management Server...")
    print(">>> Web Dashboard UI accessible at: http://localhost:8000\n")

    async def run_server():
        config = uvicorn.Config(fastapi_app, host="127.0.0.1", port=8000, log_level="warning")
        server = uvicorn.Server(config)
        await server.serve()

    async def run_simulated_agent(robot_id: str, robot_type: RobotType, delay: float):
        await asyncio.sleep(delay)
        print(f"[{robot_id}] Connecting Edge Agent to Cloud Server...")
        cfg = EdgeConfig(
            robot_id=robot_id,
            robot_type=robot_type,
            server_url="ws://127.0.0.1:8000/ws/telemetry",
            telemetry_rate_hz=20
        )
        agent = EdgeAgent(cfg)
        await agent.start()

    server_task = asyncio.create_task(run_server())
    rover_task = asyncio.create_task(run_simulated_agent("rover-alpha", RobotType.DIFFERENTIAL_DRIVE, 1.5))
    arm_task = asyncio.create_task(run_simulated_agent("arm-beta", RobotType.ROBOTIC_ARM, 2.0))

    try:
        await asyncio.gather(server_task, rover_task, arm_task)
    except asyncio.CancelledError:
        pass


async def run_drone_demo():
    """Option 3: Demonstrate custom Quadcopter hardware plugin."""
    print("\n>>> Initializing Custom Quadcopter Drone Hardware Plugin...")
    drone = QuadcopterDronePlugin("drone-sky-01")
    await drone.initialize()

    print("[Avionics] Arming rotors and commanding vertical takeoff to 5m...")
    await drone.send_command(MotorCommandData(
        target_twist=TwistData(linear=Vector3D(z=1.5)),
        parameters={"arm": True},
        requested_mode=RobotMode.AUTONOMOUS
    ))

    for step in range(3):
        await asyncio.sleep(1.0)
        t = await drone.read_telemetry()
        print(
            f"  [T+{step+1}s] Altitude: {t.pose.position.z:.2f}m | "
            f"Batt: {t.battery.percentage:.1f}% ({t.battery.voltage:.2f}V) | "
            f"Status: {t.status_message}"
        )

    print("\n[Avionics] Executing 3D translation & yaw rotation...")
    await drone.send_command(MotorCommandData(
        target_twist=TwistData(
            linear=Vector3D(x=2.5, y=1.0, z=0.0),
            angular=Vector3D(z=0.78)
        ),
        parameters={"arm": True}
    ))

    for step in range(3):
        await asyncio.sleep(1.0)
        t = await drone.read_telemetry()
        print(
            f"  [T+{step+4}s] Pos (X,Y,Z): ({t.pose.position.x:.2f}, {t.pose.position.y:.2f}, {t.pose.position.z:.2f}) | "
            f"Yaw: {t.pose.orientation_rpy.z:.2f} rad | Batt: {t.battery.percentage:.1f}%"
        )

    print("\n[Avionics] Commanding Emergency Safe Return & Landing...")
    await drone.emergency_stop()
    await asyncio.sleep(1.0)
    await drone.shutdown()
    print("[Avionics] Drone safely landed and shutdown complete.\n")


async def run_latency_benchmark():
    """Option 4: Benchmark MsgPack vs JSON telemetry serialization throughput."""
    import json
    import msgpack

    print("\n>>> Running NexusOS Telemetry Serialization & Latency Benchmark...")
    rover = DiffDriveBasePlugin("benchmark-rover")
    await rover.initialize()
    telem = await rover.read_telemetry()

    iterations = 50_000
    print(f"Benchmarking {iterations:,} telemetry serialization cycles...\n")

    # 1. JSON
    t0 = time.perf_counter()
    json_bytes = 0
    for _ in range(iterations):
        payload = json.dumps(telem.to_dict()).encode("utf-8")
        json_bytes += len(payload)
    t_json = time.perf_counter() - t0

    # 2. MsgPack
    t0 = time.perf_counter()
    mp_bytes = 0
    for _ in range(iterations):
        payload = telem.to_msgpack()
        mp_bytes += len(payload)
    t_msgpack = time.perf_counter() - t0

    print(f"| Protocol | Total Time | Throughput       | Avg Frame Size |")
    print(f"|----------|------------|------------------|----------------|")
    print(f"| JSON     | {t_json*1000:7.2f} ms | {iterations/t_json:10,.0f} ops/s | {json_bytes/iterations:6.1f} bytes   |")
    print(f"| MsgPack  | {t_msgpack*1000:7.2f} ms | {iterations/t_msgpack:10,.0f} ops/s | {mp_bytes/iterations:6.1f} bytes   |")
    speedup = t_json / t_msgpack
    bandwidth_saved = (1.0 - (mp_bytes / json_bytes)) * 100
    print(f"\n[Result] MsgPack is {speedup:.2f}x FASTER and saves {bandwidth_saved:.1f}% network bandwidth per frame!\n")
    await rover.shutdown()


def main():
    print_banner()
    print("Select a demo experience to run:")
    print("  [1] Autonomous Multi-Robot Mission (Rover + Arm + Custom Drone)")
    print("  [2] Full Cloud Fleet Stack (FastAPI Server + Simulated Agents + Dashboard)")
    print("  [3] Custom Hardware Plugin Demo (Autonomous Quadcopter UAV)")
    print("  [4] MsgPack Telemetry Performance & Latency Benchmark")
    print("  [5] Exit\n")

    choice = input("Enter choice [1-5] (default: 1): ").strip()
    if not choice:
        choice = "1"

    try:
        if choice == "1":
            asyncio.run(run_standalone_mission())
        elif choice == "2":
            asyncio.run(run_full_cloud_stack())
        elif choice == "3":
            asyncio.run(run_drone_demo())
        elif choice == "4":
            asyncio.run(run_latency_benchmark())
        elif choice == "5":
            print("Exiting NexusOS demo. Have a great day!")
        else:
            print("Invalid choice. Running default Autonomous Multi-Robot Mission...")
            asyncio.run(run_standalone_mission())
    except KeyboardInterrupt:
        print("\nDemo interrupted by user.")


if __name__ == "__main__":
    main()
