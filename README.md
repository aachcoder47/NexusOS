# NexusOS: Universal Robotics Operating System & Agent Context Middleware

[![PyPI version](https://img.shields.io/pypi/v/nexusos-robotics.svg?color=brightgreen&label=PyPI%20Package)](https://pypi.org/project/nexusos-robotics/)
[![Python Versions](https://img.shields.io/pypi/pyversions/nexusos-robotics.svg)](https://pypi.org/project/nexusos-robotics/)
[![License: NCL-1.0](https://img.shields.io/badge/License-NCL--1.0_Dual-blue.svg)](LICENSE.md)
[![Tests Status](https://img.shields.io/badge/Tests-68%20Passed%20(100%25)-emerald.svg)]()
[![Digital Twin](https://img.shields.io/badge/3D%20Digital%20Twin-Three.js%20Integrated-cyan.svg)]()

> **Bridging Physical Hardware, Real-World Physics, Autonomous Navigation, Edge AI, and Cloud Fleet Orchestration** through a zero-dependency Hardware Abstraction Layer and unified Agent Context Memory.

---

## 🌟 What is NexusOS?

**NexusOS (`nexusos-robotics`)** is a hardware-agnostic, production-grade robotics operating system engineered to eliminate the "Tower of Babel" in robotics. It replaces fragile ROS 2 DDS configurations, version lock-in, and proprietary hardware silos with an elegant, modern architecture:

1. **Zero-Heavy-Dependency Hardware Abstraction Layer (HAL)**: Normalizes actuators, mobile bases (AGVs/AMRs), 6-DOF robotic arms, quadrupeds, and drones into strict Pydantic v2 schemas.
2. **Real-World 3D Newtonian Physics Engine**: Features DC motor torque curves with back-EMF, Pacejka Magic Formula tire-ground friction, suspension spring-damper reaction forces, quadratic aerodynamic drag, and Li-Ion electrochemical battery modeling.
3. **Full Autonomous Navigation Stack**: Multi-layer costmaps (`Costmap2D`), global path planning ($A^*$, $\Theta^*$, $\text{RRT}^*$, $D^*\text{-Lite}$), and real-time dynamic obstacle avoidance (Dynamic Window Approach & Pure Pursuit).
4. **Agent Context & Temporal Memory Engine**: Real-time spatial entity deduplication, decaying environment knowledge base, and vector cosine similarity retrieval for LLM / Vision-Language-Action (VLA) AI agents.
5. **Cloud Fleet Server & 3D Digital Twin**: High-frequency bi-directional WebSocket hub with a 3D Three.js dashboard supporting interactive click-to-navigate beacons, LiDAR point clouds, motor torque HUDs, and global emergency stop overrides.
6. **Dual-Licensing Compliance Engine**: Enforces the **NexusOS Community & Commercial License (NCL-1.0)** with **non-bricking Grace Mode** (never halts or crashes a running robot; enforces safe reduced speed and telemetry watermarking).

---

## 🏗️ System Architecture

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                 3D DIGITAL TWIN WEB UI                                 │
│          (Interactive Click-to-Navigate • 360° LiDAR Cloud • Physics Telemetry)        │
└───────────────────────────────────────────▲────────────────────────────────────────────┘
                                            │ WebSockets (30 Hz Streaming)
┌───────────────────────────────────────────▼────────────────────────────────────────────┐
│                             CLOUD FLEET CONTROL SERVER                                 │
│        (FastAPI Hub • Multi-Tenant State • Command Dispatch • License Verification)    │
└───────────────────────────────────────────▲────────────────────────────────────────────┘
                                            │ Resilient WebSocket / gRPC (Auto-Reconnect)
┌───────────────────────────────────────────▼────────────────────────────────────────────┐
│                              ASYNCHRONOUS EDGE DAEMON                                  │
│         (50Hz Safety Loop • Offline Buffer • Mission Watchdog • Grace Mode)            │
├────────────────────────────────────────────────────────────────────────────────────────┤
│                                AGENT CONTEXT MEMORY                                    │
│       (Temporal State Chunking • Spatial Entity Dedup • Cosine Vector Retrieval)       │
├────────────────────────────────────────────────────────────────────────────────────────┤
│                            AUTONOMOUS NAVIGATION STACK                                 │
│       (Costmap2D • Global: A*/Theta*/RRT*/D* Lite • Local: DWA / Pure Pursuit)         │
├────────────────────────────────────────────────────────────────────────────────────────┤
│                           REAL-WORLD 3D PHYSICS ENGINE                                 │
│       (Pacejka Tires • DC Motors • Suspension • Aero Drag • Battery Electrochemistry)  │
├────────────────────────────────────────────────────────────────────────────────────────┤
│                          HARDWARE ABSTRACTION LAYER (HAL)                              │
│       (Differential Drive AGVs • 6-DOF Arms • Quadrupeds • Custom Hardware Drivers)    │
└───────────────────────────────────────────▲────────────────────────────────────────────┘
                                            │
                                 Physical Hardware / Actuators
```

---

## 📦 Monorepo Layout

```
NexusOS/
├── proto/                    # Protobuf v3 contracts (nexusos.proto)
├── nexusos/                  # Main Python Package (PyPI: nexusos-robotics)
│   ├── proto/                # Protocol Buffers and gRPC definitions
│   ├── core/                 # Core HAL, Driver Registry, and Pydantic v2 Types
│   │   ├── hal.py            # BaseRobotDriver, DiffDriveMockDriver, RoboticArm6DOFMockDriver
│   │   ├── registry.py       # Thread-safe DriverRegistry factory
│   │   └── types.py          # Pose3D, Twist3D, JointState, TelemetryFrame, CommandFrame
│   ├── memory/               # Real-time Agent Context & Spatial Memory Engine
│   │   └── engine.py         # AgentContextEngine (temporal chunking, dedup, vector index)
│   ├── agent/                # Asynchronous Edge Daemon & Telemetry Streamer
│   │   ├── daemon.py         # 50Hz isolated HAL loop & WebSocket client with backoff
│   │   └── run_edge.py       # Standalone CLI / Docker edge runner
│   ├── server/               # Cloud Fleet Control Backend
│   │   └── main.py           # FastAPI server, WebSockets, fan-out, REST endpoints
│   └── licensing/            # Licensing & Safety Compliance Engine
│       ├── verifier.py       # JWT validation, ARR tier enforcement, non-bricking Grace Mode
│       └── server.py         # Development License Server mock (port 9000)
├── physics/                  # 3D Newtonian Physics Engine
│   ├── body.py               # 6-DOF RigidBody3D with gyroscopic Euler terms & sleep
│   ├── materials.py          # 23 engineering materials & Pacejka tire-surface models
│   ├── vehicle.py            # DC Motor torque curves, Suspension, Aero drag, Li-Ion battery
│   ├── collision.py          # SAT collision detector, OBB raycasting, impulse solver
│   ├── sensors.py            # Raycast LiDAR and 6-axis IMU sensors
│   ├── shapes.py             # Box, Sphere, Cylinder collision geometry
│   ├── vector.py             # Vec3 and Quaternion mathematics
│   └── world.py              # Sub-stepped multi-body PhysicsWorld
├── navigation/               # Autonomous Navigation Stack
│   ├── costmap.py            # Costmap2D multi-layer grid with obstacle inflation
│   ├── navigator.py          # Finite State Machine (FSM) autonomous navigator
│   ├── planners/             # A*, Theta*, RRT*, D* Lite global path planners
│   └── local_planners/       # Dynamic Window Approach (DWA) & Pure Pursuit
├── cloud/                    # Legacy & Web UI Assets
│   └── static/index.html     # 3D Digital Twin (Three.js, Click-to-Navigate, Diagnostics)
├── demo/                     # Verification Demos & Warehouse Scenarios
│   └── autonomous_warehouse_demo.py # Complete logistics mission demonstration
├── docker/                   # Multi-Stage Dockerfile & Orchestration
│   └── Dockerfile            # Cloud, Edge, and License Server targets
├── docker-compose.yml        # Multi-container stack (Cloud + License + 2 Edge Robots)
├── LICENSE.md                # NexusOS Community & Commercial License (NCL-1.0)
├── pyproject.toml            # Packaging configuration for PyPI (nexusos-robotics)
└── tests/                    # Comprehensive automated test suite (68 tests)
```

---

## ⚡ Quick Start

### 1. Installation

Install from PyPI:
```bash
pip install nexusos-robotics
```

Or install from source with development dependencies:
```bash
git clone https://github.com/nexusos-robotics/NexusOS.git
cd NexusOS
pip install -e ".[dev]"
```

### 2. Launch the 3D Digital Twin & Simulation

Run the complete local cloud server, simulated differential drive robot, and 3D digital twin:
```bash
python scripts/run_sim.py
```
Open your browser at **`http://localhost:8000`**.

**Interactive Features:**
- **Click anywhere on the 3D floor** to dispatch an autonomous navigation beacon.
- Watch the vehicle plan an $A^*$ path and dynamically avoid obstacles using DWA.
- Inspect the **Real-World Physics Diagnostics HUD** (motor torque, current draw, Pacejka tire slip, suspension travel, battery SoC).
- Switch tire surfaces on the fly (e.g., *Dry Concrete* $\rightarrow$ *Ice* $\rightarrow$ *Gravel*).

---

### 3. Run Headless Warehouse Logistics Mission

```bash
python demo/autonomous_warehouse_demo.py
```
Executes a multi-stop autonomous mission (`Station Alpha` $\rightarrow$ `Station Beta` $\rightarrow$ `Charging Dock`) verifying LiDAR sensor fusion, obstacle avoidance, and goal convergence.

---

### 4. Deploy via Docker Compose

Spin up the Cloud Server, License Mock, and 2 Edge Robot instances in one command:
```bash
docker-compose up --build
```

---

## 💻 Writing a Custom Robot Driver

Subclass `BaseRobotDriver` to integrate any physical hardware:

```python
import asyncio
from nexusos.core.hal import BaseRobotDriver
from nexusos.core.types import RobotType, TelemetryFrame, CommandFrame, Pose3D, Vector3D
from nexusos.core.registry import DriverRegistry
from nexusos.agent.daemon import EdgeDaemon

class MyRoverDriver(BaseRobotDriver):
    def __init__(self, robot_id: str):
        super().__init__(robot_id=robot_id, robot_type=RobotType.DIFFERENTIAL_DRIVE)

    async def initialize(self) -> bool:
        # Connect to hardware (Serial / CAN / Modbus)
        self.is_initialized = True
        return True

    async def read_telemetry(self) -> TelemetryFrame:
        # Read wheel encoders, IMU, and battery sensors
        return TelemetryFrame(
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            pose=Pose3D(position=Vector3D(x=1.2, y=0.5, z=0.0))
        )

    async def send_command(self, command: CommandFrame) -> bool:
        # Dispatch velocity vectors to motor controllers
        if command.target_twist:
            v_x = command.target_twist.linear.x
            w_z = command.target_twist.angular.z
            # Send PWM to hardware...
        return True

    async def emergency_stop(self) -> bool:
        # Engage mechanical brakes / cut motor power
        return True

    async def shutdown(self) -> None:
        await self.emergency_stop()

# Register driver with the system
DriverRegistry.register("MY_ROVER", MyRoverDriver)

# Run with the Edge Daemon
async def main():
    driver = MyRoverDriver(robot_id="rover_01")
    daemon = EdgeDaemon(robot_id="rover_01", driver=driver, cloud_ws_url="ws://localhost:8000/ws/edge")
    await daemon.start()

if __name__ == "__main__":
    asyncio.run(main())
```

---

## 🧠 Agent Context & Spatial Memory

NexusOS enables LLM and VLA AI agents to query the robot's spatial environment and state history:

```python
from nexusos.memory.engine import AgentContextEngine
from nexusos.core.types import Vector3D

memory = AgentContextEngine(robot_id="robot_01")

# Ingest detected physical objects with automatic spatial deduplication
memory.ingest_spatial_entity(
    label="charging_dock",
    position=Vector3D(x=4.0, y=-2.0, z=0.0),
    attributes={"voltage": 24.0, "status": "AVAILABLE"}
)

# Assert temporal facts (expire automatically)
memory.assert_fact("dock_1", "is_state", "OCCUPIED", validity_sec=300.0)

# Create a snapshot block with vector embedding
block = memory.create_context_block(embedding=[0.24, -0.58, 0.91, 0.12], active_intent="GO_TO_DOCK")

# Query memory by spatial radius
nearby_entities = memory.query_spatial_entities(center=Vector3D(x=3.5, y=-1.8, z=0.0), radius_m=2.0)

# Query historical contexts by vector cosine similarity
similar_contexts = memory.query_similar_contexts(query_embedding=[0.25, -0.55, 0.90, 0.10], top_k=3)
```

---

## ⚖️ Legal Framework & Dual Licensing (NCL-1.0)

NexusOS is distributed under the **NexusOS Community & Commercial License (NCL-1.0)**:

- **Community Tier (Free & Open Source)**: Free for all non-commercial, academic, research, open-source projects, and commercial organizations with Annual Recurring Revenue (ARR) **under $100,000 USD**.
- **Commercial Tier (Revenue Share)**: Mandatory for closed-source commercial deployments with ARR **over $100,000 USD** (3% revenue share + support agreement). Contact `licensing@nexusos.io`.
- **Non-Bricking Safety Fallback (Grace Mode)**: If an unverified or expired token is detected in production, the software **will never halt, crash, or brick physical robots**. Instead, it safely operates at a capped speed ($0.5\text{ m/s}$) with watermarked telemetry and keeps all emergency stop safety loops operational.

---

## 🧪 Testing & Verification

Run the full automated test suite (unit tests covering vector math, SAT collision, planners, real-world physics, licensing, memory, and daemon):

```bash
python -m pytest tests/ -v
```

```
============================= 68 passed in 4.35s ==============================
```

---

## 👥 Contributing & Community

- **Website**: [nexusos.io](https://nexusos.io)
- **Documentation**: [USERGUIDE.md](USERGUIDE.md)
- **Contributing Guide**: [CONTRIBUTING.md](CONTRIBUTING.md)
- **Bug Tracker**: [GitHub Issues](https://github.com/nexusos-robotics/NexusOS/issues)

---

Copyright (c) 2024–2026 NexusOS Contributors and Maintainers. All rights reserved.
