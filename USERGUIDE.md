# NexusOS — User Guide

> **NexusOS** is a hardware-agnostic, real-time robotics fleet orchestration platform.
> Connect any robot — differential drive, robotic arm, drone, or custom hardware —
> to a unified cloud backend with live telemetry, teleoperation, and fleet management.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Installation](#2-installation)
3. [Running the Simulation Stack](#3-running-the-simulation-stack)
4. [Fleet Dashboard](#4-fleet-dashboard)
5. [REST API Reference](#5-rest-api-reference)
6. [WebSocket Endpoints](#6-websocket-endpoints)
7. [EdgeAgent Configuration](#7-edgeagent-configuration)
8. [Writing a Custom HAL Plugin](#8-writing-a-custom-hal-plugin)
9. [Built-in HAL Plugins](#9-built-in-hal-plugins)
10. [ROS 2 Bridge Integration](#10-ros-2-bridge-integration)
11. [Licensing](#11-licensing)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Architecture Overview

```
+--------------------------------------------------------+
|                    NEXUSOS PLATFORM                    |
|                                                        |
|  +---------------+   WebSocket    +----------------+  |
|  | Cloud Fleet   |<---------------| Edge Agent     |  |
|  | Server        | Telemetry(50Hz)| Daemon         |  |
|  | (FastAPI)     |--------------->| (per robot)    |  |
|  | port :8000    |  Commands(JSON)|                |  |
|  +------+--------+                +-------+--------+  |
|         | WebSocket /ws/dashboard          |           |
|         v                         +--------v-------+  |
|  +---------------+                | AbstractHAL    |  |
|  | Fleet UI      |                | Plugin (ABC)   |  |
|  | Dashboard     |                +--------+-------+  |
|  | (Browser)     |                         |           |
|  +---------------+          DiffDrive   RoboticArm    |
|                              Plugin      Plugin       |
+--------------------------------------------------------+
```

**Components at a glance:**

| Component | Role |
|-----------|------|
| **Cloud Fleet Server** | FastAPI backend — aggregates telemetry, dispatches commands, serves dashboard |
| **Edge Agent Daemon** | Runs on robot compute; reads HAL, streams telemetry, receives commands |
| **HAL Plugin** | Hardware-specific adapter implementing `AbstractHardwarePlugin` |
| **Fleet Dashboard** | Browser UI — live gauges, 2D map, teleoperation controls |
| **Protobuf Schema** | Shared message contracts (`proto/nexus.proto`) |

**Data flow:**
1. HAL Plugin reads raw hardware sensors → normalized `RobotTelemetryData`
2. Edge Agent serializes to **msgpack** binary (~6x faster than JSON) and streams to Cloud via WebSocket at up to 50 Hz
3. Cloud Server updates `FleetStateManager` and broadcasts to all Dashboard clients
4. Dashboard sends teleoperation commands → Cloud dispatches to correct Edge Agent
5. Edge Agent calls `hal.send_command()` on physical hardware

---

## 2. Installation

### Requirements

- Python **3.10+**
- Docker + Docker Compose (for containerized mode)
- ROS 2 Humble/Iron/Jazzy _(optional — only for ROS 2 bridge)_

### Option A — Install from PyPI _(Recommended)_

```bash
pip install nexusos-fleet
```

> Full package: **[pypi.org/project/nexusos-fleet](https://pypi.org/project/nexusos-fleet/)**

### Option B — Install from Source

```bash
git clone https://github.com/aachcoder47/NexusOS.git
cd NexusOS
pip install -e .
```

### Verify Installation

```bash
python -c "from hal.base import AbstractHardwarePlugin; print('HAL OK')"
python -c "from edge.agent import EdgeAgent; print('Edge OK')"
```

---

## 3. Running the Simulation Stack

### Option A — Local Python _(No Docker required)_

Starts Cloud Server + two simulated Edge Agents (diff-drive + robotic arm):

```bash
python scripts/run_sim.py
```

Expected output:
```
[NexusOS.CloudServer]  Uvicorn running on http://0.0.0.0:8000
[NexusOS.EdgeAgent]    Connected! robot_id=rover-alpha
[NexusOS.EdgeAgent]    Connected! robot_id=arm-beta
```

Open **http://localhost:8000** in your browser.

---

### Option B — Docker Compose _(Containerized)_

```bash
docker-compose up --build
```

| Container | Service | Port |
|-----------|---------|------|
| `nexus-cloud-server` | Cloud Fleet Backend | `:8000` |
| `nexus-edge-rover-alpha` | Diff-Drive simulator | — |
| `nexus-edge-arm-beta` | Robotic Arm simulator | — |

---

### Option C — Start Components Individually

**Terminal 1 — Cloud Server:**
```bash
uvicorn cloud.main:app --host 0.0.0.0 --port 8000 --reload
```

**Terminal 2 — Edge Agent(s):**
```bash
# python edge/agent.py <robot_id> <robot_type>
python edge/agent.py rover-alpha DIFFERENTIAL_DRIVE
python edge/agent.py arm-beta ROBOTIC_ARM
```

---

## 4. Fleet Dashboard

Navigate to **`http://localhost:8000`**.

### Panels

| Panel | Description |
|-------|-------------|
| **Robot Cards** | Live status — mode, battery %, connection state |
| **Telemetry Gauges** | Real-time velocity, angular rate, battery voltage |
| **2D Map View** | Live robot X/Y position plotted on a 2D plane |
| **Teleop Controls** | Arrow key / button teleoperation |
| **Global E-Stop** | One-click emergency stop across entire fleet |

### Keyboard Teleoperation

Select a robot card, then:

| Key | Action |
|-----|--------|
| `W` / `↑` | Forward |
| `S` / `↓` | Backward |
| `A` / `←` | Turn Left |
| `D` / `→` | Turn Right |
| `Space` | Stop |

> **Note:** Teleop only works in `MANUAL` mode. In `AUTONOMOUS` mode the onboard watchdog governs the robot.

---

## 5. REST API Reference

Base URL: `http://localhost:8000`

### Health Check

```http
GET /api/v1/health
```
```json
{ "status": "HEALTHY", "service": "NexusOS Cloud Fleet Server", "version": "1.0.0" }
```

---

### List All Robots

```http
GET /api/v1/robots
```
```json
{
  "total_robots": 2,
  "robots": {
    "rover-alpha": {
      "robot_id": "rover-alpha",
      "robot_type": "DIFFERENTIAL_DRIVE",
      "mode": "MANUAL",
      "battery_percentage": 87.3,
      "pose": { "x": 1.24, "y": -0.33, "yaw": 0.52 }
    }
  }
}
```

---

### Get Robot Telemetry

```http
GET /api/v1/robots/{robot_id}
```
```bash
curl http://localhost:8000/api/v1/robots/rover-alpha
```

---

### Send Command to Robot

```http
POST /api/v1/robots/{robot_id}/command
Content-Type: application/json
```

**Velocity command (mobile base):**
```json
{
  "requested_mode": "MANUAL",
  "twist": {
    "linear":  { "x": 0.5, "y": 0.0, "z": 0.0 },
    "angular": { "x": 0.0, "y": 0.0, "z": 0.3 }
  }
}
```

**Joint position command (robotic arm):**
```json
{
  "requested_mode": "MANUAL",
  "joints": {
    "names":     ["joint_1", "joint_2", "joint_3"],
    "positions": [0.785, 1.047, -0.524]
  }
}
```

**Switch mode:**
```json
{ "requested_mode": "AUTONOMOUS" }
```

**Single-robot E-Stop:**
```json
{ "requested_mode": "EMERGENCY_STOP" }
```

**Response:**
```json
{ "status": "DISPATCHED", "robot_id": "rover-alpha", "command": { ... } }
```

---

### Global Fleet E-Stop

```http
POST /api/v1/fleet/estop
```
```bash
curl -X POST http://localhost:8000/api/v1/fleet/estop
```
```json
{ "status": "GLOBAL_EMERGENCY_STOP_EXECUTED", "robots_notified_count": 3 }
```

---

## 6. WebSocket Endpoints

### Edge Agent Stream

```
ws://localhost:8000/ws/telemetry/{robot_id}
```

- **Inbound (from Edge Agent):** Binary msgpack frames _or_ JSON telemetry
- **Outbound (to Edge Agent):** JSON `MotorCommandData` when a command is dispatched

---

### Dashboard Live Feed

```
ws://localhost:8000/ws/dashboard
```

**Server → Browser (fleet broadcast):**
```json
{
  "event": "FLEET_UPDATE",
  "fleet": {
    "rover-alpha": { "pose": { "x": 1.2, "y": 0.3 }, "mode": "MANUAL" }
  }
}
```

**Browser → Server (send command):**
```json
{
  "action": "COMMAND",
  "robot_id": "rover-alpha",
  "command": { "twist": { "linear": { "x": 0.3 } } }
}
```

**Browser → Server (fleet E-Stop):**
```json
{ "action": "GLOBAL_ESTOP" }
```

---

## 7. EdgeAgent Configuration

```python
from edge.config import EdgeConfig
from edge.agent import EdgeAgent
from hal.base import RobotType
import asyncio

config = EdgeConfig(
    robot_id             = "my-robot-01",
    robot_type           = RobotType.DIFFERENTIAL_DRIVE,
    server_url           = "ws://192.168.1.100:8000/ws/telemetry",
    telemetry_rate_hz    = 20.0,
    watchdog_timeout_sec = 2.0,
    reconnect_interval_sec = 3.0,
    use_protobuf         = True,
)

agent = EdgeAgent(config, hal_plugin=MyPlugin(config.robot_id))
asyncio.run(agent.start())
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_id` | `"robot-01"` | Unique identifier for this robot |
| `robot_type` | `DIFFERENTIAL_DRIVE` | Robot category (see table below) |
| `server_url` | `ws://localhost:8000/ws/telemetry` | Cloud WebSocket URL |
| `telemetry_rate_hz` | `20.0` | Telemetry publish frequency (1–50 Hz) |
| `watchdog_timeout_sec` | `2.0` | E-Stop if no command received in AUTONOMOUS mode |
| `reconnect_interval_sec` | `3.0` | Seconds between reconnect attempts on disconnect |
| `use_protobuf` | `True` | Binary msgpack vs plain JSON frames |

### Robot Types

| `RobotType` | Description |
|-------------|-------------|
| `DIFFERENTIAL_DRIVE` | 2-wheel mobile base |
| `ROBOTIC_ARM` | Serial manipulator |
| `DRONE` | Multirotor UAV |
| `LEGGED` | Quadruped / biped |
| `TRACKED` | Tank-drive tracked platform |
| `GENERIC` | Any custom hardware |

---

## 8. Writing a Custom HAL Plugin

Subclass `AbstractHardwarePlugin` and implement all 5 abstract methods:

```python
from hal.base import (
    AbstractHardwarePlugin, RobotTelemetryData, MotorCommandData,
    RobotType, RobotMode, PoseData, TwistData, BatteryData,
    JointStateData, Vector3D
)
import time

class MyRobotPlugin(AbstractHardwarePlugin):

    def __init__(self, robot_id: str):
        super().__init__(robot_id=robot_id)
        # Open your hardware driver here (serial, CAN, SDK, etc.)

    async def initialize(self) -> bool:
        """Open hardware connection. Return True on success."""
        self.current_mode = RobotMode.MANUAL
        self.is_initialized = True
        return True

    async def read_telemetry(self) -> RobotTelemetryData:
        """Read sensors and return normalized telemetry."""
        return RobotTelemetryData(
            robot_id      = self.robot_id,
            robot_type    = RobotType.GENERIC,
            timestamp_sec = time.time(),
            pose    = PoseData(
                position        = Vector3D(x=0.0, y=0.0, z=0.0),
                orientation_rpy = Vector3D(x=0.0, y=0.0, z=0.0),
            ),
            twist   = TwistData(
                linear  = Vector3D(x=0.0, y=0.0, z=0.0),
                angular = Vector3D(x=0.0, y=0.0, z=0.0),
            ),
            battery = BatteryData(voltage=12.0, percentage=100.0, is_charging=False),
            mode    = self.current_mode,
            joints  = JointStateData(names=[], positions=[], velocities=[], effort=[]),
            custom_metrics = {"my_sensor": 42.0},
            status_message = "OK",
        )

    async def send_command(self, command: MotorCommandData) -> bool:
        """Apply command to hardware. Return True if accepted."""
        if command.requested_mode:
            self.current_mode = command.requested_mode
        if command.target_twist:
            vx = command.target_twist.linear.x
            wz = command.target_twist.angular.z
            # Write to your driver here
        return True

    async def emergency_stop(self) -> bool:
        """Cut all motor outputs immediately."""
        self.current_mode = RobotMode.EMERGENCY_STOP
        return True

    async def shutdown(self) -> None:
        """Close hardware connections cleanly."""
        await self.emergency_stop()
```

> See **`demo/custom_hardware_plugin.py`** for a complete `QuadcopterDronePlugin` with
> 3D kinematics, altitude control, and battery simulation.

---

## 9. Built-in HAL Plugins

### `DiffDriveBasePlugin`

```python
from hal.plugins.diff_drive import DiffDriveBasePlugin

plugin = DiffDriveBasePlugin(
    robot_id     = "rover-alpha",
    wheel_track  = 0.45,   # metres between drive wheels
    wheel_radius = 0.08,   # metres
)
```

- Unicycle kinematic model using numpy-accelerated trig (~3x faster than `math`)
- Simulates encoder odometry, battery drain, and sensor noise
- Supports `MANUAL`, `AUTONOMOUS`, `EMERGENCY_STOP` modes

### `RoboticArmPlugin`

```python
from hal.plugins.robotic_arm import RoboticArmPlugin

plugin = RoboticArmPlugin(robot_id="arm-beta", num_joints=6)
```

- Joint position control with velocity-limited interpolation
- Safety joint-limit enforcement
- Full joint telemetry (names, positions, velocities, effort)

---

## 10. ROS 2 Bridge Integration

Bridge NexusOS telemetry directly to ROS 2 topics for **RViz2** or **rqt**:

```python
from hal.bridges.ros2_bridge import NexusROS2BridgeNode

bridge = NexusROS2BridgeNode(robot_id="rover-alpha")
# Publishes:
#   /rover-alpha/pose         → geometry_msgs/PoseStamped
#   /rover-alpha/joint_states → sensor_msgs/JointState
#   /rover-alpha/cmd_vel      → geometry_msgs/Twist
```

```bash
# In a ROS 2-sourced terminal
source /opt/ros/humble/setup.bash
ros2 topic echo /rover-alpha/pose
```

> Requires ROS 2 installed. Gracefully raises `RuntimeError` if `rclpy` is not found.

---

## 11. Licensing

**Dual-license model (NCL v1.0):**

| Tier | Eligibility | Cost |
|------|-------------|------|
| **Community** | Non-commercial, personal, academic, AGPL-compliant OSS | Free |
| **Commercial** | Any entity with >$100K USD gross annual revenue | Paid |

### Evaluation Sandbox Mode

When a commercial key is missing or expired:
- ✅ Hardware control, E-Stop, and motor drivers stay **fully operational**
- ⚠️ Compliance notices are logged
- ❌ Some fleet management features are restricted

### License API

```bash
# Verify JWT license token
curl -X POST http://localhost:8000/api/v1/license/verify \
  -H "Content-Type: application/json" \
  -d '{"token": "eyJhbGc..."}'

# Report quarterly usage
curl -X POST http://localhost:8000/api/v1/license/report_usage \
  -H "Content-Type: application/json" \
  -d '{"license_token": "...", "robot_count": 5, "revenue_usd": 250000}'
```

---

## 12. Troubleshooting

### Edge Agent cannot connect

```
[WARNING] Connection lost or unreachable. Retrying in 3s...
```
- Verify Cloud Server: `curl http://localhost:8000/api/v1/health`
- Check `server_url` in `EdgeConfig` — use the machine's actual IP, not `localhost`, if agents run on separate hardware
- Docker: `docker ps` → confirm `nexus-cloud-server` is `Up (healthy)`

### Dashboard shows no robots

- Confirm Edge Agents are running and log "Connected!"
- Call `GET /api/v1/robots` — if empty, agents have not connected yet
- Hard-refresh browser (`Ctrl+Shift+R`)

### `initialize()` returns `False`

- Check hardware driver is reachable (USB, serial, CAN)
- Linux: `sudo chmod 666 /dev/ttyUSB0` or add user to `dialout` group
- Confirm hardware is powered and not locked by another process

### Watchdog E-Stop fires in Autonomous mode

```
[ERROR] WATCHDOG TIMEOUT! No command for 2.35s. Triggering E-STOP!
```
- Autonomous mode requires commands at ≥ `1 / watchdog_timeout_sec` Hz
- Increase `watchdog_timeout_sec` in `EdgeConfig`, or ensure your autonomous controller sends regular heartbeats
- To recover: `curl -X POST http://localhost:8000/api/v1/robots/my-robot/command -d '{"requested_mode":"MANUAL"}'`

### Port 8000 already in use

```bash
# Windows
netstat -ano | findstr :8000
# macOS/Linux
lsof -i :8000
# Change port in docker-compose.yml → ports: - "8001:8000"
```

### Recompile Protobuf stubs

```bash
python scripts/compile_proto.py
# Output: proto/gen/*.py
```

---

## Quick Reference

```bash
# Install
pip install nexusos-fleet

# Run everything (local)
python scripts/run_sim.py

# Run everything (Docker)
docker-compose up --build

# Dashboard
open http://localhost:8000

# Health
curl http://localhost:8000/api/v1/health

# List robots
curl http://localhost:8000/api/v1/robots

# Drive rover-alpha forward at 0.5 m/s
curl -X POST http://localhost:8000/api/v1/robots/rover-alpha/command \
  -H "Content-Type: application/json" \
  -d '{"twist":{"linear":{"x":0.5},"angular":{"z":0.0}}}'

# Fleet E-Stop
curl -X POST http://localhost:8000/api/v1/fleet/estop
```

---

*NexusOS — Hardware-agnostic robotics fleet orchestration.*
*[GitHub](https://github.com/aachcoder47/NexusOS) · [PyPI](https://pypi.org/project/nexusos-fleet/)*
