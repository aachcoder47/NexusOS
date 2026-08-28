# NexusOS Fleet Demo Suite (v1.0.1)

Welcome to the **NexusOS Fleet** demo repository! This project showcases the capabilities of the [`nexusos-fleet`](https://pypi.org/project/nexusos-fleet/) PyPI package for universal robotics hardware abstraction, real-time edge telemetry, and cloud fleet management.

---

## 📦 What's Inside This Demo

| Script | Description |
|---|---|
| [`main.py`](main.py) | **Interactive Demo Launcher** — Run full missions, benchmarks, or the cloud fleet server. |
| [`mission_controller.py`](mission_controller.py) | **Autonomous Multi-Robot Orchestration** — Controls a Rover, 6-DOF Arm, and Drone concurrently. |
| [`custom_hardware_plugin.py`](custom_hardware_plugin.py) | **Plugin Extensibility** — Demonstrates building a custom `QuadcopterDronePlugin` with 3D kinematics. |
| [`live_dashboard_client.py`](live_dashboard_client.py) | **Live Terminal Monitor** — Subscribes to the WebSocket broadcast and renders real-time telemetry tables. |
| [`requirements.txt`](requirements.txt) | Minimal dependencies needed to run the demo suite. |

---

## 🚀 Quickstart

### 1. Install Dependencies
```bash
# Create virtual environment (optional)
python -m venv venv
venv\Scripts\activate       # On Windows
source venv/bin/activate    # On Linux/macOS

# Install demo requirements (pulls nexusos-fleet from PyPI)
pip install -r requirements.txt
```

### 2. Run the Interactive Suite
```bash
python main.py
```

You will see an interactive menu:
```text
========================================================================
     _   _                       ___  ____  
    | \ | | _____  ___   _ ___  / _ \/ ___| 
    |  \| |/ _ \ \/ / | | / __|| | | \___ \ 
    | |\  |  __/>  <| |_| \__ \| |_| |___) |
    |_| \_|\___/_/\_\\__,_|___/ \___/|____/ 
                                            
    Universal Robotics Abstraction & Cloud Fleet Middleware (v1.0.1)
========================================================================

Select a demo experience to run:
  [1] Autonomous Multi-Robot Mission (Rover + Arm + Custom Drone)
  [2] Full Cloud Fleet Stack (FastAPI Server + Simulated Agents + Dashboard)
  [3] Custom Hardware Plugin Demo (Autonomous Quadcopter UAV)
  [4] MsgPack Telemetry Performance & Latency Benchmark
  [5] Exit
```

---

## 🛠️ Individual Scenarios

### Scenario A: Standalone Multi-Robot Mission
Run autonomous choreography directly without running an external server:
```bash
python mission_controller.py
```
- **Rover Alpha**: Executes square navigation waypoints with odometry tracking.
- **Arm Beta**: Performs a 6-DOF pick-and-place joint trajectory cycle with gripper actuation.
- **Drone Gamma**: Handles takeoff, 3D waypoint survey, hover scans, and landing.

### Scenario B: Cloud Fleet Stack & Web Dashboard
Launch the FastAPI backend and simulated edge robots:
```bash
# Start the stack (Option 2 in main.py)
python -c "import asyncio; from demo.main import run_full_cloud_stack; asyncio.run(run_full_cloud_stack())"
```
Open your browser to: **`http://localhost:8000`** to view the live dashboard!

### Scenario C: Live Terminal Telemetry Monitor
In a separate terminal, monitor real-time WebSocket telemetry:
```bash
python live_dashboard_client.py
```

### Scenario D: Building Custom Plugins
Extend `AbstractHardwarePlugin` to connect your own robot:
```python
from hal.base import AbstractHardwarePlugin, RobotType, RobotTelemetryData, MotorCommandData

class MyCustomRobotPlugin(AbstractHardwarePlugin):
    def __init__(self, robot_id: str):
        super().__init__(robot_id=robot_id, robot_type=RobotType.CUSTOM)

    async def initialize(self) -> bool:
        # Connect to your CAN bus, serial port, or ROS 2 node
        return True

    async def read_telemetry(self) -> RobotTelemetryData:
        # Return standard NexusOS telemetry frame
        ...

    async def send_command(self, command: MotorCommandData) -> bool:
        # Dispatch velocity vectors or joint targets
        ...
```

---

## ⚡ Performance
NexusOS features a high-throughput MsgPack binary serialization pipeline achieving **>100,000 frames/second** with deadline-compensated timing to ensure determinism across edge computing platforms.
