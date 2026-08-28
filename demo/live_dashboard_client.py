"""
Demo: Real-time Terminal Telemetry Dashboard for NexusOS Fleet.

Subscribes to the NexusOS Cloud Fleet WebSocket stream (`/ws/dashboard`)
and displays live status, positions, battery state, and alarms.
"""

import asyncio
import json
import sys
import time
from typing import Dict, Any

try:
    import websockets
except ImportError:
    print("Error: websockets is required. Install via `pip install websockets`")
    sys.exit(1)


def format_dashboard(fleet_data: Dict[str, Any]):
    """Renders formatted ASCII table of current fleet status."""
    sys.stdout.write("\033[2J\033[H")  # Clear terminal screen
    print("================================================================================")
    print("                      NEXUSOS LIVE FLEET TELEMETRY DASHBOARD                    ")
    print("================================================================================")
    print(f"Server Time: {time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime())}")
    print("--------------------------------------------------------------------------------")
    print(f"{'ROBOT ID':<15} | {'TYPE':<16} | {'MODE':<12} | {'BATTERY':<8} | {'POSITION (X,Y,Z)':<20}")
    print("--------------------------------------------------------------------------------")

    if not fleet_data:
        print("  Waiting for active robot telemetry streams...")
    else:
        for robot_id, telem in fleet_data.items():
            r_type = telem.get("robot_type", "UNKNOWN")
            r_mode = telem.get("mode", "UNKNOWN")
            batt = telem.get("battery", {})
            battery_pct = batt.get("percentage", batt.get("soc_percentage", 0.0))
            soc = f"{battery_pct:.1f}%"
            pose = telem.get("pose", {}).get("position", {})
            pos_str = f"({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f}, {pose.get('z', 0):.2f})"
            print(f"{robot_id:<15} | {r_type:<16} | {r_mode:<12} | {soc:<8} | {pos_str:<20}")

    print("--------------------------------------------------------------------------------")
    print("Press Ctrl+C to exit dashboard monitor.")


async def listen_dashboard(server_url: str = "ws://127.0.0.1:8000/ws/dashboard"):
    """Connects to NexusOS Fleet WebSocket and monitors broadcast."""
    print(f"Connecting to live telemetry broadcast at {server_url}...")
    fleet_cache: Dict[str, Any] = {}

    while True:
        try:
            async with websockets.connect(server_url) as ws:
                print("Connected! Streaming live fleet data...\n")
                while True:
                    msg = await ws.recv()
                    data = json.loads(msg)
                    
                    if "robots" in data and isinstance(data["robots"], dict):
                        fleet_cache.update(data["robots"])
                    elif "robot_id" in data:
                        fleet_cache[data["robot_id"]] = data
                    
                    format_dashboard(fleet_cache)

        except (websockets.exceptions.ConnectionClosed, OSError) as e:
            print(f"Disconnected from server ({e}). Reconnecting in 2s...")
            await asyncio.sleep(2.0)
        except Exception as e:
            print(f"Dashboard monitor error: {e}")
            await asyncio.sleep(2.0)


if __name__ == "__main__":
    url = sys.argv[1] if len(sys.argv) > 1 else "ws://127.0.0.1:8000/ws/dashboard"
    try:
        asyncio.run(listen_dashboard(url))
    except KeyboardInterrupt:
        print("\nDashboard monitor exited.")
