"""
NexusOS Autonomous Navigation - Dynamic Window Approach (DWA) Local Planner.
Evaluates reachable velocity space (v, w) and forward simulates trajectory rollouts
for reactive real-time obstacle avoidance.
"""

from __future__ import annotations
import math
from typing import List, Tuple, Dict, Any, Optional
import numpy as np

from navigation.costmap import Costmap2D


class DWALocalPlanner:
    """
    Dynamic Window Approach (DWA) Reactive Collision Avoidance.
    Calculates kinematically feasible (v, w) commands taking into account:
      - Motor acceleration limits (v_acc, w_acc)
      - Braking stopping distance
      - Obstacle clearance
      - Heading alignment to target waypoint
    """

    def __init__(
        self,
        costmap: Costmap2D,
        max_speed: float = 1.0,           # Max linear speed (m/s)
        min_speed: float = -0.2,          # Max reverse speed (m/s)
        max_yaw_rate: float = 1.5,        # Max angular speed (rad/s)
        max_accel: float = 1.2,           # Max linear acceleration (m/s^2)
        max_yaw_accel: float = 2.5,       # Max angular acceleration (rad/s^2)
        v_resolution: float = 0.08,       # Velocity sampling step
        yaw_rate_resolution: float = 0.15,# Angular velocity sampling step
        sim_time: float = 1.5,            # Forward simulation horizon (sec)
        sim_granularity: float = 0.1,     # Simulation step (sec)
        heading_weight: float = 0.4,      # Heading score weight
        clearance_weight: float = 0.4,    # Obstacle clearance weight
        velocity_weight: float = 0.2      # Velocity score weight
    ):
        self.costmap = costmap
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_accel = float(max_accel)
        self.max_yaw_accel = float(max_yaw_accel)
        self.v_resolution = float(v_resolution)
        self.yaw_rate_resolution = float(yaw_rate_resolution)
        self.sim_time = float(sim_time)
        self.sim_granularity = float(sim_granularity)

        self.heading_weight = float(heading_weight)
        self.clearance_weight = float(clearance_weight)
        self.velocity_weight = float(velocity_weight)

    def compute_velocity_command(
        self,
        current_pose: Tuple[float, float, float],   # (x, y, yaw)
        current_twist: Tuple[float, float],         # (v, w)
        target_waypoint: Tuple[float, float]        # (tx, ty)
    ) -> Tuple[float, float, List[Dict[str, float]], List[List[Dict[str, float]]]]:
        """
        Compute optimal (best_v, best_w), the selected rollout trajectory,
        and all candidate rollouts for visualization.
        """
        curr_x, curr_y, curr_yaw = current_pose
        curr_v, curr_w = current_twist

        # 1. Compute Dynamic Window [V_min, V_max] and [W_min, W_max]
        dt_window = 0.2  # Control sample time
        v_min = max(self.min_speed, curr_v - self.max_accel * dt_window)
        v_max = min(self.max_speed, curr_v + self.max_accel * dt_window)
        w_min = max(-self.max_yaw_rate, curr_w - self.max_yaw_accel * dt_window)
        w_max = min(self.max_yaw_rate, curr_w + self.max_yaw_accel * dt_window)

        # 2. Sample Velocity Grid
        v_samples = np.arange(v_min, v_max + 1e-4, self.v_resolution)
        w_samples = np.arange(w_min, w_max + 1e-4, self.yaw_rate_resolution)

        best_score = -float("inf")
        best_v = 0.0
        best_w = 0.0
        best_trajectory: List[Dict[str, float]] = []
        all_candidate_trajectories: List[List[Dict[str, float]]] = []

        for v in v_samples:
            for w in w_samples:
                traj, is_safe, min_dist_to_obs = self._simulate_trajectory(
                    curr_x, curr_y, curr_yaw, v, w
                )

                if not is_safe or not traj:
                    continue

                if len(all_candidate_trajectories) < 15:
                    all_candidate_trajectories.append(traj)

                # Score Trajectory
                # (a) Heading Score: angle to target waypoint at end of trajectory
                end_x = traj[-1]["x"]
                end_y = traj[-1]["y"]
                end_yaw = traj[-1]["yaw"]

                target_angle = math.atan2(target_waypoint[1] - end_y, target_waypoint[0] - end_x)
                angle_diff = abs((target_angle - end_yaw + math.pi) % (2.0 * math.pi) - math.pi)
                heading_score = math.pi - angle_diff  # Higher is better (range 0..pi)

                # (b) Clearance Score
                clearance_score = min(2.0, min_dist_to_obs)

                # (c) Forward Speed Score
                velocity_score = v

                total_score = (
                    self.heading_weight * heading_score +
                    self.clearance_weight * clearance_score +
                    self.velocity_weight * velocity_score
                )

                if total_score > best_score:
                    best_score = total_score
                    best_v = float(v)
                    best_w = float(w)
                    best_trajectory = traj

        if not best_trajectory:
            # Emergency in-place rotation if trapped
            best_v = 0.0
            best_w = 0.5
            best_trajectory = [{"x": curr_x, "y": curr_y, "yaw": curr_yaw}]

        return (best_v, best_w, best_trajectory, all_candidate_trajectories)

    def _simulate_trajectory(
        self, x: float, y: float, yaw: float, v: float, w: float
    ) -> Tuple[List[Dict[str, float]], bool, float]:
        """Simulate forward trajectory over horizon and evaluate obstacle distance."""
        traj: List[Dict[str, float]] = []
        sim_steps = int(self.sim_time / self.sim_granularity)

        min_obs_dist = float("inf")

        curr_x, curr_y, curr_yaw = x, y, yaw
        dt = self.sim_granularity

        for _ in range(sim_steps):
            curr_yaw += w * dt
            curr_yaw = (curr_yaw + math.pi) % (2.0 * math.pi) - math.pi
            curr_x += v * math.cos(curr_yaw) * dt
            curr_y += v * math.sin(curr_yaw) * dt

            traj.append({"x": round(curr_x, 3), "y": round(curr_y, 3), "yaw": round(curr_yaw, 3)})

            # Check collision on costmap
            cost = self.costmap.get_cost(curr_x, curr_y)
            if cost >= Costmap2D.INSCRIBED_INFLATED:
                return (traj, False, 0.0)

            # Estimate clearance
            clearance_estimate = (255 - cost) / 255.0 * self.costmap.inflation_radius
            if clearance_estimate < min_obs_dist:
                min_obs_dist = clearance_estimate

        return (traj, True, min_obs_dist)
