"""
NexusOS Autonomous Navigation - Adaptive Pure Pursuit Path Follower.
Implements curvature-based trajectory tracking with lookahead distance scaling and goal deceleration ramp.
"""

from __future__ import annotations
import math
from typing import List, Tuple, Optional, Dict, Any
import numpy as np


class PurePursuitController:
    """
    Adaptive Pure Pursuit Controller with PID speed profiling.
    """

    def __init__(
        self,
        min_lookahead_dist: float = 0.4,
        max_lookahead_dist: float = 1.5,
        lookahead_ratio: float = 0.8,      # Ld = min + ratio * v
        target_speed: float = 0.8,         # Cruise velocity m/s
        goal_tolerance_m: float = 0.15,    # 15 cm arrival tolerance
        max_angular_vel: float = 1.5       # Max rotational rate rad/s
    ):
        self.min_lookahead_dist = float(min_lookahead_dist)
        self.max_lookahead_dist = float(max_lookahead_dist)
        self.lookahead_ratio = float(lookahead_ratio)
        self.target_speed = float(target_speed)
        self.goal_tolerance_m = float(goal_tolerance_m)
        self.max_angular_vel = float(max_angular_vel)

    def compute_control(
        self,
        current_pose: Tuple[float, float, float],       # (x, y, yaw)
        current_v: float,
        path: List[Tuple[float, float]]
    ) -> Tuple[float, float, Optional[Tuple[float, float]], bool]:
        """
        Compute (target_v, target_w), lookahead_point (x, y), and goal_reached boolean flag.
        """
        if not path:
            return (0.0, 0.0, None, True)

        rx, ry, ryaw = current_pose
        goal_x, goal_y = path[-1]

        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)
        if dist_to_goal <= self.goal_tolerance_m:
            return (0.0, 0.0, (goal_x, goal_y), True)

        # 1. Compute dynamic lookahead distance
        ld = min(self.max_lookahead_dist, max(self.min_lookahead_dist, self.min_lookahead_dist + self.lookahead_ratio * abs(current_v)))

        # 2. Find lookahead target point along path
        target_pt = self._find_lookahead_point((rx, ry), ld, path)

        # 3. Transform target point into robot local coordinates
        dx = target_pt[0] - rx
        dy = target_pt[1] - ry

        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)

        # Local coordinates
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        # 4. Pure Pursuit Curvature calculation: kappa = 2 * local_y / (Ld^2)
        dist_to_target = math.hypot(local_x, local_y)
        if dist_to_target < 1e-4:
            return (0.0, 0.0, target_pt, False)

        curvature = 2.0 * local_y / (dist_to_target * dist_to_target)

        # 5. Speed Profiling & Goal Deceleration Ramp
        decel_distance = 1.2
        if dist_to_goal < decel_distance:
            v_cmd = self.target_speed * (dist_to_goal / decel_distance)
            v_cmd = max(0.15, v_cmd)  # Minimum creep speed to overcome friction
        else:
            v_cmd = self.target_speed

        # Curvature speed slowdown on sharp turns
        v_cmd = v_cmd / (1.0 + 1.5 * abs(curvature))

        # 6. Angular Velocity: w = curvature * v
        w_cmd = curvature * v_cmd
        w_cmd = max(-self.max_angular_vel, min(self.max_angular_vel, w_cmd))

        return (round(v_cmd, 3), round(w_cmd, 3), target_pt, False)

    def _find_lookahead_point(
        self, robot_pos: Tuple[float, float], lookahead: float, path: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """Find the furthest point along the path that is at distance >= lookahead."""
        rx, ry = robot_pos
        best_pt = path[-1]

        for i in range(len(path) - 1, -1, -1):
            d = math.hypot(path[i][0] - rx, path[i][1] - ry)
            if d <= lookahead:
                if i + 1 < len(path):
                    best_pt = path[i + 1]
                else:
                    best_pt = path[i]
                break

        return best_pt
