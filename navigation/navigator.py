"""
NexusOS Autonomous Navigation - Navigation Executive & Recovery State Machine.
Coordinates Global Planners (A*/Theta*/RRT*/D* Lite), Local Planners (DWA/Pure Pursuit),
Multi-layer Costmaps, and automatic obstacle recovery behaviors.
"""

from __future__ import annotations
import math
import time
import logging
from enum import Enum
from typing import List, Tuple, Optional, Dict, Any

from navigation.costmap import Costmap2D
from navigation.planners.astar import AStarPlanner
from navigation.planners.theta_star import ThetaStarPlanner
from navigation.planners.rrt_star import RRTStarPlanner
from navigation.planners.d_star_lite import DStarLitePlanner
from navigation.local_planners.dwa import DWALocalPlanner
from navigation.local_planners.pure_pursuit import PurePursuitController

logger = logging.getLogger("NexusOS.Navigator")


class NavigationState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    OBSTACLE_AVOIDANCE = "OBSTACLE_AVOIDANCE"
    RECOVERY_ROTATE = "RECOVERY_ROTATE"
    RECOVERY_BACKUP = "RECOVERY_BACKUP"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"


class AutonomousNavigator:
    """
    Complete Autonomous Navigation Executive for heterogeneous mobile robots.
    """

    def __init__(
        self,
        costmap: Optional[Costmap2D] = None,
        planner_type: str = "A_STAR",  # "A_STAR", "THETA_STAR", "RRT_STAR", "D_STAR_LITE"
        local_planner_type: str = "DWA" # "DWA", "PURE_PURSUIT"
    ):
        self.costmap = costmap if costmap else Costmap2D()
        self.planner_type = planner_type
        self.local_planner_type = local_planner_type

        # Instantiate planners
        self.astar_planner = AStarPlanner(self.costmap)
        self.theta_planner = ThetaStarPlanner(self.costmap)
        self.rrt_planner = RRTStarPlanner(self.costmap)
        self.dstar_planner = DStarLitePlanner(self.costmap)

        self.dwa_planner = DWALocalPlanner(self.costmap)
        self.pure_pursuit = PurePursuitController()

        # State
        self.state: NavigationState = NavigationState.IDLE
        self.current_goal: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.current_path: List[Tuple[float, float]] = []
        self.lookahead_point: Optional[Tuple[float, float]] = None
        self.candidate_trajectories: List[List[Dict[str, float]]] = []

        # Recovery timers
        self._recovery_start_time: float = 0.0
        self._recovery_duration: float = 2.0
        self._consecutive_replans: int = 0
        self._max_replans: int = 4

        # Mission tracking
        self.distance_to_goal: float = 0.0
        self.total_path_length: float = 0.0
        self.start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def set_goal(
        self,
        start_pose: Tuple[float, float, float],   # (x, y, yaw)
        goal_pose: Tuple[float, float, float]     # (gx, gy, gyaw)
    ) -> bool:
        """Dispatch a new navigation goal and trigger global planning."""
        self.start_pose = start_pose
        self.current_goal = goal_pose
        self.state = NavigationState.PLANNING

        sx, sy, _ = start_pose
        gx, gy, _ = goal_pose

        logger.info(f"Received Nav Goal: ({gx:.2f}, {gy:.2f}) from Start: ({sx:.2f}, {sy:.2f}) using {self.planner_type}")

        path = self._execute_global_plan((sx, sy), (gx, gy))

        if not path or len(path) < 2:
            logger.warning(f"Failed to find collision-free global path to goal ({gx:.2f}, {gy:.2f})!")
            self.state = NavigationState.FAILED
            self.current_path = []
            return False

        self.current_path = path
        self.total_path_length = self._calculate_path_length(path)
        self.state = NavigationState.NAVIGATING
        self._consecutive_replans = 0
        logger.info(f"Global Path planned successfully: {len(path)} waypoints, Length: {self.total_path_length:.2f}m")
        return True

    def cancel_goal(self):
        """Cancel active navigation and return to IDLE."""
        logger.info("Navigation goal cancelled by user/cloud.")
        self.state = NavigationState.IDLE
        self.current_goal = None
        self.current_path = []
        self.lookahead_point = None
        self.candidate_trajectories = []

    def update(
        self,
        current_pose: Tuple[float, float, float],   # (x, y, yaw)
        current_twist: Tuple[float, float],         # (v, w)
        lidar_points: Optional[List[Dict[str, float]]] = None
    ) -> Tuple[float, float]:
        """
        Step the navigation state machine and compute (target_v, target_w) motor commands.
        """
        rx, ry, ryaw = current_pose
        rv, rw = current_twist

        # 1. Update Costmap with LiDAR points if provided
        if lidar_points:
            self.costmap.update_from_lidar(lidar_points)

        # 2. State Machine Execution
        if self.state == NavigationState.IDLE or not self.current_goal:
            return (0.0, 0.0)

        gx, gy, gyaw = self.current_goal
        self.distance_to_goal = math.hypot(gx - rx, gy - ry)

        # Goal arrived check
        if self.distance_to_goal <= 0.20:
            logger.info(f"Target goal ({gx:.2f}, {gy:.2f}) reached successfully!")
            self.state = NavigationState.GOAL_REACHED
            self.current_path = []
            self.lookahead_point = None
            return (0.0, 0.0)

        # State transitions
        if self.state == NavigationState.PLANNING:
            success = self.set_goal(current_pose, self.current_goal)
            if not success:
                return (0.0, 0.0)

        elif self.state == NavigationState.RECOVERY_ROTATE:
            elapsed = time.time() - self._recovery_start_time
            if elapsed < self._recovery_duration:
                return (0.0, 0.8)  # Rotate in place to clear field of view
            else:
                # Attempt replan after rotation
                logger.info("Recovery rotation complete. Attempting replan...")
                self.state = NavigationState.PLANNING
                return (0.0, 0.0)

        elif self.state == NavigationState.RECOVERY_BACKUP:
            elapsed = time.time() - self._recovery_start_time
            if elapsed < 1.0:
                return (-0.2, 0.0)  # Gentle reverse
            else:
                self.state = NavigationState.RECOVERY_ROTATE
                self._recovery_start_time = time.time()
                return (0.0, 0.0)

        # 3. Path Validation & Local Planning
        if not self.current_path:
            self.state = NavigationState.PLANNING
            return (0.0, 0.0)

        # Check if ahead path is blocked by dynamic obstacle
        if self._is_path_blocked():
            logger.warning("Dynamic obstacle detected on active path! Triggering real-time replanning...")
            self._consecutive_replans += 1
            if self._consecutive_replans > self._max_replans:
                logger.warning("Path repeatedly blocked. Initiating recovery backup...")
                self.state = NavigationState.RECOVERY_BACKUP
                self._recovery_start_time = time.time()
                return (0.0, 0.0)

            # Replan from current pose
            new_path = self._execute_global_plan((rx, ry), (gx, gy))
            if new_path:
                self.current_path = new_path
                self.total_path_length = self._calculate_path_length(new_path)
            else:
                self.state = NavigationState.OBSTACLE_AVOIDANCE

        # 4. Local Trajectory Generation (DWA / Pure Pursuit)
        if self.local_planner_type == "DWA":
            # Select target waypoint on path (look ahead ~1.5m)
            target_wpt = self._find_local_target_waypoint((rx, ry))
            self.lookahead_point = target_wpt

            cmd_v, cmd_w, best_traj, all_trajs = self.dwa_planner.compute_velocity_command(
                current_pose, current_twist, target_wpt
            )
            self.candidate_trajectories = all_trajs
            return (cmd_v, cmd_w)

        else:  # Pure Pursuit
            cmd_v, cmd_w, target_pt, reached = self.pure_pursuit.compute_control(
                current_pose, rv, self.current_path
            )
            self.lookahead_point = target_pt
            if reached:
                self.state = NavigationState.GOAL_REACHED
                return (0.0, 0.0)
            return (cmd_v, cmd_w)

    def _execute_global_plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        if self.planner_type == "THETA_STAR":
            return self.theta_planner.plan(start, goal)
        elif self.planner_type == "RRT_STAR":
            return self.rrt_planner.plan(start, goal)
        elif self.planner_type == "D_STAR_LITE":
            return self.dstar_planner.plan(start, goal)
        else:  # Default A*
            return self.astar_planner.plan(start, goal, smooth=True)

    def _is_path_blocked(self, lookahead_points: int = 5) -> bool:
        """Check if next few waypoints on path have lethal cost."""
        if not self.current_path:
            return False
        pts_to_check = min(lookahead_points, len(self.current_path))
        for i in range(pts_to_check):
            px, py = self.current_path[i]
            if not self.costmap.is_collision_free(px, py):
                return True
        return False

    def _find_local_target_waypoint(self, robot_pos: Tuple[float, float], lookahead_m: float = 1.5) -> Tuple[float, float]:
        """Find the point along current_path that is roughly lookahead_m ahead of robot."""
        rx, ry = robot_pos
        for wpt in self.current_path:
            d = math.hypot(wpt[0] - rx, wpt[1] - ry)
            if d >= lookahead_m:
                return wpt
        return self.current_path[-1]

    def _calculate_path_length(self, path: List[Tuple[float, float]]) -> float:
        total = 0.0
        for i in range(len(path) - 1):
            total += math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        return total

    def to_dict(self) -> Dict[str, Any]:
        """Export navigation status for telemetry and cloud streaming."""
        return {
            "state": self.state.value,
            "planner_type": self.planner_type,
            "local_planner_type": self.local_planner_type,
            "has_active_goal": self.current_goal is not None,
            "goal": {"x": self.current_goal[0], "y": self.current_goal[1], "yaw": self.current_goal[2]} if self.current_goal else None,
            "distance_to_goal": round(self.distance_to_goal, 2),
            "total_path_length": round(self.total_path_length, 2),
            "waypoints_remaining": len(self.current_path),
            "path": [{"x": p[0], "y": p[1]} for p in self.current_path],
            "lookahead_point": {"x": self.lookahead_point[0], "y": self.lookahead_point[1]} if self.lookahead_point else None,
            "candidate_trajectories": self.candidate_trajectories[:6]
        }
